#!/usr/bin/env python3
"""HOME before each strike; default sweep: 300..900 deg/s, 20 trials each.

Run from Pixi: pixi run motor-sweep [--mode BLE | --mode USB --port PORT].
This is a hardware experiment runner. --dry-run only prints the plan.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import datetime as dt
import json
import math
import signal
import sys
import time
from dataclasses import dataclass, asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from hammer_protocol import LineBuffer, TraceReceiver, TraceReplacedError, analyze_trace, fields

RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
ALIASES = {'Rotary_Hammer_BLE', 'Neuro_Hammer_BLE'}
DEFAULT_SPEEDS = tuple(range(300, 901, 100))


class SweepError(RuntimeError):
    pass


def finite_number(values, key):
    try:
        value = float(values[key])
    except (KeyError, ValueError) as exc:
        raise SweepError(f'设备字段 {key} 无效') from exc
    if not math.isfinite(value):
        raise SweepError(f'设备字段 {key} 不是有限数')
    return value


@dataclass(frozen=True)
class Config:
    speeds: tuple[int, ...] = DEFAULT_SPEEDS
    repeats: int = 20
    angle: float = 50.0
    pause: float = 2.0
    grade_pause: float = 5.0
    mode: str = 'BLE'
    device: str = 'Rotary_Hammer_BLE'
    port: str | None = None
    # HOME uses the firmware's 40 deg/s approach, not JOG + arbitrary ZERO.
    settle_seconds: float = .2
    still_timeout: float = 5.0
    command_timeout: float = 2.0
    home_timeout: float = 7.0
    retract_timeout: float = 10.0
    strike_timeout: float = 4.0
    trace_timeout: float = 300.0
    chunk_timeout: float = 4.0
    master_port: str | None = None
    accel_min_peak: float = 100.

    def validate(self):
        if not self.speeds or any(not isinstance(s, int) or not 0 < s <= 3000 for s in self.speeds):
            raise ValueError('速度必须为 1..3000 的整数；输入范围不代表机械允许范围')
        if len(set(self.speeds)) != len(self.speeds):
            raise ValueError('速度档位不能重复')
        if not isinstance(self.repeats, int) or self.repeats <= 0:
            raise ValueError('每档次数必须为正整数')
        if not math.isfinite(self.angle) or not 0 < self.angle <= 360:
            raise ValueError('回撤角度需为 0..360° 内的正数')
        for key in ('pause', 'grade_pause', 'settle_seconds'):
            value = getattr(self, key)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} 不能为负数或非有限数')
        for key in ('still_timeout', 'command_timeout', 'home_timeout', 'retract_timeout',
                    'strike_timeout', 'trace_timeout', 'chunk_timeout'):
            if not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'{key} 必须为有限正数')
        if self.mode not in ('USB', 'BLE') or (self.mode == 'USB' and not self.port):
            raise ValueError('USB 模式须用 --port 指定电机端口')
        if self.mode == 'BLE' and self.port:
            raise ValueError('--port 只用于USB；请同时指定 --mode USB')
        if not math.isfinite(self.accel_min_peak) or self.accel_min_peak<=0:
            raise ValueError('加速度阈值必须为有限正数')
        if self.master_port and self.master_port==self.port:
            raise ValueError('Master与电机不能使用同一串口')

    def plan(self):
        return [(speed, trial) for speed in self.speeds for trial in range(1, self.repeats + 1)]


class HardwareLink:
    """One asyncio connection, no GUI and no reconnect/action retries."""
    def __init__(self, config: Config):
        self.config = config
        self.queue = asyncio.Queue()
        self.lines = LineBuffer()
        self.serial = self.client = self.reader = None
        self.closed = False

    async def connect(self):
        if self.config.mode == 'USB':
            import serial
            from serial.tools import list_ports
            info=next((p for p in list_ports.comports() if p.device==self.config.port),None)
            if info and (info.serial_number or '').replace(':','').upper()=='1CDBD499C8A0':
                raise SweepError('此串口属于Master，禁止作为电机端口')
            self.serial = serial.Serial(self.config.port, 115200, timeout=0, write_timeout=.3)
            await asyncio.sleep(1.2)
            self.serial.reset_input_buffer()
            self.reader = asyncio.create_task(self._read_usb())
        else:
            from bleak import BleakClient, BleakScanner
            identifier = self.config.device
            def match(device, adv):
                names = {device.name, device.address, adv.local_name}
                return identifier in names or (identifier in ALIASES and bool(names & ALIASES))
            device = await BleakScanner.find_device_by_filter(match, timeout=8)
            if device is None:
                raise SweepError(f'未找到电机 BLE：{identifier}')
            loop = asyncio.get_running_loop()
            def lost(_):
                if not self.closed:
                    loop.call_soon_threadsafe(self.queue.put_nowait, SweepError('BLE 已断开'))
            self.client = BleakClient(device, timeout=12, disconnected_callback=lost)
            await self.client.connect()
            required = {RX_UUID.lower(), TX_UUID.lower()}
            available = {ch.uuid.lower() for service in self.client.services for ch in service.characteristics}
            if not required <= available:
                raise SweepError('连接的设备缺少电机 UART 服务')
            def notify(_, data):
                loop.call_soon_threadsafe(self._feed, bytes(data))
            await self.client.start_notify(TX_UUID, notify)

    def _feed(self, data):
        if not self.closed:
            for line in self.lines.feed(data):
                self.queue.put_nowait(line)

    async def _read_usb(self):
        try:
            while not self.closed:
                self._feed(self.serial.read(min(max(1, self.serial.in_waiting), 8192)))
                await asyncio.sleep(.005)
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.queue.put_nowait(SweepError(f'USB 读取失败：{exc}'))

    async def send(self, command):
        data = (command + '\n').encode('ascii')
        if self.serial is not None and self.serial.is_open:
            if self.serial.write(data) != len(data):
                raise SweepError('USB 命令写入不完整')
        elif self.client is not None and self.client.is_connected:
            for offset in range(0, len(data), 20):
                await self.client.write_gatt_char(RX_UUID, data[offset:offset + 20], response=True)
        else:
            raise SweepError('电机未连接；不会自动重连或重复动作')

    async def receive(self, timeout):
        item = await asyncio.wait_for(self.queue.get(), timeout)
        if isinstance(item, Exception):
            raise item
        return item

    async def close(self):
        self.closed = True
        if self.reader is not None:
            self.reader.cancel()
            await self.reader
        if self.serial is not None:
            self.serial.close()
        if self.client is not None and self.client.is_connected:
            await self.client.disconnect()


SUMMARY_FIELDS = ('number', 'speed_dps', 'trial', 'retract_deg', 'fire_state', 'result', 'shot',
                  'reason', 'contact_candidate', 'early_drop_suspected', 'release_angle_deg',
                  'release_speed_dps', 'home_travel_deg', 'ready_angle_deg', 'trace_samples',
                  'speed_qualified', 'pre_drop_speed_dps', 'stable_ms', 'error',
                  'home_zero_shift_deg','accel_peak_counts','accel_raw_peak_counts','accel_peak_time_us',
                  'accel_peak_seq','accel_threshold','accel_gap_count','accel_link_fault','accel_no_event',
                  'accel_multi_event','accel_window_complete','quality_qualified')


class RunFiles:
    def __init__(self, parent: Path, config: Config):
        # A new directory for every invocation, including repeated same-second runs.
        self.directory = parent / f'motor_sweep_{dt.datetime.now():%Y%m%d_%H%M%S_%f}'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log_file = (self.directory / 'communication.log').open('w', encoding='utf-8')
        self.csv_file = (self.directory / 'summary.csv').open('w', newline='', encoding='utf-8-sig')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=SUMMARY_FIELDS, extrasaction='ignore')
        self.writer.writeheader(); self.csv_file.flush()
        self.config = config
        self.rows = []
        self.started = dt.datetime.now().isoformat(timespec='milliseconds')
        self.write_manifest('created')

    def log(self, message):
        self.log_file.write(f'[{dt.datetime.now().isoformat(timespec="milliseconds")}] {message}\n')
        self.log_file.flush()

    def json(self, relative, value):
        destination = self.directory / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + '.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(destination)

    def write_manifest(self, status, error='', stop_error=''):
        self.json('run.json', {'status': status, 'started': self.started,
            'updated': dt.datetime.now().isoformat(timespec='milliseconds'), 'config': asdict(self.config),
            'planned_fires': len(self.config.plan()), 'recorded_trials': len(self.rows),
            'completed_fires': sum(row.get('fire_state') == 'completed' for row in self.rows),
            'error': error, 'stop_error': stop_error})

    def save_trial(self, row, details, points):
        relative = Path(f'speed_{row["speed_dps"]}') / f'trial_{row["trial"]:02d}'
        self.json(relative / 'result.json', details)
        if points:
            destination = self.directory / relative / 'trace.csv'
            with destination.open('w', newline='', encoding='utf-8-sig') as file:
                writer = csv.DictWriter(file, fieldnames=(*points[0].to_dict(), 'target_dps'))
                writer.writeheader()
                for point in points:
                    writer.writerow({**point.to_dict(), 'target_dps': row['speed_dps']})
        self.rows.append(dict(row))
        self.writer.writerow(row); self.csv_file.flush()
        self.write_manifest('running')

    def close(self):
        self.log_file.close(); self.csv_file.close()


class MotorSweep:
    def __init__(self, config: Config, link, files: RunFiles, progress=print, capture=None):
        config.validate()
        self.config, self.link, self.files, self.progress = config, link, files, progress
        self.request_id = 0
        self.info = self.tuning = None
        self.stopped = False
        self.capture=capture
        if config.master_port and capture is None:
            from serial.tools import list_ports
            port=next((p for p in list_ports.comports() if p.device==config.master_port),None)
            if port and (port.serial_number or '').replace(':','').upper()=='C49E7E17C32C':
                raise SweepError('指定的Master串口实际是Motor，已拒绝连接')
            files.json('master_identity.json',{'port':config.master_port,'serial_number':port.serial_number if port else None,
                'hwid':port.hwid if port else None})
            from motor_accel_capture import AccelCapture
            self.capture=AccelCapture(config.master_port,files.directory)
        self.zero_shift=0.;self.home_count=0
        self.pose_reference={}

    def check_pose(self,phase,details):
        if not self.capture:return
        pose=self.capture.pose()
        reference=self.pose_reference.setdefault(phase,pose)
        difference=math.sqrt(sum((a-b)**2 for a,b in zip(pose,reference)))
        details[phase+'_pose']={'counts':pose,'reference_counts':reference,'difference_counts':difference,'limit_counts':6}
        if difference>6:
            raise SweepError(f'{phase}静态姿态与本组初始位置不一致：{difference:.2f} counts')

    async def send(self, command):
        self.files.log('TX ' + command)
        try:
            await asyncio.wait_for(self.link.send(command), self.config.command_timeout)
        except asyncio.TimeoutError as exc:
            raise SweepError(f'发送 {command} 超时；未自动重发') from exc

    async def receive(self, timeout, *, stopping=False):
        line = await self.link.receive(timeout)
        self.files.log('RX ' + line)
        if line.startswith('ERR '):
            raise SweepError(line)
        if line.startswith('EVT ABORT ') and not stopping:
            raise SweepError(line)
        if line.startswith('HELLO ') and self.info is not None:
            raise SweepError('测试中固件重新启动，停止本轮测试')
        return line

    async def response(self, prefix, timeout, *, stopping=False):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SweepError(f'等待 {prefix.strip()} 超时；动作命令未自动重发')
            try:
                line = await self.receive(remaining, stopping=stopping)
            except asyncio.TimeoutError as exc:
                raise SweepError(f'等待 {prefix.strip()} 超时；动作命令未自动重发') from exc
            if line.startswith(prefix):
                return fields(line)

    async def command(self, command, prefix, timeout=None):
        await self.send(command)
        return await self.response(prefix, timeout or self.config.command_timeout)

    async def synchronize(self):
        info = await self.command('INFO', 'INFO ')
        required = {'HOME', 'TRACE', 'TRACE_CRC', 'TRACE_REQUEST_ID', 'BOUNDED_STRIKE', 'BOUNDED_HOME'}
        if info.get('protocol') != '2' or not required <= set(info.get('features', '').split(',')):
            raise SweepError('需要V12：协议2、CRC轨迹及BOUNDED_STRIKE/BOUNDED_HOME行程保护')
        self.info = info
        self.tuning = await self.command('TUNING', 'TUNING ')
        self.files.json('device.json', {'info': info, 'tuning': self.tuning})

    async def wait_still(self, *, zeroed=False):
        deadline = time.monotonic() + self.config.still_timeout
        stable = None; anchor = None
        while time.monotonic() < deadline:
            status = await self.command('STATUS', 'STATUS ')
            try:
                angle, speed = finite_number(status, 'angle'), finite_number(status, 'speed')
                valid = math.isfinite(angle) and math.isfinite(speed)
            except (KeyError, ValueError) as exc:
                raise SweepError('状态回复无效') from exc
            if not valid or status.get('state') != 'IDLE' or status.get('homing') != '0':
                raise SweepError('归零/叩击前设备未处于有效IDLE状态：' + str(status))
            if zeroed and status.get('zeroed') != '1':
                raise SweepError('HOME后零位无效，叩击未启动')
            now = time.monotonic()
            if abs(speed) <= 5:
                if stable is None or abs(angle - anchor) > .25:
                    stable, anchor = now, angle
                if now - stable >= self.config.settle_seconds:
                    return status
            else:
                stable = anchor = None
            await asyncio.sleep(.1)
        raise SweepError('等待机构静止超时')

    async def download_trace(self, shot, samples):
        receiver = TraceReceiver(require_crc=True, request_ids=True, shot=shot)
        deadline = time.monotonic() + self.config.trace_timeout
        offset = retries = 0
        while time.monotonic() < deadline:
            self.request_id = (self.request_id + 1) & 0xffffffff
            await self.send(receiver.request(offset, self.request_id))
            chunk_end = min(deadline, time.monotonic() + self.config.chunk_timeout)
            ended = False
            while time.monotonic() < chunk_end:
                try:
                    line = await self.receive(chunk_end - time.monotonic())
                except asyncio.TimeoutError:
                    break
                if not line.startswith(('TRACE ', 'TRACE_BEGIN ', 'TRACE_END ')):
                    continue
                try:
                    ended = receiver.feed(line)
                except TraceReplacedError:
                    raise
                except (KeyError, ValueError, OverflowError) as exc:
                    self.files.log(f'TRACE_REJECT {exc}')
                    continue
                if receiver.total is not None and receiver.total != samples:
                    raise SweepError('轨迹点数与本击EVT SHOT不一致')
                if ended:
                    break
            if receiver.total is not None:
                missing = next((i for i in range(receiver.total) if i not in receiver.points), None)
                if ended and missing is None:
                    points = [receiver.points[i] for i in range(receiver.total)]
                    if not points or any(b.time_us <= a.time_us for a, b in zip(points, points[1:])):
                        raise SweepError('轨迹为空或时间戳不递增')
                    return points, receiver.metadata
            else:
                missing = offset
            if missing is None or missing <= offset:
                retries += 1
                if retries > 3:
                    raise SweepError('轨迹分包连续无进展，已重读3次')
            else:
                retries = 0
            offset = missing if missing is not None else offset
        raise SweepError('完整轨迹读取超时')

    async def one_trial(self, number, speed, trial):
        row = dict(number=number, speed_dps=speed, trial=trial, retract_deg=self.config.angle,
                   fire_state='not_sent', result='incomplete', error='')
        details = {'row': row, 'started': dt.datetime.now().isoformat(timespec='milliseconds')}
        points = []
        accel_rows=[]
        try:
            await self.wait_still()
            await self.command('HOME', 'OK HOME ')
            home = await self.response('EVT HOME ', self.config.home_timeout)
            if home.get('contact_candidate') != '1' or home.get('zeroed') != '1':
                raise SweepError('HOME没有建立有效接触候选零位')
            details['home'] = home
            shift=finite_number(home,'angle_before')
            if self.home_count:
                self.zero_shift+=shift
                row['home_zero_shift_deg']=self.zero_shift
                # Repeated preload/rebound corrections are not an absolute
                # physical angle reference. Check each correction and the rigid
                # hammer's independent static acceleration pose instead.
            if abs(finite_number(home,'travel'))>12:
                raise SweepError('HOME实际行程超过12°，停止检查接触位置')
            self.home_count+=1
            row['home_zero_shift_deg']=self.zero_shift
            row['home_travel_deg'] = finite_number(home, 'travel')
            details['home_status'] = await self.wait_still(zeroed=True)
            self.check_pose('contact',details)
            await self.command(f'SET {self.config.angle:g} {speed}', 'OK SET ')
            params = await self.command('PARAMS', 'PARAMS ')
            if abs(finite_number(params, 'retract_angle') - self.config.angle) > .011 or abs(finite_number(params, 'strike_speed') - speed) > .011:
                raise SweepError('参数回读不一致，不启动本击')
            details['params'] = params
            await self.command('RETRACT', 'OK RETRACT ')
            ready = await self.response('EVT READY ', self.config.retract_timeout)
            row['ready_angle_deg'] = finite_number(ready, 'actual')
            if abs(finite_number(ready, 'target') + self.config.angle) > .011 or abs(finite_number(ready, 'actual') + self.config.angle) > 1.21:
                raise SweepError('回撤到位回复与目标不一致')
            if self.capture:
                # Poll encoder during a quiet 200ms baseline while capture keeps running.
                until=time.monotonic()+getattr(self.capture,'baseline_seconds',.25)
                anchor=None
                while time.monotonic()<until:
                    self.capture.check()
                    st=await self.command('STATUS','STATUS ')
                    angle=finite_number(st,'angle')
                    if st.get('state')!='READY' or abs(finite_number(st,'speed'))>5 or abs(angle+self.config.angle)>1.21:
                        raise SweepError('静态基线期间机构没有保持静止')
                    if anchor is not None and abs(angle-anchor)>.5:raise SweepError('静态基线期间位置漂移')
                    anchor=angle;await asyncio.sleep(.05)
                self.check_pose('retract',details)
                details['accel_fire']=self.capture.mark('fire',number)
            row['fire_state'] = 'sent_unconfirmed'
            fired = await self.command('FIRE', 'OK FIRE ')
            row['fire_state'] = 'acknowledged'; row['shot'] = int(fired['shot'])
            event = await self.response('EVT SHOT ', self.config.strike_timeout)
            if int(event['shot']) != row['shot'] or abs(finite_number(event, 'target_speed') - speed) > .011:
                raise SweepError('叩击事件不是当前动作')
            row['fire_state'] = 'completed'
            details['shot_event'] = event
            if event['reason'] not in ('SPEED_DROP', 'TIMEOUT_NO_DROP'):
                raise SweepError('收到非预期的叩击结束原因：' + event['reason'])
            row.update(reason=event['reason'], contact_candidate=event.get('contact_candidate') == '1',
                       release_angle_deg=finite_number(event, 'release_angle'), release_speed_dps=finite_number(event, 'release_speed'))
            row['early_drop_suspected'] = row['contact_candidate'] and row['release_angle_deg'] < -5
            if event['reason']!='SPEED_DROP' or row['early_drop_suspected'] or row['release_angle_deg']>=8:
                row['result']=event['reason'] if not row['early_drop_suspected'] else 'early_drop_suspected'
                raise SweepError('异常叩击，整轮停止：'+row['result'])
            if self.capture:
                ended=time.monotonic();self.capture.record('shot_end',{'number':number,'shot':row['shot']})
                accel_rows,details['accel_end']=await self.capture.finish_window(number,details['accel_fire'],ended)
            samples = int(event['samples'])
            if not 0 < samples <= 2048:
                raise SweepError('设备返回的本击样本数无效')
            points, metadata = await self.download_trace(row['shot'], samples)
            if abs(finite_number(metadata, 'target') - speed) > .011 or abs(finite_number(metadata, 'retract_angle') - self.config.angle) > .011 or metadata['reason'] != event['reason']:
                raise SweepError('轨迹参数或结束原因与当前动作不一致')
            details['trace_metadata'] = metadata
            metrics = analyze_trace(points, speed, contact=row['contact_candidate'], window_us=int(metadata['window_us']))
            metrics.pop('shot_success', None)  # Encoder speed qualification is not confirmed contact.
            details['encoder_metrics'] = metrics
            row.update(trace_samples=len(points), speed_qualified=metrics['qualified'],
                       pre_drop_speed_dps=metrics.get('pre_drop_speed'), stable_ms=metrics['stable_ms'])
            row['result'] = ('early_drop_suspected' if row['early_drop_suspected'] else
                             'speed_drop_candidate' if row['contact_candidate'] else event['reason'])
            if self.capture:
                from motor_accel_analysis import analyze,FIELDS
                metrics=analyze(accel_rows,details['accel_fire'],details['accel_end'],self.config.accel_min_peak,row)
                details['accel_metrics']=metrics
                row.update({k:metrics.get(k) for k in FIELDS})
        except BaseException as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
            # Stop before writing partial results, including on Ctrl+C/disk errors.
            try:
                await self.stop()
            except Exception as stop_exc:
                details['stop_error'] = f'{type(stop_exc).__name__}: {stop_exc}'
            raise
        finally:
            if self.capture and details.get('accel_fire'):
                from motor_accel_capture import save_window
                if not accel_rows:accel_rows=self.capture.window(details['accel_fire'])
                relative=Path(f'speed_{speed}')/f'trial_{trial:02d}'/'accel.csv'
                save_window(self.files.directory/relative,accel_rows)
            self.files.save_trial(row, details, points)
        self.progress(f'[{number}/{len(self.config.plan())}] {speed}°/s 第{trial}/{self.config.repeats}次 '
                      f'{row["result"]} | 撤驱动角度 {row["release_angle_deg"]:.2f}° | {len(points)}点')

    async def stop(self):
        if self.stopped:
            return
        # Bypass file logging: a full/broken disk must not prevent the priority STOP.
        await asyncio.wait_for(self.link.send('!'), 1.0)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = await self.link.receive(deadline - time.monotonic())
            try:
                self.files.log('STOP_RX ' + line)
            except OSError:
                pass
            if line.startswith('OK STOP'):
                self.stopped = True
                return
        raise SweepError('STOP未收到确认')

    async def _trials(self):
        previous=None
        for number,(speed,trial) in enumerate(self.config.plan(),1):
            if previous is not None:
                await asyncio.sleep(self.config.grade_pause if speed!=previous else self.config.pause)
            await self.one_trial(number,speed,trial)
            previous=speed

    async def run(self):
        status, error, stop_error = 'failed', '', ''
        try:
            await self.link.connect()
            await self.synchronize()
            self.progress('已连接：' + self.info.get('firmware', '?') + '；每击先HOME，Ctrl+C停止')
            self.files.write_manifest('running')
            if self.capture:
                await self.capture.start()
                self.progress('Master已同步，已取得连续1秒有效加速度。')
                body=asyncio.create_task(self._trials());watch=asyncio.create_task(self.capture.watch())
                try:
                    done,_=await asyncio.wait((body,watch),return_when=asyncio.FIRST_COMPLETED)
                    if watch in done:
                        failure=watch.exception()
                        body.cancel()
                        await asyncio.gather(body,return_exceptions=True)
                        raise failure or SweepError('采集监视器异常退出')
                    await body
                finally:
                    body.cancel();watch.cancel()
                    await asyncio.gather(body,watch,return_exceptions=True)
            else:await self._trials()
            status = 'completed'
        except asyncio.CancelledError:
            status, error = 'interrupted', '用户中止或任务取消'
            raise
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            try:
                await self.stop()
            except Exception as exc:
                stop_error = f'{type(exc).__name__}: {exc}'
                self.progress('STOP确认失败：' + stop_error)
                if status == 'completed': status = 'stop_unconfirmed'
            cleanup_error=None
            try:
                if self.capture:await self.capture.close()
            except Exception as exc:
                cleanup_error=exc;status='failed';error=error or f'采集清理失败：{exc}'
            finally:
                try:await self.link.close()
                finally:self.files.write_manifest(status, error, stop_error)
            if cleanup_error:raise cleanup_error
            if stop_error and status == 'stop_unconfirmed':
                raise SweepError('动作已结束，但STOP未确认，请检查设备')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='每击先HOME：300–900°/s，100°/s一档，每档20次。')
    parser.add_argument('--mode', choices=('BLE', 'USB'), default='BLE')
    parser.add_argument('--device', default='Rotary_Hammer_BLE', help='BLE广播名或地址')
    parser.add_argument('--port', help='USB模式的电机串口')
    parser.add_argument('--master-port',help='同步记录加速度的Master USB串口')
    parser.add_argument('--accel-min-peak',type=float,default=None,help='事件阈值下限，counts；默认100或已保存参数')
    parser.add_argument('--analyze-run',type=Path,help='仅从已保存实验重新生成统计及图表')
    parser.add_argument('--speeds', nargs='+', type=int, default=list(DEFAULT_SPEEDS), metavar='DPS')
    parser.add_argument('--repeats', type=int, default=20, help='每档FIRE次数，默认20；不自动补敲')
    parser.add_argument('--angle', type=float, default=50, help='回撤角度，默认50°')
    parser.add_argument('--pause', type=float, default=2, help='同档每击保存完成后的休息秒数')
    parser.add_argument('--grade-pause', type=float, default=5, help='换档前休息秒数')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs' / 'motor_sweeps')
    parser.add_argument('--dry-run', action='store_true', help='仅打印计划，不连接或驱动电机')
    parser.add_argument('--grouped',action='store_true',help='每档独立启动、保存和停机；任一组失败即终止后续组')
    args = parser.parse_args(argv)
    config = Config(tuple(args.speeds), args.repeats, args.angle, args.pause, args.grade_pause,
                    args.mode, args.device, args.port,master_port=args.master_port,accel_min_peak=100. if args.accel_min_peak is None else args.accel_min_peak)
    try: config.validate()
    except ValueError as exc: parser.error(str(exc))
    return args, config


def run_groups(args,config):
    directory=args.output.resolve()/f'group_experiment_{dt.datetime.now():%Y%m%d_%H%M%S}'
    directory.mkdir(parents=True,exist_ok=False)
    async def grouped():
        connection=None;groups=[]
        try:
            for speed in config.speeds:
                cfg=replace(config,speeds=(speed,))
                files=RunFiles(directory/f'group_{speed}',cfg)
                capture=None;code=0
                print(f'开始独立分组：{speed}°/s × {config.repeats}；{files.directory}',flush=True)
                try:
                    if config.master_port:
                        from motor_accel_capture import AccelCapture
                        capture=AccelCapture(config.master_port,files.directory,serial_connection=connection,keep_open=True)
                    runner=MotorSweep(cfg,HardwareLink(cfg),files,lambda message:print(message,flush=True),capture=capture)
                    await runner.run()
                except asyncio.CancelledError:
                    code=130
                    raise
                except Exception as exc:
                    code=1;print('本组停止，后续组取消：'+str(exc),flush=True)
                finally:
                    if capture and capture.reader:connection=capture.reader.serial_port
                    files.close()
                    if config.master_port:
                        from motor_accel_analysis import report
                        try:report(files.directory)
                        except Exception as exc:
                            code=1;print('报告失败，原始数据保留：'+str(exc),flush=True)
                    groups.append({'speed_dps':speed,'exit_code':code,'directory':str(files.directory)})
                    (directory/'groups.json').write_text(json.dumps(groups,ensure_ascii=False,indent=2)+'\n')
                if code:return code
                print(f'{speed}°/s 组已结束并确认Motor STOP、Master IDLE。',flush=True)
                await asyncio.sleep(3)
            return 0
        finally:
            if connection and connection.is_open:
                try:connection.rts=False;connection.dtr=False
                except OSError:pass
                connection.close()
    async def execute():
        loop=asyncio.get_running_loop();task=asyncio.current_task()
        if sys.platform!='win32':loop.add_signal_handler(signal.SIGTERM,task.cancel)
        try:return await grouped()
        finally:
            if sys.platform!='win32':loop.remove_signal_handler(signal.SIGTERM)
    try:return asyncio.run(execute())
    except (KeyboardInterrupt,asyncio.CancelledError):return 130


def main(argv=None):
    args, config = parse_args(argv)
    if args.analyze_run:
        from motor_accel_analysis import report
        report(args.analyze_run,args.accel_min_peak)
        print('已重新生成：'+str(args.analyze_run/'report.md'));return 0
    print(f'回撤 {config.angle:g}°；每击HOME（固件40°/s接近）后叩击；共{len(config.plan())}次FIRE')
    for speed in config.speeds:
        print(f'  {speed}°/s × {config.repeats}')
    if args.dry_run:
        print('仅预览，没有连接设备。'); return 0
    if args.grouped:
        return run_groups(args,config)
    files = RunFiles(args.output.resolve(), config)
    print('输出目录：' + str(files.directory), flush=True)
    sweep = MotorSweep(config, HardwareLink(config), files, lambda text: print(text, flush=True))
    async def run():
        loop = asyncio.get_running_loop()
        task = asyncio.current_task()
        if sys.platform != 'win32': loop.add_signal_handler(signal.SIGTERM, task.cancel)
        try: await sweep.run()
        finally:
            if sys.platform != 'win32': loop.remove_signal_handler(signal.SIGTERM)
    try:
        asyncio.run(run())
    except (KeyboardInterrupt, asyncio.CancelledError):
        print('测试已中止；已尝试STOP，已记录的数据保留。'); return 130
    except Exception as exc:
        print(f'测试停止：{exc}', file=sys.stderr); return 1
    finally:
        files.close()
        if config.master_port:
            from motor_accel_analysis import report
            try:report(files.directory)
            except Exception as exc:print('报告生成失败，原始数据保留：'+str(exc),file=sys.stderr)
        print('结果：' + str(files.directory / 'summary.csv'), flush=True)
    print('完成；已STOP，下一次使用前请重新建立零位。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
