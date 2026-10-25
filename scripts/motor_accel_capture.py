"""Headless Master adapter; the stable scope GUI and its parser stay unchanged."""
from __future__ import annotations
import asyncio
import csv
import json
import queue
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from scope48_mvc import AcquisitionModel, SerialReader

COLS = ('host_monotonic','host_rx_time','timestamp_us','sample_seq','batch_seq','rx_flags',
        'x','y','z','seq_unwrapped','time_unwrapped_us')

def unwrap(value, reference):
    return value if reference is None else reference + ((value-reference+2**31) % 2**32-2**31)

class CaptureModel(AcquisitionModel):
    def __init__(self, owner):
        super().__init__(max_points=2000)
        self.owner=owner
    def append_emg48(self, *args, **kwargs):
        pass  # SerialReader still parses and verifies every frame; no EMG waveform storage.
    def append_imu_sample(self, source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x,y,z):
        if source_id == 2:
            self.owner.sample(timestamp_us,sample_seq,batch_seq,rx_flags,x,y,z)
        elif source_id == 0x7e:
            super().append_imu_sample(source_id,timestamp_us,sample_seq,batch_seq,rx_flags,x,y,z)
    def update_sync_diag(self,*args):
        super().update_sync_diag(*args)
        self.owner.record('sync', dict(zip(('source','state','beacon_seq','offset_us','drift_ppm',
                                          'residual_us','beacon_count'),args)))
    def update_state_event(self,*args):
        super().update_state_event(*args)
        self.owner.record('state',dict(zip(('source','state','command_seq','effective_master_time_us'),args)))
    def update_diag(self,*args,**kwargs):
        super().update_diag(*args,**kwargs)
        self.owner.record('link',{'args':args,**kwargs})
    def note_reader_error(self,exc):
        super().note_reader_error(exc)
        self.owner.error=RuntimeError(f'Master串口读取失败：{exc}')

class CaptureReader(SerialReader):
    def close(self):
        if self.serial_port is not None and self.serial_port.is_open:
            # Arduino TinyUSB recognizes 01->11->10->00 as a reboot gesture.
            # Release RTS first from a connected 11 state, ending at idle 00;
            # do not leave the kernel's close/HUPCL transition half-armed.
            try:
                self.serial_port.rts=False
                self.serial_port.dtr=False
            except (OSError,RuntimeError):pass
        super().close()
    def classify_data_frame_sequence(self,frame_type,sequence,modulus):
        role=super().classify_data_frame_sequence(frame_type,sequence,modulus)
        # Allow late/duplicate IMU frames through BOTH CRC checks. The model
        # deduplicates by the full source sample sequence, not an 8-bit PC frame.
        return 'late_or_duplicate' if frame_type==0x34 and role=='duplicate' else role

class AccelCapture:
    def __init__(self,port,directory,baud=4000000,serial_connection=None,keep_open=False):
        from serial.tools import list_ports
        entry=next((p for p in list_ports.comports() if p.device==port),None)
        if entry and (entry.serial_number or '').replace(':','').upper()=='C49E7E17C32C':
            raise RuntimeError('此USB设备属于Motor，禁止作为Master打开')
        self.directory=Path(directory);self.port=port;self.baud=baud
        if entry:
            (self.directory/'master_identity.json').write_text(json.dumps({'port':port,'serial_number':entry.serial_number,'hwid':entry.hwid},indent=2)+'\n')
        self.serial_connection=serial_connection;self.keep_open=keep_open
        self.lock=threading.Lock();self.samples={};self.order=deque()
        self.seq=self.stamp=None;self.last_new=None;self.error=None
        self.queue=queue.Queue(maxsize=100000);self.closing=threading.Event()
        self.model=CaptureModel(self);self.reader=None;self.writer=None
        self.stream_started=None;self.duplicates=0;self.active=False
        self.raw=(self.directory/'accel_raw.csv').open('w',newline='',encoding='utf-8-sig')
        self.diag=(self.directory/'capture_events.jsonl').open('w',encoding='utf-8')
        self.csv=csv.DictWriter(self.raw,fieldnames=COLS);self.csv.writeheader();self.raw.flush()
        self.writer=threading.Thread(target=self._write,daemon=True);self.writer.start()
    def put(self,item):
        try:self.queue.put_nowait(item)
        except queue.Full:self.error=RuntimeError('加速度写盘队列已满')
    def record(self,kind,values):
        self.put(('event',{'kind':kind,'host_monotonic':time.monotonic(),'host_rx_time':time.time(),**values}))
    def sample(self,t,s,b,f,x,y,z):
        now=time.monotonic()
        with self.lock:
            seq=unwrap(s,self.seq);stamp=unwrap(t,self.stamp)
            row=dict(zip(COLS,(now,time.time(),t,s,b,f,x,y,z,seq,stamp)))
            self.put(('sample',row)) # Keep duplicates in the continuous raw file for auditing.
            if seq in self.samples:
                self.duplicates+=1
                # A repeated copy must not hide any originally observed link fault.
                self.samples[seq]['rx_flags'] |= f
                return
            self.samples[seq]=row;self.order.append(seq)
            if self.seq is None or seq>self.seq:
                self.seq=seq;self.stamp=stamp;self.last_new=now
            while len(self.order)>30000:
                self.samples.pop(self.order.popleft(),None)
    def _write(self):
        last=time.monotonic()
        try:
            while not self.closing.is_set() or not self.queue.empty():
                try:kind,value=self.queue.get(timeout=.05)
                except queue.Empty:continue
                if kind=='sample':self.csv.writerow(value)
                else:self.diag.write(json.dumps(value,ensure_ascii=False)+'\n')
                self.queue.task_done()
                if time.monotonic()-last>=.2:
                    self.raw.flush();self.diag.flush();last=time.monotonic()
            self.raw.flush();self.diag.flush()
        except BaseException as exc:
            self.error=RuntimeError(f'加速度写盘失败：{exc}')
    def check(self,require_data=True):
        if self.error:raise self.error
        if self.reader and not self.reader.connected:raise RuntimeError('Master串口已断开')
        if require_data and self.stream_started is not None:
            if time.monotonic()-(self.last_new or self.stream_started)>1.:
                raise RuntimeError('加速度连续1秒没有新样本')
    async def command(self,cmd):
        self.record('command',{'command':cmd})
        await asyncio.wait_for(asyncio.to_thread(self.reader.send_text_command,cmd),2)
    async def start(self):
        self.reader=CaptureReader(self.port,self.baud,self.model,enable_resend=True,live_decimation=1000000)
        if self.serial_connection is not None:
            if not self.serial_connection.is_open:raise RuntimeError('组间Master USB连接已断开，不自动重连')
            self.reader.serial_port=self.serial_connection
        else:self.reader.open()
        if self.reader.serial_port:self.reader.serial_port.write_timeout=.5
        self.reader.start();await self.command('STOP');await asyncio.sleep(.3)
        await self.command('START_SYNC')
        last_sync=time.monotonic()
        deadline=time.monotonic()+30
        while True:
            self.check(False)
            _,diags,_=self.model.sync_snapshot();d=diags.get(2,[])
            if d and time.time()-d[-1].host_rx_time<1 and d[-1].beacon_count>=30 and abs(d[-1].residual_us)<=3000:
                break
            if time.monotonic()>deadline:raise RuntimeError('30秒内未取得加速度有效同步')
            # Before any motor action, repeat a missed wake command only while
            # the node has not acknowledged SYNC or sent a fresh sync diagnostic.
            _,_,states=self.model.sync_snapshot()
            awake=bool(d and time.time()-d[-1].host_rx_time<2) or any(
                e.source_id==2 and e.state==1 and time.time()-e.host_rx_time<30 for e in states)
            if not awake and time.monotonic()-last_sync>2:
                await self.command('START_SYNC');last_sync=time.monotonic()
            await asyncio.sleep(.05)
        self.reader.reset_stream_session()
        with self.lock:self.samples.clear();self.order.clear();self.seq=self.stamp=None;self.last_new=None
        self.stream_started=time.monotonic();await self.command('START_STREAM')
        stable=None;last=None
        while time.monotonic()<deadline:
            self.check()
            with self.lock:
                recent=sorted(self.samples.values(),key=lambda r:r['seq_unwrapped'])
            if recent:
                newest=recent[-1]
                valid=(newest['rx_flags']&2)!=0 and (newest['rx_flags']&4)==0
                if not valid:stable=None
                elif last != newest['seq_unwrapped']:
                    tail=[r for r in recent if r['time_unwrapped_us']>=newest['time_unwrapped_us']-1000000]
                    complete=all(b['seq_unwrapped']==a['seq_unwrapped']+1 and
                        0<b['time_unwrapped_us']-a['time_unwrapped_us']<3000 and (b['rx_flags']&6)==2
                        for a,b in zip(tail,tail[1:]))
                    if complete and len(tail)>=900 and tail[-1]['time_unwrapped_us']-tail[0]['time_unwrapped_us']>=990000:
                        self.active=True;self.record('ready',{'sample_seq':self.seq});return
                last=newest['seq_unwrapped']
            await asyncio.sleep(.05)
        raise RuntimeError('同步后未取得连续1秒有效加速度')
    async def watch(self):
        while True:self.check();await asyncio.sleep(.05)
    def mark(self,kind,number):
        self.check()
        with self.lock:mark={'host_monotonic':time.monotonic(),'sample_seq':self.seq,'timestamp_us':self.stamp}
        self.record(kind,{'number':number,**mark});return mark
    def pose(self):
        import numpy as np
        self.check()
        with self.lock:
            rows=[r for _,r in sorted(self.samples.items()) if r['time_unwrapped_us']>=self.stamp-200000]
        if len(rows)<160 or rows[-1]['time_unwrapped_us']-rows[0]['time_unwrapped_us']<180000:
            raise RuntimeError('静态姿态检查缺少200ms加速度')
        if any((r['rx_flags']&6)!=2 for r in rows) or any(
            b['seq_unwrapped']!=a['seq_unwrapped']+1 for a,b in zip(rows,rows[1:])):
            raise RuntimeError('静态姿态基线有丢样或链路故障')
        return np.median([[r[k] for k in ('x','y','z')] for r in rows],axis=0).tolist()
    async def finish_window(self,number,fire,ended):
        deadline=ended+1.
        while time.monotonic()<deadline:self.check();await asyncio.sleep(.02)
        end=self.mark('window_end',number)
        await asyncio.sleep(.5);self.check()
        rows=self.window(fire,end)
        self.record('window_closed',{'number':number,'samples':len(rows),'end_seq':end['sample_seq']})
        return rows,end
    def window(self,fire,end=None):
        with self.lock:
            hi=(end or {'sample_seq':self.seq})['sample_seq']
            return [dict(r) for _,r in sorted(self.samples.items()) if r['seq_unwrapped']<=hi
                    and r['time_unwrapped_us']>=fire['timestamp_us']-200000]
    async def close(self):
        # Caller must issue motor STOP first, even when this serial link/writer fails.
        stop_error=None
        try:
            if self.reader and self.reader.connected:
                sent=time.time();await self.command('STOP')
                deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    state,_,events=self.model.sync_snapshot()
                    if state==0 and any(e.source_id==0 and e.state==0 and e.host_rx_time>=sent for e in events):break
                    await asyncio.sleep(.02)
                else:raise RuntimeError('Master STOP未收到IDLE确认')
        except Exception as exc:stop_error=exc
        if self.reader:
            self.reader.stop()
            if self.reader.raw_reader_thread:await asyncio.to_thread(self.reader.raw_reader_thread.join,1)
            if self.reader.ident is not None:await asyncio.to_thread(self.reader.join,2)
            if not self.keep_open:self.reader.close()
        self.record('parser_final',{'parser':asdict(self.model.diag_snapshot()[-1]),'duplicate_samples':self.duplicates})
        self.closing.set()
        if self.writer:await asyncio.to_thread(self.writer.join,3)
        if self.writer and self.writer.is_alive():raise RuntimeError('加速度写盘线程未正常退出')
        try:self.raw.close();self.diag.close()
        finally:
            if self.error:raise self.error
            if stop_error:raise stop_error

def save_window(path,rows):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=COLS);w.writeheader();w.writerows(rows)
