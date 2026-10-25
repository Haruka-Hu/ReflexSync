"""Hardware-free tests of the real sweep runner and its result files."""
from __future__ import annotations
import asyncio
import csv
import json
import sys
import tempfile
import unittest
import zlib
from unittest.mock import patch
from dataclasses import replace
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from motor_speed_sweep import Config, MotorSweep, RunFiles, SweepError, parse_args, main


def crc(line):
    return f'{line} crc={zlib.crc32(line.encode()):08x}'


class FakeMotor:
    def __init__(self, *, fault=None):
        self.commands=[]; self.queue=asyncio.Queue(); self.fault=fault; self.closed=False
        self.angle=50.;self.speed=300;self.shot=0;self.zeroed=False;self.state='IDLE'
        self.trace_requests=0;self.corrupted=False;self.home_count=0
        self.fire_wait=asyncio.Event()
    async def connect(self):pass
    def put(self,line):self.queue.put_nowait(line)
    async def send(self,command):
        self.commands.append(command)
        if command=='INFO':
            features='HOME,TRACE,TRACE_CRC,TRACE_REQUEST_ID,BOUNDED_STRIKE,BOUNDED_HOME'
            if self.fault=='old':features='HOME,TRACE'
            self.put('INFO firmware=HAMMER_COAST_V9 protocol=2 features='+features)
        elif command=='TUNING':self.put('TUNING kp=3 ki=20 ks=-96.7 kv=.4477 kg=3.25')
        elif command=='STATUS':
            self.put(f'STATUS state={self.state} zeroed={int(self.zeroed)} angle=0 speed=0 homing=0')
        elif command=='HOME':
            self.home_count+=1;self.zeroed=True
            self.put('OK HOME speed=40')
            if self.fault=='home_abort':
                self.put('EVT ABORT reason=HOME_TIMEOUT angle=0 speed=0')
            else:
                self.put(f'EVT HOME contact_candidate=1 zeroed={0 if self.fault=="zero" else 1} angle_before=0 travel=1 peak_speed=40')
        elif command.startswith('SET '):
            _,angle,speed=command.split();self.angle=float(angle);self.speed=int(speed)
            self.put(f'OK SET retract_angle={angle} strike_speed={speed}')
        elif command=='PARAMS':
            speed='nan' if self.fault=='params_nan' else str(self.speed)
            self.put(f'PARAMS retract_angle={self.angle} strike_speed={speed}')
        elif command=='RETRACT':
            self.state='READY';self.put(f'OK RETRACT target={-self.angle}')
            self.put(f'EVT READY target={-self.angle} actual={-self.angle}')
        elif command=='FIRE':
            assert self.home_count==self.shot+1, 'every FIRE needs a fresh HOME'
            assert self.zeroed and self.state=='READY'
            self.state='IDLE';self.shot+=1
            self.fire_wait.set()
            if self.fault=='fire_ack_lost':return
            self.put(f'OK FIRE shot={self.shot} target_speed={self.speed}')
            if self.fault=='cancel':return
            if self.fault=='disconnect':
                self.put(SweepError('BLE disconnected'));return
            release=-40 if self.fault=='early' else 0
            reason='TIMEOUT_NO_DROP' if self.fault=='timeout_drop' else 'SPEED_DROP'
            candidate=0 if self.fault=='timeout_drop' else 1
            self.put(f'EVT SHOT reason={reason} contact_candidate={candidate} shot={self.shot} '
                f'target_speed={self.speed} release_angle={release} release_speed=90 samples=16 zeroed=1')
        elif command.startswith('TRACE '):
            self.trace_requests+=1
            _,offset,request=command.split();offset=int(offset);end=min(16,offset+8)
            reason='TIMEOUT_NO_DROP' if self.fault=='timeout_drop' else 'SPEED_DROP'
            candidate=0 if self.fault=='timeout_drop' else 1
            self.put(crc(f'TRACE_BEGIN shot={self.shot} total=16 offset={offset} request={request} '
                f'target={self.speed} window_us=6000 reason={reason} contact_candidate={candidate} '
                f'retract_angle={self.angle} kp=3 ki=20 ks=-96.7 kv=.4477 kg=3.25'))
            for index in range(offset,end):
                line=crc(f'TRACE shot={self.shot} request={request} i={index} us={index*1000} '
                         f'angle={-50+index*3} speed={self.speed} raw=0 pwm=200 drop=300 baseline=300')
                if index==2 and (self.fault=='permanent_crc' or self.fault=='crc' and not self.corrupted):
                    line=line.replace('pwm=200','pwm=201');self.corrupted=True
                self.put(line)
            self.put(crc(f'TRACE_END shot={self.shot} next={end} total=16 offset={offset} request={request}'))
        elif command=='!':
            if self.fault=='stop_ack_lost':return
            self.state='IDLE';self.zeroed=False
            self.put('EVT ABORT reason=USER_STOP');self.put('OK STOP')
        else:raise AssertionError(command)
    async def receive(self,timeout):
        item=await asyncio.wait_for(self.queue.get(),timeout)
        if isinstance(item,Exception):raise item
        return item
    async def close(self):self.closed=True


class SweepTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.config=Config(speeds=(300,),repeats=1,pause=0,grade_pause=0,settle_seconds=0,
                           command_timeout=.05,chunk_timeout=.02,strike_timeout=.05)
    def runner(self,fault=None,config=None):
        config=config or self.config
        files=RunFiles(Path(self.tmp.name),config);self.addCleanup(files.close)
        motor=FakeMotor(fault=fault)
        return MotorSweep(config,motor,files,lambda _:None),motor,files
    async def test_full_default_plan_140_fires_and_140_fresh_homes(self):
        config=replace(self.config,speeds=tuple(range(300,901,100)),repeats=20)
        runner,motor,files=self.runner(config=config);await runner.run()
        self.assertEqual(motor.commands.count('HOME'),140)
        self.assertEqual(motor.commands.count('FIRE'),140)
        self.assertEqual(motor.commands[-1],'!');self.assertTrue(motor.closed)
        with (files.directory/'summary.csv').open(encoding='utf-8-sig') as file:
            rows=list(csv.DictReader(file))
        self.assertEqual([int(r['speed_dps']) for r in rows],[s for s in config.speeds for _ in range(20)])
        self.assertTrue(all(r['fire_state']=='completed' for r in rows))
        for speed in config.speeds:
            self.assertEqual(len(list((files.directory/f'speed_{speed}').glob('trial_*/trace.csv'))),20)
        manifest=json.loads((files.directory/'run.json').read_text())
        self.assertEqual(manifest['status'],'completed');self.assertEqual(manifest['completed_fires'],140)
    async def test_crc_damage_only_retries_trace(self):
        runner,motor,files=self.runner('crc');await runner.run()
        self.assertGreater(motor.trace_requests,2)
        self.assertEqual(motor.commands.count('HOME'),1);self.assertEqual(motor.commands.count('FIRE'),1)
        self.assertEqual(files.rows[0]['trace_samples'],16)
    async def test_bad_home_and_nan_parameters_never_fire(self):
        for fault in ('home_abort','zero','params_nan','old'):
            with self.subTest(fault=fault):
                runner,motor,files=self.runner(fault)
                with self.assertRaises(SweepError):await runner.run()
                self.assertNotIn('FIRE',motor.commands);self.assertIn('!',motor.commands)
                self.assertEqual(json.loads((files.directory/'run.json').read_text())['status'],'failed')
    async def test_fatal_link_and_permanent_trace_error_stop_without_extra_strikes(self):
        for fault in ('disconnect','permanent_crc','fire_ack_lost'):
            with self.subTest(fault=fault):
                runner,motor,files=self.runner(fault)
                with self.assertRaises(SweepError):await runner.run()
                self.assertEqual(motor.commands.count('FIRE'),1);self.assertIn('!',motor.commands)
                self.assertEqual(len(files.rows),1)
                self.assertNotEqual(files.rows[0]['result'],'speed_drop_candidate')
    async def test_early_drop_and_no_drop_stop_the_entire_run(self):
        for fault in ('early','timeout_drop'):
            runner,motor,files=self.runner(fault,replace(self.config,repeats=2))
            with self.assertRaises(SweepError):await runner.run()
            self.assertEqual(motor.commands.count('FIRE'),1)
            self.assertEqual(motor.commands[-1],'!')
            self.assertEqual(len(files.rows),1)
    async def test_cancel_sends_stop_before_partial_result_save(self):
        runner,motor,files=self.runner('cancel')
        original=files.save_trial
        def save(*args):
            self.assertEqual(motor.commands[-1],'!')
            original(*args)
        files.save_trial=save
        task=asyncio.create_task(runner.run());await motor.fire_wait.wait();task.cancel()
        with self.assertRaises(asyncio.CancelledError):await task
        self.assertEqual(motor.commands.count('FIRE'),1)
        self.assertEqual(json.loads((files.directory/'run.json').read_text())['status'],'interrupted')
    async def test_missing_stop_ack_cannot_report_completed(self):
        runner,motor,files=self.runner('stop_ack_lost')
        with self.assertRaises(SweepError):await runner.run()
        manifest=json.loads((files.directory/'run.json').read_text())
        self.assertEqual(manifest['status'],'stop_unconfirmed')
        self.assertTrue(manifest['stop_error']);self.assertTrue(motor.closed)

    async def test_data_write_error_still_stops(self):
        runner,motor,files=self.runner()
        def fail(*args):raise OSError('disk full')
        files.save_trial=fail
        with self.assertRaises(OSError):await runner.run()
        self.assertIn('!',motor.commands);self.assertTrue(motor.closed)
    async def test_trace_log_disk_error_does_not_block_priority_stop(self):
        runner,motor,files=self.runner()
        original=files.log
        def log(message):
            if message.startswith('RX TRACE'):raise OSError('disk full')
            original(message)
        files.log=log
        with self.assertRaises(OSError):await runner.run()
        self.assertIn('!',motor.commands)


class PlanTests(unittest.TestCase):
    def test_group_failure_never_starts_a_later_grade(self):
        calls=[]
        async def fake_run(runner):
            calls.append(runner.config.speeds[0])
            if len(calls)==2:raise SweepError('group failed')
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(MotorSweep,'run',fake_run):
                result=main(['--grouped','--output',tmp,'--speeds','300','400','500'])
            self.assertEqual(result,1);self.assertEqual(calls,[300,400])
    def test_defaults_and_cli_preview(self):
        args,config=parse_args(['--dry-run'])
        self.assertTrue(args.dry_run);self.assertEqual(len(config.plan()),140)
        self.assertEqual(config.speeds,tuple(range(300,901,100)))
        self.assertEqual(config.angle,50);self.assertEqual(config.mode,'BLE')
    def test_invalid_config(self):
        for config in (Config(repeats=0),Config(angle=float('nan')),Config(pause=-1),
                       Config(mode='USB'),Config(mode='BLE',port='COM9'),Config(speeds=(300,300)),Config(speeds=(0,))):
            with self.assertRaises(ValueError):config.validate()

if __name__=='__main__':unittest.main()
