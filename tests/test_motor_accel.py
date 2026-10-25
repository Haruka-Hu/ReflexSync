import asyncio
import csv
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from motor_accel_capture import AccelCapture,CaptureReader,unwrap
from motor_accel_analysis import analyze,stats,report
from scope48_mvc import crc16_ccitt
from motor_speed_sweep import Config,RunFiles,MotorSweep
from test_motor_speed_sweep import FakeMotor

def wave(start=0,peak=500):
    return [dict(host_monotonic=i/1000,host_rx_time=i/1000,timestamp_us=i*1000,
        sample_seq=i,batch_seq=i//20,rx_flags=2,x=10+(peak if i==250 else 0),y=20,z=-40,
        seq_unwrapped=i+start,time_unwrapped_us=i*1000+start*1000) for i in range(1400)]

def frame(pc,seq=0,t=0,inner_bad=False,outer_bad=False):
    wire=bytearray([0x20,2])+int(pc).to_bytes(2,'little')+int(seq).to_bytes(4,'little')+bytes([20,2])+int(t).to_bytes(4,'little')
    for i in range(20):wire+=int(i*1000).to_bytes(2,'little')+b'\x0a\x00\x14\x00\xd8\xff'
    wire+=int(crc16_ccitt(wire) ^ (1 if inner_bad else 0)).to_bytes(2,'little')
    data=bytearray(b'\xaa\x55\x34')+bytes([pc])+wire
    data+=int(crc16_ccitt(data) ^ (1 if outer_bad else 0)).to_bytes(2,'little')
    return data

class AnalysisTests(unittest.TestCase):
    def test_known_peak_baseline_stats_and_absent_values(self):
        rows=wave();fire={'sample_seq':200,'timestamp_us':200000}
        m=analyze(rows,fire,{'sample_seq':1399},motor={'speed_qualified':True,'contact_candidate':True})
        self.assertEqual(m['baseline_counts'],[10,20,-40]);self.assertEqual(m['accel_peak_counts'],500)
        self.assertTrue(m['quality_qualified']);self.assertEqual(m['accel_peak_seq'],250)
        s=stats([100,None,200,300]);self.assertEqual(s['mean'],200);self.assertEqual(s['sd'],100);self.assertEqual(s['cv_pct'],50)
        self.assertIsNone(stats([100])['sd']);self.assertEqual(stats([])['n'],0)
    def test_gaps_faults_multi_no_event_and_recovered_duplicate(self):
        fire={'sample_seq':200,'timestamp_us':200000};end={'sample_seq':1399}
        for fault in ('gap','link','multi','none','recovered'):
            rows=wave(peak=50 if fault=='none' else 500)
            if fault=='gap':del rows[300]
            if fault=='link':rows[260]['rx_flags']|=4
            if fault=='multi':rows[450]['x']+=500
            if fault=='recovered':rows.append(dict(rows[260],rx_flags=3));rows=rows[::-1]
            m=analyze(rows,fire,end,motor={'speed_qualified':True,'contact_candidate':True})
            self.assertEqual(m['quality_qualified'],fault=='recovered',fault)
    def test_uint32_reorder_and_rollover(self):
        self.assertEqual(unwrap(5,2**32-5),2**32+5)
        self.assertEqual(unwrap(2**32-6,2**32+5),2**32-6)
    def test_report_preserves_aborted_attempt_without_acceleration(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);trial=directory/'speed_300/trial_01';trial.mkdir(parents=True)
            (directory/'run.json').write_text(json.dumps({'status':'failed','config':{'speeds':[300],'repeats':20}}))
            row={'number':1,'speed_dps':300,'trial':1,'fire_state':'not_sent','error':'HOME drift'}
            (trial/'result.json').write_text(json.dumps({'row':row}))
            ss=report(directory);self.assertEqual(ss[0]['recorded'],0);self.assertEqual(ss[0]['missing'],20)
            self.assertEqual(ss[0]['attempted'],1)
            with (directory/'summary.csv').open(encoding='utf-8-sig') as f:out=list(csv.DictReader(f))
            self.assertEqual(out[0]['error'],'HOME drift')

class CaptureTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.c=AccelCapture('fake',self.tmp.name)
    async def asyncTearDown(self):
        try:await self.c.close()
        except RuntimeError:pass
    async def test_actual_parser_double_crc_reorder_duplicates_and_wrap(self):
        c=self.c;r=CaptureReader('fake',4000000,c.model,enable_resend=True)
        fake=SimpleNamespace(is_open=True,write=lambda data:len(data),flush=lambda:None)
        fake.close=lambda:setattr(fake,'is_open',False)
        r.serial_port=fake;r._raw_reader_loop=lambda:None;c.reader=r;r.start()
        start=2**32-10;stamp=2**32-5000
        packets=[frame(0,start,stamp),frame(2,30,35000),frame(1,10,15000),frame(1,10,15000),
                 frame(3,50,55000,inner_bad=True),frame(4,70,75000,outer_bad=True),
                 frame(3,50,55000),frame(4,70,75000)]
        data=b''.join(packets)
        for i in range(0,len(data),37):r.raw_queue.put(data[i:i+37])
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            if len(c.samples)==100:break
            await asyncio.sleep(.01)
        self.assertEqual(len(c.samples),100);self.assertEqual(c.seq,2**32+89)
        self.assertGreaterEqual(c.model.diag_snapshot()[-1].crc_imu_fails,2)
        self.assertGreaterEqual(c.duplicates,20)
        fire={'sample_seq':2**32+10,'timestamp_us':2**32+15000}
        rows=c.window(fire,{'sample_seq':2**32+29})
        self.assertEqual(rows[-1]['seq_unwrapped'],2**32+29)
    async def test_no_new_data_and_write_failure_are_fatal(self):
        self.c.stream_started=time.monotonic()-2
        with self.assertRaisesRegex(RuntimeError,'1秒'):self.c.check()
        self.c.csv=SimpleNamespace(writerow=lambda row:(_ for _ in ()).throw(OSError('disk full')))
        self.c.sample(1000,1,0,2,1,2,3)
        await asyncio.sleep(.1)
        with self.assertRaisesRegex(RuntimeError,'写盘失败'):self.c.check()
    async def test_group_close_stops_capture_without_closing_shared_usb(self):
        c=self.c;c.keep_open=True
        port=SimpleNamespace(is_open=True)
        c.reader=SimpleNamespace(connected=True,serial_port=port,raw_reader_thread=None,ident=None,stop=lambda:None,
            close=lambda:(_ for _ in ()).throw(AssertionError('shared USB closed')))
        async def command(text):c.model.update_state_event(0,0,1,0)
        c.command=command
        await c.close();self.assertTrue(port.is_open)
        c.reader=None  # tearDown must not close an already closed writer again.

class FakeCapture:
    baseline_seconds=0
    def __init__(self,motor,fail=False):self.motor=motor;self.fail=fail;self.current=0;self.windows=[];self.closed=False
    async def start(self):pass
    def check(self):pass
    def pose(self):return [14.,2.,-40.]
    def record(self,*args):pass
    async def watch(self):
        if self.fail:
            await self.motor.fire_wait.wait();raise RuntimeError('accel dropout')
        await asyncio.Future()
    def mark(self,kind,number):
        self.current=number
        return {'host_monotonic':number,'sample_seq':number*10000+200,'timestamp_us':number*10000000+200000}
    async def finish_window(self,number,fire,ended):
        rows=wave(start=number*10000,peak=number+500);self.windows.append(number)
        return rows,{'sample_seq':number*10000+1399}
    def window(self,fire):return wave(start=self.current*10000)
    async def close(self):
        assert self.motor.commands[-1]=='!', 'motor STOP must precede Master STOP'
        self.closed=True

class IntegratedTests(unittest.IsolatedAsyncioTestCase):
    async def test_changed_static_contact_pose_stops_before_next_fire(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg=Config(speeds=(300,),repeats=2,pause=0,settle_seconds=0)
            motor=FakeMotor();capture=FakeCapture(motor);files=RunFiles(Path(tmp),cfg)
            poses=iter(([14.,2.,-40.],[14.,2.,-40.],[30.,2.,-40.]))
            capture.pose=lambda:next(poses)
            try:
                with self.assertRaisesRegex(RuntimeError,'静态姿态'):
                    await MotorSweep(cfg,motor,files,lambda _:None,capture=capture).run()
            finally:files.close()
            self.assertEqual(motor.commands.count('FIRE'),1);self.assertTrue(capture.closed)
    async def test_complete_140_trial_pairing_and_offline_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg=Config(pause=0,grade_pause=0,settle_seconds=0)
            motor=FakeMotor();capture=FakeCapture(motor);files=RunFiles(Path(tmp),cfg)
            try:await MotorSweep(cfg,motor,files,lambda _:None,capture=capture).run()
            finally:files.close()
            self.assertEqual(motor.commands.count('HOME'),140);self.assertEqual(capture.windows,list(range(1,141)))
            paths=list(files.directory.glob('speed_*/trial_*/result.json'));self.assertEqual(len(paths),140)
            for p in paths:
                d=json.loads(p.read_text());n=d['row']['number'];self.assertEqual(d['row']['accel_peak_counts'],n+500)
                self.assertEqual(d['accel_metrics']['accel_peak_seq'],n*10000+250)
            ss=report(files.directory);self.assertEqual(ss[0]['n'],20)
            self.assertTrue((files.directory/'grade_comparison.png').exists())
    async def test_dropout_priority_stop_no_extra_home_or_fire(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg=Config(repeats=2,pause=0,settle_seconds=0)
            motor=FakeMotor(fault='cancel');capture=FakeCapture(motor,fail=True);files=RunFiles(Path(tmp),cfg)
            try:
                with self.assertRaisesRegex(RuntimeError,'dropout'):
                    await MotorSweep(cfg,motor,files,lambda _:None,capture=capture).run()
            finally:files.close()
            self.assertTrue(capture.closed);self.assertEqual(motor.commands.count('FIRE'),1)
            self.assertEqual(motor.commands.count('HOME'),1)

if __name__=='__main__':unittest.main()
