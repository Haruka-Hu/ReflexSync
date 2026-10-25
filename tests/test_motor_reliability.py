"""Offline regression: pixi run python -m unittest discover -s tests -v.

Native encoder coverage also needs a C++17 compiler (c++ or clang++).
No ports are opened and no device/motor commands leave the fake worker.
"""
from __future__ import annotations
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import random
import shutil
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication
from hammer_gui import MainWindow
from hammer_protocol import LineBuffer, TracePoint, TraceReceiver, TraceReplacedError


def frame(payload):
    return f'{payload} crc={zlib.crc32(payload.encode("ascii")):08x}'


def chunk(offset=0,request=1,*,total=16,shot=44,crc=True):
    end=min(total,offset+8)
    rows=[f'TRACE_BEGIN shot={shot} total={total} offset={offset} request={request} '
          'target=500 window_us=6000 reason=SPEED_DROP contact_candidate=1 '
          'retract_angle=50 kp=3 ki=20 ks=-96.7 kv=.4477 kg=3.25']
    rows += [f'TRACE shot={shot} request={request} i={i} us={i*1000} '
             f'angle={-50+i*.5:.3f} speed=500 raw=416.667 pwm=200 drop=500 baseline=500'
             for i in range(offset,end)]
    rows += [f'TRACE_END shot={shot} next={end} total={total} offset={offset} request={request}']
    return [frame(row) for row in rows] if crc else rows


class FakeWorker(QObject):
    line_received=Signal(str)
    connected_changed=Signal(bool)
    status_changed=Signal(str)
    error_occurred=Signal(str)
    def __init__(self):
        super().__init__(); self.sent=[]
    def send(self,command): self.sent.append(command)
    def close(self): pass


class HostReliability(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=QApplication.instance() or QApplication([])
    def setUp(self):
        self.worker=FakeWorker(); self.window=MainWindow(self.worker)
        w=self.window; w.timer.stop(); w.port_timer.stop()
        w.connected=w.ready=w.zeroed=True; w.state='IDLE'
        w.mode.setCurrentText('BLE')
        w.firmware_info={'protocol':'2','features':'TRACE_CRC,TRACE_REQUEST_ID,PCNT'}
    def tearDown(self): self.window.close()
    def begin(self): self.window.download_trace(expected_shot=44)
    def deliver(self,rows):
        for row in rows: self.window._line(row)
    def requested(self):
        words=self.worker.sent[-1].split()
        return int(words[1]),int(words[2]) if len(words)>2 else 0
    def finish(self,total=16,crc=True):
        while self.window.trace_busy:
            offset,request=self.requested()
            self.deliver(chunk(offset,request,total=total,crc=crc))
    def test_fragmented_ble_and_complete(self):
        self.begin(); buffer=LineBuffer(); rng=random.Random(48)
        while self.window.trace_busy:
            offset,request=self.requested()
            data=('\n'.join(chunk(offset,request))+'\n').encode()
            while data:
                n=rng.randint(1,182)
                self.deliver(buffer.feed(data[:n])); data=data[n:]
        self.assertTrue(self.window.trace_complete)
        self.assertEqual(list(sorted(self.window.trace_points)),list(range(16)))
    def test_missing_and_corrupt_rows_retry_from_first_hole(self):
        self.begin(); offset,request=self.requested(); rows=chunk(offset,request)
        rows[5]=rows[5].replace('speed=500','speed=501')  # numeric damage, valid syntax
        del rows[7]
        self.deliver(rows)
        self.assertEqual(self.requested()[0],4)
        self.assertTrue(self.window.trace_busy)
        self.finish(); self.assertTrue(self.window.trace_complete)
    def test_bad_trace_does_not_clear_set_ack(self):
        self.begin(); pending=('SET 50 500','OK SET',None,float('inf'))
        self.window.pending=pending
        self.deliver(['TRACE shot=44 i=1 angle=1TRACE_END',frame('TRACE shot=44 request=1 i=1 us=1 angle=nan speed=1 raw=1 pwm=0 drop=0 baseline=0')])
        self.assertEqual(self.window.pending,pending)
        self.window.cancel_trace(); self.window._line('ERR TRACE busy')
        self.assertEqual(self.window.pending,pending)
        self.window._line('OK SET retract_angle=50 strike_speed=500')
        self.assertIsNone(self.window.pending)
    def test_cancel_and_restart_ignores_delayed_old_chunk(self):
        self.begin(); old=self.requested()[1]
        self.window.cancel_trace(); self.begin()
        self.deliver(chunk(request=old,shot=43))
        self.assertEqual(self.window.trace_total,0)
        self.assertFalse(self.window.trace_points)
        self.finish(); self.assertTrue(self.window.trace_complete)
    def test_bad_begin_end_timeout_and_bounded_retry(self):
        self.begin(); rows=chunk()
        self.deliver(rows[1:])  # lost BEGIN: no row may be committed
        self.assertFalse(self.window.trace_points)
        for _ in range(4):
            self.window.trace_deadline=0; self.window._tick()
        self.assertFalse(self.window.trace_busy)
        self.assertFalse(self.window.trace_complete)
        self.assertFalse(self.window.export_button.isEnabled())
    def test_legacy_v7_missing_us_recovers(self):
        self.window.firmware_info={'protocol':'2'}; self.begin()
        rows=chunk(request=0,crc=False)
        rows[3]=rows[3].replace('us=2000 ','')
        self.deliver(rows)
        self.assertEqual(self.requested()[0],2)
        self.finish(crc=False); self.assertTrue(self.window.trace_complete)
    def test_changed_shot_aborts_without_export(self):
        self.begin(); self.deliver(chunk(shot=45))
        self.assertFalse(self.window.trace_busy)
        self.assertFalse(self.window.trace_complete)
    def test_zero_samples_is_valid_empty_result(self):
        self.begin(); self.deliver(chunk(total=0))
        self.assertFalse(self.window.trace_busy)
        self.assertFalse(self.window.trace_complete)
    def test_timestamp_corruption_blocks_export(self):
        self.begin(); rows=chunk(total=8)
        payload=rows[3].split(' crc=')[0].replace('us=2000','us=0')
        rows[3]=frame(payload)
        self.deliver(rows)
        self.assertFalse(self.window.trace_complete)
        self.assertFalse(self.window.export_button.isEnabled())
    def test_abort_reason_and_raw_diagnostics_are_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'session.log'
            self.window.session_log=path.open('w',encoding='utf-8')
            self.window._line('DIAG encoder=PCNT count=-120 isr_count=-120')
            self.window._line('EVT ABORT reason=BLE_DISCONNECT angle=-25 speed=0 shot=0 samples=0')
            self.window.session_log.close(); self.window.session_log=None
            content=path.read_text()
        self.assertIn('count=-120',content)
        self.assertIn('reason=BLE_DISCONNECT',content)
        self.assertFalse(self.window.zeroed)
        self.assertEqual(self.window.state,'IDLE')

    def test_log_disk_failure_does_not_interrupt_stop_handling(self):
        class FailedLog:
            def write(self,text):raise OSError('disk full')
            def close(self):raise OSError('disk full')
        self.window.session_log=FailedLog()
        self.window._line('EVT ABORT reason=USER_STOP angle=-25 speed=0 shot=0 samples=0')
        self.assertIsNone(self.window.session_log)
        self.assertFalse(self.window.zeroed)
        self.assertEqual(self.window.state,'IDLE')

    def test_persistent_crc_damage_stops_after_three_retries(self):
        self.begin()
        for _ in range(4):
            offset,request=self.requested(); rows=chunk(offset,request)
            rows[1]=rows[1].replace('us=0','us=1')
            self.deliver(rows)
        self.assertFalse(self.window.trace_busy)
        self.assertFalse(self.window.trace_complete)


class ProtocolReliability(unittest.TestCase):
    def test_syntactically_valid_corruption_rejected(self):
        receiver=TraceReceiver(require_crc=True,request_ids=True)
        receiver.request(0,1)
        rows=chunk(); receiver.feed(rows[0])
        with self.assertRaises(ValueError): receiver.feed(rows[1].replace('angle=-50','angle=-40'))
        self.assertFalse(receiver.points)
    def test_out_of_chunk_and_nonfinite_rejected(self):
        receiver=TraceReceiver(require_crc=True,request_ids=True); receiver.request(0,1)
        receiver.feed(chunk()[0])
        for payload in ('TRACE shot=44 request=1 i=99 us=1 angle=0 speed=0 raw=0 pwm=0 drop=0 baseline=0',
                        'TRACE shot=44 request=1 i=1 us=1 angle=inf speed=0 raw=0 pwm=0 drop=0 baseline=0'):
            with self.assertRaises(ValueError): receiver.feed(frame(payload))
    def test_concatenation_and_missing_crc_rejected(self):
        receiver=TraceReceiver(require_crc=True,request_ids=True); receiver.request(0,1)
        rows=chunk()
        with self.assertRaises(ValueError): receiver.feed(rows[0]+rows[1])
        with self.assertRaises(ValueError): receiver.feed(rows[0].split(' crc=')[0])
        self.assertIsNone(receiver.total)
    def test_lost_fragment_recovered_across_many_chunks(self):
        rng=random.Random(375); receiver=TraceReceiver(require_crc=True,request_ids=True)
        buffer=LineBuffer(); request=0; damaged=0
        while len(receiver.points)<256:
            request+=1
            self.assertLess(request,150)
            offset=next(i for i in range(256) if i not in receiver.points)
            receiver.request(offset,request)
            data=('\n'.join(chunk(offset,request,total=256))+'\n').encode()
            fragments=[data[i:i+20] for i in range(0,len(data),20)]
            drop=rng.randrange(len(fragments)) if request%3==0 else -1
            for i,fragment in enumerate(fragments):
                if i==drop: damaged+=1; continue
                for line in buffer.feed(fragment):
                    try: receiver.feed(line)
                    except (KeyError,ValueError): pass
        self.assertGreater(damaged,0)
        self.assertEqual(len(receiver.points),256)


class NativeEncoder(unittest.TestCase):
    def test_production_encoder_and_controller(self):
        compiler=shutil.which('c++')
        if compiler is None: self.skipTest('C++17 compiler required')
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'encoder_test')
            subprocess.run([compiler,'-std=c++17','-DARDUINO_ARCH_ESP32',
                '-I'+str(ROOT/'tests/motor_fakes'),'-I'+str(ROOT/'src/motor'),
                *(str(ROOT/p) for p in ('tests/motor_encoder_test.cpp',
                'src/motor/hall_encoder.cpp','src/motor/motion_controller.cpp',
                'src/motor/drv8871_motor.cpp')),'-o',binary],check=True)
            subprocess.run([binary],check=True)

    def test_actual_firmware_trace_framing_matches_host_crc(self):
        compiler=shutil.which('c++')
        if compiler is None: self.skipTest('C++17 compiler required')
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'protocol_test')
            subprocess.run([compiler,'-std=c++17','-DARDUINO_ARCH_ESP32',
                '-I'+str(ROOT/'tests/motor_fakes'),'-I'+str(ROOT/'src/motor'),
                *(str(ROOT/p) for p in ('tests/motor_protocol_test.cpp',
                'src/motor/hall_encoder.cpp','src/motor/motion_controller.cpp',
                'src/motor/drv8871_motor.cpp')),'-o',binary],check=True)
            output=subprocess.check_output([binary],text=True)
        receiver=TraceReceiver(require_crc=True,request_ids=True)
        receiver.request(0,0xffffffff)
        complete=False
        for line in output.splitlines(): complete=receiver.feed(line)
        self.assertTrue(complete)
        self.assertEqual(receiver.total,0)

if __name__=='__main__': unittest.main()
