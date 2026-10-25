"""Never let the Master's uploader reset/write the simultaneously connected motor."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import upload_master_tinyusb as uploader

class PortIdentityTests(unittest.TestCase):
    def test_motor_is_rejected_before_any_reset(self):
        motor=SimpleNamespace(device='/dev/cu.usbmodem1101',serial_number='C4:9E:7E:17:C3:2C')
        with patch.object(uploader.list_ports,'comports',return_value=[motor]),patch.object(uploader.serial,'Serial') as port:
            with self.assertRaises(RuntimeError):uploader.enter_bootloader(rom_port=motor.device)
            with self.assertRaises(RuntimeError):uploader.enter_bootloader(runtime_port=motor.device)
            port.assert_not_called()
    def test_master_rom_is_selected_by_identity_not_port_number(self):
        motor=SimpleNamespace(device='/dev/cu.usbmodem1101',serial_number='C4:9E:7E:17:C3:2C')
        master=SimpleNamespace(device='/dev/cu.usbmodem101',serial_number='1C:DB:D4:99:C8:A0')
        with patch.object(uploader.list_ports,'comports',return_value=[motor,master]):
            self.assertEqual(uploader.enter_bootloader(),master.device)
