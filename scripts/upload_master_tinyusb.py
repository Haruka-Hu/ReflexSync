#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import time

import serial
from serial.tools import list_ports

RUNTIME_PORT = "/dev/cu.usbmodem1CDBD499C8A01"
MASTER_SERIAL = "1CDBD499C8A0"
MOTOR_SERIAL = "C49E7E17C32C"

def device_serial(port: str) -> str:
    entry=next((p for p in list_ports.comports() if p.device==port),None)
    return (entry.serial_number or '').replace(':','').upper() if entry else ''


def wait_for_port(path: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path in {port.device for port in list_ports.comports()}:
            return True
        time.sleep(0.1)
    return False


def enter_bootloader(runtime_port: str = RUNTIME_PORT, rom_port: str | None = None) -> str:
    # USB names change at reset. In this bench usbmodem1101 belongs to MOTOR;
    # its mere presence must never be treated as the Master's ROM interface.
    if rom_port:
        if device_serial(rom_port)!=MASTER_SERIAL:
            raise RuntimeError('指定引导口的芯片身份不是Master；拒绝烧录')
        return rom_port
    for entry in list_ports.comports():
        if device_serial(entry.device)==MASTER_SERIAL and entry.device!=runtime_port:
            return entry.device
    if not wait_for_port(runtime_port, 2.0):
        ports = ", ".join(port.device for port in list_ports.comports()) or "none"
        raise RuntimeError(f"Master TinyUSB port not found; detected: {ports}")
    if device_serial(runtime_port)!=MASTER_SERIAL:
        raise RuntimeError('运行口的芯片身份不是已确认的Master；拒绝复位或烧录')

    # Arduino-ESP32 TinyUSB uses a 1200-baud line-coding request to reboot
    # persistently into the ROM serial bootloader.
    port = serial.Serial(runtime_port, 1200, timeout=0.1)
    port.close()
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        for entry in list_ports.comports():
            if entry.device!=runtime_port and device_serial(entry.device)==MASTER_SERIAL:
                return entry.device
        time.sleep(.1)
    raise RuntimeError('未发现与Master芯片身份匹配的引导口；拒绝使用其他ESP32端口')


def main() -> None:
    parser = argparse.ArgumentParser(description="Enter TinyUSB bootloader and upload Master firmware")
    parser.add_argument("--port", default=RUNTIME_PORT, help="Master runtime USB port; specify on a different computer")
    parser.add_argument("--rom-port", help="可选；仍须通过Master芯片身份校验，默认自动发现")
    args = parser.parse_args()
    rom_port=enter_bootloader(args.port, args.rom_port)
    if device_serial(rom_port)!=MASTER_SERIAL:
        raise RuntimeError('上传前Master身份复核失败')
    subprocess.run(
        [
            "platformio",
            "run",
            "-e",
            "master",
            "-t",
            "upload",
            "--upload-port",
            rom_port,
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
