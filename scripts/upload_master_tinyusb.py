#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import time

import serial
from serial.tools import list_ports

RUNTIME_PORT = "/dev/cu.usbmodem1CDBD499C8A01"
ROM_PORT = "/dev/cu.usbmodem1101"


def wait_for_port(path: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path in {port.device for port in list_ports.comports()}:
            return True
        time.sleep(0.1)
    return False


def enter_bootloader(runtime_port: str = RUNTIME_PORT, rom_port: str = ROM_PORT) -> None:
    if wait_for_port(rom_port, 0.2):
        return
    if not wait_for_port(runtime_port, 2.0):
        ports = ", ".join(port.device for port in list_ports.comports()) or "none"
        raise RuntimeError(f"Master TinyUSB port not found; detected: {ports}")

    # Arduino-ESP32 TinyUSB uses a 1200-baud line-coding request to reboot
    # persistently into the ROM serial bootloader.
    port = serial.Serial(runtime_port, 1200, timeout=0.1)
    port.close()
    if not wait_for_port(rom_port, 8.0):
        ports = ", ".join(port.device for port in list_ports.comports()) or "none"
        raise RuntimeError(f"ROM bootloader port did not appear; detected: {ports}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Enter TinyUSB bootloader and upload Master firmware")
    parser.add_argument("--port", default=RUNTIME_PORT, help="Master runtime USB port; specify on a different computer")
    parser.add_argument("--rom-port", default=ROM_PORT, help="ROM bootloader USB port")
    args = parser.parse_args()
    enter_bootloader(args.port, args.rom_port)
    subprocess.run(
        [
            "platformio",
            "run",
            "-e",
            "master",
            "-t",
            "upload",
            "--upload-port",
            args.rom_port,
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
