"""A single background asyncio thread for either USB serial or BLE."""
from __future__ import annotations

import asyncio
import threading

import serial
from bleak import BleakClient, BleakScanner
from PySide6.QtCore import QObject, Signal

from hammer_protocol import LineBuffer

NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
DEFAULT_DEVICE = 'Neuro_Hammer_BLE'
DEVICE_ALIASES = {'Neuro_Hammer_BLE', 'Rotary_Hammer_BLE'}


class TransportWorker(QObject):
    line_received = Signal(str)
    connected_changed = Signal(bool)
    status_changed = Signal(str)
    error_occurred = Signal(str)

    def __init__(self):
        super().__init__()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.serial = None
        self.client = None
        self.reader = None
        self.generation = 0
        self.started = threading.Event()
        self.thread.start()
        if not self.started.wait(2):
            raise RuntimeError("通信线程启动失败")

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.connection_lock = asyncio.Lock()
        self.write_lock = asyncio.Lock()
        self.started.set()
        self.loop.run_forever()

    def _submit(self, coroutine):
        future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        def done(f):
            try:
                f.result()
            except Exception as exc:
                self.error_occurred.emit(str(exc))
        future.add_done_callback(done)
        return future

    def connect_device(self, mode: str, identifier: str):
        self._submit(self._connect(mode, identifier))

    def disconnect_device(self):
        self._submit(self._disconnect())

    def send(self, command: str):
        self._submit(self._send(command,self.generation))

    async def _connect(self, mode, identifier):
        async with self.connection_lock:
            await self._disconnect_unlocked(announce=False)
            self.status_changed.emit('正在连接…')
            self.generation += 1
            token = self.generation
            lines = LineBuffer()
            try:
                if mode == 'USB':
                    self.serial = serial.Serial(identifier, 115200, timeout=0, write_timeout=.2)
                    # Opening the native USB port can reset the ESP32. Let setup finish.
                    await asyncio.sleep(1.2)
                    self.serial.reset_input_buffer()
                    self.reader = asyncio.create_task(self._read_serial(self.serial, token, lines))
                else:
                    self.status_changed.emit('正在扫描 BLE：'+identifier)
                    def match(d,a):
                        names={d.name,d.address,a.local_name}
                        return identifier in names or (identifier in DEVICE_ALIASES and bool(names & DEVICE_ALIASES))
                    device = await BleakScanner.find_device_by_filter(match,timeout=8)
                    if device is None:
                        raise RuntimeError(f'未找到 BLE 设备：{identifier}')
                    self.status_changed.emit('正在连接 BLE：'+(device.name or device.address))
                    client = BleakClient(device, timeout=12, disconnected_callback=lambda c:
                        self.loop.call_soon_threadsafe(self._ble_lost, token))
                    self.client = client
                    await client.connect()
                    def notify(_sender, data):
                        if token == self.generation:
                            for line in lines.feed(bytes(data)):
                                self.line_received.emit(line)
                    required={NUS_RX_UUID.lower(),NUS_TX_UUID.lower()}
                    available={ch.uuid.lower() for service in client.services for ch in service.characteristics}
                    if not required <= available:raise RuntimeError('设备没有电机 UART 服务，请核对固件')
                    await client.start_notify(NUS_TX_UUID, notify)
                self.connected_changed.emit(True)
                self.status_changed.emit(f'已连接 {mode}：{identifier}')
            except Exception:
                await self._disconnect_unlocked()
                raise

    async def _read_serial(self, connection, token, lines):
        try:
            while token == self.generation and connection.is_open:
                data = connection.read(min(connection.in_waiting, 8192))
                for line in lines.feed(data):
                    self.line_received.emit(line)
                await asyncio.sleep(.005)
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.error_occurred.emit(f'串口中断：{exc}')
            if token == self.generation:
                await self._disconnect_unlocked()

    def _ble_lost(self, token):
        if token == self.generation:
            self.generation += 1
            self.client = None
            self.connected_changed.emit(False)
            self.status_changed.emit('BLE 已断开，请手动重新连接')

    async def _send(self, command, token=None):
        if token is not None and token != self.generation:return
        data = (command.strip()+'\n').encode('ascii')
        async with self.write_lock:
            if token is not None and token != self.generation:return
            try:
                if self.serial is not None and self.serial.is_open:
                    if self.serial.write(data) != len(data):
                        raise RuntimeError('串口写入不完整')
                elif self.client is not None and self.client.is_connected:
                    for offset in range(0, len(data), 20):
                        await self.client.write_gatt_char(NUS_RX_UUID, data[offset:offset+20], response=True)
                else:
                    raise RuntimeError('尚未连接设备')
            except Exception:
                await self._disconnect_unlocked()
                raise

    async def _disconnect(self):
        async with self.connection_lock:
            try:
                if self.serial is not None or self.client is not None:
                    await self._send('!')
                    await asyncio.sleep(.08)
            except Exception:
                pass
            await self._disconnect_unlocked()

    async def _disconnect_unlocked(self, announce=True):
        self.generation += 1
        reader, self.reader = self.reader, None
        if reader is not None and reader is not asyncio.current_task():
            reader.cancel()
        connection, self.serial = self.serial, None
        if connection is not None:
            connection.close()
        client, self.client = self.client, None
        if client is not None and client.is_connected:
            await client.disconnect()
        if announce:
            self.connected_changed.emit(False)
            self.status_changed.emit('未连接')

    def close(self):
        try:
            self._submit(self._disconnect()).result(timeout=4)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2)
