#!/usr/bin/env python3
"""USB/BLE rotary hammer controller: pixi run gui."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget, QHeaderView, QTabWidget,
)
from serial.tools import list_ports

from hammer_protocol import TracePoint, analyze_trace, fields
from hammer_transport import DEFAULT_DEVICE, TransportWorker

CONFIG_FILE = Path(__file__).resolve().parents[1] / 'hammer_profiles.json'
DEFAULT_PROFILES = [{'angle': 60.0, 'speed': float(s)} for s in (200,400,600,800,1000)]
TUNING_KEYS = ('kp','ki','ks','kv','kg')
TUNING_DEFAULTS = (3,20,-96.7,.4477,3.25)


class MainWindow(QMainWindow):
    def __init__(self, worker=None):
        super().__init__()
        self.worker = worker or TransportWorker()
        self.connected = self.connecting = False
        self.state = 'DISCONNECTED'
        self.zeroed = False
        self.homing = False
        self.received_status = False
        self.ready = False
        self.firmware_info = {}
        self.connection_error = ""
        self.status_inflight = False
        self.last_status_received = 0
        self.request_attempts = 0
        self.jog_speed = 0
        self.last_jog_send = 0
        self.pending = None
        self.trace_busy = False
        self.trace_points = {}
        self.trace_shot = -1
        self.trace_total = 0
        self.trace_offset = 0
        self.trace_retries = 0
        self.trace_deadline = 0
        self.trace_target = 200.0
        self.trace_window = 6000
        self.shot_event = {}
        self.metrics = {}
        self.trace_settings = {}
        self.confirmed_settings = {}
        self.confirmed_tuning = {}
        self.live = deque(maxlen=600)
        self.last_status_request = 0
        self.profiles = self._load_profiles()
        self.setWindowTitle('旋转叩击电机 · 速度闭环')
        self.resize(1000,790)
        self.setMinimumSize(820,680)
        self._build()
        self.worker.line_received.connect(self._line)
        self.worker.connected_changed.connect(self._connected)
        self.worker.status_changed.connect(self.connection_label.setText)
        self.worker.error_occurred.connect(self._error)
        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(40)
        self.refresh_ports()
        self._buttons()

    @staticmethod
    def _load_profiles():
        try:
            profiles = json.loads(CONFIG_FILE.read_text())['profiles']
            if len(profiles) != 5 or any(not all(math.isfinite(float(p[k])) and
                    0 < float(p[k]) <= (360 if k=='angle' else 3000)
                    for k in ('angle','speed')) for p in profiles):
                raise ValueError('invalid profile')
            return profiles
        except (OSError, ValueError, KeyError, TypeError):
            return [dict(p) for p in DEFAULT_PROFILES]

    @staticmethod
    def _spin(value, minimum, maximum, decimals=2):
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def _build(self):
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)
        layout.setSpacing(9)
        header=QHBoxLayout()
        self.mode=QComboBox(); self.mode.addItems(['USB','BLE'])
        self.ports=QComboBox(); self.ports.setMinimumWidth(210)
        self.ble_device=QLineEdit(DEFAULT_DEVICE); self.ble_device.setPlaceholderText('BLE 名称或地址')
        self.refresh_button=QPushButton('刷新串口'); self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button=QPushButton('连接'); self.connect_button.clicked.connect(self.toggle_connection)
        for w in (self.mode,self.ports,self.ble_device,self.refresh_button,self.connect_button):header.addWidget(w)
        self.connection_label=QLabel('未连接'); self.connection_label.setWordWrap(True)
        header.addWidget(self.connection_label,1); layout.addLayout(header)
        self.mode.currentTextChanged.connect(self._mode_changed); self._mode_changed()
        self.state_label=QLabel('未连接'); self.state_label.setStyleSheet('font-size:16px;font-weight:bold')
        layout.addWidget(self.state_label)
        self.help_label=QLabel(); self.help_label.setWordWrap(True); layout.addWidget(self.help_label)
        self.motion_label=QLabel('本次动作：尚未启动'); self.motion_label.setWordWrap(True); layout.addWidget(self.motion_label)
        self.tabs=QTabWidget(); layout.addWidget(self.tabs,1)

        panel=QWidget(); controls=QVBoxLayout(panel); self.tabs.addTab(panel,'电机操作')
        gears=QGroupBox('五档快捷叩击 · 使用表格中的角度与速度'); row=QHBoxLayout(gears)
        self.gear_buttons=[]
        for i,p in enumerate(self.profiles):
            b=QPushButton(f'{i+1}档\n{float(p["speed"]):g} °/s'); b.setMinimumHeight(58)
            b.clicked.connect(lambda _=False,n=i:self.profile_start(n)); row.addWidget(b); self.gear_buttons.append(b)
        controls.addWidget(gears)
        self.table=QTableWidget(5,2); self.table.setHorizontalHeaderLabels(['回撤角度 (°)','目标速度 (°/s)'])
        self.table.setVerticalHeaderLabels([f'{i}档' for i in range(1,6)])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(28); self.table.setMaximumHeight(180)
        for i,p in enumerate(self.profiles):
            self.table.setItem(i,0,QTableWidgetItem(str(p['angle'])))
            self.table.setItem(i,1,QTableWidgetItem(str(p['speed'])))
        self.table.itemChanged.connect(self._profile_labels); controls.addWidget(self.table)
        save=QPushButton('保存五档配置到本地'); save.clicked.connect(self.save_profiles); controls.addWidget(save)
        params=QGroupBox('单次动作参数'); form=QGridLayout(params)
        self.angle=self._spin(60,.1,360); self.speed=self._spin(200,1,3000)
        form.addWidget(QLabel('回撤角度 (°)'),0,0); form.addWidget(self.angle,0,1)
        form.addWidget(QLabel('目标速度 (°/s)'),0,2); form.addWidget(self.speed,0,3)
        self.apply_button=QPushButton('下发参数'); self.apply_button.clicked.connect(self.apply_settings)
        form.addWidget(self.apply_button,0,4)
        self.confirmed_label=QLabel('设备参数：等待连接读取'); self.confirmed_label.setWordWrap(True)
        form.addWidget(self.confirmed_label,1,0,1,5); controls.addWidget(params)
        actions=QHBoxLayout()
        self.start_button=QPushButton('一键回撤 → 叩击'); self.start_button.setMinimumHeight(48)
        self.start_button.clicked.connect(self.start)
        self.retract_button=QPushButton('仅回撤'); self.retract_button.clicked.connect(lambda:self.apply_settings('RETRACT'))
        self.fire_button=QPushButton('叩击 FIRE'); self.fire_button.clicked.connect(self.fire)
        for w in (self.start_button,self.retract_button,self.fire_button):actions.addWidget(w)
        controls.addLayout(actions)
        home=QHBoxLayout()
        self.zero_button=QPushButton('当前位置归零'); self.zero_button.clicked.connect(self.zero)
        self.home_button=QPushButton('慢速接近靶面并归零'); self.home_button.clicked.connect(self.home)
        home.addWidget(self.zero_button); home.addWidget(self.home_button); controls.addLayout(home)
        manual=QHBoxLayout()
        self.forward_button=QPushButton('按住正转 · 40°/s')
        self.reverse_button=QPushButton('按住反转 · 40°/s')
        for w,v in ((self.forward_button,40),(self.reverse_button,-40)):
            w.setMinimumHeight(40); w.setAutoRepeat(False)
            w.pressed.connect(lambda value=v:self.jog(value)); w.released.connect(self.stop); manual.addWidget(w)
        controls.addLayout(manual)
        note=QLabel('手动移动或 STOP 后需重新归零。1000°/s 档的接触检测尚未通过实机验收。')
        note.setWordWrap(True); controls.addWidget(note); controls.addStretch()

        plots=QWidget(); rv=QVBoxLayout(plots); self.tabs.addTab(plots,'速度与轨迹')
        pg.setConfigOptions(antialias=True)
        self.graph=pg.GraphicsLayoutWidget(); self.graph.setBackground('#17212b')
        self.speed_plot=self.graph.addPlot(row=0,col=0,title='角速度'); self.speed_plot.addLegend()
        self.speed_plot.getAxis('left').enableAutoSIPrefix(False)
        self.curve_target=self.speed_plot.plot(pen=pg.mkPen('#aeb5be',style=Qt.DashLine),name='设定')
        self.curve_speed=self.speed_plot.plot(pen=pg.mkPen('#4dd3c0',width=2),name='反馈')
        self.curve_raw=self.speed_plot.plot(pen=pg.mkPen('#657484',width=1),name='原始计数测速')
        self.speed_plot.setLabel('left','速度','°/s')
        self.angle_plot=self.graph.addPlot(row=1,col=0,title='角度')
        self.curve_angle=self.angle_plot.plot(pen='#f3c96a'); self.angle_plot.setLabel('left','角度','°')
        self.pwm_plot=self.graph.addPlot(row=2,col=0,title='PWM（有符号；BRAKE 表示双高制动）')
        self.curve_pwm=self.pwm_plot.plot(pen='#cb9ef0'); self.pwm_plot.setLabel('left','PWM')
        self.angle_plot.setXLink(self.speed_plot); self.pwm_plot.setXLink(self.speed_plot)
        for plot in (self.speed_plot,self.angle_plot,self.pwm_plot):
            plot.showGrid(x=True,y=True,alpha=.15); plot.setLabel('bottom','时间','ms')
        rv.addWidget(self.graph,1)
        self.result_label=QLabel('叩击结束后自动读取完整轨迹；可取消读取后继续操作')
        self.result_label.setWordWrap(True); rv.addWidget(self.result_label)
        er=QHBoxLayout()
        self.trace_button=QPushButton('读取最近轨迹'); self.trace_button.clicked.connect(self.download_trace)
        self.cancel_trace_button=QPushButton('取消读取'); self.cancel_trace_button.clicked.connect(self.cancel_trace)
        self.export_button=QPushButton('导出 CSV + 参数快照'); self.export_button.clicked.connect(self.export_trace)
        for w in (self.trace_button,self.cancel_trace_button,self.export_button):er.addWidget(w)
        rv.addLayout(er)
        tune=QWidget(); tf=QFormLayout(tune); self.tuning={}; self.tabs.addTab(tune,'PI / 前馈调参')
        for key,value,maximum in zip(TUNING_KEYS,TUNING_DEFAULTS,(100,1000,1000,10,1000)):
            spin=self._spin(value,-1000 if key in ('ks','kg') else 0,maximum,4)
            self.tuning[key]=spin; tf.addRow(key.upper(),spin)
        self.tune_button=QPushButton('下发并回读控制参数'); self.tune_button.clicked.connect(self.apply_tuning)
        tf.addRow(self.tune_button)
        self.tuning_label=QLabel('设备调参值：等待读取'); self.tuning_label.setWordWrap(True); tf.addRow(self.tuning_label)
        self.sync_button=QPushButton('重新读取设备参数'); self.sync_button.clicked.connect(self.synchronize)
        tf.addRow(self.sync_button)
        logpanel=QWidget(); loglayout=QVBoxLayout(logpanel); self.tabs.addTab(logpanel,'通信日志')
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(800); loglayout.addWidget(self.log)
        foot=QHBoxLayout()
        self.feedback_label=QLabel('角度 --   速度 --   PWM --'); self.feedback_label.setWordWrap(True)
        foot.addWidget(self.feedback_label,1)
        self.stop_button=QPushButton('STOP 停止'); self.stop_button.setMinimumSize(170,48)
        self.stop_button.setStyleSheet('background:#b52d36;color:white;font-weight:bold;font-size:17px')
        self.stop_button.clicked.connect(self.stop); foot.addWidget(self.stop_button); layout.addLayout(foot)

    def _profile_labels(self):
        for i,b in enumerate(self.gear_buttons):
            item=self.table.item(i,1)
            b.setText(f'{i+1}档\n{item.text() if item else "--"} °/s')

    def _mode_changed(self):
        usb=self.mode.currentText()=='USB'
        self.ports.setVisible(usb); self.refresh_button.setVisible(usb); self.ble_device.setVisible(not usb)

    def refresh_ports(self):
        current=self.ports.currentText()
        ports=[p.device for p in list_ports.comports()]
        if ports != [self.ports.itemText(i) for i in range(self.ports.count())]:
            self.ports.clear(); self.ports.addItems(ports)
            if current in ports: self.ports.setCurrentText(current)
            elif ports:
                self.ports.setCurrentText(next((p for p in ports if 'usbmodem' in p or 'COM' in p),ports[0]))

    def toggle_connection(self):
        if self.connected:
            self.worker.disconnect_device(); return
        identifier=self.ports.currentText() if self.mode.currentText()=='USB' else self.ble_device.text().strip()
        if not identifier: self._error('请选择串口或填写 BLE 名称'); return
        self.connecting=True; self._buttons()
        self.worker.connect_device(self.mode.currentText(),identifier)

    def _connected(self, connected):
        self.connected=connected; self.connecting=False; self.ready=False; self.connection_error=""
        self.pending=None; self.trace_busy=False; self.status_inflight=False; self.jog_speed=0
        self.received_status=False; self.zeroed=False; self.homing=False
        self.confirmed_settings={}; self.confirmed_tuning={}; self.firmware_info={}
        self.state='CONNECTING' if connected else 'DISCONNECTED'
        if connected:
            self.trace_shot=-1; self.trace_points={}; self.metrics={}; self.live.clear()
            self.synchronize()
        self._buttons()

    def synchronize(self):
        if not self.connected:return
        self.cancel_trace(); self.pending=None; self.ready=False; self.connection_error=""
        def info(_):
            if self.firmware_info.get('protocol')!='2':
                self._error('固件协议不匹配，请烧录当前工程固件后重新连接'); return
            self._request('STATUS','STATUS ',lambda _:self._request('PARAMS','PARAMS ',
                lambda _:self._request('TUNING','TUNING ',finished)))
        def finished(_):
            self.ready=True
            self.connection_label.setText('已连接 · 协议与参数已确认')
            self._append('连接就绪，可以操作；查询状态不会锁住按钮')
            self._buttons()
        self._request('INFO','INFO ',info)

    def _set_state(self,state):
        self.state=state; self._buttons()

    def _buttons(self):
        usable=self.connected and self.ready and self.pending is None and not self.jog_speed
        idle=usable and self.state=='IDLE'
        self.connect_button.setText('断开' if self.connected else '连接中…' if self.connecting else '连接')
        self.connect_button.setEnabled(not self.connecting)
        for w in (self.mode,self.ports,self.ble_device):w.setEnabled(not self.connected and not self.connecting)
        for b in (self.apply_button,self.tune_button,self.zero_button,self.home_button):b.setEnabled(idle)
        for b in (self.start_button,self.retract_button,*self.gear_buttons):b.setEnabled(idle and self.zeroed)
        self.fire_button.setEnabled(usable and self.state=='READY')
        self.forward_button.setEnabled(idle or bool(self.jog_speed>0))
        self.reverse_button.setEnabled(idle or bool(self.jog_speed<0))
        self.stop_button.setEnabled(self.connected)
        self.trace_button.setEnabled(idle and not self.trace_busy)
        self.cancel_trace_button.setEnabled(self.trace_busy)
        self.sync_button.setEnabled(self.connected and self.pending is None and self.state in ('IDLE','CONNECTING'))
        self.export_button.setEnabled(bool(self.trace_points) and len(self.trace_points)==self.trace_total and not self.trace_busy)
        names={'IDLE':'空闲','RETRACT':'回撤 / 手动移动','READY':'回撤已到位','STRIKE':'叩击中','CONNECTING':'正在确认设备','DISCONNECTED':'未连接'}
        self.state_label.setText(names.get(self.state,self.state)+(' · 慢速复位中' if self.homing else '')+(' · 零点有效' if self.zeroed else ' · 待归零'))
        if not self.connected:hint='先选择 USB 串口或 BLE，点击连接。'
        elif not self.ready:hint=self.connection_error or '正在读取固件版本和参数；可在调参页点击重新读取设备参数。'
        elif self.pending:hint='正在执行：'+self.pending[0]+'；STOP 随时可用。'
        elif self.jog_speed:hint='松开按钮即停止；手动移动后请重新确认零点。'
        elif self.state=='IDLE' and not self.zeroed:hint='机构静止在约定物理零位后，点击「当前位置归零」，叩击按钮即可启用。'
        elif self.state=='IDLE':hint='可以点击五档按钮或一键叩击。'+(' 正在后台读取轨迹，启动新动作将取消读取。' if self.trace_busy else '')
        elif self.state=='READY':hint='回撤已到位，点击 FIRE 叩击，或 STOP 取消。'
        else:hint='动作执行中；可随时 STOP。'
        self.help_label.setText(hint)

    def _request(self,command,prefix,callback=None):
        if not self.connected:
            self._error('尚未连接设备，命令未发送'); return False
        if self.pending is not None:
            self._append('命令未发送：上一操作尚未完成（'+self.pending[0]+'）'); return False
        self.request_attempts=0
        self.pending=(command,prefix,callback,time.monotonic()+2)
        self.worker.send(command); self._append('→ '+command); self._buttons(); return True

    def cancel_trace(self):
        if self.trace_busy:self._append('已取消轨迹读取，未下载完整的轨迹不会作为完整结果导出')
        self.trace_busy=False

    def fire(self):
        self.cancel_trace()
        self._request('FIRE','OK FIRE',lambda _:self._set_state('STRIKE'))

    def jog(self,speed):
        if not self.ready or self.state!='IDLE' or self.pending:return
        self.cancel_trace(); self.jog_speed=speed; self.zeroed=False
        self.state='RETRACT'; self.last_jog_send=time.monotonic()
        self.worker.send(f'JOG {speed:g}'); self._buttons()

    def apply_settings(self,after=None):
        if isinstance(after,bool): after=None
        self.cancel_trace()
        requested={'retract_angle':self.angle.value(),'strike_speed':self.speed.value()}
        def verify(_):
            if any(abs(self.confirmed_settings[k]-value)>.011 for k,value in requested.items()):
                self._error('参数回读不一致，动作未启动'); return
            if after:
                self.live.clear(); self.trace_settings={'settings':dict(self.confirmed_settings),
                                                       'tuning':dict(self.confirmed_tuning)}
                self._request(after,'OK '+after,lambda _:self._set_state('RETRACT'))
        self._request(f"SET {requested['retract_angle']:.2f} {requested['strike_speed']:.2f}",'OK SET ',
                      lambda _:self._request('PARAMS','PARAMS ',verify))

    def start(self):
        self.apply_settings('START')

    def profile_start(self,index):
        try:
            profiles=self._table_profiles(); p=profiles[index]
        except ValueError as exc: self._error(str(exc)); return
        self.angle.setValue(p['angle']); self.speed.setValue(p['speed']); self.start()

    def _table_profiles(self):
        profiles=[]
        try:
            for row in range(5):
                angle=float(self.table.item(row,0).text()); speed=float(self.table.item(row,1).text())
                if not (math.isfinite(angle) and math.isfinite(speed) and 0<angle<=360 and 0<speed<=3000):
                    raise ValueError
                profiles.append({'angle':angle,'speed':speed})
        except (ValueError, AttributeError):
            raise ValueError('档位表需填写有限正数：角度 ≤360°，速度 ≤3000°/s；这些是输入范围，不代表机构允许范围')
        return profiles

    def save_profiles(self):
        try:
            self.profiles=self._table_profiles()
            CONFIG_FILE.write_text(json.dumps({'profiles':self.profiles},ensure_ascii=False,indent=2)+'\n')
            self._append('档位已保存：'+str(CONFIG_FILE))
        except (ValueError,OSError) as exc: self._error(str(exc))

    def apply_tuning(self):
        self.cancel_trace()
        requested={k:w.value() for k,w in self.tuning.items()}
        def verify(_):
            if any(abs(self.confirmed_tuning[k]-v)>.00011 for k,v in requested.items()):
                self._error('控制参数回读不一致'); return
            self._append('控制参数已回读确认')
        self._request('TUNE '+' '.join(f'{requested[k]:.4f}' for k in TUNING_KEYS),'OK TUNE ',
                      lambda _:self._request('TUNING','TUNING ',verify))

    def home(self):
        self.cancel_trace()
        def started(_):
            self.homing=True; self.zeroed=False; self.live.clear()
            self._set_state('RETRACT')
        self._request('HOME','OK HOME ',started)

    def zero(self):
        self.cancel_trace()
        if QMessageBox.question(self,'确认物理零位','确认机构静止，并且当前锤头位于约定的物理零位？')==QMessageBox.Yes:
            def done(_):
                self.zeroed=True; self._request('STATUS','STATUS ')
            self._request('ZERO','OK ZERO ',done)

    def stop(self):
        self.pending=None; self.trace_busy=False; self.jog_speed=0
        self.worker.send('!'); self._append('→ ! STOP')
        self.zeroed=False; self._buttons()

    def _motion_reason(self,reason):
        descriptions={
            'RETRACT_TIMEOUT':'回撤超时，未稳定到位；已制动，请重新确认零位',
            'RETRACT_NO_PROGRESS':'回撤没有位置进展；已制动，请检查机构并重新归零',
            'CONTROL_OR_SENSOR':'控制周期或反馈异常；已制动',
            'USER_STOP':'已手动停止，需重新归零',
            'RETRACTING':'正在回撤，需误差≤1.2°且速度≤20°/s持续130ms',
            'READY':'回撤已稳定到位，等待FIRE',
            'STRIKING':'正在叩击',
            'SPEED_DROP':'检测到速度突降，已撤驱动',
            'TIMEOUT_NO_DROP':'下击超时，未检测到接触；已撤驱动',
            'ZEROED':'已将当前物理位置设为零位',
            'HOMING':'正在慢速接近靶面', 'HOME_DONE':'慢速复位完成',
            'JOGGING':'按住手动移动中', 'JOG_RELEASE':'手动移动刷新停止，已撤驱动',
        }
        if reason in descriptions:self.motion_label.setText('本次动作：'+descriptions[reason])

    def _line(self,line):
        p=fields(line)
        previous_state=self.state
        try:
            if line.startswith('INFO ') and 'protocol' in p:
                self.firmware_info=p
            elif line.startswith('STATUS '):
                self.received_status=True; self.status_inflight=False; self.last_status_received=time.monotonic()
                self.state=p['state']; self.zeroed=p.get('zeroed')=='1'; self.homing=p.get('homing')=='1'
                self._motion_reason(p.get('reason',''))
                self.feedback_label.setText(f"角度 {p['angle']}°   速度 {p['speed']}°/s   PWM {p['pwm']}   {p.get('output','')}")
                if self.state=='RETRACT' and not self.homing and 'pos_error' in p and not self.jog_speed:
                    self.feedback_label.setText(self.feedback_label.text()+f"   到位误差 {p['pos_error']}° / 稳定 {p.get('settled_ms','0')}ms")
                if self.state in ('RETRACT','READY','STRIKE') and not self.trace_busy:
                    self.live.append((time.monotonic()*1000,float(p['angle']),float(p['speed']),float(p['raw']),float(p['pwm'])))
                    self._plot_live()
            elif line.startswith('PARAMS '):
                self.confirmed_settings={k:float(p[k]) for k in ('retract_angle','strike_speed')}
                if not self.ready:
                    self.angle.setValue(self.confirmed_settings['retract_angle']); self.speed.setValue(self.confirmed_settings['strike_speed'])
                self.confirmed_label.setText(f"设备已确认：{p['retract_angle']}° / {p['strike_speed']}°/s")
            elif line.startswith('TUNING '):
                self.confirmed_tuning={k:float(p[k]) for k in TUNING_KEYS}
                for k,w in self.tuning.items(): w.setValue(self.confirmed_tuning[k])
                self.tuning_label.setText('设备已确认：'+'  '.join(f'{k}={p[k]}' for k in TUNING_KEYS))
            elif line.startswith('EVT JOG_END '):
                self.jog_speed=0; self.state='IDLE'; self.zeroed=False
            elif line.startswith('EVT HOME '):
                self.homing=False; self.state='IDLE'; self.zeroed=p.get('zeroed')=='1'
                self.result_label.setText('慢速复位完成：靶面接触候选，PWM已归零')
            elif line.startswith('EVT READY'):
                self.state='READY'
            elif line.startswith('OK FIRE '):
                self.state='STRIKE'; self.live.clear()
                self.trace_settings={'settings':dict(self.confirmed_settings),'tuning':dict(self.confirmed_tuning)}
            elif line.startswith('EVT SHOT '):
                self.state='IDLE'; self.zeroed=p.get('zeroed')=='1'; self.shot_event=p
                self.trace_target=float(p['target_speed'])
                self.result_label.setText('结束：'+p['reason']+'；正在读取轨迹…')
                QTimer.singleShot(100,self.download_trace)
            elif line.startswith('EVT ABORT '):
                self.state='IDLE'; self.zeroed=False; self.homing=False; self.jog_speed=0; self.shot_event=p
                self.result_label.setText('已停止：'+p['reason'])
                self._motion_reason(p['reason'])
            elif line.startswith('TRACE_BEGIN ') and self.trace_busy:
                shot=int(p['shot'])
                if self.trace_shot>=0 and self.trace_shot!=shot: raise ValueError('轨迹已被下一次叩击覆盖')
                self.trace_shot=shot; self.trace_total=int(p['total']); self.trace_target=float(p['target'])
                self.trace_window=int(p['window_us'])
                self.result_label.setText(f'正在读取轨迹：{len(self.trace_points)} / {self.trace_total} 点；可取消')
                if 'reason' in p:
                    self.shot_event={k:p[k] for k in ('shot','reason','contact_candidate','target')}
                if all(k in p for k in (*TUNING_KEYS,'retract_angle')):
                    self.trace_settings={'settings':{'retract_angle':float(p['retract_angle']),
                        'strike_speed':float(p['target'])},'tuning':{k:float(p[k]) for k in TUNING_KEYS}}
            elif line.startswith('TRACE ') and self.trace_busy and int(p['shot'])==self.trace_shot:
                point=TracePoint.parse(line); self.trace_points[point.index]=point
            elif line.startswith('TRACE_END ') and self.trace_busy and int(p['shot'])==self.trace_shot:
                if int(p['total'])!=self.trace_total: raise ValueError('轨迹总数不一致')
                if int(p['next'])>self.trace_offset or self.trace_total==0:
                    self._next_trace_chunk()
            elif line.startswith('ERR '):
                if self.pending and self.pending[0]=='INFO':
                    line='固件不支持当前协议，请烧录本工程固件后重新连接（原始回复：'+line+'）'
                self.jog_speed=0
                self.pending=None
                if self.trace_busy: self.trace_busy=False
                self._error(line)
        except (KeyError,ValueError,OverflowError) as exc:
            self.pending=None; self.trace_busy=False; self._error(f'协议解析失败：{exc}')
        if not line.startswith(('STATUS ','TRACE ','TRACE_BEGIN ','TRACE_END ','OK JOG ')):
            self._append('← '+line)
        if self.pending is not None and line.startswith(self.pending[1]):
            callback=self.pending[2]; self.pending=None
            if callback: callback(line)
        self._buttons()
        if previous_state=='STRIKE' and self.state=='IDLE' and not line.startswith('EVT SHOT'):
            if int(p.get('samples','0'))>0 and int(p.get('shot','-1'))!=self.trace_shot:
                QTimer.singleShot(150,self.download_trace)

    def download_trace(self):
        if not self.connected or self.state!='IDLE' or self.trace_busy: return
        if self.pending is not None:
            QTimer.singleShot(150,self.download_trace); return
        self.trace_busy=True; self.trace_points={}; self.trace_shot=-1; self.trace_total=0
        self.trace_offset=0; self.trace_retries=0
        self._request_trace(0); self._buttons()

    def _request_trace(self,offset):
        self.trace_offset=offset; self.trace_deadline=time.monotonic()+4
        self.worker.send(f'TRACE {offset}')

    def _next_trace_chunk(self):
        missing=next((i for i in range(self.trace_total) if i not in self.trace_points),None)
        if missing is not None:
            if missing<=self.trace_offset:
                self.trace_retries+=1
                if self.trace_retries>3:
                    self.trace_busy=False; self._error('轨迹分包缺失，请重新读取'); return
            else: self.trace_retries=0
            self._request_trace(missing); return
        self.trace_busy=False
        points=sorted(self.trace_points.values(),key=lambda p:p.index)
        if not points: self.result_label.setText('设备尚无叩击轨迹'); self._buttons(); return
        if any(b.time_us<=a.time_us for a,b in zip(points,points[1:])):
            self._error('轨迹时间不连续，未计算精度'); return
        contact=self.shot_event.get('reason')=='SPEED_DROP' and int(self.shot_event.get('shot','-1'))==self.trace_shot
        self.metrics=analyze_trace(points,self.trace_target,contact=contact,window_us=self.trace_window)
        self._plot_trace(points)
        m=self.metrics
        reason=self.shot_event.get('reason','未知（只读出缓存轨迹）')
        steady=m.get('steady_mean')
        self.result_label.setText(
            f"结束：{reason} | 目标 {self.trace_target:g}°/s | "
            f"{'接触前稳速达标' if m['qualified'] and contact else '未检测到接触，速度统计仅供参考' if not contact else '未满足 ±10% / 10ms'}\n"
            f"末段稳速 {m['stable_ms']:.1f}ms | 超调 {m.get('overshoot_pct',0):.1f}% | "
            f"稳速均值 {f'{steady:.1f}' if steady is not None else '--'}°/s | "
            f"轨迹 {len(points)} 点；接触轨迹回溯速降起点后统计，起点为编码器估计")
        self._append(f'完整轨迹已读取：shot={self.trace_shot}，{len(points)} 点')
        self._buttons()

    def _plot_live(self):
        if not self.live: return
        rows=list(self.live); origin=rows[0][0]
        self._plot([r[0]-origin for r in rows],[r[1] for r in rows],[r[2] for r in rows],
                   [r[3] for r in rows],[r[4] for r in rows],40 if self.homing else self.confirmed_settings.get('strike_speed',self.speed.value()))

    def _plot_trace(self,points):
        self._plot([p.time_us/1000 for p in points],[p.angle for p in points],[p.speed for p in points],
                   [p.raw for p in points],[p.pwm for p in points],self.trace_target)

    def _plot(self,x,angle,speed,raw,pwm,target):
        self.curve_target.setData(x,[target]*len(x)); self.curve_speed.setData(x,speed)
        self.curve_raw.setData(x,raw); self.curve_angle.setData(x,angle); self.curve_pwm.setData(x,pwm)

    def export_trace(self):
        if not self.trace_points or self.trace_busy or len(self.trace_points)!=self.trace_total: return
        name,_=QFileDialog.getSaveFileName(self,'保存轨迹',f'hammer_{dt.datetime.now():%Y%m%d_%H%M%S}.csv','CSV (*.csv)')
        if not name: return
        try:
            path=Path(name).with_suffix('.csv'); points=sorted(self.trace_points.values(),key=lambda p:p.index)
            with path.open('w',newline='',encoding='utf-8-sig') as f:
                writer=csv.DictWriter(f,fieldnames=(*points[0].to_dict(),'target'))
                writer.writeheader()
                for p in points: writer.writerow({**p.to_dict(),'target':self.trace_target})
            snapshot={'shot':self.trace_shot,'target':self.trace_target,'event':self.shot_event,
                'parameters':self.trace_settings,'metrics':self.metrics,'window_us':self.trace_window,
                'exported_at':dt.datetime.now().isoformat()}
            path.with_suffix('.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+'\n')
            self._append('已导出：'+str(path))
        except OSError as exc: self._error(str(exc))

    def _tick(self):
        now=time.monotonic()
        if self.jog_speed and self.connected and now-self.last_jog_send>=.1:
            self.last_jog_send=now; self.worker.send(f'JOG {self.jog_speed:g}')
        if self.pending and now>self.pending[3]:
            command,prefix,callback,_=self.pending
            if command in ('INFO','STATUS','PARAMS','TUNING','PING') and self.request_attempts<2:
                self.request_attempts+=1; self.pending=(command,prefix,callback,now+2)
                self.worker.send(command); self._append('重试只读查询：'+command)
            else:
                self.pending=None
                self._error(f'命令未收到确认：{command}；未自动重发动作，请检查通信日志')
        if self.trace_busy and now>self.trace_deadline:
            self.trace_retries+=1
            if self.trace_retries>3:
                self.trace_busy=False; self._error('轨迹读取超时，可手动重新读取')
            else:self._request_trace(self.trace_offset)
        interval=.2 if self.mode.currentText()=='USB' else .6
        if self.status_inflight and now-self.last_status_request>2:self.status_inflight=False
        if self.connected and self.ready and self.pending is None and not self.trace_busy and not self.status_inflight and now-self.last_status_request>interval:
            self.last_status_request=now; self.status_inflight=True
            # Telemetry is independent of action transactions: never locks buttons.
            self.worker.send('STATUS')
        self._buttons()

    def _append(self,text):
        self.log.appendPlainText(f'[{dt.datetime.now():%H:%M:%S}] {text}')

    def _error(self,text):
        self.connecting=False
        if self.connected and not self.ready:self.connection_error=text
        self._append('错误：'+text)
        self.connection_label.setText(text); self._buttons()

    def changeEvent(self,event):
        if event.type()==event.Type.ActivationChange and not self.isActiveWindow() and self.jog_speed:
            self.stop()
        super().changeEvent(event)

    def closeEvent(self,event):
        self.timer.stop(); self.port_timer.stop(); self.worker.close(); event.accept()


def main():
    parser=argparse.ArgumentParser(description='旋转叩击电机 USB / BLE 控制器')
    parser.add_argument('--device',default=DEFAULT_DEVICE,help='BLE 名称或地址')
    parser.add_argument('--port',help='USB 串口路径')
    parser.add_argument('--mode',choices=('USB','BLE'),default='USB')
    args=parser.parse_args()
    app=QApplication(sys.argv); app.setStyle('Fusion')
    window=MainWindow(); window.mode.setCurrentText(args.mode)
    window.ble_device.setText(args.device)
    if args.port:
        if window.ports.findText(args.port)<0:window.ports.addItem(args.port)
        window.ports.setCurrentText(args.port)
    window.show()
    return app.exec()


if __name__=='__main__':
    raise SystemExit(main())
