#!/usr/bin/env python3
"""Python GUI for EMG / IMU CSV conversion and correlation; run with pixi run csv-mat."""
from __future__ import annotations

import csv
import os
from pathlib import Path
import sys
import signal
import tempfile
import traceback

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "emg-imu-csv-mat-mpl"))
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets as W
from matplotlib import font_manager, rcParams
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from csv_mat_core import active_channels, analyze, capture_payload, load_capture, save_unique_mat

_font_names = {font.name for font in font_manager.fontManager.ttflist}
_cjk_font = next((name for name in ("PingFang SC", "Heiti TC", "Microsoft YaHei", "SimHei", "Arial Unicode MS", "Noto Sans CJK SC")
                  if name in _font_names), None)
if _cjk_font:
    rcParams["font.sans-serif"] = [_cjk_font, "DejaVu Sans"]
rcParams["axes.unicode_minus"] = False


class Signals(QtCore.QObject):
    done = QtCore.Signal(object)
    error = QtCore.Signal(str)


class Job(QtCore.QRunnable):
    def __init__(self, function):
        super().__init__()
        self.function = function
        self.signals = Signals()

    def run(self):
        try:
            self.signals.done.emit(self.function())
        except Exception:
            self.signals.error.emit(traceback.format_exc())


class DropList(W.QListWidget):
    files_dropped = QtCore.Signal(list)

    def __init__(self):
        super().__init__()
        self.setAcceptDrops(True)
        self.setDragDropMode(W.QAbstractItemView.DropOnly)
        self.setMinimumWidth(260)
        self.setToolTip("把一个或多个 CSV 文件拖到这里；点击文件加载详情。")

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls() and any(u.isLocalFile() and u.toLocalFile().lower().endswith(".csv") for u in event.mimeData().urls()):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event):
        self.dragEnterEvent(event)

    def dropEvent(self, event):
        paths = [u.toLocalFile() for u in event.mimeData().urls() if u.isLocalFile() and u.toLocalFile().lower().endswith(".csv")]
        self.files_dropped.emit(paths)
        event.acceptProposedAction()


def draw_heatmap(figure, result):
    figure.clear()
    ax = figure.add_subplot(111)
    matrix = result["correlation"]
    names = list(result["channel_names"])
    im = ax.imshow(matrix, vmin=-1, vmax=1, cmap="RdBu_r")
    figure.colorbar(im, ax=ax, label="Pearson r", shrink=.85)
    ax.set_xticks(range(len(names)), names, rotation=90, fontsize=8)
    ax.set_yticks(range(len(names)), names, fontsize=8)
    if len(names) <= 12:
        for i in range(len(names)):
            for j in range(len(names)):
                ax.text(j, i, f"{matrix[i,j]:.2f}", ha="center", va="center", fontsize=8,
                        color="white" if abs(matrix[i, j]) > .65 else "black")
    figure.suptitle(Path(result["source_file"]).name, fontsize=10)
    ax.set_title(f"Pearson correlation | n={result['sample_count']:,} | "
                 f"{result['time_s'][0]:.3f}-{result['time_s'][-1]:.3f} s\n"
                 f"mean r={result['mean_r']:.4f}, mean |r|={result['mean_abs_r']:.4f}")
    figure.tight_layout()


class MainWindow(W.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EMG / IMU · CSV 转 MAT 与通道相关性")
        self.resize(1250, 850)
        self.capture = None
        self.result = None
        self.busy = False
        self.job = None
        self.pool = QtCore.QThreadPool(self)
        self.pool.setMaxThreadCount(1)
        self.output_directory = None
        root = W.QWidget()
        self.setCentralWidget(root)
        layout = W.QVBoxLayout(root)
        title = W.QLabel("EMG / IMU 数据工作台")
        title.setStyleSheet("font-size:24px;font-weight:600;color:#183b56;padding:8px 0")
        layout.addWidget(title)
        layout.addWidget(W.QLabel("拖入 CSV → 查看数据 → 转为 MAT；EMG 可选择通道和时间范围分析相关性"))
        splitter = W.QSplitter()
        layout.addWidget(splitter, 1)
        left = W.QWidget()
        ll = W.QVBoxLayout(left)
        ll.setContentsMargins(0, 8, 8, 0)
        self.add_button = W.QPushButton("选择 CSV 文件…")
        self.add_button.clicked.connect(self.choose_files)
        ll.addWidget(self.add_button)
        ll.addWidget(W.QLabel("文件列表（也可拖放多个 CSV 到下方）"))
        self.files = DropList()
        self.files.files_dropped.connect(self.add_files)
        self.files.currentItemChanged.connect(self.select_file)
        ll.addWidget(self.files, 1)
        self.clear_button = W.QPushButton("清空列表")
        self.clear_button.clicked.connect(self.clear_files)
        ll.addWidget(self.clear_button)
        self.out_button = W.QPushButton("选择输出目录…")
        self.out_button.clicked.connect(self.choose_output)
        ll.addWidget(self.out_button)
        self.out_label = W.QLabel("默认：每个 CSV 旁的 csv_mat_exports 文件夹")
        self.out_label.setWordWrap(True)
        ll.addWidget(self.out_label)
        self.convert_button = W.QPushButton("转换当前文件 → MAT")
        self.convert_button.clicked.connect(self.convert_current)
        self.batch_button = W.QPushButton("批量转换列表中的全部文件")
        self.batch_button.clicked.connect(self.convert_all)
        ll.addWidget(self.convert_button)
        ll.addWidget(self.batch_button)
        self.open_button = W.QPushButton("打开输出目录")
        self.open_button.clicked.connect(self.open_output)
        ll.addWidget(self.open_button)
        splitter.addWidget(left)
        right = W.QWidget()
        rl = W.QVBoxLayout(right)
        rl.setContentsMargins(8, 8, 0, 0)
        self.info = W.QLabel("尚未加载文件。支持 emg48_capture、imu_capture、accel CSV。")
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        rl.addWidget(self.info)
        self.tabs = W.QTabWidget()
        rl.addWidget(self.tabs, 1)
        preview = W.QWidget()
        pl = W.QVBoxLayout(preview)
        self.preview = W.QTableWidget()
        self.preview.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        pl.addWidget(self.preview)
        pl.addWidget(W.QLabel("预览前 30 行；转换和分析使用完整数据。所有原始列均保留。"))
        self.tabs.addTab(preview, "数据预览")
        analysis = W.QWidget()
        al = W.QVBoxLayout(analysis)
        self.settings = W.QWidget()
        sl = W.QVBoxLayout(self.settings)
        sl.setContentsMargins(0, 0, 0, 0)
        self.channels = W.QListWidget()
        self.channels.setFlow(W.QListView.LeftToRight)
        self.channels.setWrapping(True)
        self.channels.setResizeMode(W.QListView.Adjust)
        self.channels.setMaximumHeight(100)
        self.channels.itemChanged.connect(self.invalidate)
        sl.addWidget(W.QLabel("参与分析的通道（默认勾选有变化的通道，可取消饱和或不需要的通道）"))
        sl.addWidget(self.channels)
        row = W.QHBoxLayout()
        for text, callback in (("仅勾选有变化的通道", self.check_active), ("全选", lambda: self.check_all(True)), ("全不选", lambda: self.check_all(False))):
            button = W.QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        row.addStretch()
        sl.addLayout(row)
        row = W.QHBoxLayout()
        self.start = self.number(0, 1e9, 0, 4)
        self.end = self.number(0, 1e9, 0, 4)
        self.end.setSpecialValueText("记录末尾")
        self.fs = self.number(0, 1e7, 0, 3)
        self.fs.setSpecialValueText("自动检测")
        for label, widget in (("起点 (s)", self.start), ("终点 (s)", self.end), ("采样率 (Hz)", self.fs)):
            row.addWidget(W.QLabel(label))
            row.addWidget(widget)
            widget.valueChanged.connect(self.invalidate)
        sl.addLayout(row)
        row = W.QHBoxLayout()
        self.band = W.QCheckBox("带通 (Hz)")
        self.low = self.number(.01, 1e6, 20, 2)
        self.high = self.number(.01, 1e6, 450, 2)
        self.notch = W.QComboBox()
        for label, value in (("不陷波", "none"), ("仅工频", "fundamental"), ("工频及谐波", "harmonics")):
            self.notch.addItem(label, value)
        self.line = W.QComboBox()
        self.line.addItems(["50 Hz", "60 Hz"])
        for w in (self.band, self.low, W.QLabel("至"), self.high, self.notch, self.line):
            row.addWidget(w)
        self.band.toggled.connect(self.invalidate)
        self.low.valueChanged.connect(self.invalidate)
        self.high.valueChanged.connect(self.invalidate)
        self.notch.currentIndexChanged.connect(self.invalidate)
        self.line.currentIndexChanged.connect(self.invalidate)
        sl.addLayout(row)
        al.addWidget(self.settings)
        row = W.QHBoxLayout()
        self.analyze_button = W.QPushButton("计算通道相关性")
        self.analyze_button.clicked.connect(self.run_analysis)
        self.export_button = W.QPushButton("导出分析 MAT + CSV + PNG")
        self.export_button.clicked.connect(self.export_analysis)
        self.enlarge_button = W.QPushButton("放大热力图")
        self.enlarge_button.clicked.connect(self.enlarge_heatmap)
        row.addWidget(self.analyze_button)
        row.addWidget(self.enlarge_button)
        row.addWidget(self.export_button)
        al.addLayout(row)
        self.stats = W.QLabel("Pearson r 保留正负号。默认分析原始值；选择滤波后先处理整段，再截取时间区间 [起点, 终点)。")
        self.stats.setWordWrap(True)
        al.addWidget(self.stats)
        self.figure = Figure(figsize=(7, 5))
        self.canvas = FigureCanvasQTAgg(self.figure)
        al.addWidget(NavigationToolbar2QT(self.canvas, self))
        al.addWidget(self.canvas, 1)
        self.tabs.addTab(analysis, "EMG 通道相关性")
        splitter.addWidget(right)
        splitter.setSizes([300, 950])
        self.log = W.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(100)
        self.log.setPlaceholderText("处理结果与数据质量提示")
        layout.addWidget(self.log)
        self.progress = W.QProgressBar()
        self.progress.setMaximumHeight(7)
        self.progress.hide()
        layout.addWidget(self.progress)
        self.statusBar().showMessage("就绪")
        self.update_buttons()

    @staticmethod
    def number(low, high, value, decimals):
        widget = W.QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(decimals)
        widget.setValue(value)
        return widget

    def update_buttons(self):
        self.files.setEnabled(not self.busy)
        for button in (self.add_button, self.clear_button, self.out_button):
            button.setEnabled(not self.busy)
        self.convert_button.setEnabled(not self.busy and self.capture is not None)
        self.batch_button.setEnabled(not self.busy and self.files.count() > 0)
        emg = self.capture is not None and self.capture.kind == "EMG"
        self.settings.setEnabled(not self.busy and emg)
        self.analyze_button.setEnabled(not self.busy and emg)
        self.export_button.setEnabled(not self.busy and self.result is not None)
        self.enlarge_button.setEnabled(not self.busy and self.result is not None)

    def work(self, function, callback, message):
        self.busy = True
        self.update_buttons()
        self.progress.setRange(0, 0)
        self.progress.show()
        self.statusBar().showMessage(message)
        self.job = Job(function)
        def done(result):
            self.finish_work()
            try:
                callback(result)
            except Exception:
                self.show_error(traceback.format_exc())
        self.job.signals.done.connect(done)
        self.job.signals.error.connect(self.show_error)
        self.pool.start(self.job)

    def finish_work(self):
        self.busy = False
        self.progress.hide()
        self.statusBar().showMessage("就绪")
        self.update_buttons()

    def show_error(self, message):
        self.finish_work()
        self.log.appendPlainText(message)
        box = W.QMessageBox(self)
        box.setWindowTitle("处理失败")
        box.setText(message.strip().splitlines()[-1])
        box.setDetailedText(message)
        box.setIcon(W.QMessageBox.Warning)
        box.exec()

    def choose_files(self):
        paths, _ = W.QFileDialog.getOpenFileNames(self, "选择采集 CSV", "", "CSV 文件 (*.csv *.CSV)")
        self.add_files(paths)

    def add_files(self, paths):
        if self.busy:
            return
        known = {self.files.item(i).data(QtCore.Qt.UserRole) for i in range(self.files.count())}
        self.files.blockSignals(True)
        for raw in paths:
            path = Path(raw).resolve()
            if path.is_file() and path.suffix.lower() == ".csv" and str(path) not in known:
                item = W.QListWidgetItem(path.name)
                item.setData(QtCore.Qt.UserRole, str(path))
                item.setToolTip(str(path))
                self.files.addItem(item)
                known.add(str(path))
        self.files.blockSignals(False)
        if self.files.currentItem() is None and self.files.count():
            self.files.setCurrentRow(0)
        self.update_buttons()

    def clear_files(self):
        self.files.clear()
        self.capture = None
        self.preview.clear()
        self.preview.setRowCount(0)
        self.preview.setColumnCount(0)
        self.channels.clear()
        self.info.setText("尚未加载文件。")
        self.invalidate()

    def select_file(self, item, previous=None):
        if item is None:
            return
        self.capture = None
        self.invalidate()
        path = item.data(QtCore.Qt.UserRole)
        self.info.setText(f"正在加载：{path}")
        self.work(lambda: load_capture(path), self.loaded, "正在读取 CSV…")

    def loaded(self, capture):
        self.capture = capture
        self.start.setValue(0)
        self.end.setValue(0)
        self.fs.setValue(0)
        self.channels.clear()
        selected = set(active_channels(capture))
        for name in capture.channels:
            item = W.QListWidgetItem(name)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked if name in selected else QtCore.Qt.Unchecked)
            self.channels.addItem(item)
        stream_info = "；".join(f"{s['label']}: {len(s['row_indices']):,} 点 / {s['fs_hz']:.3f} Hz" for s in capture.streams)
        self.info.setText(f"{capture.path.name}\n{capture.kind} · {capture.size:,} 行 · {len(capture.columns)} 列\n{stream_info}")
        self.preview.clear()
        self.preview.setColumnCount(len(capture.columns))
        self.preview.setHorizontalHeaderLabels(list(capture.columns))
        self.preview.setRowCount(min(30, capture.size))
        for col, values in enumerate(capture.columns.values()):
            for row, value in enumerate(values[:30]):
                self.preview.setItem(row, col, W.QTableWidgetItem(str(value)))
        self.preview.resizeColumnsToContents()
        for warning in capture.warnings:
            self.log.appendPlainText("提示：" + warning)
        self.log.appendPlainText(f"已加载 {capture.path}")
        self.tabs.setCurrentIndex(1 if capture.kind == "EMG" else 0)
        self.update_buttons()

    def invalidate(self, *args):
        self.result = None
        self.figure.clear()
        self.canvas.draw_idle()
        self.stats.setText("设置已更新，点击“计算通道相关性”。" if self.capture else "加载 EMG 后可分析通道相关性。")
        self.update_buttons()

    def check_all(self, checked):
        for i in range(self.channels.count()):
            self.channels.item(i).setCheckState(QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)

    def check_active(self):
        active = set(active_channels(self.capture)) if self.capture else set()
        for i in range(self.channels.count()):
            item = self.channels.item(i)
            item.setCheckState(QtCore.Qt.Checked if item.text() in active else QtCore.Qt.Unchecked)

    def choose_output(self):
        path = W.QFileDialog.getExistingDirectory(self, "选择输出目录")
        if path:
            self.output_directory = Path(path)
            self.out_label.setText(str(self.output_directory))

    def output_for(self, path):
        return self.output_directory or Path(path).parent / "csv_mat_exports"

    def open_output(self):
        if self.output_directory or self.capture:
            path = self.output_directory or self.output_for(self.capture.path)
            path.mkdir(parents=True, exist_ok=True)
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def convert_current(self):
        capture = self.capture
        output = self.output_for(capture.path)
        self.work(lambda: save_unique_mat(output, capture.path.stem, capture_payload(capture)),
                  lambda path: self.log.appendPlainText(f"MAT 已保存：{path}"), "正在转换 MAT…")

    def convert_all(self):
        paths = [Path(self.files.item(i).data(QtCore.Qt.UserRole)) for i in range(self.files.count())]
        destinations = [self.output_for(path) for path in paths]
        def convert():
            reports = []
            for path, output in zip(paths, destinations):
                try:
                    capture = load_capture(path)
                    result = save_unique_mat(output, path.stem, capture_payload(capture))
                    reports.append(f"成功：{result}")
                except Exception as exc:
                    reports.append(f"失败：{path}：{exc}")
            return reports
        self.work(convert, lambda reports: self.log.appendPlainText("\n".join(reports)), "批量转换中…")

    def run_analysis(self):
        channels = [self.channels.item(i).text() for i in range(self.channels.count())
                    if self.channels.item(i).checkState() == QtCore.Qt.Checked]
        capture = self.capture
        args = dict(channels=channels, start=self.start.value(), end=self.end.value() or None,
                    fs_override=self.fs.value() or None, bandpass=self.band.isChecked(),
                    low=self.low.value(), high=self.high.value(), notch=self.notch.currentData(),
                    line_hz=50. if self.line.currentIndex() == 0 else 60.)
        self.result = None
        self.work(lambda: analyze(capture, **args), self.analyzed, "正在计算通道相关性…")

    def analyzed(self, result):
        self.result = result
        self.stats.setText(f"{len(result['channel_names'])} 通道，{result['sample_count']:,} 点；"
                           f"平均 r = {result['mean_r']:.6f}；平均 |r| = {result['mean_abs_r']:.6f}；"
                           f"范围 [{result['min_r']:.6f}, {result['max_r']:.6f}]。")
        draw_heatmap(self.figure, result)
        self.canvas.draw_idle()
        if len(result['excluded_constant_channels']):
            self.log.appendPlainText("排除区间内恒定通道：" + ", ".join(result['excluded_constant_channels']))
        for warning in result["warnings"]:
            self.log.appendPlainText("提示：" + str(warning))
        self.update_buttons()

    def export_analysis(self):
        # Rendering stays on the GUI thread; numerical work and MAT writes use workers.
        output = self.output_for(self.capture.path)
        output.mkdir(parents=True, exist_ok=True)
        stem = self.capture.path.stem + "_correlation"
        serial = 0
        while True:
            folder = output / f"{stem}{'_' + str(serial) if serial else ''}"
            try:
                folder.mkdir()
                break
            except FileExistsError:
                serial += 1
        try:
            export_figure = Figure(figsize=(max(8, len(self.result["channel_names"]) * .22),
                                             max(7, len(self.result["channel_names"]) * .22)))
            draw_heatmap(export_figure, self.result)
            export_figure.savefig(folder / "correlation.png", dpi=180)
            with (folder / "correlation.csv").open("x", newline="", encoding="utf-8-sig") as fp:
                writer = csv.writer(fp)
                writer.writerow(["channel", *self.result["channel_names"]])
                for name, row in zip(self.result["channel_names"], self.result["correlation"]):
                    writer.writerow([name, *row])
            result = self.result
            self.work(lambda: save_unique_mat(folder, "analysis", result),
                      lambda path: self.log.appendPlainText(f"分析已导出：{folder}"), "正在保存分析…")
        except Exception:
            self.show_error(traceback.format_exc())

    def enlarge_heatmap(self):
        dialog = W.QDialog(self)
        dialog.setWindowTitle("EMG 通道相关性热力图")
        dialog.resize(950, 850)
        layout = W.QVBoxLayout(dialog)
        canvas = FigureCanvasQTAgg(Figure(figsize=(9, 8)))
        layout.addWidget(NavigationToolbar2QT(canvas, dialog))
        layout.addWidget(canvas, 1)
        draw_heatmap(canvas.figure, self.result)
        dialog.exec()

    def closeEvent(self, event):
        if self.busy:
            self.statusBar().showMessage("正在处理数据，请等待处理结束后关闭。")
            event.ignore()
        else:
            event.accept()


def main():
    app = W.QApplication(sys.argv)
    app.setApplicationName("EMG-IMU CSV MAT")
    app.setOrganizationName("HarukaLab")
    app.setStyle("Fusion")
    app.setStyleSheet("QPushButton {padding:7px 10px;} QGroupBox {font-weight:600;} QListWidget, QTableWidget {background:#fff;} QMainWindow {background:#f4f7fa;}")
    signal.signal(signal.SIGINT, lambda *_: app.closeAllWindows())
    signal_timer = QtCore.QTimer(app)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(250)
    window = MainWindow()
    window.show()
    paths = [p for p in sys.argv[1:] if p.lower().endswith(".csv")]
    if paths:
        QtCore.QTimer.singleShot(0, lambda: window.add_files(paths))
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
