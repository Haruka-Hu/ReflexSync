#!/usr/bin/env python3
"""Wireless capture v3 host scope: 48CH EMG, IMU and MVC feedback."""

from __future__ import annotations

import argparse
import binascii
import csv
import json
import os
import queue
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
import scipy.signal as signal
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStyle,
    QTabWidget,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

# Resolve storage from this checkout, independently of the terminal's cwd.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV_DIR = PROJECT_ROOT / "data"
DEFAULT_WAVEFORM_DIR = PROJECT_ROOT / "outputs" / "waveforms"


def resolve_recording_dir(directory: Path) -> Path:
    directory = Path(directory).expanduser()
    if not directory.is_absolute():
        directory = PROJECT_ROOT / directory
    return directory.resolve()


FRAME_HEADER = b"\xAA\x55"
PC_MSG_SAMPLE_BATCH = 0x30
PC_MSG_SYNC_DIAG = 0x31
PC_MSG_STATE_EVENT = 0x32
PC_MSG_EMG48_BATCH = 0x33
PC_MSG_IMU_WIRE_BATCH = 0x34
PC_HEADER_SIZE = 5
PC_EMG48_HEADER_SIZE = 6
PC_SAMPLE_SIZE = 24
PC_SAMPLE_STRUCT = struct.Struct("<BIIHBiii")
PC_MAX_BATCH_SAMPLES = 16
PC_SYNC_DIAG_SIZE = 24
PC_SYNC_DIAG_STRUCT = struct.Struct("<BBHiiiH")
PC_STATE_EVENT_SIZE = 14
PC_STATE_EVENT_STRUCT = struct.Struct("<BBHI")
EMG48_CHANNEL_COUNT = 48
EMG48_CHANNEL_WIRE_SIZE = 3
EMG48_BATCH_META_STRUCT = struct.Struct("<IIIBB")
EMG48_BATCH_META_SIZE = EMG48_BATCH_META_STRUCT.size
EMG48_SAMPLE_SIZE = EMG48_CHANNEL_COUNT * EMG48_CHANNEL_WIRE_SIZE
EMG48_SAMPLE_PERIOD_US = 500
EMG48_MAX_BATCH_SAMPLES = 4
IMU_BATCH_TYPE = 0x20
IMU_BATCH_SIZE = 20
IMU_BATCH_HEADER_SIZE = 14
IMU_SAMPLE_SIZE = 8
IMU_BATCH_WIRE_SIZE = IMU_BATCH_HEADER_SIZE + IMU_BATCH_SIZE * IMU_SAMPLE_SIZE + 2
PC_IMU_WIRE_HEADER_SIZE = 4
PC_IMU_WIRE_PACKET_SIZE = PC_IMU_WIRE_HEADER_SIZE + IMU_BATCH_WIRE_SIZE + 2

SOURCE_EMG = 0x00
SOURCE_GYRO = 0x01
SOURCE_ACCEL = 0x02
SOURCE_DIAG = 0x7E
SOURCE_NAMES = {
    SOURCE_EMG: "EMG48",
    SOURCE_GYRO: "Gyro",
    SOURCE_ACCEL: "Accel",
    SOURCE_DIAG: "Diag",
}
STATE_NAMES = {
    0: "IDLE",
    1: "SYNC",
    2: "STREAM_PENDING",
    3: "STREAM",
}

UINT32_WRAP = 2**32
INT32_HALF_WRAP = 2**31
MIN_FRAME_SIZE = 7
TIMESTAMP_RELOCK_GAP_US = 250_000
TIMESTAMP_RELOCK_BACKWARD_US = 2_000
SYNC_STABLE_WINDOW_S = 3.0
SYNC_MIN_BEACON_COUNT = 30
SYNC_USABLE_RESIDUAL_US = 3_000
SYNC_STABLE_RESIDUAL_US = 1_000
SYNC_EXCELLENT_RESIDUAL_US = 500
SYNC_STABLE_DRIFT_RANGE_PPM = 100
SYNC_EXCELLENT_DRIFT_RANGE_PPM = 50

# ADS1298 conversion constants. The PGA gain is supplied by --ads-gain.
ADS_REFERENCE_V = 2.4
ADS_FULL_SCALE_COUNTS = 2**23 - 1
EMG_MIN_RAW_RANGE = 1000.0
EMG_MIN_HALF_RANGES_UV = (100.0, 250.0, 500.0, 1000.0)
EMG_DEFAULT_MIN_HALF_RANGE_UV = 250.0
SERIAL_READ_SIZE = 65536
RAW_QUEUE_CHUNKS = 256
RECORD_QUEUE_ROWS = 50000
MAX_RESEND_REQUESTS_PER_GAP = 8
MAX_PENDING_RESEND_SEQUENCES = 512
MAX_RESEND_COMMANDS_PER_SECOND = 20
MVC_GROUP_SIZE = 16
MVC_HISTORY_SECONDS = 10.0
MVC_BASELINE_SECONDS = 5.0
MVC_COUNTDOWN_SECONDS = 3.0
MVC_TRIAL_SECONDS = 5.0
MVC_REST_SECONDS = 30.0
MVC_TRIAL_COUNT = 3
MVC_TRIAL_SCORE_SECONDS = 0.5


def emg_scale_factor_uv(ads_gain: float) -> float:
    if ads_gain <= 0:
        raise ValueError("ADS gain must be positive")
    return ADS_REFERENCE_V * 1_000_000.0 / (float(ads_gain) * ADS_FULL_SCALE_COUNTS)


def calculate_emg_y_range(
    values: np.ndarray,
    minimum_half_range: float,
) -> tuple[float, float] | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    p05 = float(np.percentile(finite, 5))
    p95 = float(np.percentile(finite, 95))
    center = (p05 + p95) * 0.5
    half_range = max((p95 - p05) * 0.65, float(minimum_half_range))
    return center - half_range, center + half_range


class EmgLiveFilter:
    """Continuous 2 kHz EMG filtering followed by display decimation."""

    def __init__(
        self,
        sample_rate: float,
        decimation: int,
        window_seconds: float,
        ads_gain: float,
    ) -> None:
        self.sample_rate = float(sample_rate)
        self.decimation = max(1, min(2, int(decimation)))
        self.scale_factor_uv = emg_scale_factor_uv(ads_gain)
        notch_b, notch_a = signal.iirnotch(50.0, 30.0, self.sample_rate)
        notch_sos = signal.tf2sos(notch_b, notch_a)
        bandpass_sos = signal.butter(
            4,
            [20.0, 450.0],
            btype="bandpass",
            fs=self.sample_rate,
            output="sos",
        )
        self.sos = np.vstack((notch_sos, bandpass_sos))
        self.steady_zi = signal.sosfilt_zi(self.sos)
        self.zi: np.ndarray | None = None
        self.last_sample_seq: int | None = None
        self.last_timestamp_us: int | None = None
        max_points = max(
            100,
            int(window_seconds * self.sample_rate / self.decimation * 1.3),
        )
        self.timestamps_us: deque[int] = deque(maxlen=max_points)
        self.channels: list[deque[float]] = [
            deque(maxlen=max_points) for _ in range(EMG48_CHANNEL_COUNT)
        ]

    def reset(self, clear_output: bool = True) -> None:
        self.zi = None
        self.last_sample_seq = None
        self.last_timestamp_us = None
        if clear_output:
            self.timestamps_us.clear()
            for channel in self.channels:
                channel.clear()

    def _reset_state_for(self, first_values: np.ndarray) -> None:
        self.zi = self.steady_zi[:, :, None] * first_values[None, None, :]

    def _filter_contiguous(
        self,
        timestamps_us: np.ndarray,
        sample_seq: np.ndarray,
        values_uv: np.ndarray,
    ) -> np.ndarray:
        if values_uv.size == 0:
            return np.empty((0, EMG48_CHANNEL_COUNT), dtype=np.float64)
        if self.zi is None:
            self._reset_state_for(values_uv[0])
        filtered, self.zi = signal.sosfilt(
            self.sos,
            values_uv,
            axis=0,
            zi=self.zi,
        )
        keep = (sample_seq % self.decimation) == 0
        for timestamp_us, row in zip(timestamps_us[keep], filtered[keep]):
            self.timestamps_us.append(int(timestamp_us))
            for channel_index, value in enumerate(row):
                self.channels[channel_index].append(float(value))
        return filtered

    def consume(
        self,
        timestamps_us: np.ndarray,
        sample_seq: np.ndarray,
        raw_channels: list[np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        empty_timestamps = np.array([], dtype=np.int64)
        empty_sequences = np.array([], dtype=np.uint64)
        empty_values = np.empty((0, EMG48_CHANNEL_COUNT), dtype=np.float64)
        if timestamps_us.size == 0 or sample_seq.size == 0:
            return empty_timestamps, empty_sequences, empty_values
        if len(raw_channels) != EMG48_CHANNEL_COUNT:
            return empty_timestamps, empty_sequences, empty_values

        start = 0
        if self.last_sample_seq is not None:
            start = -1
            for index, sequence in enumerate(sample_seq):
                delta = (int(sequence) - self.last_sample_seq) & 0xFFFFFFFF
                if 0 < delta < INT32_HALF_WRAP:
                    start = index
                    break
            if start < 0:
                return empty_timestamps, empty_sequences, empty_values

        new_timestamps = timestamps_us[start:]
        new_sequences = sample_seq[start:]
        values_uv = np.column_stack(
            [channel[start:] for channel in raw_channels]
        ).astype(np.float64, copy=False)
        values_uv *= self.scale_factor_uv
        if values_uv.size == 0:
            return empty_timestamps, empty_sequences, empty_values

        boundaries = [0]
        if new_sequences.size > 1:
            sequence_steps = (
                new_sequences[1:].astype(np.uint64)
                - new_sequences[:-1].astype(np.uint64)
            ) & np.uint64(0xFFFFFFFF)
            timestamp_steps = (
                new_timestamps[1:].astype(np.int64)
                - new_timestamps[:-1].astype(np.int64)
                + INT32_HALF_WRAP
            ) % UINT32_WRAP - INT32_HALF_WRAP
            discontinuity = (
                (sequence_steps != 1)
                | (timestamp_steps <= 0)
                | (timestamp_steps > EMG48_SAMPLE_PERIOD_US * 4)
            )
            boundaries.extend((np.flatnonzero(discontinuity) + 1).tolist())
        boundaries.append(len(new_sequences))

        previous_sequence = self.last_sample_seq
        emitted_timestamps: list[np.ndarray] = []
        emitted_sequences: list[np.ndarray] = []
        emitted_values: list[np.ndarray] = []
        for segment_index, (left, right) in enumerate(zip(boundaries, boundaries[1:])):
            if left >= right:
                continue
            first_sequence = int(new_sequences[left])
            first_timestamp_us = int(new_timestamps[left])
            if previous_sequence is not None:
                forward = (first_sequence - previous_sequence) & 0xFFFFFFFF
                timestamp_forward = (
                    first_timestamp_us
                    - int(self.last_timestamp_us or first_timestamp_us)
                    + INT32_HALF_WRAP
                ) % UINT32_WRAP - INT32_HALF_WRAP
                if (
                    forward != 1
                    or timestamp_forward <= 0
                    or timestamp_forward > EMG48_SAMPLE_PERIOD_US * 4
                ):
                    self._reset_state_for(values_uv[left])
            elif segment_index > 0:
                self._reset_state_for(values_uv[left])
            filtered = self._filter_contiguous(
                new_timestamps[left:right],
                new_sequences[left:right],
                values_uv[left:right],
            )
            emitted_timestamps.append(new_timestamps[left:right])
            emitted_sequences.append(new_sequences[left:right])
            emitted_values.append(filtered)
            previous_sequence = int(new_sequences[right - 1])
            self.last_timestamp_us = int(new_timestamps[right - 1])

        self.last_sample_seq = int(new_sequences[-1])
        if not emitted_values:
            return empty_timestamps, empty_sequences, empty_values
        return (
            np.concatenate(emitted_timestamps),
            np.concatenate(emitted_sequences),
            np.vstack(emitted_values),
        )

    def snapshot(
        self,
        reference_timestamp_us: int | None,
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        empty = np.array([], dtype=np.float64)
        if not self.timestamps_us or reference_timestamp_us is None:
            return empty, [empty for _ in range(EMG48_CHANNEL_COUNT)]
        timestamps = np.fromiter(self.timestamps_us, dtype=np.int64)
        rel_time_s = (
            signed_delta_us(timestamps, reference_timestamp_us).astype(np.float64)
            / 1_000_000.0
        )
        channels = [
            np.fromiter(channel, dtype=np.float64) for channel in self.channels
        ]
        return rel_time_s, channels


@dataclass(frozen=True)
class MvcFeedbackPoint:
    host_time: float
    timestamp_us: int
    group_start: int
    metric_uv_rms: float
    rest_uv_rms: float
    mvc_uv_rms: float
    actual_percent: float
    target_percent: float
    tolerance_percent: float
    phase: str
    trial_number: int


class MvcCalibration:
    """Guided rest + three-trial EMG MVC calibration state machine."""

    def __init__(self, update_seconds: float) -> None:
        self.update_seconds = max(0.01, float(update_seconds))
        self.clear()

    def clear(self) -> None:
        self.phase = "idle"
        self.phase_started = 0.0
        self.baseline_values: list[float] = []
        self.trial_values: list[float] = []
        self.trial_scores: list[float] = []
        self.rest_uv_rms = float("nan")
        self.mvc_uv_rms = float("nan")
        self.trial_invalid = False
        self.message = "未标定"

    @property
    def valid(self) -> bool:
        return (
            self.phase == "complete"
            and np.isfinite(self.rest_uv_rms)
            and np.isfinite(self.mvc_uv_rms)
            and self.mvc_uv_rms > self.rest_uv_rms
        )

    @property
    def active(self) -> bool:
        return self.phase in {"baseline", "countdown", "trial", "rest", "invalid"}

    @property
    def trial_number(self) -> int:
        return min(MVC_TRIAL_COUNT, len(self.trial_scores) + 1)

    def start(self, now: float) -> None:
        self.clear()
        self.phase = "baseline"
        self.phase_started = now
        self.message = "保持放松，正在记录静息基线"

    def cancel(self) -> None:
        self.clear()
        self.phase = "cancelled"
        self.message = "标定已取消"

    def note_gap(self) -> None:
        if self.phase == "trial":
            self.trial_invalid = True

    def redo_current(self, now: float) -> None:
        if self.phase == "rest" and self.trial_scores:
            self.trial_scores.pop()
        elif self.phase == "invalid" and len(self.trial_scores) >= MVC_TRIAL_COUNT:
            self.trial_scores.pop()
        elif self.phase == "complete" and self.trial_scores:
            self.trial_scores.pop()
            self.mvc_uv_rms = float("nan")
        if self.phase not in {"idle", "baseline", "cancelled"}:
            self.phase = "countdown"
            self.phase_started = now
            self.trial_values = []
            self.trial_invalid = False
            self.message = f"第 {self.trial_number} 次将在倒计时后开始"

    def percent(self, metric_uv_rms: float) -> float:
        if not self.valid:
            return float("nan")
        denominator = self.mvc_uv_rms - self.rest_uv_rms
        return 100.0 * max(metric_uv_rms - self.rest_uv_rms, 0.0) / denominator

    def _finish_trial(self, now: float) -> None:
        minimum_points = max(1, int(round(4.0 / self.update_seconds)))
        score_points = max(1, int(round(MVC_TRIAL_SCORE_SECONDS / self.update_seconds)))
        if self.trial_invalid or len(self.trial_values) < minimum_points:
            self.phase = "invalid"
            self.phase_started = now
            self.message = f"第 {self.trial_number} 次数据不完整，请重做"
            return
        values = np.asarray(self.trial_values, dtype=np.float64)
        if values.size < score_points:
            self.phase = "invalid"
            self.message = f"第 {self.trial_number} 次有效数据不足，请重做"
            return
        kernel = np.ones(score_points, dtype=np.float64) / score_points
        stable_mean = np.convolve(values, kernel, mode="valid")
        self.trial_scores.append(float(np.max(stable_mean)))
        self.trial_values = []
        if len(self.trial_scores) >= MVC_TRIAL_COUNT:
            self.mvc_uv_rms = max(self.trial_scores)
            if not np.isfinite(self.rest_uv_rms) or self.mvc_uv_rms <= self.rest_uv_rms:
                self.phase = "invalid"
                self.message = "MVC 未明显高于静息基线，请重新标定"
                return
            self.phase = "complete"
            self.message = f"标定完成：MVC {self.mvc_uv_rms:.1f} µV RMS"
        else:
            self.phase = "rest"
            self.phase_started = now
            self.message = f"第 {len(self.trial_scores)} 次完成，请休息"

    def update(self, now: float, metric_uv_rms: float) -> None:
        if not np.isfinite(metric_uv_rms):
            return
        elapsed = max(0.0, now - self.phase_started)
        if self.phase == "baseline":
            self.baseline_values.append(float(metric_uv_rms))
            if elapsed >= MVC_BASELINE_SECONDS:
                self.rest_uv_rms = float(np.median(self.baseline_values))
                self.phase = "countdown"
                self.phase_started = now
                self.message = "第 1 次将在倒计时后开始"
        elif self.phase == "countdown":
            remaining = max(0, int(np.ceil(MVC_COUNTDOWN_SECONDS - elapsed)))
            self.message = f"第 {self.trial_number} 次：{remaining}"
            if elapsed >= MVC_COUNTDOWN_SECONDS:
                self.phase = "trial"
                self.phase_started = now
                self.trial_values = []
                self.trial_invalid = False
                self.message = f"第 {self.trial_number} 次：最大等长用力"
        elif self.phase == "trial":
            self.trial_values.append(float(metric_uv_rms))
            remaining = max(0.0, MVC_TRIAL_SECONDS - elapsed)
            self.message = f"第 {self.trial_number} 次最大用力，还剩 {remaining:.1f}s"
            if elapsed >= MVC_TRIAL_SECONDS:
                self._finish_trial(now)
        elif self.phase == "rest":
            remaining = max(0, int(np.ceil(MVC_REST_SECONDS - elapsed)))
            self.message = f"休息 {remaining}s，随后进行第 {self.trial_number} 次"
            if elapsed >= MVC_REST_SECONDS:
                self.phase = "countdown"
                self.phase_started = now
                self.message = f"第 {self.trial_number} 次将在倒计时后开始"

    def metadata(self) -> dict[str, object]:
        return {
            "valid": self.valid,
            "phase": self.phase,
            "rest_uv_rms": None if not np.isfinite(self.rest_uv_rms) else self.rest_uv_rms,
            "mvc_uv_rms": None if not np.isfinite(self.mvc_uv_rms) else self.mvc_uv_rms,
            "trial_scores_uv_rms": list(self.trial_scores),
            "baseline_seconds": MVC_BASELINE_SECONDS,
            "countdown_seconds": MVC_COUNTDOWN_SECONDS,
            "trial_seconds": MVC_TRIAL_SECONDS,
            "rest_seconds": MVC_REST_SECONDS,
            "trial_score_seconds": MVC_TRIAL_SCORE_SECONDS,
        }


class MvcTracker:
    """Rolling per-channel RMS and spatial median for one 16-channel FPC."""

    def __init__(
        self,
        sample_rate: float,
        rms_window_ms: float,
        update_ms: int,
        tolerance_percent: float,
    ) -> None:
        self.sample_rate = float(sample_rate)
        self.rms_window_ms = float(rms_window_ms)
        self.update_seconds = max(0.01, float(update_ms) / 1000.0)
        self.tolerance_percent = max(0.0, float(tolerance_percent))
        self.window_samples = max(1, int(round(self.sample_rate * self.rms_window_ms / 1000.0)))
        self.group_start = 0
        self.target_percent = 10.0
        self.calibration = MvcCalibration(self.update_seconds)
        self.history_maxlen = max(100, int(MVC_HISTORY_SECONDS / self.update_seconds * 1.5))
        self.history: deque[MvcFeedbackPoint] = deque(maxlen=self.history_maxlen)
        self._squares: deque[np.ndarray] = deque(maxlen=self.window_samples)
        self._square_sum = np.zeros(MVC_GROUP_SIZE, dtype=np.float64)
        self.last_sample_seq: int | None = None
        self.last_emit_time = 0.0
        self.last_point: MvcFeedbackPoint | None = None
        self.gap_count = 0

    def reset_signal(self, clear_history: bool = True) -> None:
        self._squares.clear()
        self._square_sum.fill(0.0)
        self.last_sample_seq = None
        self.last_emit_time = 0.0
        self.last_point = None
        if clear_history:
            self.history.clear()

    def set_group_start(self, group_start: int) -> None:
        if group_start not in (0, 16, 32):
            raise ValueError("MVC channel group must start at 0, 16, or 32")
        if group_start != self.group_start:
            self.group_start = group_start
            self.reset_signal(clear_history=True)
            self.calibration.clear()

    def set_target(self, target_percent: float) -> None:
        self.target_percent = min(100.0, max(1.0, float(target_percent)))

    def _reset_window_for_gap(self) -> None:
        self._squares.clear()
        self._square_sum.fill(0.0)
        self.gap_count += 1
        self.calibration.note_gap()

    def consume(
        self,
        timestamps_us: np.ndarray,
        sample_seq: np.ndarray,
        filtered_uv: np.ndarray,
        now: float,
    ) -> MvcFeedbackPoint | None:
        if filtered_uv.ndim != 2 or filtered_uv.shape[1] != EMG48_CHANNEL_COUNT:
            return None
        if len(filtered_uv) != len(sample_seq) or len(sample_seq) != len(timestamps_us):
            return None
        left = self.group_start
        right = left + MVC_GROUP_SIZE
        for timestamp, sequence, row in zip(timestamps_us, sample_seq, filtered_uv):
            sequence_value = int(sequence)
            if self.last_sample_seq is not None:
                step = (sequence_value - self.last_sample_seq) & 0xFFFFFFFF
                if step == 0 or step >= INT32_HALF_WRAP:
                    continue
                if step != 1:
                    self._reset_window_for_gap()
            self.last_sample_seq = sequence_value
            square = np.square(row[left:right].astype(np.float64, copy=False))
            if len(self._squares) == self.window_samples:
                self._square_sum -= self._squares.popleft()
            self._squares.append(square)
            self._square_sum += square

        if len(self._squares) < self.window_samples:
            return None
        if self.last_emit_time and now - self.last_emit_time < self.update_seconds * 0.8:
            return None
        self.last_emit_time = now
        channel_rms = np.sqrt(np.maximum(self._square_sum / len(self._squares), 0.0))
        metric = float(np.median(channel_rms))
        self.calibration.update(now, metric)
        actual = self.calibration.percent(metric)
        point = MvcFeedbackPoint(
            host_time=time.time(),
            timestamp_us=int(timestamps_us[-1]),
            group_start=self.group_start,
            metric_uv_rms=metric,
            rest_uv_rms=self.calibration.rest_uv_rms,
            mvc_uv_rms=self.calibration.mvc_uv_rms,
            actual_percent=actual,
            target_percent=self.target_percent,
            tolerance_percent=self.tolerance_percent,
            phase=self.calibration.phase,
            trial_number=self.calibration.trial_number,
        )
        self.history.append(point)
        self.last_point = point
        return point

    def history_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        finite_points = [point for point in self.history if np.isfinite(point.actual_percent)]
        if not finite_points:
            return np.array([], dtype=np.float64), np.array([], dtype=np.float64)
        latest = finite_points[-1].host_time
        return (
            np.asarray([point.host_time - latest for point in finite_points], dtype=np.float64),
            np.asarray([point.actual_percent for point in finite_points], dtype=np.float64),
        )


def signed_delta_us(timestamp_us: np.ndarray, reference_us: int) -> np.ndarray:
    raw_delta = (timestamp_us.astype(np.int64) - int(reference_us) + INT32_HALF_WRAP) % UINT32_WRAP
    return raw_delta - INT32_HALF_WRAP


def crc16_ccitt(data: bytes | bytearray) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def decode_i24_le(data: bytes | bytearray, offset: int) -> int:
    raw = int(data[offset]) | (int(data[offset + 1]) << 8) | (int(data[offset + 2]) << 16)
    if raw & 0x800000:
        raw -= 0x1000000
    return raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="无线采集系统 v3：48CH EMG / Gyro / Accel 示波器。")
    parser.add_argument("--port", help="可选，手动指定串口；若不提供则自动搜索。")
    parser.add_argument("--baud", type=int, default=4_000_000, help="串口波特率，默认 4000000。")
    parser.add_argument("--window-seconds", type=float, default=2.0, help="显示窗口长度，默认 2 秒。")
    parser.add_argument("--expected-rate", type=float, default=2000.0, help="EMG 缓冲区预估采样率，默认 2000Hz。")
    parser.add_argument("--emg-display-rate", type=float, default=2000.0, help="EMG 显示滤波器采样率，默认 2000Hz。")
    parser.add_argument("--ads-gain", type=float, default=4.0, help="ADS1298 PGA 增益，默认 4，必须与固件一致。")
    parser.add_argument("--mvc-rms-window-ms", type=float, default=250.0, help="EMG %%MVC 的滑动 RMS 窗口，默认 250ms。")
    parser.add_argument("--mvc-update-ms", type=int, default=50, help="EMG %%MVC 计算与曲线更新周期，默认 50ms。")
    parser.add_argument("--mvc-tolerance", type=float, default=5.0, help="目标线容差，单位为 MVC 百分点，默认 ±5。")
    parser.add_argument("--csv-dir", type=Path, default=DEFAULT_CSV_DIR, help="录制目录，默认当前 v3 工程的 data/；相对路径也以工程目录为基准。")
    parser.add_argument("--plot-interval-ms", type=int, default=250, help="实时波形刷新间隔，默认 250ms。")
    resend_group = parser.add_mutually_exclusive_group()
    resend_group.add_argument("--enable-resend", dest="enable_resend", action="store_true", help="开启 PC 端自动请求 Master 重发缺失帧（默认）。")
    resend_group.add_argument("--disable-resend", dest="enable_resend", action="store_false", help="关闭 PC 端自动重传请求。")
    parser.set_defaults(enable_resend=True)
    parser.add_argument(
        "--live-decimation",
        type=int,
        choices=(1, 2),
        default=2,
        help="滤波后的 EMG 实时显示抽取倍率，可选 1/2，默认 2；录制仍保存全量。",
    )
    parser.add_argument("--list-ports", action="store_true", help="列出当前所有串口后退出。")
    args = parser.parse_args()
    args.csv_dir = resolve_recording_dir(args.csv_dir)
    return args


def list_ports() -> None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未发现可用串口。")
        return
    print("可用串口：")
    for port in ports:
        print(f"  {port.device}  {port.description}")


def auto_find_port() -> str | None:
    preferred_keywords = ("usbmodem", "esp32", "espressif", "usb jtag", "usb serial", "cdc", "tinyusb")
    fallback_keywords = ("usbserial", "wchusb", "ttyacm", "ttyusb", "cp210", "ch340", "uart")
    reject_keywords = ("bluetooth", "incoming-port", "airplay")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    scored: list[tuple[int, str]] = []
    for port in ports:
        haystack = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
        if any(keyword in haystack for keyword in reject_keywords):
            continue
        score = 0
        if any(keyword in haystack for keyword in preferred_keywords):
            score += 10
        if any(keyword in haystack for keyword in fallback_keywords):
            score += 3
        if score > 0:
            scored.append((score, port.device))
    if scored:
        scored.sort(reverse=True)
        return scored[0][1]
    return None


@dataclass
class Emg48Buffer:
    timestamps_us: deque[int]
    sample_seq: deque[int]
    drdy_count: deque[int]
    flags: deque[int]
    channels: list[deque[int]]
    last_host_rx_time: float = 0.0
    received_count: int = 0


@dataclass
class SourceBuffer:
    timestamps_us: deque[int]
    sample_seq: deque[int]
    batch_seq: deque[int]
    rx_flags: deque[int]
    x: deque[int]
    y: deque[int]
    z: deque[int]
    last_host_rx_time: float = 0.0
    received_count: int = 0


@dataclass
class ParserStats:
    crc_fails: int = 0
    crc_emg_fails: int = 0
    crc_imu_fails: int = 0
    crc_sync_fails: int = 0
    crc_state_fails: int = 0
    bad_headers: int = 0
    bad_sources: int = 0
    bad_counts: int = 0
    last_bad_count: int = 0
    pc_sequence_gaps: int = 0
    pc_resend_requests: int = 0
    pc_resend_recovered: int = 0
    pc_duplicate_frames: int = 0
    host_queue_drops: int = 0
    recording_queue_drops: int = 0
    raw_queue_max_depth: int = 0
    bad_timestamps: int = 0
    resync_bytes: int = 0
    reader_errors: int = 0
    last_reader_error: str = ""


@dataclass
class SyncDiag:
    host_rx_time: float
    source_id: int
    state: int
    beacon_seq: int
    offset_us: int
    drift_ppm: int
    residual_us: int
    beacon_count: int


@dataclass
class StateEvent:
    host_rx_time: float
    source_id: int
    state: int
    command_seq: int
    effective_master_time_us: int


class AcquisitionModel:
    def __init__(self, max_points: int) -> None:
        self.lock = threading.Lock()
        self.emg = Emg48Buffer(
            timestamps_us=deque(maxlen=max_points),
            sample_seq=deque(maxlen=max_points),
            drdy_count=deque(maxlen=max_points),
            flags=deque(maxlen=max_points),
            channels=[deque(maxlen=max_points) for _ in range(EMG48_CHANNEL_COUNT)],
        )
        self.sources = {
            source_id: SourceBuffer(
                timestamps_us=deque(maxlen=max_points),
                sample_seq=deque(maxlen=max_points),
                batch_seq=deque(maxlen=max_points),
                rx_flags=deque(maxlen=max_points),
                x=deque(maxlen=max_points),
                y=deque(maxlen=max_points),
                z=deque(maxlen=max_points),
            )
            for source_id in (SOURCE_GYRO, SOURCE_ACCEL)
        }
        self.is_recording = False
        self.emg_csv_fp = None
        self.emg_csv_writer = None
        self.emg_raw_fp = None
        self.emg_raw_path: Path | None = None
        self.imu_csv_fp = None
        self.imu_csv_writer = None
        self.sync_csv_fp = None
        self.sync_csv_writer = None
        self.record_queue: queue.Queue[tuple[str, object] | None] | None = None
        self.record_writer_thread: threading.Thread | None = None
        self.diag_forwarded_per_sec = 0
        self.diag_errors_per_sec = 0
        self.diag_online_slave_count = 0
        self.diag_link_events_per_sec = 0
        self.diag_queue_drops_per_sec = 0
        self.diag_ack_fails_per_sec = 0
        self.diag_errors_total = 0
        self.diag_link_events_total = 0
        self.diag_queue_drops_total = 0
        self.diag_ack_fails_total = 0
        self.system_state = 0
        self.state_events: deque[StateEvent] = deque(maxlen=200)
        self.sync_diag = {
            SOURCE_GYRO: deque(maxlen=500),
            SOURCE_ACCEL: deque(maxlen=500),
        }
        self.parser_stats = ParserStats()

    def clear_all(self) -> None:
        with self.lock:
            self.emg.timestamps_us.clear()
            self.emg.sample_seq.clear()
            self.emg.drdy_count.clear()
            self.emg.flags.clear()
            for channel in self.emg.channels:
                channel.clear()
            self.emg.received_count = 0
            self.emg.last_host_rx_time = 0.0
            for source in self.sources.values():
                source.timestamps_us.clear()
                source.sample_seq.clear()
                source.batch_seq.clear()
                source.rx_flags.clear()
                source.x.clear()
                source.y.clear()
                source.z.clear()
                source.received_count = 0
                source.last_host_rx_time = 0.0

    def reset_session(self) -> None:
        """Reset all host-side state before attaching a new serial device."""
        self.clear_all()
        with self.lock:
            self.diag_forwarded_per_sec = 0
            self.diag_errors_per_sec = 0
            self.diag_online_slave_count = 0
            self.diag_link_events_per_sec = 0
            self.diag_queue_drops_per_sec = 0
            self.diag_ack_fails_per_sec = 0
            self.diag_errors_total = 0
            self.diag_link_events_total = 0
            self.diag_queue_drops_total = 0
            self.diag_ack_fails_total = 0
            self.system_state = 0
            self.state_events.clear()
            for values in self.sync_diag.values():
                values.clear()
            self.parser_stats = ParserStats()

    def clear_emg_on_relock(self) -> None:
        with self.lock:
            self.emg.timestamps_us.clear()
            self.emg.sample_seq.clear()
            self.emg.drdy_count.clear()
            self.emg.flags.clear()
            for channel in self.emg.channels:
                channel.clear()

    def append_emg48(
        self,
        timestamp_us: int,
        sample_seq: int,
        drdy_count: int,
        flags: int,
        channels: tuple[int, ...] | None,
        include_display: bool = True,
    ) -> None:
        with self.lock:
            self.emg.last_host_rx_time = time.time()
            self.emg.received_count += 1
            if include_display and channels is not None:
                self.emg.timestamps_us.append(timestamp_us)
                self.emg.sample_seq.append(sample_seq)
                self.emg.drdy_count.append(drdy_count)
                self.emg.flags.append(flags)
                for idx, value in enumerate(channels):
                    self.emg.channels[idx].append(value)

    def append_imu_sample(
        self,
        source_id: int,
        timestamp_us: int,
        sample_seq: int,
        batch_seq: int,
        rx_flags: int,
        x: int,
        y: int,
        z: int,
    ) -> None:
        if source_id == SOURCE_DIAG:
            self.update_diag(
                x,
                y,
                z,
                link_events_per_sec=sample_seq,
                queue_drops_per_sec=batch_seq,
                ack_fails_per_sec=rx_flags,
            )
            return
        with self.lock:
            source = self.sources.get(source_id)
            if source is None:
                return
            source.timestamps_us.append(timestamp_us)
            source.sample_seq.append(sample_seq)
            source.batch_seq.append(batch_seq)
            source.rx_flags.append(rx_flags)
            source.x.append(x)
            source.y.append(y)
            source.z.append(z)
            source.last_host_rx_time = time.time()
            source.received_count += 1
            record_queue = self.record_queue if self.is_recording else None
        if record_queue is not None:
            name = SOURCE_NAMES.get(source_id, f"Unknown({source_id})")
            vector_norm = float(np.sqrt(float(x) * float(x) + float(y) * float(y) + float(z) * float(z)))
            self.enqueue_record_row(
                "imu",
                [
                    f"{time.time():.6f}",
                    name,
                    source_id,
                    timestamp_us,
                    sample_seq,
                    batch_seq,
                    rx_flags,
                    x,
                    y,
                    z,
                    f"{vector_norm:.3f}",
                ],
            )

    def update_diag(
        self,
        forwarded_per_sec: int,
        errors_per_sec: int,
        online_slave_count: int,
        link_events_per_sec: int = 0,
        queue_drops_per_sec: int = 0,
        ack_fails_per_sec: int = 0,
    ) -> None:
        with self.lock:
            self.diag_forwarded_per_sec = forwarded_per_sec
            self.diag_errors_per_sec = errors_per_sec
            self.diag_online_slave_count = online_slave_count
            self.diag_link_events_per_sec = link_events_per_sec
            self.diag_queue_drops_per_sec = queue_drops_per_sec
            self.diag_ack_fails_per_sec = ack_fails_per_sec
            self.diag_errors_total += max(0, errors_per_sec)
            self.diag_link_events_total += max(0, link_events_per_sec)
            self.diag_queue_drops_total += max(0, queue_drops_per_sec)
            self.diag_ack_fails_total += max(0, ack_fails_per_sec)

    def update_sync_diag(
        self,
        source_id: int,
        state: int,
        beacon_seq: int,
        offset_us: int,
        drift_ppm: int,
        residual_us: int,
        beacon_count: int,
    ) -> None:
        diag = SyncDiag(time.time(), source_id, state, beacon_seq, offset_us, drift_ppm, residual_us, beacon_count)
        with self.lock:
            if source_id not in self.sync_diag:
                self.sync_diag[source_id] = deque(maxlen=500)
            self.sync_diag[source_id].append(diag)
            record_queue = self.record_queue if self.is_recording else None
        if record_queue is not None:
            self.enqueue_record_row(
                "sync",
                [
                    f"{diag.host_rx_time:.6f}",
                    SOURCE_NAMES.get(source_id, f"Unknown({source_id})"),
                    source_id,
                    STATE_NAMES.get(state, str(state)),
                    beacon_seq,
                    offset_us,
                    drift_ppm,
                    residual_us,
                    beacon_count,
                ],
            )

    def update_state_event(self, source_id: int, state: int, command_seq: int, effective_master_time_us: int) -> None:
        event = StateEvent(time.time(), source_id, state, command_seq, effective_master_time_us)
        with self.lock:
            self.state_events.append(event)
            if source_id == SOURCE_EMG:
                self.system_state = state

    def reference_timestamp_us(self) -> int | None:
        with self.lock:
            latest_timestamp: int | None = None
            latest_host_rx_time = -1.0
            if self.emg.timestamps_us and self.emg.last_host_rx_time >= latest_host_rx_time:
                latest_host_rx_time = self.emg.last_host_rx_time
                latest_timestamp = int(self.emg.timestamps_us[-1])
            for source_id in (SOURCE_GYRO, SOURCE_ACCEL):
                source = self.sources[source_id]
                if source.timestamps_us and source.last_host_rx_time >= latest_host_rx_time:
                    latest_host_rx_time = source.last_host_rx_time
                    latest_timestamp = int(source.timestamps_us[-1])
            return latest_timestamp

    def emg_snapshot(
        self,
        reference_timestamp_us: int | None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray], int, float, float, int]:
        with self.lock:
            empty = np.array([], dtype=np.float64)
            if not self.emg.timestamps_us or reference_timestamp_us is None:
                return (
                    empty,
                    np.array([], dtype=np.int64),
                    np.array([], dtype=np.uint64),
                    [empty for _ in range(EMG48_CHANNEL_COUNT)],
                    self.emg.received_count,
                    0.0,
                    0.0,
                    0,
                )
            timestamps = np.fromiter(self.emg.timestamps_us, dtype=np.int64)
            sample_seq = np.fromiter(self.emg.sample_seq, dtype=np.uint64)
            rel_time_s = signed_delta_us(timestamps, reference_timestamp_us).astype(np.float64) / 1_000_000.0
            channels = [np.fromiter(channel, dtype=np.float64) for channel in self.emg.channels]
            handled_rate = 0.0
            drdy_rate = 0.0
            if len(self.emg.timestamps_us) >= 2:
                dt = (int(self.emg.timestamps_us[-1]) - int(self.emg.timestamps_us[0])) / 1_000_000.0
                if dt > 0:
                    handled_rate = (int(self.emg.sample_seq[-1]) - int(self.emg.sample_seq[0])) / dt
                    drdy_rate = (int(self.emg.drdy_count[-1]) - int(self.emg.drdy_count[0])) / dt
            flags = int(self.emg.flags[-1]) if self.emg.flags else 0
            return (
                rel_time_s,
                timestamps,
                sample_seq,
                channels,
                self.emg.received_count,
                handled_rate,
                drdy_rate,
                flags,
            )

    def imu_snapshot(self, source_id: int, reference_timestamp_us: int | None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
        with self.lock:
            source = self.sources[source_id]
            empty = np.array([], dtype=np.float64)
            if not source.timestamps_us or reference_timestamp_us is None:
                return empty, empty, empty, empty, source.received_count
            timestamps = np.fromiter(source.timestamps_us, dtype=np.int64)
            x = np.fromiter(source.x, dtype=np.float64)
            y = np.fromiter(source.y, dtype=np.float64)
            z = np.fromiter(source.z, dtype=np.float64)
            rel_time_s = signed_delta_us(timestamps, reference_timestamp_us).astype(np.float64) / 1_000_000.0
            if rel_time_s.size > 1:
                order = np.argsort(rel_time_s)
                rel_time_s = rel_time_s[order]
                x = x[order]
                y = y[order]
                z = z[order]
            return rel_time_s, x, y, z, source.received_count

    def source_status_snapshot(self) -> dict[int, tuple[int, float]]:
        with self.lock:
            status = {
                SOURCE_EMG: (self.emg.received_count, self.emg.last_host_rx_time),
            }
            status.update(
                {
                    source_id: (source.received_count, source.last_host_rx_time)
                    for source_id, source in self.sources.items()
                }
            )
            return status

    def diag_snapshot(self) -> tuple[int, int, int, int, int, int, int, int, int, int, ParserStats]:
        with self.lock:
            return (
                self.diag_forwarded_per_sec,
                self.diag_errors_per_sec,
                self.diag_online_slave_count,
                self.diag_link_events_per_sec,
                self.diag_queue_drops_per_sec,
                self.diag_ack_fails_per_sec,
                self.diag_errors_total,
                self.diag_link_events_total,
                self.diag_queue_drops_total,
                self.diag_ack_fails_total,
                ParserStats(
                    crc_fails=self.parser_stats.crc_fails,
                    crc_emg_fails=self.parser_stats.crc_emg_fails,
                    crc_imu_fails=self.parser_stats.crc_imu_fails,
                    crc_sync_fails=self.parser_stats.crc_sync_fails,
                    crc_state_fails=self.parser_stats.crc_state_fails,
                    bad_headers=self.parser_stats.bad_headers,
                    bad_sources=self.parser_stats.bad_sources,
                    bad_counts=self.parser_stats.bad_counts,
                    last_bad_count=self.parser_stats.last_bad_count,
                    pc_sequence_gaps=self.parser_stats.pc_sequence_gaps,
                    pc_resend_requests=self.parser_stats.pc_resend_requests,
                    pc_resend_recovered=self.parser_stats.pc_resend_recovered,
                    pc_duplicate_frames=self.parser_stats.pc_duplicate_frames,
                    host_queue_drops=self.parser_stats.host_queue_drops,
                    recording_queue_drops=self.parser_stats.recording_queue_drops,
                    raw_queue_max_depth=self.parser_stats.raw_queue_max_depth,
                    bad_timestamps=self.parser_stats.bad_timestamps,
                    resync_bytes=self.parser_stats.resync_bytes,
                    reader_errors=self.parser_stats.reader_errors,
                    last_reader_error=self.parser_stats.last_reader_error,
                ),
            )

    def sync_snapshot(self) -> tuple[int, dict[int, list[SyncDiag]], list[StateEvent]]:
        with self.lock:
            return (
                self.system_state,
                {source_id: list(values) for source_id, values in self.sync_diag.items()},
                list(self.state_events),
            )

    def note_crc_fail(self, frame_type: int | None = None) -> None:
        with self.lock:
            self.parser_stats.crc_fails += 1
            if frame_type == PC_MSG_EMG48_BATCH:
                self.parser_stats.crc_emg_fails += 1
            elif frame_type in (PC_MSG_SAMPLE_BATCH, PC_MSG_IMU_WIRE_BATCH):
                self.parser_stats.crc_imu_fails += 1
            elif frame_type == PC_MSG_SYNC_DIAG:
                self.parser_stats.crc_sync_fails += 1
            elif frame_type == PC_MSG_STATE_EVENT:
                self.parser_stats.crc_state_fails += 1

    def note_bad_header(self) -> None:
        with self.lock:
            self.parser_stats.bad_headers += 1

    def note_bad_source(self) -> None:
        with self.lock:
            self.parser_stats.bad_sources += 1

    def note_bad_count(self, count: int) -> None:
        with self.lock:
            self.parser_stats.bad_counts += 1
            self.parser_stats.last_bad_count = count

    def note_pc_sequence_gap(self, missing_frames: int) -> None:
        if missing_frames <= 0:
            return
        with self.lock:
            self.parser_stats.pc_sequence_gaps += missing_frames

    def note_pc_resend_request(self, count: int = 1) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.pc_resend_requests += count

    def note_pc_resend_recovered(self) -> None:
        with self.lock:
            self.parser_stats.pc_resend_recovered += 1

    def note_pc_duplicate_frame(self) -> None:
        with self.lock:
            self.parser_stats.pc_duplicate_frames += 1

    def note_host_queue_drop(self, count: int = 1) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.host_queue_drops += count

    def note_recording_queue_drop(self, count: int = 1) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.recording_queue_drops += count

    def note_raw_queue_depth(self, depth: int) -> None:
        if depth <= 0:
            return
        with self.lock:
            if depth > self.parser_stats.raw_queue_max_depth:
                self.parser_stats.raw_queue_max_depth = depth

    def enqueue_record_row(self, kind: str, row: list) -> None:
        record_queue = self.record_queue
        if record_queue is None:
            return
        try:
            record_queue.put_nowait((kind, row))
        except queue.Full:
            self.note_recording_queue_drop()

    def _record_writer_loop(self) -> None:
        record_queue = self.record_queue
        if record_queue is None:
            return
        while True:
            item = record_queue.get()
            try:
                if item is None:
                    return
                kind, row = item
                if kind == "emg_raw" and self.emg_raw_fp is not None:
                    self.emg_raw_fp.write(row)
                elif kind == "imu" and self.imu_csv_writer is not None:
                    self.imu_csv_writer.writerow(row)
                elif kind == "sync" and self.sync_csv_writer is not None:
                    self.sync_csv_writer.writerow(row)
            finally:
                record_queue.task_done()

    def note_bad_timestamp(self) -> None:
        with self.lock:
            self.parser_stats.bad_timestamps += 1

    def note_resync_bytes(self, count: int) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.resync_bytes += count

    def note_reader_error(self, exc: Exception) -> None:
        with self.lock:
            self.parser_stats.reader_errors += 1
            self.parser_stats.last_reader_error = f"{type(exc).__name__}: {exc}"

    def start_recording(self, csv_dir: Path) -> tuple[Path, Path, Path]:
        csv_dir = resolve_recording_dir(csv_dir)
        with self.lock:
            if self.is_recording:
                raise RuntimeError("CSV 已经处于录制状态。")
            csv_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            emg_path = csv_dir / f"emg48_capture_{stamp}.csv"
            imu_path = csv_dir / f"imu_capture_{stamp}.csv"
            sync_path = csv_dir / f"sync_diag_{stamp}.csv"

            self.emg_raw_path = emg_path.with_suffix(".bin")
            self.emg_raw_fp = self.emg_raw_path.open("wb")
            self.imu_csv_fp = imu_path.open("w", newline="", encoding="utf-8")
            self.imu_csv_writer = csv.writer(self.imu_csv_fp)
            self.imu_csv_writer.writerow(
                ["host_rx_time", "source", "source_id", "timestamp_us", "sample_seq", "batch_seq", "rx_flags", "x", "y", "z", "vector_norm"]
            )
            self.sync_csv_fp = sync_path.open("w", newline="", encoding="utf-8")
            self.sync_csv_writer = csv.writer(self.sync_csv_fp)
            self.sync_csv_writer.writerow(
                ["host_rx_time", "source", "source_id", "state", "beacon_seq", "offset_us", "drift_ppm", "residual_us", "beacon_count"]
            )
            self.record_queue = queue.Queue(maxsize=RECORD_QUEUE_ROWS)
            self.record_writer_thread = threading.Thread(target=self._record_writer_loop, name="csvRecordWriter", daemon=True)
            self.record_writer_thread.start()
            self.is_recording = True
            return emg_path, imu_path, sync_path

    def stop_recording(self) -> None:
        with self.lock:
            self.is_recording = False
            record_queue = self.record_queue
            record_writer_thread = self.record_writer_thread
            self.record_queue = None
        if record_queue is not None:
            record_queue.join()
            record_queue.put(None)
            record_queue.join()
        if record_writer_thread is not None:
            record_writer_thread.join(timeout=2.0)
        with self.lock:
            for fp in (self.emg_csv_fp, self.emg_raw_fp, self.imu_csv_fp, self.sync_csv_fp):
                if fp is not None:
                    fp.close()
            self.emg_csv_fp = None
            self.emg_csv_writer = None
            self.emg_raw_fp = None
            self.emg_raw_path = None
            self.imu_csv_fp = None
            self.imu_csv_writer = None
            self.sync_csv_fp = None
            self.sync_csv_writer = None
            self.record_writer_thread = None


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, model: AcquisitionModel, enable_resend: bool = False, live_decimation: int = 2) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.model = model
        self.enable_resend = enable_resend
        self.live_decimation = max(1, int(live_decimation))
        self.stop_event = threading.Event()
        self.serial_port: serial.Serial | None = None
        self.last_timestamp_us: dict[int, int] = {}
        self.write_lock = threading.Lock()
        self.last_data_frame_sequence: dict[int, int] = {}
        self.pending_resend_sequences: set[tuple[int, int]] = set()
        self.resend_command_times: deque[float] = deque()
        self.raw_queue: queue.Queue[bytes] = queue.Queue(maxsize=RAW_QUEUE_CHUNKS)
        self.raw_reader_thread: threading.Thread | None = None
        self.stream_reset_event = threading.Event()

    @property
    def connected(self) -> bool:
        return (
            self.serial_port is not None
            and self.serial_port.is_open
            and not self.stop_event.is_set()
        )

    def open(self) -> None:
        # Clear TinyUSB's partially armed DTR/RTS reboot sequence left by the
        # preceding close, before asserting the connected (True, True) state.
        port = serial.Serial(port=None, baudrate=self.baud, timeout=0.02)
        port.dtr = False
        port.rts = False
        port.port = self.port
        port.open()
        self.serial_port = port
        try:
            self.serial_port.set_buffer_size(rx_size=4 * 1024 * 1024, tx_size=64 * 1024)
        except (AttributeError, OSError, serial.SerialException):
            pass
        self.serial_port.dtr = True
        self.serial_port.rts = True

    def close(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

    def stop(self) -> None:
        self.stop_event.set()

    def _clear_stream_tracking(self) -> None:
        self.last_timestamp_us.clear()
        self.last_data_frame_sequence.clear()
        self.pending_resend_sequences.clear()
        self.resend_command_times.clear()

    def reset_stream_session(self) -> None:
        """Forget stream-local sequence state before Master starts a new run."""
        self._clear_stream_tracking()
        while True:
            try:
                self.raw_queue.get_nowait()
                self.raw_queue.task_done()
            except queue.Empty:
                break
        self.stream_reset_event.set()

    def send_text_command(self, command: str) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("串口尚未打开。")
        with self.write_lock:
            self.serial_port.write(f"{command.strip()}\n".encode("utf-8"))
            self.serial_port.flush()

    def discard_corrupt_candidate(self, raw_buffer: bytearray) -> bytearray:
        next_header = raw_buffer.find(FRAME_HEADER, 2)
        if next_header < 0:
            discard_count = max(1, len(raw_buffer) - 1)
            self.model.note_resync_bytes(discard_count)
            return raw_buffer[-1:]
        self.model.note_resync_bytes(next_header)
        return raw_buffer[next_header:]

    def _raw_reader_loop(self) -> None:
        if self.serial_port is None:
            return
        while not self.stop_event.is_set():
            try:
                chunk = self.serial_port.read(SERIAL_READ_SIZE)
                if not chunk:
                    continue
                try:
                    self.raw_queue.put_nowait(chunk)
                    self.model.note_raw_queue_depth(self.raw_queue.qsize())
                except queue.Full:
                    self.model.note_host_queue_drop(len(chunk))
            except serial.SerialException as exc:
                self.model.note_reader_error(exc)
                self.stop_event.set()
                break
            except Exception as exc:
                self.model.note_reader_error(exc)

    def request_resend(self, frame_type: int, sequence: int, modulus: int) -> None:
        if not self.enable_resend:
            return
        seq = sequence % modulus
        key = (frame_type, seq)
        if key in self.pending_resend_sequences:
            return
        if len(self.pending_resend_sequences) >= MAX_PENDING_RESEND_SEQUENCES:
            return
        now = time.monotonic()
        while self.resend_command_times and now - self.resend_command_times[0] > 1.0:
            self.resend_command_times.popleft()
        if len(self.resend_command_times) >= MAX_RESEND_COMMANDS_PER_SECOND:
            return
        self.pending_resend_sequences.add(key)
        try:
            if frame_type == PC_MSG_EMG48_BATCH:
                self.send_text_command(f"RESEND EMG {seq}")
            else:
                self.send_text_command(f"RESEND {seq}")
            self.resend_command_times.append(now)
            self.model.note_pc_resend_request()
        except Exception as exc:
            self.pending_resend_sequences.discard(key)
            self.model.note_reader_error(exc)

    def classify_data_frame_sequence(self, frame_type: int, sequence: int, modulus: int) -> str:
        seq = sequence % modulus
        key = (frame_type, seq)
        if key in self.pending_resend_sequences:
            self.pending_resend_sequences.remove(key)
            self.model.note_pc_resend_recovered()
            return "recovered"

        previous = self.last_data_frame_sequence.get(frame_type)
        if previous is None:
            self.last_data_frame_sequence[frame_type] = seq
            return "current"

        diff = (seq - previous) % modulus
        if diff == 0:
            self.model.note_pc_duplicate_frame()
            return "duplicate"
        if diff < (modulus // 2):
            missing = diff - 1
            if missing > 0:
                self.model.note_pc_sequence_gap(missing)
                for offset in range(1, min(missing, MAX_RESEND_REQUESTS_PER_GAP) + 1):
                    self.request_resend(frame_type, previous + offset, modulus)
            self.last_data_frame_sequence[frame_type] = seq
            return "current"

        self.model.note_pc_duplicate_frame()
        return "duplicate"

    def note_timestamp(self, source_id: int, timestamp_us: int) -> None:
        if source_id not in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL):
            return
        if source_id != SOURCE_EMG:
            return
        last_timestamp = self.last_timestamp_us.get(source_id)
        if last_timestamp is None:
            self.last_timestamp_us[source_id] = timestamp_us
            return
        delta_us = (int(timestamp_us) - int(last_timestamp) + INT32_HALF_WRAP) % UINT32_WRAP - INT32_HALF_WRAP
        if delta_us < -TIMESTAMP_RELOCK_BACKWARD_US or delta_us > TIMESTAMP_RELOCK_GAP_US:
            self.model.clear_emg_on_relock()
        elif delta_us <= 0:
            self.model.note_bad_timestamp()
        self.last_timestamp_us[source_id] = timestamp_us

    def run(self) -> None:
        if self.serial_port is None:
            raise RuntimeError("串口尚未打开。")
        self.raw_reader_thread = threading.Thread(target=self._raw_reader_loop, name="serialRawReader", daemon=True)
        self.raw_reader_thread.start()
        raw_buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                try:
                    chunk = self.raw_queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if self.stream_reset_event.is_set():
                    raw_buffer.clear()
                    self._clear_stream_tracking()
                    self.stream_reset_event.clear()
                raw_buffer += chunk

                while len(raw_buffer) >= MIN_FRAME_SIZE:
                    idx = raw_buffer.find(FRAME_HEADER)
                    if idx == -1:
                        self.model.note_resync_bytes(max(0, len(raw_buffer) - 1))
                        raw_buffer = raw_buffer[-1:]
                        break
                    if idx > 0:
                        self.model.note_resync_bytes(idx)
                        raw_buffer = raw_buffer[idx:]
                    if len(raw_buffer) < 4:
                        break

                    frame_type = raw_buffer[2]
                    if frame_type == PC_MSG_SYNC_DIAG:
                        if len(raw_buffer) < PC_SYNC_DIAG_SIZE:
                            break
                        frame = bytes(raw_buffer[:PC_SYNC_DIAG_SIZE])
                        if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = self.discard_corrupt_candidate(raw_buffer)
                            continue
                        self.model.update_sync_diag(*PC_SYNC_DIAG_STRUCT.unpack(frame[4:-2]))
                        raw_buffer = raw_buffer[PC_SYNC_DIAG_SIZE:]
                        continue

                    if frame_type == PC_MSG_STATE_EVENT:
                        if len(raw_buffer) < PC_STATE_EVENT_SIZE:
                            break
                        frame = bytes(raw_buffer[:PC_STATE_EVENT_SIZE])
                        if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = self.discard_corrupt_candidate(raw_buffer)
                            continue
                        self.model.update_state_event(*PC_STATE_EVENT_STRUCT.unpack(frame[4:-2]))
                        raw_buffer = raw_buffer[PC_STATE_EVENT_SIZE:]
                        continue

                    if frame_type == PC_MSG_EMG48_BATCH:
                        if len(raw_buffer) < PC_EMG48_HEADER_SIZE:
                            break
                        sample_count = raw_buffer[3]
                        if sample_count == 0 or sample_count > EMG48_MAX_BATCH_SAMPLES:
                            self.model.note_bad_count(sample_count)
                            raw_buffer = raw_buffer[1:]
                            continue
                        frame_sequence = int.from_bytes(raw_buffer[4:6], "little")
                        packet_size = (
                            PC_EMG48_HEADER_SIZE
                            + EMG48_BATCH_META_SIZE
                            + sample_count * EMG48_SAMPLE_SIZE
                            + 2
                        )
                        if len(raw_buffer) < packet_size:
                            break
                        frame = bytes(raw_buffer[:packet_size])
                        if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = self.discard_corrupt_candidate(raw_buffer)
                            continue
                        frame_role = self.classify_data_frame_sequence(PC_MSG_EMG48_BATCH, frame_sequence, 65536)
                        if frame_role == "duplicate":
                            raw_buffer = raw_buffer[packet_size:]
                            continue
                        with self.model.lock:
                            record_queue = self.model.record_queue if self.model.is_recording else None
                        if record_queue is not None:
                            self.model.enqueue_record_row("emg_raw", frame)
                        try:
                            base_timestamp_us, base_sample_seq, base_drdy_count, flags, _reserved = (
                                EMG48_BATCH_META_STRUCT.unpack(
                                    frame[
                                        PC_EMG48_HEADER_SIZE :
                                        PC_EMG48_HEADER_SIZE + EMG48_BATCH_META_SIZE
                                    ]
                                )
                            )
                        except struct.error:
                            self.model.note_bad_source()
                            raw_buffer = raw_buffer[packet_size:]
                            continue
                        payload_offset = PC_EMG48_HEADER_SIZE + EMG48_BATCH_META_SIZE
                        for sample_index in range(sample_count):
                            sample_bytes = frame[payload_offset : payload_offset + EMG48_SAMPLE_SIZE]
                            timestamp_us = (base_timestamp_us + sample_index * EMG48_SAMPLE_PERIOD_US) & 0xFFFFFFFF
                            sample_seq = (base_sample_seq + sample_index) & 0xFFFFFFFF
                            drdy_count = (base_drdy_count + sample_index) & 0xFFFFFFFF
                            payload_offset += EMG48_SAMPLE_SIZE
                            include_display = frame_role == "current"
                            channels = None
                            if include_display:
                                try:
                                    channels = tuple(
                                        decode_i24_le(sample_bytes, ch * EMG48_CHANNEL_WIRE_SIZE)
                                        for ch in range(EMG48_CHANNEL_COUNT)
                                    )
                                except IndexError:
                                    self.model.note_bad_source()
                                    break
                                self.note_timestamp(SOURCE_EMG, timestamp_us)
                            self.model.append_emg48(
                                timestamp_us,
                                sample_seq,
                                drdy_count,
                                flags,
                                channels,
                                include_display=include_display,
                            )
                        raw_buffer = raw_buffer[packet_size:]
                        continue

                    if frame_type == PC_MSG_IMU_WIRE_BATCH:
                        if len(raw_buffer) < PC_IMU_WIRE_PACKET_SIZE:
                            break
                        frame = bytes(raw_buffer[:PC_IMU_WIRE_PACKET_SIZE])
                        if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = self.discard_corrupt_candidate(raw_buffer)
                            continue
                        frame_sequence = frame[3]
                        frame_role = self.classify_data_frame_sequence(PC_MSG_IMU_WIRE_BATCH, frame_sequence, 256)
                        if frame_role == "duplicate":
                            raw_buffer = raw_buffer[PC_IMU_WIRE_PACKET_SIZE:]
                            continue

                        wire = frame[PC_IMU_WIRE_HEADER_SIZE:-2]
                        if (
                            len(wire) != IMU_BATCH_WIRE_SIZE
                            or wire[0] != IMU_BATCH_TYPE
                            or int.from_bytes(wire[-2:], "little") != crc16_ccitt(wire[:-2])
                        ):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = raw_buffer[PC_IMU_WIRE_PACKET_SIZE:]
                            continue

                        source_id = wire[1]
                        batch_seq = int.from_bytes(wire[2:4], "little")
                        sample_start_seq = int.from_bytes(wire[4:8], "little")
                        sample_count = wire[8]
                        rx_flags = wire[9]
                        base_timestamp_us = int.from_bytes(wire[10:14], "little")
                        if source_id not in (SOURCE_GYRO, SOURCE_ACCEL):
                            self.model.note_bad_source()
                            raw_buffer = raw_buffer[PC_IMU_WIRE_PACKET_SIZE:]
                            continue
                        if sample_count == 0 or sample_count > IMU_BATCH_SIZE:
                            self.model.note_bad_count(sample_count)
                            raw_buffer = raw_buffer[PC_IMU_WIRE_PACKET_SIZE:]
                            continue

                        sample_offset = IMU_BATCH_HEADER_SIZE
                        for sample_index in range(sample_count):
                            sample = wire[sample_offset : sample_offset + IMU_SAMPLE_SIZE]
                            dt_us = int.from_bytes(sample[0:2], "little")
                            timestamp_us = (base_timestamp_us + dt_us) & 0xFFFFFFFF
                            sample_seq = (sample_start_seq + sample_index) & 0xFFFFFFFF
                            x = int.from_bytes(sample[2:4], "little", signed=True)
                            y = int.from_bytes(sample[4:6], "little", signed=True)
                            z = int.from_bytes(sample[6:8], "little", signed=True)
                            self.model.append_imu_sample(
                                source_id,
                                timestamp_us,
                                sample_seq,
                                batch_seq,
                                rx_flags,
                                x,
                                y,
                                z,
                            )
                            sample_offset += IMU_SAMPLE_SIZE
                        raw_buffer = raw_buffer[PC_IMU_WIRE_PACKET_SIZE:]
                        continue

                    if frame_type == PC_MSG_SAMPLE_BATCH:
                        if len(raw_buffer) < PC_HEADER_SIZE:
                            break
                        sample_count = raw_buffer[3]
                        if sample_count == 0 or sample_count > PC_MAX_BATCH_SAMPLES:
                            self.model.note_bad_count(sample_count)
                            raw_buffer = raw_buffer[1:]
                            continue
                        frame_sequence = raw_buffer[4]
                        packet_size = PC_HEADER_SIZE + sample_count * PC_SAMPLE_SIZE + 2
                        if len(raw_buffer) < packet_size:
                            break
                        frame = bytes(raw_buffer[:packet_size])
                        if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
                            self.model.note_crc_fail(frame_type)
                            raw_buffer = self.discard_corrupt_candidate(raw_buffer)
                            continue
                        frame_role = self.classify_data_frame_sequence(PC_MSG_SAMPLE_BATCH, frame_sequence, 256)
                        if frame_role == "duplicate":
                            raw_buffer = raw_buffer[packet_size:]
                            continue
                        payload_offset = PC_HEADER_SIZE
                        for _ in range(sample_count):
                            sample_bytes = frame[payload_offset : payload_offset + PC_SAMPLE_SIZE]
                            payload_offset += PC_SAMPLE_SIZE
                            try:
                                source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z = PC_SAMPLE_STRUCT.unpack(sample_bytes)
                            except struct.error:
                                self.model.note_bad_source()
                                continue
                            if source_id not in (SOURCE_GYRO, SOURCE_ACCEL, SOURCE_DIAG):
                                self.model.note_bad_source()
                                continue
                            if frame_role == "current":
                                self.note_timestamp(source_id, timestamp_us)
                            self.model.append_imu_sample(source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z)
                        raw_buffer = raw_buffer[packet_size:]
                        continue

                    self.model.note_bad_header()
                    raw_buffer = raw_buffer[1:]
            except serial.SerialException:
                break
            except Exception as exc:
                self.model.note_reader_error(exc)
                continue


class MainWindow(QMainWindow):
    def __init__(
        self,
        model: AcquisitionModel,
        csv_dir: Path,
        initial_port: str | None,
        baud: int,
        window_seconds: float,
        emg_display_rate: float,
        plot_interval_ms: int,
        ads_gain: float,
        mvc_rms_window_ms: float,
        mvc_update_ms: int,
        mvc_tolerance: float,
        enable_resend: bool,
        live_decimation: int,
    ) -> None:
        super().__init__()
        self.model = model
        self.reader: SerialReader | None = None
        self.csv_dir = resolve_recording_dir(csv_dir)
        self.waveform_dir = DEFAULT_WAVEFORM_DIR
        self.enable_resend = bool(enable_resend)
        self.live_decimation = max(1, int(live_decimation))
        self.initial_port = initial_port
        self.port = ""
        self.baud = baud
        self.connection_error = ""
        self.disconnecting = False
        self.last_port_scan: tuple[str, ...] = ()
        self.last_system_state = 0
        self.window_seconds = window_seconds
        self.plot_interval_ms = max(100, int(plot_interval_ms))
        self.ads_gain = float(ads_gain)
        self.mvc_update_ms = max(20, int(mvc_update_ms))
        self.last_counts = {SOURCE_EMG: -1, SOURCE_GYRO: -1, SOURCE_ACCEL: -1}
        self.source_rates = {SOURCE_EMG: 0.0, SOURCE_GYRO: 0.0, SOURCE_ACCEL: 0.0}
        self.rate_last_counts = {SOURCE_EMG: 0, SOURCE_GYRO: 0, SOURCE_ACCEL: 0}
        self.rate_last_time = time.time()
        self.control_status_text = ""
        self.control_status_until = 0.0
        self.analysis_status_text = "录制分析结果 --"
        self.analysis_status_visible_text = ""
        self.analysis_status_until = 0.0
        self.emg_quality_text = "EMG48 --"
        self.current_recording_paths: tuple[Path, Path, Path] | None = None
        self.current_mvc_recording_paths: tuple[Path, Path] | None = None
        self.mvc_csv_fp = None
        self.mvc_csv_writer = None
        self.mvc_stable_since: float | None = None
        self.mvc_last_recorded_timestamp: int | None = None
        self.emg_plots: list[pg.PlotItem] = []
        self.emg_curves = []
        self.imu_curves: dict[int, tuple] = {}

        self.emg_live_filter = EmgLiveFilter(
            sample_rate=emg_display_rate,
            decimation=self.live_decimation,
            window_seconds=self.window_seconds,
            ads_gain=self.ads_gain,
        )
        self.mvc_tracker = MvcTracker(
            sample_rate=emg_display_rate,
            rms_window_ms=mvc_rms_window_ms,
            update_ms=self.mvc_update_ms,
            tolerance_percent=mvc_tolerance,
        )

        self.setWindowTitle("无线采集系统 v3 · 48CH EMG + MVC反馈")
        self.resize(1380, 900)
        self.setMinimumSize(1080, 760)
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #F2F4F7;
                color: #1F2933;
                font-size: 13px;
            }
            QWidget#headerBand, QWidget#statusBand {
                background: transparent;
            }
            QWidget#connectionPanel, QWidget#controlPanel, QWidget#settingsPanel, QWidget#analysisGroup {
                background: #FFFFFF;
                border: 1px solid #D7DDE5;
                border-radius: 3px;
            }
            QWidget#headerBand QLabel, QWidget#statusBand QLabel,
            QWidget#connectionPanel QLabel, QWidget#controlPanel QLabel, QWidget#settingsPanel QLabel,
            QWidget#analysisGroup QLabel {
                background: transparent;
            }
            QLabel#sectionLabel {
                color: #52606D;
                font-size: 11px;
                font-weight: 600;
            }
            QPushButton, QToolButton, QComboBox {
                min-height: 30px;
                border: 1px solid #C8D0DA;
                border-radius: 5px;
                background: #FFFFFF;
                padding: 0 10px;
            }
            QPushButton:hover, QToolButton:hover, QComboBox:hover {
                border-color: #7B8A9A;
                background: #F8FAFC;
            }
            QPushButton:disabled {
                color: #9AA5B1;
                background: #EEF1F4;
            }
            QPushButton#streamButton {
                background: #1769E0;
                border-color: #1769E0;
                color: #FFFFFF;
                font-weight: 600;
            }
            QPushButton#stopButton {
                color: #B42318;
                border-color: #E2A39D;
            }
            QPushButton#recordButton {
                background: #16803A;
                border-color: #16803A;
                color: #FFFFFF;
                font-weight: 600;
            }
            QPushButton#recordingButton {
                background: #C4322B;
                border-color: #C4322B;
                color: #FFFFFF;
                font-weight: 600;
            }
            QTextEdit#analysisPanel {
                color: #1F2933;
                background: #F8FAFC;
                border: 1px solid #D7DDE5;
                border-radius: 5px;
                padding: 4px;
                selection-background-color: #B8D4FF;
            }
            QTabWidget::pane {
                border: 1px solid #C9D1DB;
                background: #101418;
            }
            QTabBar::tab {
                min-width: 78px;
                min-height: 28px;
                padding: 0 14px;
            }
            QTabWidget::tab-bar {
                alignment: center;
            }
            QLabel#statusPrimary {
                font-weight: 600;
                color: #26323D;
                background: #F8FAFC;
                border: 1px solid #D7DDE5;
                border-radius: 2px;
                padding: 4px 6px;
            }
            QLabel#statusCell {
                color: #52606D;
                background: #F8FAFC;
                border: 1px solid #D7DDE5;
                border-radius: 2px;
                padding: 4px 6px;
            }
            """
        )

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        header_band = QWidget()
        header_band.setObjectName("headerBand")
        header_layout = QHBoxLayout(header_band)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(6)

        controls_layout = QVBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(7)

        connection_panel = QWidget()
        connection_panel.setObjectName("connectionPanel")
        connection_row = QHBoxLayout(connection_panel)
        connection_row.setContentsMargins(8, 5, 8, 5)
        connection_row.setSpacing(6)
        connection_label = QLabel("串口")
        connection_label.setObjectName("sectionLabel")
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.port_combo.setMinimumWidth(260)
        self.port_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        if self.port_combo.lineEdit() is not None:
            self.port_combo.lineEdit().setPlaceholderText("选择或输入串口路径")
        self.refresh_ports_button = QToolButton()
        self.refresh_ports_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload))
        self.refresh_ports_button.setToolTip("刷新串口列表")
        self.connect_button = QPushButton("连接")
        self.connect_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
        self.connect_button.setMinimumWidth(88)
        self.connection_status_label = QLabel("未连接")
        self.connection_status_label.setObjectName("sectionLabel")
        connection_row.addWidget(connection_label)
        connection_row.addWidget(self.port_combo, stretch=1)
        connection_row.addWidget(self.refresh_ports_button)
        connection_row.addWidget(self.connect_button)
        connection_row.addWidget(self.connection_status_label)
        controls_layout.addWidget(connection_panel)

        action_panel = QWidget()
        action_panel.setObjectName("controlPanel")
        action_row = QHBoxLayout(action_panel)
        action_row.setContentsMargins(8, 5, 8, 5)
        action_row.setSpacing(6)
        action_label = QLabel("采集控制")
        action_label.setObjectName("sectionLabel")
        self.sync_button = QPushButton("开始同步")
        self.stream_button = QPushButton("开始采集")
        self.stop_button = QPushButton("停止")
        self.clear_button = QPushButton("清空")
        self.record_button = QPushButton("开始录制 CSV")
        self.stream_button.setObjectName("streamButton")
        self.stop_button.setObjectName("stopButton")
        self.record_button.setObjectName("recordButton")
        self.sync_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload))
        self.stream_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay))
        self.stop_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop))
        self.clear_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogResetButton))
        self.record_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogSaveButton))
        self.sync_button.setToolTip("广播同步信标并等待 Slave 收敛")
        self.stream_button.setToolTip("统一启动 EMG48 与 IMU 数据流")
        self.stop_button.setToolTip("停止采集并返回空闲状态")
        self.clear_button.setToolTip("清空当前实时波形缓存")
        self.record_button.setToolTip("录制 EMG48、IMU 和同步诊断数据")

        action_row.addWidget(action_label)
        for button in (self.sync_button, self.stream_button, self.stop_button, self.clear_button):
            button.setMinimumWidth(88)
            action_row.addWidget(button)
        action_row.addStretch(1)
        self.record_button.setMinimumWidth(132)
        action_row.addWidget(self.record_button)
        controls_layout.addWidget(action_panel)

        self.storage_path_label = QLabel(
            f"录制目录：{self.csv_dir}\n波形目录：{self.waveform_dir}"
        )
        self.storage_path_label.setWordWrap(True)
        self.storage_path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        controls_layout.addWidget(self.storage_path_label)

        settings_panel = QWidget()
        settings_panel.setObjectName("settingsPanel")
        settings_row = QHBoxLayout(settings_panel)
        settings_row.setContentsMargins(8, 5, 8, 5)
        settings_row.setSpacing(6)
        wireless_label = QLabel("无线")
        wireless_label.setObjectName("sectionLabel")
        self.channel_combo = QComboBox()
        for channel in range(1, 14):
            self.channel_combo.addItem(f"CH{channel}", channel)
        self.channel_combo.setCurrentIndex(0)
        self.channel_combo.setMinimumWidth(72)
        self.rate_combo = QComboBox()
        self.rate_combo.addItem("2Mbps", "2M")
        self.rate_combo.addItem("1Mbps", "1M")
        self.rate_combo.setMinimumWidth(84)
        self.radio_button = QPushButton("应用无线")
        self.radio_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton))
        self.radio_button.setToolTip("将无线信道和速率应用到 Master 与 Slave")
        self.display_combo = QComboBox()
        self.display_combo.addItem("EMG Raw Counts", "raw")
        self.display_combo.addItem("EMG uV 20-450Hz + 50Hz Notch", "filtered")
        self.display_combo.setMinimumWidth(250)
        self.display_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        display_label = QLabel("显示")
        display_label.setObjectName("sectionLabel")
        amplitude_label = QLabel("纵轴最小")
        amplitude_label.setObjectName("sectionLabel")
        self.emg_min_range_combo = QComboBox()
        for half_range_uv in EMG_MIN_HALF_RANGES_UV:
            self.emg_min_range_combo.addItem(f"±{half_range_uv:g} µV", half_range_uv)
        default_scale_index = self.emg_min_range_combo.findData(EMG_DEFAULT_MIN_HALF_RANGE_UV)
        self.emg_min_range_combo.setCurrentIndex(max(0, default_scale_index))
        self.emg_min_range_combo.setMinimumWidth(100)
        self.emg_min_range_combo.setToolTip("静息时的最小纵轴半幅；发力信号超出后仍会自动扩展")
        window_label = QLabel(f"{self.window_seconds:g} s 窗口 · {1000.0 / self.plot_interval_ms:.0f} FPS")
        window_label.setObjectName("sectionLabel")

        self.sync_button.clicked.connect(lambda: self.send_control_command("START_SYNC"))
        self.stream_button.clicked.connect(lambda: self.send_control_command("START_STREAM"))
        self.stop_button.clicked.connect(lambda: self.send_control_command("STOP"))
        self.clear_button.clicked.connect(self.clear_live_buffers)
        self.radio_button.clicked.connect(self.send_radio_command)
        self.record_button.clicked.connect(self.toggle_recording)
        self.display_combo.currentIndexChanged.connect(self._update_emg_scale_control)
        self.refresh_ports_button.clicked.connect(self.refresh_serial_ports)
        self.connect_button.clicked.connect(self.toggle_serial_connection)

        settings_row.addWidget(wireless_label)
        settings_row.addWidget(self.channel_combo)
        settings_row.addWidget(self.rate_combo)
        settings_row.addWidget(self.radio_button)
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.VLine)
        divider.setFrameShadow(QFrame.Shadow.Sunken)
        settings_row.addWidget(divider)
        settings_row.addWidget(display_label)
        settings_row.addWidget(self.display_combo, stretch=1)
        settings_row.addWidget(amplitude_label)
        settings_row.addWidget(self.emg_min_range_combo)
        settings_row.addWidget(window_label)
        self._update_emg_scale_control()
        controls_layout.addWidget(settings_panel)
        header_layout.addLayout(controls_layout, stretch=3)

        analysis_group = QWidget()
        analysis_group.setObjectName("analysisGroup")
        analysis_layout = QVBoxLayout(analysis_group)
        analysis_layout.setContentsMargins(8, 5, 8, 5)
        analysis_layout.setSpacing(3)
        analysis_title_row = QHBoxLayout()
        analysis_title = QLabel("录制分析")
        analysis_title.setObjectName("sectionLabel")
        self.copy_analysis_button = QToolButton()
        self.copy_analysis_button.setText("复制")
        self.copy_analysis_button.setToolTip("复制完整录制分析结果")
        self.copy_analysis_button.clicked.connect(self.copy_analysis_result)
        analysis_title_row.addWidget(analysis_title)
        analysis_title_row.addStretch(1)
        analysis_title_row.addWidget(self.copy_analysis_button)
        analysis_layout.addLayout(analysis_title_row)
        self.analysis_status_label = QTextEdit()
        self.analysis_status_label.setObjectName("analysisPanel")
        self.analysis_status_label.setReadOnly(True)
        self.analysis_status_label.setPlainText("录制分析结果 --")
        self.analysis_status_label.setLineWrapMode(QTextEdit.LineWrapMode.WidgetWidth)
        self.analysis_status_label.setMinimumWidth(360)
        self.analysis_status_label.setMaximumWidth(520)
        self.analysis_status_label.setFixedHeight(76)
        analysis_layout.addWidget(self.analysis_status_label)
        header_layout.addWidget(analysis_group, stretch=2)
        main_layout.addWidget(header_band)

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.emg_graph = pg.GraphicsLayoutWidget()
        self.imu_graph = pg.GraphicsLayoutWidget()
        self.mvc_tab = QWidget()
        self.emg_graph.ci.layout.setSpacing(2)
        self.imu_graph.ci.layout.setSpacing(4)
        self.tabs.addTab(self.emg_graph, "EMG48")
        self.tabs.addTab(self.imu_graph, "IMU")
        self.tabs.addTab(self.mvc_tab, "MVC反馈")
        main_layout.addWidget(self.tabs, stretch=1)

        mvc_layout = QVBoxLayout(self.mvc_tab)
        mvc_layout.setContentsMargins(10, 10, 10, 10)
        mvc_layout.setSpacing(8)
        mvc_controls = QWidget()
        mvc_controls.setObjectName("settingsPanel")
        mvc_controls_layout = QHBoxLayout(mvc_controls)
        mvc_controls_layout.setContentsMargins(10, 7, 10, 7)
        mvc_controls_layout.setSpacing(7)
        mvc_controls_layout.addWidget(QLabel("反馈通道"))
        self.mvc_group_combo = QComboBox()
        self.mvc_group_combo.addItem("FPC1 · CH01–16", 0)
        self.mvc_group_combo.addItem("FPC2 · CH17–32", 16)
        self.mvc_group_combo.addItem("FPC3 · CH33–48", 32)
        self.mvc_group_combo.setMinimumWidth(145)
        mvc_controls_layout.addWidget(self.mvc_group_combo)
        self.mvc_start_button = QPushButton("开始MVC标定")
        self.mvc_cancel_button = QPushButton("取消标定")
        self.mvc_redo_button = QPushButton("重做当前试次")
        self.mvc_clear_button = QPushButton("清除MVC")
        self.mvc_start_button.setToolTip("5秒静息基线，随后完成三次5秒最大等长收缩")
        self.mvc_cancel_button.setToolTip("取消当前标定但不停止数据采集")
        self.mvc_redo_button.setToolTip("丢弃并重新进行当前试次")
        self.mvc_clear_button.setToolTip("清除MVC基准和实时反馈历史")
        for button in (
            self.mvc_start_button,
            self.mvc_cancel_button,
            self.mvc_redo_button,
            self.mvc_clear_button,
        ):
            mvc_controls_layout.addWidget(button)
        mvc_controls_layout.addStretch(1)
        mvc_controls_layout.addWidget(QLabel("目标"))
        self.mvc_target_spin = QDoubleSpinBox()
        self.mvc_target_spin.setRange(1.0, 100.0)
        self.mvc_target_spin.setDecimals(0)
        self.mvc_target_spin.setSingleStep(1.0)
        self.mvc_target_spin.setSuffix(" %MVC")
        self.mvc_target_spin.setValue(10.0)
        self.mvc_target_spin.setMinimumWidth(105)
        mvc_controls_layout.addWidget(self.mvc_target_spin)
        for target in (10, 20, 30, 40, 50):
            button = QPushButton(f"{target}%")
            button.setMaximumWidth(54)
            button.clicked.connect(lambda _checked=False, value=target: self.mvc_target_spin.setValue(value))
            mvc_controls_layout.addWidget(button)
        mvc_layout.addWidget(mvc_controls)

        mvc_status_panel = QWidget()
        mvc_status_panel.setObjectName("controlPanel")
        mvc_status_layout = QGridLayout(mvc_status_panel)
        mvc_status_layout.setContentsMargins(10, 7, 10, 7)
        self.mvc_current_label = QLabel("当前 EMG %MVC：--")
        self.mvc_target_label = QLabel("目标：10% MVC · 容差 ±5")
        self.mvc_hold_label = QLabel("容差带内保持：0.0 s")
        self.mvc_calibration_label = QLabel("标定状态：未标定")
        self.mvc_metric_label = QLabel(f"空间RMS：-- · PGA {self.ads_gain:g}")
        for label in (
            self.mvc_current_label,
            self.mvc_target_label,
            self.mvc_hold_label,
            self.mvc_calibration_label,
            self.mvc_metric_label,
        ):
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        mvc_status_layout.addWidget(self.mvc_current_label, 0, 0)
        mvc_status_layout.addWidget(self.mvc_target_label, 0, 1)
        mvc_status_layout.addWidget(self.mvc_hold_label, 0, 2)
        mvc_status_layout.addWidget(self.mvc_calibration_label, 1, 0, 1, 2)
        mvc_status_layout.addWidget(self.mvc_metric_label, 1, 2)
        mvc_status_layout.setColumnStretch(0, 1)
        mvc_status_layout.setColumnStretch(1, 1)
        mvc_status_layout.setColumnStretch(2, 1)
        mvc_layout.addWidget(mvc_status_panel)

        self.mvc_plot = pg.PlotWidget()
        self.mvc_plot.showGrid(x=True, y=True, alpha=0.22)
        self.mvc_plot.setLabel("left", "EMG", units="%MVC")
        self.mvc_plot.setLabel("bottom", "最近时间", units="s")
        self.mvc_plot.setXRange(-MVC_HISTORY_SECONDS, 0.0, padding=0.0)
        self.mvc_plot.setYRange(0.0, 150.0, padding=0.02)
        self.mvc_plot.setMouseEnabled(x=False, y=False)
        self.mvc_tolerance_region = pg.LinearRegionItem(
            values=(5.0, 15.0),
            orientation="horizontal",
            movable=False,
            brush=pg.mkBrush(23, 105, 224, 45),
            pen=pg.mkPen(None),
        )
        self.mvc_target_line = pg.InfiniteLine(pos=10.0, angle=0, movable=False, pen=pg.mkPen("#1769E0", width=2))
        self.mvc_curve = self.mvc_plot.plot([], [], pen=pg.mkPen("#00D18F", width=2.2))
        self.mvc_plot.addItem(self.mvc_tolerance_region)
        self.mvc_plot.addItem(self.mvc_target_line)
        mvc_layout.addWidget(self.mvc_plot, stretch=1)

        self.mvc_group_combo.currentIndexChanged.connect(self._change_mvc_group)
        self.mvc_target_spin.valueChanged.connect(self._change_mvc_target)
        self.mvc_start_button.clicked.connect(self.start_mvc_calibration)
        self.mvc_cancel_button.clicked.connect(self.cancel_mvc_calibration)
        self.mvc_redo_button.clicked.connect(self.redo_mvc_trial)
        self.mvc_clear_button.clicked.connect(self.clear_mvc_calibration)

        status_band = QWidget()
        status_band.setObjectName("statusBand")
        status_grid = QGridLayout(status_band)
        status_grid.setContentsMargins(9, 6, 9, 6)
        status_grid.setHorizontalSpacing(16)
        status_grid.setVerticalSpacing(3)
        self.primary_status_label = QLabel(f"未连接串口 @ {self.baud}")
        self.primary_status_label.setObjectName("statusPrimary")
        self.emg_status_label = QLabel("EMG48 --")
        self.link_status_label = QLabel("Slave-01 -- | Slave-02 --")
        self.transport_status_label = QLabel("CRC 0 | gap 0 | host drop 0")
        self.sync_status_label = QLabel("State IDLE | Sync --")
        for label in (
            self.emg_status_label,
            self.link_status_label,
            self.transport_status_label,
            self.sync_status_label,
        ):
            label.setObjectName("statusCell")
        for label in (
            self.primary_status_label,
            self.emg_status_label,
            self.link_status_label,
            self.transport_status_label,
            self.sync_status_label,
        ):
            label.setWordWrap(True)
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        status_grid.addWidget(self.primary_status_label, 0, 0)
        status_grid.addWidget(self.emg_status_label, 0, 1)
        status_grid.addWidget(self.link_status_label, 1, 0)
        status_grid.addWidget(self.transport_status_label, 1, 1)
        status_grid.addWidget(self.sync_status_label, 2, 0, 1, 2)
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 1)
        main_layout.insertWidget(1, status_band)

        for row in range(6):
            for col in range(8):
                idx = row * 8 + col
                plot = self.emg_graph.addPlot(row=row, col=col, title=f"ch{idx + 1:02d}")
                plot.setTitle(f"ch{idx + 1:02d}", size="8pt")
                self._setup_small_emg_plot(plot)
                curve = plot.plot([], [], pen=pg.mkPen("#00D18F", width=1.0))
                self.emg_plots.append(plot)
                self.emg_curves.append(curve)

        first_imu_plot = None
        for row, (source_id, title, channels, colors) in enumerate(
            (
                (SOURCE_GYRO, "Slave-01 Gyroscope", ("gx", "gy", "gz"), ("#00C853", "#FF9100", "#AA00FF")),
                (SOURCE_ACCEL, "Slave-02 Accelerometer", ("ax", "ay", "az"), ("#26C6DA", "#EC407A", "#9CCC65")),
            )
        ):
            plot = self.imu_graph.addPlot(row=row, col=0, title=title)
            self._setup_imu_plot(plot)
            if first_imu_plot is None:
                first_imu_plot = plot
            else:
                plot.setXLink(first_imu_plot)
            plot.addLegend(offset=(12, 12))
            curves = []
            for label, color in zip(channels, colors):
                curves.append(plot.plot([], [], pen=pg.mkPen(color, width=1.4), name=label))
            self.imu_curves[source_id] = (*curves, plot)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(self.plot_interval_ms)
        self.mvc_timer = QTimer(self)
        self.mvc_timer.timeout.connect(self.update_mvc_feedback)
        self.mvc_timer.start(self.mvc_update_ms)
        self.port_scan_timer = QTimer(self)
        self.port_scan_timer.timeout.connect(self._poll_serial_connection)
        self.port_scan_timer.start(1000)
        self.refresh_serial_ports(force=True)
        self._update_control_states(0)
        if self.initial_port:
            self.port_combo.setEditText(self.initial_port)
            QTimer.singleShot(0, self.connect_serial)

    def _serial_port_sort_key(self, port_info) -> tuple[int, str]:
        text = f"{port_info.device} {port_info.description} {port_info.manufacturer or ''}".lower()
        if "usbmodem" in text or "ttyacm" in text:
            priority = 0
        elif "usbserial" in text or "wch" in text or "cp210" in text or "ch340" in text:
            priority = 1
        elif "bluetooth" in text or "incoming-port" in text or "debug-console" in text:
            priority = 3
        else:
            priority = 2
        return priority, port_info.device

    def refresh_serial_ports(self, force: bool = False) -> None:
        if self.reader is not None:
            return
        ports = sorted(serial.tools.list_ports.comports(), key=self._serial_port_sort_key)
        devices = tuple(port.device for port in ports)
        if not force and devices == self.last_port_scan:
            return
        previous_text = self.port_combo.currentText().strip()
        self.last_port_scan = devices
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for port_info in ports:
            self.port_combo.addItem(port_info.device, port_info.device)
            index = self.port_combo.count() - 1
            description = port_info.description or "无描述"
            manufacturer = f" | {port_info.manufacturer}" if port_info.manufacturer else ""
            self.port_combo.setItemData(index, f"{description}{manufacturer}", Qt.ItemDataRole.ToolTipRole)
        if previous_text:
            self.port_combo.setEditText(previous_text)
        elif ports:
            preferred = next(
                (index for index, port_info in enumerate(ports) if self._serial_port_sort_key(port_info)[0] < 3),
                None,
            )
            if preferred is None:
                self.port_combo.setCurrentIndex(-1)
                self.port_combo.setEditText("")
            else:
                self.port_combo.setCurrentIndex(preferred)
        self.port_combo.blockSignals(False)
        if not ports and not previous_text:
            self.connection_status_label.setText("未发现串口")
        elif not self.connection_error:
            self.connection_status_label.setText("未连接")

    def toggle_serial_connection(self) -> None:
        if self.reader is None:
            self.connect_serial()
        else:
            self.disconnect_serial("已手动断开")

    def connect_serial(self) -> None:
        if self.reader is not None:
            return
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "无法连接", "请先选择或输入串口路径。")
            return

        self.connection_status_label.setText("连接中…")
        self.connect_button.setEnabled(False)
        QApplication.processEvents()
        reader = SerialReader(
            port=port,
            baud=self.baud,
            model=self.model,
            enable_resend=self.enable_resend,
            live_decimation=self.live_decimation,
        )
        try:
            reader.open()
            self.model.reset_session()
            self._reset_live_signal_state(preserve_completed_mvc=False)
            reader.start()
        except Exception as exc:
            reader.stop()
            reader.close()
            self.connection_error = str(exc)
            self.connection_status_label.setText("连接失败")
            self.primary_status_label.setText(f"串口连接失败: {exc}")
            self.connect_button.setEnabled(True)
            self._update_control_states(0)
            QMessageBox.warning(self, "串口连接失败", f"{port}\n{exc}")
            return

        self.reader = reader
        self.port = port
        self.last_system_state = 0
        self.connection_error = ""
        self.port_combo.setEnabled(False)
        self.refresh_ports_button.setEnabled(False)
        self.connect_button.setEnabled(True)
        self.connect_button.setText("断开")
        self.connect_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogCloseButton))
        self.connection_status_label.setText("已连接")
        self.primary_status_label.setText(f"已连接 {self.port} @ {self.baud}")
        self._update_control_states(0)

    def disconnect_serial(self, reason: str = "已断开") -> None:
        if self.disconnecting:
            return
        self.disconnecting = True
        try:
            self._finish_recording()
            reader = self.reader
            self.reader = None
            if reader is not None:
                reader.stop()
                reader.close()
                if reader.is_alive():
                    reader.join(timeout=1.5)
                if reader.raw_reader_thread is not None and reader.raw_reader_thread.is_alive():
                    reader.raw_reader_thread.join(timeout=0.5)
            self.port = ""
            self.last_system_state = 0
            self.model.reset_session()
            self._reset_live_signal_state(preserve_completed_mvc=False)
            self.port_combo.setEnabled(True)
            self.refresh_ports_button.setEnabled(True)
            self.connect_button.setText("连接")
            self.connect_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
            self.connection_status_label.setText(reason)
            self.primary_status_label.setText(f"{reason} @ {self.baud}")
            self._update_control_states(0)
            self.refresh_serial_ports(force=True)
        finally:
            self.disconnecting = False

    def _poll_serial_connection(self) -> None:
        reader = self.reader
        if reader is None:
            self.refresh_serial_ports()
            return
        if reader.stop_event.is_set() or not reader.is_alive() or not reader.connected:
            _diag = self.model.diag_snapshot()[-1]
            detail = _diag.last_reader_error or "串口读取已停止"
            self.disconnect_serial(f"连接中断: {detail}")

    def _require_reader(self) -> SerialReader:
        reader = self.reader
        if reader is None or not reader.connected:
            raise RuntimeError("请先选择并连接串口。")
        return reader

    def _invalidate_active_mvc(self, message: str) -> None:
        calibration = self.mvc_tracker.calibration
        if calibration.active:
            calibration.trial_invalid = True
            calibration.phase = "invalid"
            calibration.message = message

    def _reset_live_signal_state(self, preserve_completed_mvc: bool = True) -> None:
        self.model.clear_all()
        self.emg_live_filter.reset(clear_output=True)
        self.mvc_tracker.reset_signal(clear_history=True)
        if preserve_completed_mvc:
            self._invalidate_active_mvc("数据流已中断，请重做当前试次")
        else:
            self.mvc_tracker.calibration.clear()
        self.mvc_curve.clear()
        self.mvc_stable_since = None
        self.emg_quality_text = "EMG48 --"
        self.last_counts = {SOURCE_EMG: -1, SOURCE_GYRO: -1, SOURCE_ACCEL: -1}
        self.source_rates = {SOURCE_EMG: 0.0, SOURCE_GYRO: 0.0, SOURCE_ACCEL: 0.0}
        self.rate_last_counts = {SOURCE_EMG: 0, SOURCE_GYRO: 0, SOURCE_ACCEL: 0}
        self.rate_last_time = time.time()
        for curve in self.emg_curves:
            curve.clear()
        for source_id in (SOURCE_GYRO, SOURCE_ACCEL):
            cx, cy, cz, _plot = self.imu_curves[source_id]
            cx.clear()
            cy.clear()
            cz.clear()
        self._refresh_mvc_labels()

    def copy_analysis_result(self) -> None:
        QApplication.clipboard().setText(self.analysis_status_text)
        self.control_status_text = "录制分析结果已复制"
        self.control_status_until = time.time() + 2.0

    def clear_live_buffers(self) -> None:
        self._reset_live_signal_state(preserve_completed_mvc=True)

    def _change_mvc_group(self) -> None:
        group_start = int(self.mvc_group_combo.currentData())
        self.mvc_tracker.set_group_start(group_start)
        self.mvc_stable_since = None
        self.mvc_curve.clear()
        self._refresh_mvc_labels()

    def _change_mvc_target(self, value: float) -> None:
        self.mvc_tracker.set_target(value)
        target = self.mvc_tracker.target_percent
        tolerance = self.mvc_tracker.tolerance_percent
        self.mvc_target_line.setValue(target)
        self.mvc_tolerance_region.setRegion((max(0.0, target - tolerance), target + tolerance))
        self.mvc_stable_since = None
        self._refresh_mvc_labels()

    def start_mvc_calibration(self) -> None:
        system_state, _sync_diag, _events = self.model.sync_snapshot()
        if system_state != 3 or self.mvc_tracker.last_point is None:
            QMessageBox.warning(self, "无法开始MVC标定", "请先开始采集，并等待实时EMG数据稳定。")
            return
        self.mvc_tracker.calibration.start(time.monotonic())
        self.mvc_tracker.history.clear()
        self.mvc_stable_since = None
        self.tabs.setCurrentWidget(self.mvc_tab)
        self._refresh_mvc_labels()

    def cancel_mvc_calibration(self) -> None:
        self.mvc_tracker.calibration.cancel()
        self.mvc_stable_since = None
        self._refresh_mvc_labels()

    def redo_mvc_trial(self) -> None:
        self.mvc_tracker.calibration.redo_current(time.monotonic())
        self.mvc_stable_since = None
        self._refresh_mvc_labels()

    def clear_mvc_calibration(self) -> None:
        self.mvc_tracker.calibration.clear()
        self.mvc_tracker.history.clear()
        self.mvc_curve.clear()
        self.mvc_stable_since = None
        self._refresh_mvc_labels()

    def _refresh_mvc_labels(self) -> None:
        tracker = self.mvc_tracker
        calibration = tracker.calibration
        point = tracker.last_point
        actual = point.actual_percent if point is not None else float("nan")
        actual_text = f"{actual:.1f}%" if np.isfinite(actual) else "--"
        metric_text = f"{point.metric_uv_rms:.1f} µV RMS" if point is not None else "--"
        hold_seconds = 0.0 if self.mvc_stable_since is None else max(0.0, time.monotonic() - self.mvc_stable_since)
        self.mvc_current_label.setText(f"当前 EMG %MVC：{actual_text}")
        self.mvc_target_label.setText(
            f"目标：{tracker.target_percent:.0f}% MVC · 容差 ±{tracker.tolerance_percent:g}"
        )
        self.mvc_hold_label.setText(f"容差带内保持：{hold_seconds:.1f} s")
        self.mvc_calibration_label.setText(f"标定状态：{calibration.message}")
        self.mvc_metric_label.setText(f"空间RMS：{metric_text} · PGA {self.ads_gain:g}")
        self.mvc_group_combo.setEnabled(not calibration.active)
        connected = self.reader is not None and self.reader.connected
        self.mvc_start_button.setEnabled(connected and not calibration.active)
        self.mvc_cancel_button.setEnabled(calibration.active)
        self.mvc_redo_button.setEnabled(calibration.phase in {"countdown", "trial", "rest", "invalid", "complete"})

    def _record_mvc_point(self, point: MvcFeedbackPoint) -> None:
        if self.mvc_csv_writer is None or point.timestamp_us == self.mvc_last_recorded_timestamp:
            return
        self.mvc_last_recorded_timestamp = point.timestamp_us
        self.mvc_csv_writer.writerow(
            [
                f"{point.host_time:.6f}",
                point.timestamp_us,
                f"CH{point.group_start + 1:02d}-CH{point.group_start + MVC_GROUP_SIZE:02d}",
                f"{point.metric_uv_rms:.6f}",
                "" if not np.isfinite(point.rest_uv_rms) else f"{point.rest_uv_rms:.6f}",
                "" if not np.isfinite(point.mvc_uv_rms) else f"{point.mvc_uv_rms:.6f}",
                "" if not np.isfinite(point.actual_percent) else f"{point.actual_percent:.6f}",
                f"{point.target_percent:.3f}",
                f"{point.tolerance_percent:.3f}",
                point.phase,
                point.trial_number,
            ]
        )
        if self.mvc_csv_fp is not None:
            self.mvc_csv_fp.flush()

    def update_mvc_feedback(self) -> None:
        reference_timestamp_us = self.model.reference_timestamp_us()
        if reference_timestamp_us is None:
            self._refresh_mvc_labels()
            return
        (
            raw_t,
            timestamps_us,
            sample_seq,
            raw_channels,
            _emg_count,
            _handled_rate,
            _drdy_rate,
            _flags,
        ) = self.model.emg_snapshot(reference_timestamp_us)
        if raw_t.size > 1:
            new_timestamps, new_sequences, filtered_uv = self.emg_live_filter.consume(
                timestamps_us,
                sample_seq,
                raw_channels,
            )
            if new_sequences.size:
                point = self.mvc_tracker.consume(
                    new_timestamps,
                    new_sequences,
                    filtered_uv,
                    time.monotonic(),
                )
                if point is not None:
                    if np.isfinite(point.actual_percent) and abs(point.actual_percent - point.target_percent) <= point.tolerance_percent:
                        if self.mvc_stable_since is None:
                            self.mvc_stable_since = time.monotonic()
                    else:
                        self.mvc_stable_since = None
                    self._record_mvc_point(point)
        t, values = self.mvc_tracker.history_arrays()
        self.mvc_curve.setData(t, values)
        self.mvc_plot.setXRange(-MVC_HISTORY_SECONDS, 0.0, padding=0.0)
        if values.size:
            upper = max(150.0, float(np.nanmax(values)) * 1.1)
            self.mvc_plot.setYRange(0.0, upper, padding=0.02)
        self._refresh_mvc_labels()

    def _set_recording_button_state(self, recording: bool) -> None:
        self.record_button.setText("停止录制" if recording else "开始录制 CSV")
        icon = QStyle.StandardPixmap.SP_MediaStop if recording else QStyle.StandardPixmap.SP_DialogSaveButton
        self.record_button.setIcon(self.style().standardIcon(icon))
        self.record_button.setObjectName("recordingButton" if recording else "recordButton")
        self.record_button.style().unpolish(self.record_button)
        self.record_button.style().polish(self.record_button)

    def _update_control_states(self, system_state: int) -> None:
        connected = self.reader is not None and self.reader.connected
        self.sync_button.setEnabled(connected and system_state not in (1, 2, 3))
        self.stream_button.setEnabled(connected and system_state not in (2, 3))
        self.stop_button.setEnabled(connected and system_state != 0)
        self.radio_button.setEnabled(connected)
        self.record_button.setEnabled(connected or self.model.is_recording)

    def _setup_small_emg_plot(self, plot: pg.PlotItem) -> None:
        plot.showGrid(x=True, y=True, alpha=0.16)
        plot.setXRange(-self.window_seconds, 0.0, padding=0.0)
        plot.setMouseEnabled(x=False, y=False)
        plot.hideButtons()
        plot.setClipToView(True)
        plot.setDownsampling(auto=True, mode="peak")
        plot.getAxis("left").setStyle(showValues=False)
        plot.getAxis("bottom").setStyle(showValues=False)
        plot.getViewBox().setDefaultPadding(0.02)

    def _setup_imu_plot(self, plot: pg.PlotItem) -> None:
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setLabel("left", "Raw")
        plot.setLabel("bottom", "Time", units="s")
        plot.setXRange(-self.window_seconds, 0.0, padding=0.0)
        plot.setClipToView(True)
        plot.setDownsampling(mode="peak")

    def _prepare_raw_emg_display(
        self,
        rel_time_s: np.ndarray,
        sample_seq: np.ndarray,
        channels: list[np.ndarray],
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        if sample_seq.size == 0:
            return rel_time_s, channels
        keep = (sample_seq % self.live_decimation) == 0
        return rel_time_s[keep], [values[keep] for values in channels]

    def _format_emg_quality(self, channels: list[np.ndarray], handled_rate: float, drdy_rate: float, flags: int) -> str:
        finite = [values[np.isfinite(values)] for values in channels if values.size]
        if not finite:
            return "EMG48 --"
        peaks = np.array([float(np.nanmax(np.abs(values))) for values in finite if values.size], dtype=np.float64)
        rms_values = np.array([float(np.sqrt(np.mean(np.square(values)))) for values in finite if values.size], dtype=np.float64)
        if peaks.size == 0 or rms_values.size == 0:
            return "EMG48 --"
        rate_state = "OK" if 1900.0 <= handled_rate <= 2100.0 and 1900.0 <= drdy_rate <= 2100.0 else "LOW"
        unit = "uV" if self.display_combo.currentData() == "filtered" else "cnt"
        return (
            f"EMG48 handled {handled_rate:.1f}Hz {rate_state} | drdy {drdy_rate:.1f}Hz | "
            f"flags 0x{flags:02X} | rms med {np.median(rms_values):.1f}{unit} max {np.max(rms_values):.1f}{unit} | "
            f"pk max {np.max(peaks):.1f}{unit}"
        )

    def _scale_emg_plot(self, plot: pg.PlotItem, values: np.ndarray) -> None:
        if values.size == 0:
            return
        if self.display_combo.currentData() == "filtered":
            minimum_half_range = float(self.emg_min_range_combo.currentData())
        else:
            minimum_half_range = EMG_MIN_RAW_RANGE * 0.65
        y_range = calculate_emg_y_range(values, minimum_half_range)
        if y_range is not None:
            plot.setYRange(*y_range, padding=0.02)

    def _update_emg_scale_control(self) -> None:
        self.emg_min_range_combo.setEnabled(self.display_combo.currentData() == "filtered")

    def _smooth(self, values: np.ndarray) -> np.ndarray:
        if values.size < 5:
            return values
        kernel = np.ones(5, dtype=np.float64) / 5.0
        return np.convolve(values, kernel, mode="same")

    def _update_source_rates(self, counts: dict[int, int], now: float) -> None:
        elapsed = now - self.rate_last_time
        if elapsed < 1.0:
            return
        for source_id, count in counts.items():
            previous_count = self.rate_last_counts.get(source_id, count)
            self.source_rates[source_id] = max(0.0, float(count - previous_count) / elapsed)
            self.rate_last_counts[source_id] = count
        self.rate_last_time = now

    def _format_link_status(
        self, name: str, source_id: int, last_rx_time: float, now: float,
        system_state: int, sync_values: list[SyncDiag],
    ) -> str:
        if last_rx_time > 0.0 and (now - last_rx_time) <= 1.5:
            state = "采样在线"
        elif system_state in (1, 2) and sync_values and now - sync_values[-1].host_rx_time <= 1.5:
            state = "同步在线，待采集"
        elif system_state == 3:
            state = "无采样数据" if last_rx_time == 0.0 else "采样中断"
        else:
            state = "离线"
        return f"{name} {state} {self.source_rates[source_id]:.0f}/s"

    def _latest_master_state_event(self, state_events: list[StateEvent]) -> StateEvent | None:
        for event in reversed(state_events):
            if event.source_id == SOURCE_EMG:
                return event
        return None

    def _format_emg_status(self, system_state: int, emg_count: int, state_events: list[StateEvent], now: float) -> str:
        if system_state == 0:
            return "EMG48 等待采集态 (IDLE 中不输出 EMG)"
        if system_state == 1:
            return "EMG48 等待采集态 (SYNC 中不输出 EMG)"
        if system_state == 2:
            return "EMG48 等待统一启动 (STREAM_PENDING)"
        latest_master_event = self._latest_master_state_event(state_events)
        stream_elapsed = now - latest_master_event.host_rx_time if latest_master_event is not None else 0.0
        if emg_count == 0 and stream_elapsed >= 1.0:
            return "EMG48 STREAM 已进入但无数据，请检查 ADS1298/DRDY/接线"
        return f"EMG48 {emg_count} 点 {self.source_rates[SOURCE_EMG]:.0f}/s | {self.emg_quality_text}"

    def _stream_sync_warning(self) -> str:
        _, sync_diag, _ = self.model.sync_snapshot()
        warnings = []
        for source_id, name in ((SOURCE_GYRO, "Gyro"), (SOURCE_ACCEL, "Accel")):
            values = sync_diag.get(source_id, [])
            if not values:
                warnings.append(f"{name} 无同步诊断")
                continue
            latest = values[-1]
            if latest.beacon_count < SYNC_MIN_BEACON_COUNT:
                warnings.append(f"{name} Beacon 不足")
            elif abs(latest.residual_us) > SYNC_USABLE_RESIDUAL_US:
                warnings.append(f"{name} residual {latest.residual_us}us")
        return "" if not warnings else "同步质量不足: " + ", ".join(warnings)

    def send_control_command(self, command: str) -> None:
        try:
            reader = self._require_reader()
            warning_text = self._stream_sync_warning() if command == "START_STREAM" else ""
            if command == "START_STREAM":
                self._reset_live_signal_state(preserve_completed_mvc=True)
                reader.reset_stream_session()
            reader.send_text_command(command)
            if command == "STOP":
                self._finish_recording()
                self._reset_live_signal_state(preserve_completed_mvc=True)
            self.control_status_text = f"已发送控制命令: {command}"
            if warning_text:
                self.control_status_text += f"；{warning_text}"
            self.control_status_until = time.time() + 3.0
            self.primary_status_label.setText(self.control_status_text)
        except Exception as exc:
            QMessageBox.warning(self, "命令发送失败", str(exc))

    def send_radio_command(self) -> None:
        try:
            reader = self._require_reader()
            channel = int(self.channel_combo.currentData())
            rate = str(self.rate_combo.currentData())
            command = f"RADIO {channel} {rate}"
            reader.send_text_command(command)
            self.control_status_text = f"已发送无线配置: CH{channel} / {rate}，请重新同步后采集"
            self.control_status_until = time.time() + 5.0
            self.primary_status_label.setText(self.control_status_text)
        except Exception as exc:
            QMessageBox.warning(self, "无线配置失败", str(exc))

    def _format_sync_line(self, name: str, values: list[SyncDiag], now: float) -> str:
        if not values:
            return f"{name} --"
        latest = values[-1]
        recent = [item for item in values if now - item.host_rx_time <= SYNC_STABLE_WINDOW_S]
        quality = "未同步"
        if latest.beacon_count >= SYNC_MIN_BEACON_COUNT:
            quality = "可用" if abs(latest.residual_us) <= SYNC_USABLE_RESIDUAL_US else "未稳"
        if len(recent) >= 5:
            drift_values = [item.drift_ppm for item in recent]
            drift_range = max(drift_values) - min(drift_values)
            stable = (
                latest.beacon_count >= SYNC_MIN_BEACON_COUNT
                and all(abs(item.residual_us) <= SYNC_STABLE_RESIDUAL_US for item in recent)
                and drift_range <= SYNC_STABLE_DRIFT_RANGE_PPM
            )
            excellent = (
                latest.beacon_count >= SYNC_MIN_BEACON_COUNT
                and all(abs(item.residual_us) <= SYNC_EXCELLENT_RESIDUAL_US for item in recent)
                and drift_range <= SYNC_EXCELLENT_DRIFT_RANGE_PPM
            )
            if excellent:
                quality = "优秀"
            elif stable:
                quality = "稳定"
        return (
            f"{name} {quality} off {latest.offset_us}us drift {latest.drift_ppm}ppm "
            f"res {latest.residual_us}us n {latest.beacon_count}"
        )

    def update_plots(self) -> None:
        reference_timestamp_us = self.model.reference_timestamp_us()
        current_counts: dict[int, int] = {}
        (
            raw_t,
            timestamps_us,
            sample_seq,
            raw_channels,
            emg_count,
            handled_rate,
            drdy_rate,
            flags,
        ) = self.model.emg_snapshot(reference_timestamp_us)
        current_counts[SOURCE_EMG] = emg_count
        if emg_count != self.last_counts[SOURCE_EMG] and raw_t.size > 1:
            if self.display_combo.currentData() == "filtered":
                t, display_channels = self.emg_live_filter.snapshot(
                    reference_timestamp_us
                )
            else:
                t, display_channels = self._prepare_raw_emg_display(
                    raw_t,
                    sample_seq,
                    raw_channels,
                )
            xmin = max(float(t[0]), -self.window_seconds) if t.size else -self.window_seconds
            for idx, curve in enumerate(self.emg_curves):
                values = display_channels[idx]
                curve.setData(t, values)
                self.emg_plots[idx].setXRange(xmin, 0.0, padding=0.0)
                self._scale_emg_plot(self.emg_plots[idx], values)
            self.emg_quality_text = self._format_emg_quality(display_channels, handled_rate, drdy_rate, flags)
            self.last_counts[SOURCE_EMG] = emg_count

        for source_id in (SOURCE_GYRO, SOURCE_ACCEL):
            ti, x, y, z, count = self.model.imu_snapshot(source_id, reference_timestamp_us)
            current_counts[source_id] = count
            if count == self.last_counts[source_id] or ti.size < 2:
                continue
            cx, cy, cz, plot = self.imu_curves[source_id]
            cx.setData(ti, self._smooth(x))
            cy.setData(ti, self._smooth(y))
            cz.setData(ti, self._smooth(z))
            plot.setXRange(-self.window_seconds, 0.0, padding=0.0)
            self.last_counts[source_id] = count

        now = time.time()
        status_snapshot = self.model.source_status_snapshot()
        for source_id, (count, _) in status_snapshot.items():
            current_counts[source_id] = count
        self._update_source_rates(current_counts, now)

        (
            diag_forwarded,
            diag_errors,
            diag_online,
            diag_link_events,
            diag_queue_drops,
            diag_ack_fails,
            diag_errors_total,
            diag_link_events_total,
            diag_queue_drops_total,
            diag_ack_fails_total,
            parser_stats,
        ) = self.model.diag_snapshot()
        system_state, sync_diag, state_events = self.model.sync_snapshot()
        if system_state == 0 and self.last_system_state in (2, 3):
            self._reset_live_signal_state(preserve_completed_mvc=True)
            emg_count = 0
            handled_rate = 0.0
            drdy_rate = 0.0
            current_counts = {SOURCE_EMG: 0, SOURCE_GYRO: 0, SOURCE_ACCEL: 0}
        self.last_system_state = system_state
        self._update_control_states(system_state)
        record_state = "录制中" if self.model.is_recording else "未录制"
        reader_error_text = (
            f" | reader err {parser_stats.reader_errors} {parser_stats.last_reader_error}"
            if parser_stats.reader_errors
            else ""
        )
        slave1_last_rx = status_snapshot.get(SOURCE_GYRO, (0, 0.0))[1]
        slave2_last_rx = status_snapshot.get(SOURCE_ACCEL, (0, 0.0))[1]
        slave1_text = self._format_link_status("Slave-01 Gyro", SOURCE_GYRO, slave1_last_rx, now,
                                               system_state, sync_diag.get(SOURCE_GYRO, []))
        slave2_text = self._format_link_status("Slave-02 Accel", SOURCE_ACCEL, slave2_last_rx, now,
                                               system_state, sync_diag.get(SOURCE_ACCEL, []))
        emg_status = self._format_emg_status(system_state, emg_count, state_events, now)
        control_status = f" | {self.control_status_text}" if now < self.control_status_until else ""

        resend_state = "resend on" if self.enable_resend else "resend off"
        if self.reader is not None and self.reader.connected:
            self.primary_status_label.setText(
                f"{STATE_NAMES.get(system_state, system_state)} | 串口 {self.port} | "
                f"{record_state} | {resend_state} | live /{self.live_decimation}{control_status}"
            )
        self.emg_status_label.setText(emg_status)
        desired_analysis_text = self.analysis_status_text if (now < self.analysis_status_until or self.model.is_recording) else "录制分析结果 --"
        if desired_analysis_text != self.analysis_status_visible_text:
            self.analysis_status_visible_text = desired_analysis_text
            self.analysis_status_label.setPlainText(desired_analysis_text)
        self.link_status_label.setText(
            f"{slave1_text} | {slave2_text} | "
            f"Gyro {self.last_counts[SOURCE_GYRO] if self.last_counts[SOURCE_GYRO] >= 0 else 0} 点 | "
            f"Accel {self.last_counts[SOURCE_ACCEL] if self.last_counts[SOURCE_ACCEL] >= 0 else 0} 点 | "
            f"fwd {diag_forwarded}/s | online {diag_online}"
        )
        self.transport_status_label.setText(
            f"CRC {parser_stats.crc_fails} "
            f"(emg {parser_stats.crc_emg_fails}, imu {parser_stats.crc_imu_fails}, "
            f"sync {parser_stats.crc_sync_fails}, state {parser_stats.crc_state_fails}) | "
            f"gap {parser_stats.pc_sequence_gaps} | hostdrop {parser_stats.host_queue_drops}B | "
            f"qdrop {diag_queue_drops_total}/{parser_stats.recording_queue_drops} | "
            f"resync {parser_stats.resync_bytes}B | bad {parser_stats.bad_headers}/{parser_stats.bad_sources}/"
            f"{parser_stats.bad_counts} | "
            f"resend {parser_stats.pc_resend_requests}/{parser_stats.pc_resend_recovered} | "
            f"dup {parser_stats.pc_duplicate_frames} | err {diag_errors_total} | "
            f"ackfail {diag_ack_fails_total}{reader_error_text}"
        )
        gyro_sync = self._format_sync_line("Gyro", sync_diag.get(SOURCE_GYRO, []), now)
        accel_sync = self._format_sync_line("Accel", sync_diag.get(SOURCE_ACCEL, []), now)
        last_event = state_events[-1] if state_events else None
        event_text = ""
        if last_event is not None:
            event_text = (
                f" | last ACK {SOURCE_NAMES.get(last_event.source_id, last_event.source_id)} "
                f"{STATE_NAMES.get(last_event.state, last_event.state)} seq {last_event.command_seq}"
            )
        self.sync_status_label.setText(
            f"State {STATE_NAMES.get(system_state, system_state)} | {gyro_sync} | {accel_sync}{event_text}"
        )

    def toggle_recording(self) -> None:
        if not self.model.is_recording:
            try:
                self._require_reader()
            except Exception as exc:
                QMessageBox.warning(self, "无法录制", str(exc))
                return
            try:
                paths = self.model.start_recording(self.csv_dir)
                mvc_paths = self._start_mvc_recording(paths[0])
            except Exception as exc:
                self.model.stop_recording()
                QMessageBox.critical(self, "录制失败", str(exc))
                return
            self.current_recording_paths = paths
            self.current_mvc_recording_paths = mvc_paths
            self._set_recording_button_state(True)
            self.primary_status_label.setText(f"正在录制: {paths[0]}")
        else:
            self._finish_recording()

    def _finish_recording(self) -> None:
        if not self.model.is_recording and self.current_recording_paths is None:
            return
        paths = self.current_recording_paths
        mvc_paths = self.current_mvc_recording_paths
        self.model.stop_recording()
        self._stop_mvc_recording()
        self.current_recording_paths = None
        self.current_mvc_recording_paths = None
        self._set_recording_button_state(False)
        if paths is not None:
            self.analysis_status_text = f"录制已停止，正在分析: {paths[0].name}"
            self.analysis_status_until = time.time() + 90.0
            threading.Thread(
                target=self._analyze_recording_files,
                args=(*paths, *(mvc_paths or (None, None))),
                daemon=True,
            ).start()

    def _start_mvc_recording(self, emg_path: Path) -> tuple[Path, Path]:
        stamp = emg_path.stem.removeprefix("emg48_capture_")
        feedback_path = emg_path.parent / f"mvc_feedback_{stamp}.csv"
        calibration_path = emg_path.parent / f"mvc_calibration_{stamp}.json"
        self.mvc_csv_fp = feedback_path.open("w", newline="", encoding="utf-8")
        self.mvc_csv_writer = csv.writer(self.mvc_csv_fp)
        self.mvc_csv_writer.writerow(
            [
                "host_time",
                "timestamp_us",
                "channel_group",
                "metric_uv_rms",
                "rest_uv_rms",
                "mvc_uv_rms",
                "actual_emg_percent_mvc",
                "target_percent_mvc",
                "tolerance_percent_points",
                "calibration_phase",
                "trial_number",
            ]
        )
        self.mvc_last_recorded_timestamp = None
        return feedback_path, calibration_path

    def _mvc_metadata(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "feedback_kind": "EMG amplitude normalized to MVC; not force or torque",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "channel_group": (
                f"CH{self.mvc_tracker.group_start + 1:02d}-"
                f"CH{self.mvc_tracker.group_start + MVC_GROUP_SIZE:02d}"
            ),
            "ads_gain": self.ads_gain,
            "sample_rate_hz": self.mvc_tracker.sample_rate,
            "filter": {
                "bandpass_hz": [20.0, 450.0],
                "notch_hz": 50.0,
                "notch_q": 30.0,
            },
            "rms_window_ms": self.mvc_tracker.rms_window_ms,
            "update_ms": self.mvc_update_ms,
            "spatial_aggregation": "median of 16 per-channel RMS values",
            "target_percent_mvc": self.mvc_tracker.target_percent,
            "tolerance_percent_points": self.mvc_tracker.tolerance_percent,
            "calibration": self.mvc_tracker.calibration.metadata(),
        }

    def _stop_mvc_recording(self) -> None:
        paths = self.current_mvc_recording_paths
        if self.mvc_csv_fp is not None:
            self.mvc_csv_fp.flush()
            self.mvc_csv_fp.close()
        self.mvc_csv_fp = None
        self.mvc_csv_writer = None
        self.mvc_last_recorded_timestamp = None
        if paths is not None:
            _feedback_path, calibration_path = paths
            calibration_path.write_text(
                json.dumps(self._mvc_metadata(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    def _analyze_recording_files(
        self,
        emg_path: Path,
        imu_path: Path,
        sync_path: Path,
        mvc_feedback_path: Path | None,
        mvc_calibration_path: Path | None,
    ) -> None:
        try:
            project_root = PROJECT_ROOT
            raw_path = emg_path.with_suffix(".bin")
            if raw_path.exists():
                self._convert_emg48_raw_to_csv(raw_path, emg_path)
            summaries = [self._analyze_emg48_csv(emg_path)]
            imu_summary = self._analyze_imu_csv(project_root, imu_path)
            if imu_summary:
                summaries.append(imu_summary)
            if mvc_feedback_path is not None:
                mvc_summary = self._analyze_mvc_feedback_csv(mvc_feedback_path)
                if mvc_summary:
                    summaries.append(mvc_summary)
            emg_waveform = self._generate_emg48_waveform(project_root, emg_path)
            imu_waveform = self._generate_imu_waveform(project_root, imu_path)
            waveform_parts = [part for part in (emg_waveform, imu_waveform) if part]
            summary_text = "\n".join(part for part in summaries if part)
            waveform_text = "\n".join(waveform_parts)
            self.analysis_status_text = f"录制分析完成:\n{summary_text}"
            if waveform_text:
                self.analysis_status_text += f"\n{waveform_text}"
            self.analysis_status_until = time.time() + 180.0
        except Exception as exc:
            self.analysis_status_text = f"录制分析异常: {type(exc).__name__}: {exc}"
            self.analysis_status_until = time.time() + 90.0

    def _analyze_mvc_feedback_csv(self, path: Path) -> str:
        if not path.exists() or path.stat().st_size == 0:
            return "EMG %MVC反馈：无有效记录"
        groups: dict[float, list[tuple[float, float, float]]] = {}
        with path.open("r", newline="", encoding="utf-8") as fp:
            for row in csv.DictReader(fp):
                try:
                    actual = float(row["actual_emg_percent_mvc"])
                    target = float(row["target_percent_mvc"])
                    tolerance = float(row["tolerance_percent_points"])
                    host_time = float(row["host_time"])
                except (KeyError, TypeError, ValueError):
                    continue
                groups.setdefault(target, []).append((host_time, actual, tolerance))
        if not groups:
            return "EMG %MVC反馈：录制期间尚未完成标定"
        lines = ["EMG %MVC反馈："]
        for target in sorted(groups):
            rows = groups[target]
            actual = np.asarray([row[1] for row in rows], dtype=np.float64)
            inside = np.asarray([abs(row[1] - target) <= row[2] for row in rows], dtype=bool)
            longest = 0.0
            run_start: float | None = None
            previous_time: float | None = None
            for (host_time, _value, _tolerance), is_inside in zip(rows, inside):
                if is_inside:
                    if run_start is None or (previous_time is not None and host_time - previous_time > self.mvc_tracker.update_seconds * 3):
                        run_start = host_time
                    longest = max(longest, host_time - run_start)
                else:
                    run_start = None
                previous_time = host_time
            lines.append(
                f"  目标 {target:.0f}%：实际 {np.mean(actual):.1f}% ± {np.std(actual):.1f}%，"
                f"带内 {np.mean(inside) * 100.0:.1f}%，最长保持 {longest:.1f}s"
            )
        return "\n".join(lines)

    def _iter_emg48_frame_samples(self, frame: bytes):
        sample_count = frame[3]
        base_timestamp_us, base_sample_seq, base_drdy_count, flags, _reserved = EMG48_BATCH_META_STRUCT.unpack(
            frame[PC_EMG48_HEADER_SIZE : PC_EMG48_HEADER_SIZE + EMG48_BATCH_META_SIZE]
        )
        payload_offset = PC_EMG48_HEADER_SIZE + EMG48_BATCH_META_SIZE
        host_rx_time = time.time()
        for sample_index in range(sample_count):
            sample_bytes = frame[payload_offset : payload_offset + EMG48_SAMPLE_SIZE]
            timestamp_us = (base_timestamp_us + sample_index * EMG48_SAMPLE_PERIOD_US) & 0xFFFFFFFF
            sample_seq = (base_sample_seq + sample_index) & 0xFFFFFFFF
            drdy_count = (base_drdy_count + sample_index) & 0xFFFFFFFF
            channels = [
                decode_i24_le(sample_bytes, ch * EMG48_CHANNEL_WIRE_SIZE)
                for ch in range(EMG48_CHANNEL_COUNT)
            ]
            yield [f"{host_rx_time:.6f}", timestamp_us, sample_seq, drdy_count, flags, *channels]
            payload_offset += EMG48_SAMPLE_SIZE

    def _convert_emg48_raw_to_csv(self, raw_path: Path, emg_path: Path) -> None:
        raw = raw_path.read_bytes()
        offset = 0
        with emg_path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(["host_rx_time", "timestamp_us", "sample_seq", "drdy_count", "flags", *[f"ch{i:02d}" for i in range(1, 49)]])
            while offset + MIN_FRAME_SIZE <= len(raw):
                if raw[offset : offset + 2] != FRAME_HEADER or raw[offset + 2] != PC_MSG_EMG48_BATCH:
                    next_header = raw.find(FRAME_HEADER, offset + 1)
                    if next_header < 0:
                        break
                    offset = next_header
                    continue
                sample_count = raw[offset + 3]
                if sample_count == 0 or sample_count > EMG48_MAX_BATCH_SAMPLES:
                    offset += 1
                    continue
                packet_size = (
                    PC_EMG48_HEADER_SIZE
                    + EMG48_BATCH_META_SIZE
                    + sample_count * EMG48_SAMPLE_SIZE
                    + 2
                )
                if offset + packet_size > len(raw):
                    break
                frame = raw[offset : offset + packet_size]
                if int.from_bytes(frame[-2:], "little") == crc16_ccitt(frame[:-2]):
                    for row in self._iter_emg48_frame_samples(frame):
                        writer.writerow(row)
                    offset += packet_size
                else:
                    offset += 1

    def _analyze_emg48_csv(self, emg_path: Path) -> str:
        if not emg_path.exists() or emg_path.stat().st_size == 0:
            return "EMG48 无文件"
        rows: list[tuple[int, int, int, int]] = []
        with emg_path.open("r", newline="", encoding="utf-8") as fp:
            reader = csv.DictReader(fp)
            for row in reader:
                try:
                    rows.append((
                        int(row["sample_seq"]),
                        int(row["drdy_count"]),
                        int(row["timestamp_us"]),
                        int(row.get("flags", "0")),
                    ))
                except (KeyError, TypeError, ValueError):
                    continue
        if len(rows) < 2:
            return f"EMG48 样本不足 {len(rows)}"
        rows.sort(key=lambda item: item[0])
        deduped: list[tuple[int, int, int, int]] = []
        duplicate_rows = 0
        last_seq: int | None = None
        for row in rows:
            if last_seq is not None and row[0] == last_seq:
                duplicate_rows += 1
                continue
            deduped.append(row)
            last_seq = row[0]
        sample_seq = [row[0] for row in deduped]
        drdy_count = [row[1] for row in deduped]
        timestamp_us = [row[2] for row in deduped]
        flags = [row[3] for row in deduped]
        seq_lost = sum(max(0, b - a - 1) for a, b in zip(sample_seq, sample_seq[1:]))
        drdy_gaps = sum(max(0, b - a - 1) for a, b in zip(drdy_count, drdy_count[1:]))
        duration_s = (timestamp_us[-1] - timestamp_us[0]) / 1_000_000.0
        handled_hz = (sample_seq[-1] - sample_seq[0]) / duration_s if duration_s > 0 else 0.0
        drdy_hz = (drdy_count[-1] - drdy_count[0]) / duration_s if duration_s > 0 else 0.0
        ts_deltas = np.diff(np.asarray(timestamp_us, dtype=np.int64)) / 1000.0
        ts_mean = float(np.nanmean(ts_deltas)) if ts_deltas.size else 0.0
        ts_std = float(np.nanstd(ts_deltas)) if ts_deltas.size else 0.0
        seq_loss_rate = seq_lost / (len(sample_seq) + seq_lost) if (len(sample_seq) + seq_lost) > 0 else 0.0
        drdy_gap_rate = drdy_gaps / (len(sample_seq) + drdy_gaps) if (len(sample_seq) + drdy_gaps) > 0 else 0.0
        flag_or = 0
        overrun_flags = 0
        queue_drop_flags = 0
        config_fail_flags = 0
        for value in flags:
            flag_or |= value
            overrun_flags += 1 if value & 0x01 else 0
            queue_drop_flags += 1 if value & 0x02 else 0
            config_fail_flags += 1 if value & 0x04 else 0
        return (
            f"EMG48:\n"
            f"  总点 {len(sample_seq)}，重复 {duplicate_rows}\n"
            f"  录制后仍缺 sample_seq {seq_lost} ({seq_loss_rate:.3%})\n"
            f"  录制后仍缺 drdy_count {drdy_gaps} ({drdy_gap_rate:.3%})\n"
            f"  handled {handled_hz:.1f} Hz, drdy {drdy_hz:.1f} Hz\n"
            f"  timestamp {ts_mean:.3f} ms ± {ts_std:.3f} ms\n"
            f"  flags OR 0x{flag_or:02X}, overrun {overrun_flags}, qdrop {queue_drop_flags}, cfgfail {config_fail_flags}"
        )

    def _analyze_imu_csv(self, project_root: Path, imu_path: Path) -> str:
        if not imu_path.exists() or imu_path.stat().st_size == 0:
            return ""
        try:
            with imu_path.open("r", encoding="utf-8") as fp:
                line_count = max(0, sum(1 for _ in fp) - 1)
            if line_count == 0:
                return "IMU 无样本"
            result = subprocess.run(
                [sys.executable, str(project_root / "tools" / "analyze_data.py"), str(imu_path)],
                cwd=project_root,
                text=True,
                capture_output=True,
                timeout=60,
                check=False,
            )
            if result.returncode != 0:
                detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
                return f"IMU 分析失败 {detail[-120:]}"
            useful = []
            for line in result.stdout.splitlines():
                stripped = line.strip()
                if stripped.startswith("===") or stripped.startswith("总接收包数") or stripped.startswith("丢包率"):
                    useful.append(stripped)
            return "IMU:\n" + "\n".join(useful[:8]) if useful else "IMU 分析完成"
        except Exception as exc:
            return f"IMU 分析异常 {type(exc).__name__}: {exc}"

    def _recording_waveform_path(self, csv_path: Path) -> Path:
        try:
            relative_csv = csv_path.resolve().relative_to(self.csv_dir)
        except ValueError:
            relative_csv = Path(csv_path.name)
        return self.waveform_dir / relative_csv.with_suffix(".png")

    def _generate_emg48_waveform(self, project_root: Path, emg_path: Path) -> str:
        if not emg_path.exists() or emg_path.stat().st_size == 0:
            return ""
        waveform_script = project_root / "tools" / "view_emg48.py"
        waveform_path = self._recording_waveform_path(emg_path)
        waveform_path.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [
                sys.executable,
                str(waveform_script),
                str(emg_path),
                "--max-points",
                "80000",
                "--save",
                str(waveform_path),
                "--no-show",
            ],
            cwd=project_root,
            text=True,
            capture_output=True,
            timeout=120,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
            return f"EMG48 波形生成失败 {detail[-120:]}"
        self._open_waveform_image(waveform_path)
        return f"EMG48 波形已打开 {waveform_path.name}"

    def _generate_imu_waveform(self, project_root: Path, imu_path: Path) -> str:
        if not imu_path.exists() or imu_path.stat().st_size == 0:
            return ""
        try:
            with imu_path.open("r", encoding="utf-8") as fp:
                if sum(1 for _ in fp) <= 1:
                    return ""
        except OSError:
            return ""
        waveform_script = project_root / "tools" / "view_waveform.py"
        if not waveform_script.exists():
            return ""
        waveform_path = self._recording_waveform_path(imu_path)
        waveform_path.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [
                sys.executable,
                str(waveform_script),
                str(imu_path),
                "--channels",
                "x,y,z,vector_norm",
                "--max-points",
                "80000",
                "--save",
                str(waveform_path),
                "--no-show",
            ],
            cwd=project_root,
            text=True,
            capture_output=True,
            timeout=120,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
            return f"IMU 波形生成失败 {detail[-120:]}"
        self._open_waveform_image(waveform_path)
        return f"IMU 波形已打开 {waveform_path.name}"

    def _open_waveform_image(self, waveform_path: Path) -> None:
        try:
            if sys.platform == "darwin":
                subprocess.Popen(["open", str(waveform_path)])
            elif os.name == "nt":
                os.startfile(waveform_path)  # type: ignore[attr-defined]
            else:
                subprocess.Popen(["xdg-open", str(waveform_path)])
        except Exception:
            pass

    def shutdown(self) -> None:
        if hasattr(self, "port_scan_timer"):
            self.port_scan_timer.stop()
        if self.reader is not None:
            self.disconnect_serial("已关闭")
        else:
            self._finish_recording()
            self.model.stop_recording()
            self._stop_mvc_recording()

    def closeEvent(self, event) -> None:
        self.shutdown()
        super().closeEvent(event)


def main() -> int:
    args = parse_args()
    if args.list_ports:
        list_ports()
        return 0
    if args.ads_gain <= 0:
        print("--ads-gain 必须大于 0。", file=sys.stderr)
        return 2

    max_points = max(100, int(args.window_seconds * args.expected_rate * 1.2))
    model = AcquisitionModel(max_points=max_points)
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=False, foreground="#EAEAEA", background="#101418")
    window = MainWindow(
        model,
        args.csv_dir,
        args.port,
        args.baud,
        args.window_seconds,
        args.emg_display_rate,
        args.plot_interval_ms,
        args.ads_gain,
        args.mvc_rms_window_ms,
        args.mvc_update_ms,
        args.mvc_tolerance,
        args.enable_resend,
        args.live_decimation,
    )
    window.show()

    exit_code = 0
    try:
        exit_code = app.exec()
    finally:
        window.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
