"""CSV/MAT conversion and zero-lag EMG correlation for csv_mat_gui.py. No GUI imports."""
from __future__ import annotations

import csv
import hashlib
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy import signal
from scipy.io import savemat


@dataclass
class Capture:
    path: Path
    columns: dict[str, np.ndarray]
    kind: str
    channels: list[str]
    time_s: np.ndarray
    streams: list[dict]
    warnings: list[str] = field(default_factory=list)
    sha256: str = ""

    @property
    def size(self):
        return len(next(iter(self.columns.values())))

    @property
    def fs(self):
        return self.streams[0]["fs_hz"] if len(self.streams) == 1 else float("nan")


def typed_column(values):
    """Keep integer counters exact, including integers above float64's 2**53."""
    if all(re.fullmatch(r"[+-]?\d+", s.strip()) for s in values):
        numbers = [int(s) for s in values]
        try:
            return np.asarray(numbers, dtype=np.int64)
        except OverflowError:
            try:
                return np.asarray(numbers, dtype=np.uint64)
            except OverflowError:
                return np.asarray(values, dtype=object)
    try:
        return np.asarray([float(s) if s.strip() else np.nan for s in values])
    except ValueError:
        return np.asarray(values, dtype=object)


def elapsed_time(values, microseconds=True):
    values = np.asarray(values)
    # Subtract the origin before converting integer timestamps to float.
    if values.dtype.kind in "iu":
        relative = np.asarray([int(v) - int(values[0]) for v in values], dtype=float)
    else:
        relative = values.astype(float) - float(values[0])
    delta = np.diff(relative)
    if microseconds:
        relative[1:] += np.cumsum(np.where(delta < -(2**31), 2**32, 0))
    return relative / (1e6 if microseconds else 1)


def load_capture(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open("rb") as raw:
        for chunk in iter(lambda: raw.read(1024 * 1024), b""):
            digest.update(chunk)
    encoding = "utf-8-sig"
    try:
        with path.open(encoding=encoding) as fp:
            fp.read()
    except UnicodeDecodeError:
        encoding = "gb18030"
    with path.open(newline="", encoding=encoding) as fp:
        sample = fp.read(8192)
        fp.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel
        reader = csv.reader(fp, dialect)
        header = next(reader, [])
        if not header or any(not h.strip() for h in header) or len(set(header)) != len(header):
            raise ValueError("CSV 表头为空、存在空列名或重复列名。")
        data = [[] for _ in header]
        for line, row in enumerate(reader, 2):
            if not row:
                continue
            if len(row) != len(header):
                raise ValueError(f"第 {line} 行有 {len(row)} 列，表头为 {len(header)} 列；请检查原文件。")
            for col, value in zip(data, row):
                col.append(value)
    if not data[0]:
        raise ValueError("CSV 只有表头，没有样本。")
    columns = {name: typed_column(values) for name, values in zip(header, data)}
    channels = sorted([h for h in header if re.fullmatch(r"ch\d+", h, re.I)],
                      key=lambda h: int(h[2:]))
    kind = "EMG" if channels else "IMU" if {"x", "y", "z"} <= columns.keys() else "TABLE"
    count = len(data[0])
    time_s = np.full(count, np.nan)
    warnings = []
    stream_rows = {}
    for i in range(count):
        key = tuple(str(columns[k][i]) for k in ("source", "source_id") if k in columns)
        stream_rows.setdefault(key or (kind,), []).append(i)
    streams = []
    time_key = next((k for k in ("time_unwrapped_us", "timestamp_us", "time_s", "t_s") if k in columns), None)
    for key, indices in stream_rows.items():
        indices = np.asarray(indices, dtype=np.int64)
        fs = float("nan")
        if time_key:
            try:
                t = elapsed_time(columns[time_key][indices], time_key.endswith("_us"))
                time_s[indices] = t
                delta = np.diff(t)
                if len(delta) and np.isfinite(t).all() and np.all(delta > 0):
                    fs = float(1 / np.median(delta))
                else:
                    warnings.append(f"{' / '.join(key)}：时间戳缺失、重复或倒退；保留原行，不能自动确定采样率。")
            except (ValueError, TypeError, OverflowError):
                warnings.append(f"{' / '.join(key)}：时间列不是有效数值。")
        streams.append(dict(label=" / ".join(key), row_indices=indices + 1,
                            time_s=time_s[indices], fs_hz=fs))
    if not time_key:
        warnings.append("没有设备时间列；MAT 保留原始数据，相关性分析需手动指定采样率。")
    if kind == "TABLE":
        warnings.append("未识别为 EMG 或 IMU；按通用表完整转换。")
    return Capture(path, columns, kind, channels, time_s, streams, warnings, digest.hexdigest())


def matlab_columns(columns):
    used, mapping, result = set(), [], {}
    for name, values in columns.items():
        key = re.sub(r"[^A-Za-z0-9_]", "_", name)
        if not key or not key[0].isalpha():
            key = "col_" + key
        key = key[:55]
        base, serial = key, 2
        while key in used:
            key = f"{base}_{serial}"
            serial += 1
        used.add(key)
        mapping.append(key)
        result[key] = values
    return result, np.asarray(mapping, dtype=object)


def capture_payload(capture):
    columns, keys = matlab_columns(capture.columns)
    payload = dict(format_version="1.0", source_file=str(capture.path),
                   source_sha256=capture.sha256, data_type=capture.kind,
                   created_utc=datetime.now(timezone.utc).isoformat(),
                   column_names=np.asarray(list(capture.columns), dtype=object),
                   column_keys=keys, columns=columns,
                   time_s=capture.time_s, streams=np.asarray(capture.streams, dtype=object),
                   warnings=np.asarray(capture.warnings, dtype=object),
                   time_definition="Seconds relative to first row of each source/source_id stream; original row order.",
                   units="Original CSV values; no physical-unit calibration or resampling applied.")
    if capture.kind == "EMG":
        payload.update(emg=np.column_stack([capture.columns[c] for c in capture.channels]),
                       channel_names=np.asarray(capture.channels, dtype=object), fs_hz=capture.fs)
    elif capture.kind == "IMU":
        payload["imu_xyz"] = np.column_stack([capture.columns[c] for c in ("x", "y", "z")])
    return payload


def save_unique_mat(directory, stem, payload):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    serial = 0
    while True:
        path = directory / f"{stem}{'_' + str(serial) if serial else ''}.mat"
        try:
            fp = path.open("xb")
            break
        except FileExistsError:
            serial += 1
    try:
        with fp:
            savemat(fp, payload, do_compression=True, long_field_names=True, oned_as="column")
    except BaseException:
        path.unlink(missing_ok=True)
        raise
    return path


def active_channels(capture):
    result = []
    for name in capture.channels:
        try:
            values = capture.columns[name].astype(float)
            finite = values[np.isfinite(values)]
            if len(finite) > 1 and np.ptp(finite) > 0:
                result.append(name)
        except (ValueError, TypeError):
            pass
    return result


def analyze(capture, channels, start=0., end=None, fs_override=None,
            bandpass=False, low=20., high=450., notch="none", line_hz=50.):
    if capture.kind != "EMG" or len(capture.streams) != 1:
        raise ValueError("相关性分析需要单个数据流的 EMG chNN 宽表。")
    if len(set(channels)) < 2 or any(c not in capture.channels for c in channels):
        raise ValueError("请选择至少两个不同的 EMG 通道。")
    fs = float(fs_override) if fs_override is not None else capture.fs
    if not np.isfinite(fs) or fs <= 0:
        raise ValueError("无法确定采样率，请填写正确的采样率 Hz。")
    t = capture.time_s.copy()
    warnings = list(capture.warnings)
    if not np.isfinite(t).all():
        if fs_override is None:
            raise ValueError("时间列无效，需要手动指定采样率。")
        t = np.arange(capture.size) / fs
        warnings.append("使用手动采样率生成时间轴，未验证原始采样连续性。")
    if not np.all(np.diff(t) > 0):
        raise ValueError("时间戳重复或倒退；请分离重复采集会话后重试。")
    if not np.isfinite(start) or start < 0 or (end is not None and (not np.isfinite(end) or end <= start)):
        raise ValueError("时间范围要求 0 ≤ 起点 < 终点。")
    mask = (t >= start) & (t < end if end is not None else True)
    if mask.sum() < 3:
        raise ValueError("所选时间范围不足 3 个样本。")
    raw = np.column_stack([capture.columns[c].astype(float) for c in channels])
    filtered = bandpass or notch != "none"
    step_error = np.abs(np.diff(t) * fs - 1)
    bad_steps = step_error > .05
    if np.any(bad_steps):
        warnings.append(f"有 {int(bad_steps.sum())} 处采样间隔偏离 1/fs 超过 5%。")
        if filtered and np.any(step_error > .1):
            raise ValueError("采样间隔偏离 1/fs 超过 10%，存在间断或采样率不匹配；可使用原始相关性或先截取连续 CSV。")
        if filtered:
            warnings.append("时间间隔抖动在 10% 内；滤波按样本顺序和所用采样率近似等间隔处理，未重采样。")
    if "sample_seq" in capture.columns:
        seq = capture.columns["sample_seq"]
        try:
            bad_seq = np.asarray([(int(b) - int(a)) % 2**32 != 1 for a, b in zip(seq[:-1], seq[1:])])
            if np.any(bad_seq):
                warnings.append(f"sample_seq 有 {int(bad_seq.sum())} 处不连续，原始行未补点。")
                if filtered:
                    raise ValueError("sample_seq 不连续，不能直接对整段滤波。")
        except (TypeError, OverflowError):
            raise ValueError("sample_seq 不是有效整数。")
    if not np.isfinite(raw if filtered else raw[mask]).all():
        raise ValueError("所需数据含 NaN/Inf 或非数值；不会自动填补或丢弃样本。")
    valid = np.ptp(raw[mask], axis=0) > 0
    excluded = [c for c, ok in zip(channels, valid) if not ok]
    channels = [c for c, ok in zip(channels, valid) if ok]
    raw = raw[:, valid]
    if len(channels) < 2:
        raise ValueError("排除所选区间内的恒定通道后，不足两个通道。")
    rail = np.sum((raw[mask] >= 8388607) | (raw[mask] <= -8388608), axis=0)
    if rail.any():
        warnings.append("检测到 ADS1298 满量程值：" + ", ".join(f"{c}={n}" for c, n in zip(channels, rail) if n))
    y = raw.copy()
    filters = []
    if bandpass:
        if not 0 < low < high < fs / 2:
            raise ValueError("带通范围需满足 0 < 下限 < 上限 < 采样率/2。")
        try:
            y = signal.sosfiltfilt(signal.butter(4, [low, high], btype="bandpass", fs=fs, output="sos"), y, axis=0)
        except ValueError as exc:
            raise ValueError("记录太短，无法进行零相位带通滤波。") from exc
        filters.append(f"Butterworth order 4, {low:g}-{high:g} Hz, zero phase")
    if notch not in ("none", "fundamental", "harmonics"):
        raise ValueError("未知陷波模式。")
    if notch != "none":
        if not 0 < line_hz < fs / 2:
            raise ValueError("工频需大于 0 且小于采样率/2。")
        limit = min(high if bandpass else fs / 2, fs / 2)
        frequencies = [line_hz] if notch == "fundamental" else np.arange(line_hz, limit, line_hz)
        for hz in frequencies:
            b, a = signal.iirnotch(hz, 30, fs)
            try:
                y = signal.filtfilt(b, a, y, axis=0)
            except ValueError as exc:
                raise ValueError("记录太短，无法进行零相位陷波。") from exc
            filters.append(f"Notch {hz:g} Hz Q=30, zero phase")
    y = y[mask]
    if not np.isfinite(y).all() or np.any(np.std(y, axis=0) == 0):
        raise ValueError("处理后有非有限值或恒定通道，无法计算 Pearson r。")
    corr = np.corrcoef(y, rowvar=False)
    pairs = corr[np.triu_indices(len(channels), 1)]
    return dict(channel_names=np.asarray(channels, dtype=object), correlation=corr,
                excluded_constant_channels=np.asarray(excluded, dtype=object),
                time_s=t[mask], processed_emg=y, fs_hz=fs,
                requested_start_s=start, requested_end_s=float(end) if end is not None else np.nan,
                sample_count=int(mask.sum()), mean_r=float(np.mean(pairs)),
                mean_abs_r=float(np.mean(np.abs(pairs))), min_r=float(pairs.min()), max_r=float(pairs.max()),
                filters=np.asarray(filters or ["Raw CSV values"], dtype=object),
                warnings=np.asarray(warnings, dtype=object),
                method="Unrectified zero-lag Pearson; unique off-diagonal pairs. Filter full recording before window [start,end).",
                sampling_assumption="Filtering assumes uniform samples at fs_hz; timestamp jitter up to 10% tolerated and over 5% reported; no resampling.",
                source_file=str(capture.path), source_sha256=capture.sha256)
