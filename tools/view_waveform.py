#!/usr/bin/env python3
"""
离线查看 imu_capture_*.csv 波形。

示例：
  pixi run python tools/view_waveform.py data/data_0711_lab/s01/right/imu_capture_20260711_100606.csv
  pixi run python tools/view_waveform.py data/data_0711_lab/s01/right/imu_capture_20260711_100606.csv --source EMG --demean
  pixi run python tools/view_waveform.py data/data_0711_lab/s01/right/imu_capture_20260711_100606.csv --start 30 --end 40 --channels x,y,z,vector_norm
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_CHANNELS = ("x", "y", "z")
PLOTTABLE_CHANNELS = ("x", "y", "z", "vector_norm")
EMG_SCALE_FACTOR_UV = 2.4 * 1_000_000.0 / (12.0 * (2**23 - 1))


def split_csv_values(values: list[str] | None) -> list[str]:
    if not values:
        return []
    result: list[str] = []
    for value in values:
        result.extend(item.strip() for item in value.split(",") if item.strip())
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="离线查看无线采集系统保存的 CSV 波形。")
    parser.add_argument("csv_file", type=Path, help="要查看的 imu_capture_*.csv 文件。")
    parser.add_argument(
        "--source",
        action="append",
        help="只显示指定 source/source_id，可重复或用逗号分隔，例如 --source EMG,Gyro 或 --source 0。",
    )
    parser.add_argument(
        "--channels",
        default=",".join(DEFAULT_CHANNELS),
        help="要显示的列，默认 x,y,z；可选 x,y,z,vector_norm。",
    )
    parser.add_argument("--start", type=float, help="开始时间，单位秒；相对本文件第一条 timestamp_us。")
    parser.add_argument("--end", type=float, help="结束时间，单位秒；相对本文件第一条 timestamp_us。")
    parser.add_argument("--max-points", type=int, default=80_000, help="每个 source 最多绘制点数，默认 80000。")
    parser.add_argument("--demean", action="store_true", help="每条曲线减去自身均值，便于观察交流波形。")
    parser.add_argument("--emg-uv", action="store_true", help="将 EMG 的 x/y/z 原始计数换算为 uV 后显示。")
    parser.add_argument("--list", action="store_true", help="只打印文件摘要和可用 source/列，不打开窗口。")
    parser.add_argument("--save", type=Path, help="保存图片路径，例如 outputs/waveform.png。")
    parser.add_argument("--no-show", action="store_true", help="不弹出窗口，适合配合 --save 使用。")
    parser.add_argument("--title", help="图标题；默认使用 CSV 文件名。")
    return parser.parse_args()


def read_capture_csv(csv_file: Path) -> pd.DataFrame:
    if not csv_file.exists():
        raise FileNotFoundError(f"找不到文件: {csv_file}")

    df = pd.read_csv(csv_file)
    required = {"source", "source_id", "timestamp_us"}
    missing = required - set(df.columns)
    if missing:
        missing_text = ", ".join(sorted(missing))
        raise ValueError(f"CSV 缺少必要列: {missing_text}")

    for column in ("source_id", "timestamp_us", *PLOTTABLE_CHANNELS):
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

    df = df.dropna(subset=["timestamp_us"]).copy()
    if df.empty:
        raise ValueError("CSV 中没有有效 timestamp_us 数据。")
    return df


def print_summary(df: pd.DataFrame, csv_file: Path) -> None:
    print(f"文件: {csv_file}")
    print(f"行数: {len(df):,}")
    print("列:", ", ".join(df.columns))
    print()
    print("source 摘要:")
    for (source, source_id), group in df.groupby(["source", "source_id"], sort=False):
        timestamps = group["timestamp_us"].to_numpy(dtype=np.float64)
        duration_s = (timestamps[-1] - timestamps[0]) / 1_000_000.0 if len(timestamps) > 1 else 0.0
        diffs = np.diff(np.sort(timestamps))
        median_dt_us = float(np.nanmedian(diffs)) if len(diffs) else math.nan
        rate = 1_000_000.0 / median_dt_us if median_dt_us and not math.isnan(median_dt_us) else math.nan
        rate_text = f"{rate:.1f} Hz" if not math.isnan(rate) else "n/a"
        print(f"  {source} (id={int(source_id)}): {len(group):,} 点, {duration_s:.3f} s, 约 {rate_text}")


def filter_sources(df: pd.DataFrame, requested_sources: list[str]) -> pd.DataFrame:
    if not requested_sources:
        return df

    source_text = df["source"].astype(str).str.lower()
    source_id_text = df["source_id"].astype("Int64").astype(str).str.lower()
    mask = pd.Series(False, index=df.index)
    for item in requested_sources:
        key = item.lower()
        mask |= source_text.eq(key) | source_id_text.eq(key)

    filtered = df[mask].copy()
    if filtered.empty:
        available = sorted({str(value) for value in df["source"].dropna().unique()})
        available_ids = sorted({str(int(value)) for value in df["source_id"].dropna().unique()})
        raise ValueError(
            f"没有匹配的 source: {', '.join(requested_sources)}；"
            f"可用 source={available}, source_id={available_ids}"
        )
    return filtered


def filter_time_range(df: pd.DataFrame, start: float | None, end: float | None) -> pd.DataFrame:
    if start is None and end is None:
        return df
    if start is not None and end is not None and end <= start:
        raise ValueError("--end 必须大于 --start。")

    t0_us = float(df["timestamp_us"].min())
    elapsed_s = (df["timestamp_us"] - t0_us) / 1_000_000.0
    mask = pd.Series(True, index=df.index)
    if start is not None:
        mask &= elapsed_s >= start
    if end is not None:
        mask &= elapsed_s <= end
    filtered = df[mask].copy()
    if filtered.empty:
        raise ValueError("指定时间范围内没有数据。")
    return filtered


def downsample(group: pd.DataFrame, max_points: int) -> pd.DataFrame:
    if max_points <= 0 or len(group) <= max_points:
        return group
    step = int(math.ceil(len(group) / max_points))
    return group.iloc[::step].copy()


def selected_channels(channel_arg: str, df: pd.DataFrame) -> list[str]:
    channels = [item.strip() for item in channel_arg.split(",") if item.strip()]
    invalid = [item for item in channels if item not in PLOTTABLE_CHANNELS]
    if invalid:
        raise ValueError(f"不支持的通道: {', '.join(invalid)}；可选: {', '.join(PLOTTABLE_CHANNELS)}")
    missing = [item for item in channels if item not in df.columns]
    if missing:
        raise ValueError(f"CSV 中没有这些通道列: {', '.join(missing)}")
    if not channels:
        raise ValueError("至少需要指定一个通道。")
    return channels


def plot_waveforms(df: pd.DataFrame, csv_file: Path, args: argparse.Namespace) -> None:
    channels = selected_channels(args.channels, df)
    t0_us = float(df["timestamp_us"].min())
    source_groups = list(df.groupby(["source", "source_id"], sort=False))
    fig_height = max(3.5, 2.7 * len(source_groups))
    fig, axes = plt.subplots(len(source_groups), 1, sharex=True, figsize=(13, fig_height), squeeze=False)
    colors = {
        "x": "#d62728",
        "y": "#2ca02c",
        "z": "#1f77b4",
        "vector_norm": "#6f42c1",
    }

    for ax, ((source, source_id), group) in zip(axes[:, 0], source_groups, strict=True):
        group = group.sort_values("timestamp_us")
        plot_group = downsample(group, args.max_points)
        time_s = (plot_group["timestamp_us"].to_numpy(dtype=np.float64) - t0_us) / 1_000_000.0
        source_name = str(source)

        for channel in channels:
            values = plot_group[channel].to_numpy(dtype=np.float64)
            label = channel
            if args.emg_uv and source_name.upper() == "EMG" and channel in {"x", "y", "z"}:
                values = values * EMG_SCALE_FACTOR_UV
                label = f"{channel} (uV)"
            if args.demean:
                values = values - np.nanmean(values)
                label = f"{label} - mean"
            ax.plot(time_s, values, linewidth=0.9, color=colors.get(channel), label=label)

        title = f"{source_name} (id={int(source_id)}, {len(group):,} points"
        if len(plot_group) != len(group):
            title += f", plotted {len(plot_group):,}"
        title += ")"
        ax.set_title(title, loc="left", fontsize=10)
        ax.set_ylabel("value")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", ncols=min(len(channels), 4), fontsize=9)

    axes[-1, 0].set_xlabel("time (s)")
    fig.suptitle(args.title or csv_file.name, fontsize=13)
    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=180)
        print(f"已保存图片: {args.save}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    args = parse_args()
    requested_sources = split_csv_values(args.source)

    try:
        df = read_capture_csv(args.csv_file)
        print_summary(df, args.csv_file)
        if args.list:
            return 0

        df = filter_sources(df, requested_sources)
        df = filter_time_range(df, args.start, args.end)
        plot_waveforms(df, args.csv_file, args)
    except Exception as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
