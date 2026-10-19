#!/usr/bin/env python3
"""Offline viewer for emg48_capture_*.csv files."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CHANNELS = [f"ch{i:02d}" for i in range(1, 49)]


def parse_channels(text: str | None) -> list[str]:
    if not text:
        return CHANNELS
    result: list[str] = []
    for item in text.split(','):
        item = item.strip().lower()
        if not item:
            continue
        name = item if item.startswith('ch') else f"ch{int(item):02d}"
        if name not in CHANNELS:
            raise ValueError(f"bad channel: {item}")
        result.append(name)
    return result or CHANNELS


def main() -> int:
    parser = argparse.ArgumentParser(description="View EMG48 wide CSV waveforms.")
    parser.add_argument('csv_file', type=Path)
    parser.add_argument('--channels', help='Comma list such as ch01,ch08,17. Default: all 48.')
    parser.add_argument('--start', type=float, help='Start time in seconds relative to first row.')
    parser.add_argument('--end', type=float, help='End time in seconds relative to first row.')
    parser.add_argument('--max-points', type=int, default=80000)
    parser.add_argument('--demean', action='store_true')
    parser.add_argument('--save', type=Path)
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    channels = parse_channels(args.channels)
    missing = [ch for ch in channels if ch not in df.columns]
    if missing:
        raise SystemExit(f"Missing columns: {missing}")
    t = (pd.to_numeric(df['timestamp_us']) - pd.to_numeric(df['timestamp_us']).iloc[0]) / 1_000_000.0
    mask = np.ones(len(df), dtype=bool)
    if args.start is not None:
        mask &= t >= args.start
    if args.end is not None:
        mask &= t <= args.end
    df = df.loc[mask].copy()
    t = t.loc[mask].to_numpy(dtype=float)
    if len(df) > args.max_points:
        step = int(np.ceil(len(df) / args.max_points))
        df = df.iloc[::step]
        t = t[::step]

    rows = int(np.ceil(len(channels) / 8))
    cols = min(8, len(channels))
    fig, axes = plt.subplots(rows, cols, figsize=(16, max(3, 2.1 * rows)), squeeze=False, sharex=True)
    for ax in axes.ravel():
        ax.set_visible(False)
    for i, ch in enumerate(channels):
        ax = axes[i // cols][i % cols]
        ax.set_visible(True)
        y = pd.to_numeric(df[ch]).to_numpy(dtype=float)
        if args.demean:
            y = y - np.nanmean(y)
        ax.plot(t, y, linewidth=0.7)
        ax.set_title(ch)
        ax.grid(True, alpha=0.25)
    fig.suptitle(args.csv_file.name)
    fig.tight_layout()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=180)
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
