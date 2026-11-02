#!/usr/bin/env python3
"""Plot existing reflex analysis and acceleration on matched device-time axes."""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/reflex-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
GROUP_COLORS = ['#217da5', '#d78520', '#8058b0']
AXIS_COLORS = ['#db675c', '#3c9d85', '#7088ca']


def draw_pair(ax_acc, ax_emg, rel_acc, xyz, rel_emg, live, row,
              limits, detail=False):
    baseline = (rel_acc >= -60) & (rel_acc < -20)
    assert baseline.sum() >= 10
    centered = xyz - np.median(xyz[baseline], axis=0)
    if detail:
        for c, label in enumerate(['X', 'Y', 'Z']):
            ax_acc.plot(rel_acc, centered[:, c], color=AXIS_COLORS[c],
                        lw=.7, alpha=.8, label=f'{label} - local baseline')
    ax_acc.plot(rel_acc, np.linalg.norm(centered, axis=1), color='#333333',
                lw=.9, label='Dynamic vector magnitude')
    for g in range(3):
        ax_emg.plot(rel_emg, np.median(live[:, g*16:(g+1)*16], axis=1),
                    lw=.8, color=GROUP_COLORS[g],
                    label=f'CH{g*16+1:02d}-{(g+1)*16:02d} median')
    for ax in (ax_acc, ax_emg):
        ax.axvline(0, color='black', lw=.8, label='Contact candidate')
        ax.axvline(row.emg_onset_estimate_ms, color='#bb3939', lw=.8,
                   ls='--', label='Automatic onset candidate')
        ax.axvline(row.raw_major_peak_delay_ms, color='#2f9662', lw=.8,
                   ls=':', label='Raw EMG main-peak reference')
        ax.set_xlim(*limits)
        ax.grid(alpha=.18)
        if row.motor_event == 'ABORT':
            ax.set_facecolor('#fff0e9')
    ax_acc.set_ylabel('Accel counts')
    ax_emg.set_ylabel('Filtered EMG counts')
    ax_emg.set_xlabel('Device time from contact candidate (ms)')
    ax_acc.tick_params(labelbottom=False)


def run(record):
    source = ROOT / 'outputs' / f'reflex_latency_{record}'
    out = source / 'accel_emg_comparison'
    out.mkdir(exist_ok=True)
    info = json.loads((source / 'analysis.json').read_text())
    windows = np.load(source / 'reflex_windows.npz')
    rows = pd.read_csv(source / 'latency_summary.csv')
    # Verify the very same recording used for the existing latency estimates.
    for key, path in info['source_paths'].items():
        actual = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        assert actual == info['sha256_before'][key], key
    imu = pd.read_csv(info['source_paths']['imu'])
    imu = imu[imu.source == 'Accel']
    emg_start = int(pd.read_csv(info['source_paths']['emg'], nrows=1).timestamp_us.iloc[0])
    accel_time = (imu.timestamp_us.to_numpy(np.int64) - emg_start) / 1e6
    xyz = imu[['x', 'y', 'z']].to_numpy(float)
    plt.rcParams.update({'font.size':9, 'axes.spines.top':False,
                         'axes.spines.right':False})
    overview = plt.figure(figsize=(17, 24), layout='constrained')
    outer = overview.add_gridspec(7, 2, hspace=.12)
    metrics = []
    for index, row in enumerate(rows.itertuples(index=False)):
        tag = f'shot{row.shot:02d}'
        contact = row.contact_candidate_s
        mask = (accel_time >= contact-.201) & (accel_time <= contact+.801)
        ra = (accel_time[mask] - contact) * 1000
        re = windows[tag+'_rel'] * 1000
        live = windows[tag+'_live']
        # Preserve the two native sample grids; no resampling or host-time shift.
        nested = outer[index//2, index%2].subgridspec(2, 1, hspace=.02,
                                                     height_ratios=[1, 1.5])
        aa = overview.add_subplot(nested[0])
        ae = overview.add_subplot(nested[1], sharex=aa)
        draw_pair(aa, ae, ra, xyz[mask], re, live, row, (-50, 300))
        aa.set_title(f'Shot {row.shot:02d} | {row.motor_event} | onset candidate {row.emg_onset_estimate_ms:.1f} ms',
                     fontsize=10)
        if index == 0:
            ae.legend(loc='upper right', fontsize=6, ncol=2)
        detail, axes = plt.subplots(2, 2, figsize=(15, 7), sharex='col',
                                    gridspec_kw={'height_ratios':[1, 1.5]},
                                    layout='constrained')
        for column, limits in enumerate([(-200, 800), (-20, 100)]):
            draw_pair(axes[0, column], axes[1, column], ra, xyz[mask],
                      re, live, row, limits, detail=True)
            axes[0, column].set_title('Full response and tail' if column == 0 else 'Contact and EMG onset zoom')
        axes[0, 0].legend(fontsize=7, ncol=2, loc='upper right')
        axes[1, 0].legend(fontsize=7, ncol=2, loc='upper right')
        detail.suptitle(f'Shot {row.shot:02d} | {row.motor_event} | contact candidate {contact:.6f} s\n'
                        f'Black: contact candidate; red dashed: automatic onset {row.emg_onset_estimate_ms:.1f} ms; '
                        f'green dotted: RAW EMG main peak {row.raw_major_peak_delay_ms:.1f} ms\n'
                        'EMG: causal 20-450 Hz + 50 Hz notch; group medians; onset uses separate detector; timing uncalibrated', fontsize=10)
        detail.savefig(out / f'shot_{row.shot:02d}_accel_emg.png', dpi=160)
        plt.close(detail)
        metrics.append({'shot':int(row.shot), 'motor_event':row.motor_event,
                        'accel_samples':int(mask.sum()), 'emg_samples':len(re),
                        'native_time_grids_preserved':True})
    overview.suptitle(f'Acceleration and filtered EMG on matched device-time axes | {record}\n'
                      'Each pair: acceleration above, EMG below. Black: contact candidate; red: automatic onset; green: raw EMG main-peak reference.\n'
                      'All 13 events retained; orange background = ABORT. Units are uncalibrated counts. No filter-delay compensation.', fontsize=12)
    overview.savefig(out / 'all_trials_accel_emg.png', dpi=160)
    plt.close(overview)
    lines = ['# 加速度与滤波肌电同时间轴对照', '',
             f'数据：{record}。所有13次动作保留，第11次为限位中止。潜伏期数值沿用既有分析，本次仅重新排图。', '',
             '每对图上方是加速度，下方是20–450 Hz带通加50 Hz陷波后的肌电；上下横坐标完全一致。',
             '单次详图左列覆盖接触前200至后800 ms，右列放大前20至后100 ms。肌电按CH01–16、CH17–32、CH33–48分别取中位数，完整48通道图保留在上一层all_channels目录。', '',
             '黑线=接触候选；红虚线=自动起始候选；绿点线=原始肌电主波峰参考。绿色位置来自原始波形，不表示滤波后波峰的位置。',
             '加速度详图保留X/Y/Z相对接触前60至20 ms局部基线的变化及动态向量模长；总览仅显示模长。单位均保留未标定counts，两个纵轴的数值不能直接比较。',
             '两种信号使用各自设备时间戳，未按峰值重新对齐、未插值到同一采样网格。接触点仍是加速度候选，自动肌电起始仍可能受伪影与滤波影响，尚未补偿系统延迟。', '',
             '![全部动作对照](all_trials_accel_emg.png)', '',
             '| 动作 | 状态 | 起始候选/ms | 原始主波峰/ms | 对照图 |',
             '|---|---|---:|---:|---|']
    for r in rows.itertuples(index=False):
        lines.append(f'| {r.shot} | {r.motor_event} | {r.emg_onset_estimate_ms:.1f} | {r.raw_major_peak_delay_ms:.1f} | [查看](shot_{r.shot:02d}_accel_emg.png) |')
    lines += ['', '[上一层完整潜伏期分析](../整理说明.md)', '',
              f'复现：`.pixi/envs/default/bin/python scripts/plot_reflex_accel_emg.py --record {record}`。']
    (out/'加速度肌电对照说明.md').write_text('\n'.join(lines)+'\n')
    (out/'verification.json').write_text(json.dumps({'source_hashes_verified':True,
            'onset_estimates_reused_without_change':True, 'trials':metrics}, indent=2))
    print(out)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', default='20261026_145447')
    run(parser.parse_args().record)
