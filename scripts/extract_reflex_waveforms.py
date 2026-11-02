#!/usr/bin/env python3
"""Extract and plot paired EMG/accelerometer windows on their device time grids."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal

CH = [f'ch{i:02d}' for i in range(1, 49)]
WRAP = 2**32


def unwrap(values):
    x = np.asarray(values, dtype=np.int64)
    delta = np.diff(x)
    if np.any((delta < 0) & (delta > -WRAP // 2)):
        raise ValueError('时间戳出现非回绕的倒退，无法安全对齐')
    return x + np.r_[0, np.cumsum(delta < -WRAP // 2)] * WRAP


def required(df, cols, name):
    missing = sorted(set(cols) - set(df.columns))
    if missing:
        raise ValueError(f'{name} 缺少列: {", ".join(missing)}')


def emg_response(rel, y):
    """Return channel-wise threshold candidates, not a physiological diagnosis."""
    env = np.sqrt(np.maximum(0, signal.lfilter(np.ones(6) / 6, [1], y*y, axis=0)))
    baseline = (rel >= -.5) & (rel < -.2)
    if baseline.sum() < 500:
        return {'onset_candidate_ms': None, 'responding_channels': 0, 'baseline_valid': False}
    threshold = np.maximum(env[baseline].mean(axis=0) + 3*env[baseline].std(axis=0, ddof=1), 1.)
    found = []
    for c in range(y.shape[1]):
        run = np.convolve((env[:, c] > threshold[c]).astype(int), np.ones(6, dtype=int), mode='valid')
        ids = np.flatnonzero((rel[:len(run)] >= .005) & (rel[:len(run)] <= .1) & (run == 6))
        if len(ids):
            found.append(rel[ids[0]] * 1000)
    return {'onset_candidate_ms':float(np.median(found)) if found else None,
            'responding_channels':len(found), 'baseline_valid':True}


def main():
    parser = argparse.ArgumentParser(description='截取 EMG/加速度叩击窗口；不重采样，所有标记均为候选。')
    parser.add_argument('--emg', required=True, help='48 通道肌电 CSV')
    parser.add_argument('--imu', required=True, help='加速度 CSV')
    parser.add_argument('--out', required=True, help='新建输出目录，不覆盖已有目录')
    parser.add_argument('--contact-s', nargs='+', type=float, help='手动指定接触候选秒数，相对 EMG 第一条设备时间戳')
    parser.add_argument('--events', nargs='+', type=int, help='按时间排序的双峰候选编号；省略则画前 3 个肌电阳性候选')
    parser.add_argument('--threshold', type=float, default=80, help='加速度变化的最低峰值（原始 counts），默认 80')
    parser.add_argument('--min-gap-ms', type=float, default=100, help='相邻冲击峰的最小间隔，默认 100 ms')
    parser.add_argument('--pair-min-ms', type=float, default=80, help='双峰最小间隔，默认 80 ms')
    parser.add_argument('--pair-max-ms', type=float, default=250, help='双峰最大间隔，默认 250 ms')
    parser.add_argument('--min-channels', type=int, default=12, help='肌电越阈通道数下限，默认 12/48')
    parser.add_argument('--imu-offset-ms', type=float, default=0, help='加到 IMU 设备时间戳上的已知校正值，默认 0 ms')
    args = parser.parse_args()
    if args.events and args.contact_s:
        parser.error('--events 和 --contact-s 不能同时使用')
    if args.threshold <= 0 or args.min_gap_ms <= 0 or not 0 < args.pair_min_ms < args.pair_max_ms or not 1 <= args.min_channels <= 48:
        parser.error('阈值和最小间隔必须大于零')
    out = Path(args.out)
    if out.exists():
        parser.error(f'输出目录已存在，避免覆盖: {out}')
    print('读取 CSV，准备按设备时间戳对齐……', flush=True)
    emg = pd.read_csv(args.emg)
    imu = pd.read_csv(args.imu)
    required(emg, ['timestamp_us', *CH], 'EMG')
    required(imu, ['timestamp_us', 'x', 'y', 'z'], 'IMU')
    if 'source' in imu:
        imu = imu[imu.source.str.lower() == 'accel'].copy()
    if len(emg) < 2000 or len(imu) < 100:
        raise ValueError('数据不足，或 IMU 文件没有 Accel 行')
    et = unwrap(emg.timestamp_us)
    it = unwrap(imu.timestamp_us)
    # Resolve the wrap epoch nearest the beginning of EMG; no interpolation.
    it = it + round((int(et[0]) - int(it[0])) / WRAP)*WRAP
    t = (et - et[0])/1e6
    at = (it - et[0])/1e6 + args.imu_offset_ms/1000
    if np.median(np.diff(et)) != 500:
        raise ValueError('此脚本要求 EMG 为 2000 Hz（设备时间戳中位间隔 500 µs）')
    if np.any(np.diff(et) > 2000) or np.any(np.diff(it) > 3000):
        print('警告：时间戳存在较大间隔，请检查丢样对结果的影响。', flush=True)
    x = emg[CH].to_numpy(dtype=float)
    xyz = imu[['x','y','z']].to_numpy(dtype=float)
    if not np.isfinite(x).all() or not np.isfinite(xyz).all():
        raise ValueError('数据包含缺失值或非有限值')
    sos = signal.butter(2, [20, 450], btype='bandpass', fs=2000, output='sos')
    live_sos = np.vstack([signal.tf2sos(*signal.iirnotch(50, 30, fs=2000)), signal.butter(4, [20,450], btype='bandpass', fs=2000, output='sos')])
    def filt(s):
        zi = signal.sosfilt_zi(s)[:,:,None]*x[0][None,None,:]
        return signal.sosfilt(s, x, axis=0, zi=zi)[0]
    filtered = filt(sos)
    live = filt(live_sos)
    candidates = []
    detected = []
    unpaired = []
    if args.contact_s:
        candidates = [{'id': f'{i:02d}', 'contact': c, 'event': 'manual'} for i,c in enumerate(args.contact_s, 1)]
    else:
        an = np.linalg.norm(xyz - np.median(xyz, axis=0), axis=1)
        sample_ms = np.median(np.diff(at))*1000
        if sample_ms <= 0:
            raise ValueError('IMU 时间戳必须递增')
        peaks, _ = signal.find_peaks(an, height=args.threshold, distance=max(1, int(np.ceil(args.min_gap_ms/sample_ms))))
        # Greedy chronological pairing. Unpaired peaks are retained for review.
        i = 0
        while i < len(peaks):
            first = int(peaks[i])
            if i+1 >= len(peaks):
                unpaired.append({'peak_s':float(at[first]),'peak_counts':float(an[first]),'reason':'no_following_peak'})
                break
            second = int(peaks[i+1]); gap = (at[second]-at[first])*1000
            if not args.pair_min_ms <= gap <= args.pair_max_ms:
                unpaired.append({'peak_s':float(at[first]),'peak_counts':float(an[first]),'reason':'no_adjacent_peak_in_pair_window'})
                i += 1
                continue
            i += 2
            near = (at >= at[second]-.06) & (at < at[second]-.02)
            if near.sum() < 10:
                unpaired.append({'peak_s':float(at[second]),'peak_counts':float(an[second]),'reason':'insufficient_accel_baseline'})
                continue
            local_base = np.median(xyz[near],axis=0)
            region = np.flatnonzero((at>=at[second]-.06)&(at<=at[second]))
            av = np.linalg.norm(xyz[region]-local_base,axis=1)
            nb = (at[region]>=at[second]-.06)&(at[region]<at[second]-.02)
            base = np.median(av[nb]); mad = 1.4826*np.median(np.abs(av[nb]-base))
            thr = base+max(6*mad,.1*(av[-1]-base))
            before = np.flatnonzero((at[region]>=at[second]-.025)&(av<=thr))
            contact = at[region[min(int(before[-1]+1),len(region)-1)]] if len(before) else at[second]
            if contact-.5 < t[0] or contact+.8 > min(t[-1],at[-1]) or contact-.2 < at[0]:
                unpaired.append({'peak_s':float(at[second]),'peak_counts':float(an[second]),'reason':'incomplete_window'})
                continue
            em = (t>=contact-.65)&(t<=contact+.9)
            response = emg_response(t[em]-contact, filtered[em])
            # Flag gaps in the baseline and response interval: samples are never resampled.
            lo = np.searchsorted(t,contact-.5); hi = np.searchsorted(t,contact+.11)
            alo = np.searchsorted(at,at[first]-.03); ahi = np.searchsorted(at,at[second]+.11)
            gap_flag = bool(np.any(np.diff(et[lo:hi])>1000) or np.any(np.diff(it[alo:ahi])>2000))
            latency = response['onset_candidate_ms']
            flags = []
            if gap_flag: flags.append('sample_gap')
            if latency is not None and latency <= 6: flags.append('onset_near_search_boundary')
            if not before.size: flags.append('contact_rising_edge_not_found')
            if contact-.5 <= at[first] < contact-.2: flags.append('first_peak_in_emg_baseline')
            enough = response['baseline_valid'] and response['responding_channels'] >= args.min_channels
            assessment = 'review_no_clear_emg' if not enough else ('review_ambiguous' if flags else 'emg_change_candidate')
            detected.append({'id':str(len(detected)+1), 'first_peak_s':float(at[first]),'second_peak_s':float(at[second]),
                             'first_peak_counts':float(an[first]),'second_peak_counts':float(an[second]),'pair_gap_ms':float(gap),
                             'contact':float(contact),'event':'paired_accel_candidate',**response,
                             'assessment':assessment,
                             'flags':';'.join(flags)})
        wanted = args.events if args.events else [int(c['id']) for c in detected if c['assessment']=='emg_change_candidate'][:3]
        if not wanted and detected and not args.events:
            wanted = [int(c['id']) for c in detected[:3]]
        if any(i < 1 or i > len(detected) for i in wanted):
            raise ValueError(f'候选编号超出范围，实际检测到 {len(detected)} 组')
        candidates = [detected[i-1] for i in wanted]
    if not candidates:
        raise ValueError('未找到可截取的双峰候选；请检查阈值、双峰间隔和记录范围，或使用 --contact-s')
    for c in candidates:
        if c['contact']-.5 < t[0] or c['contact']+.8 > min(t[-1],at[-1]) or c['contact']-.2 < at[0]:
            raise ValueError(f"事件 {c['id']} 缺少完整的基线或截取窗口")
    out.mkdir(parents=True)
    results=[]
    for c in candidates:
        contact=c['contact']; em=(t>=contact-.65)&(t<=contact+.9); rel=t[em]-contact
        response=emg_response(rel,filtered[em]); on=response['onset_candidate_ms']; window=(t>=contact-.2)&(t<=contact+.8); am=(at>=contact-.2)&(at<=contact+.8)
        base=(at>=contact-.15)&(at<contact-.05)
        if base.sum()<10:
            raise ValueError(f"事件 {c['id']} 的 IMU 基线不足")
        accel=np.linalg.norm(xyz[am]-np.median(xyz[base],axis=0),axis=1)
        tt=(t[window]-contact)*1000; aa=(at[am]-contact)*1000
        stem=f"shot_{c['id']}"
        eout=emg.loc[window].copy().reset_index(drop=True);eout.insert(0,'relative_ms',tt)
        eout=pd.concat([eout,pd.DataFrame(live[window],columns=[f'{name}_filtered' for name in CH])],axis=1)
        eout.to_csv(out/f'{stem}_emg.csv',index=False)
        aout=imu.loc[am].copy();aout.insert(0,'relative_ms',aa);aout['accel_change_counts']=accel;aout.to_csv(out/f'{stem}_accel.csv',index=False)
        fig,axes=plt.subplots(2,2,figsize=(14,7),sharex='col',constrained_layout=True)
        for j,lim in enumerate([(-200,800),(-50,120)]):
            axes[0,j].plot(aa,accel,color='#3a4750',lw=.8);axes[0,j].set_xlim(*lim);axes[0,j].set_ylabel('Acceleration change (counts)');axes[0,j].set_title(['Full window','Near impact'][j])
            if 'first_peak_s' in c:
                axes[0,j].axvline((c['first_peak_s']-contact)*1000,color='#ca8a04',ls=':',lw=.9,label='First acceleration peak')
                axes[0,j].legend(fontsize=8)
            for g,col in enumerate(['#247ba0','#d17b29','#6857a8']):axes[1,j].plot(tt,np.median(live[window,16*g:16*(g+1)],axis=1),color=col,lw=.8,label=f'CH{16*g+1:02d}-{16*(g+1):02d} median')
            for ax in axes[:,j]:ax.axvline(0,color='black',lw=.8);ax.grid(alpha=.2)
            if on is not None:axes[1,j].axvline(on,color='#bf4036',ls='--',lw=.9,label='Onset candidate')
            axes[1,j].set_ylabel('Filtered EMG (ADC counts)');axes[1,j].set_xlabel('Time from contact candidate (ms)')
        axes[1,0].legend(fontsize=8,ncol=2);fig.suptitle(f"Event {c['id']} ({c.get('assessment','manual')}): contact candidate at 0; onset {on if on is not None else 'none'} ms; channels {response['responding_channels']}/48")
        fig.savefig(out/f'{stem}.png',dpi=160);plt.close(fig)
        results.append({**c,**response})
    (out/'summary.json').write_text(json.dumps({'emg':str(Path(args.emg).resolve()),'imu':str(Path(args.imu).resolve()),'detected_candidates':detected,'unpaired_peaks':unpaired,'parameters':{'threshold':args.threshold,'min_gap_ms':args.min_gap_ms,'pair_min_ms':args.pair_min_ms,'pair_max_ms':args.pair_max_ms,'min_channels':args.min_channels},'imu_offset_ms':args.imu_offset_ms,'notes':'设备时间轴名义对齐；接触和起始均为候选，未验证生理来源或同步误差。','events':results},ensure_ascii=False,indent=2),encoding='utf-8')
    if detected:
        pd.DataFrame(detected).to_csv(out/'paired_candidates.csv',index=False)
    pd.DataFrame(unpaired,columns=['peak_s','peak_counts','reason']).to_csv(out/'unpaired_peaks.csv',index=False)
    print(f'检测到 {len(detected)} 组双峰候选，其中 {sum(c["assessment"]=="emg_change_candidate" for c in detected)} 组肌电变化候选；输出 {len(results)} 组波形到 {out.resolve()}。所有结果须人工复核。')

if __name__ == '__main__':
    main()
