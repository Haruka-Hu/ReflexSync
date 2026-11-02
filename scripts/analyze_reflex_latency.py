#!/usr/bin/env python3
"""Impact windows and provisional EMG onset latency; no device access.

pixi run python scripts/analyze_reflex_latency.py --record 20261026_145447
Uses accelerometer device timestamps for latency; motor wall-clock logs only
select the contact candidate. Onsets remain automatic, not physiological proof.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
os.environ.setdefault('MPLCONFIGDIR','/tmp/reflex-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd
from scipy import signal

ROOT=Path(__file__).resolve().parents[1]
FS=2000
CH=[f'ch{i:02d}' for i in range(1,49)]
BP=signal.butter(2,[20,450],btype='bandpass',fs=FS,output='sos')
LIVE=np.vstack([signal.tf2sos(*signal.iirnotch(50,30,fs=FS)),
                signal.butter(4,[20,450],btype='bandpass',fs=FS,output='sos')])


def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()


def filt(sos,x):
    zi=signal.sosfilt_zi(sos)[:,:,None]*x[0][None,None,:]
    return signal.sosfilt(sos,x,axis=0,zi=zi)[0]


def envelope(y):
    return np.sqrt(np.maximum(0,signal.lfilter(np.ones(6)/6,[1],y*y,axis=0)))


def onset(t,env,baseline,k=3,hold=6):
    threshold=env[baseline].mean(axis=0)+k*env[baseline].std(axis=0,ddof=1)
    threshold=np.maximum(threshold,1.)
    above=env>threshold
    found=[]
    for c in range(env.shape[1]):
        run=np.convolve(above[:,c].astype(int),np.ones(hold,dtype=int),mode='valid')
        ids=np.flatnonzero((t[:len(run)]>=.005)&(t[:len(run)]<=.100)&(run==hold))
        found.append(t[ids[0]]*1000 if len(ids) else np.nan)
    return np.array(found),threshold


def med(x):return float(np.nanmedian(x)) if np.any(np.isfinite(x)) else np.nan


def run(record):
    out=ROOT/'outputs'/f'reflex_latency_{record}'
    (out/'trials').mkdir(parents=True,exist_ok=True)
    (out/'all_channels').mkdir(exist_ok=True)
    (out/'csv').mkdir(exist_ok=True)
    files={k:ROOT/'data'/f'{stem}_{record}.{ext}' for k,stem,ext in [
        ('emg','emg48_capture','csv'),('imu','imu_capture','csv'),
        ('sync','sync_diag','csv'),('calibration','mvc_calibration','json')]}
    motor=ROOT/'outputs/motor_logs/motor_20261026_145334_241448.log'
    motor_bytes=motor.read_bytes()
    motor_snapshot=out/'motor_log_snapshot.log'
    motor_snapshot.write_bytes(motor_bytes)
    hashes={k:digest(f) for k,f in files.items() if f.exists()}
    hashes['motor_snapshot']=hashlib.sha256(motor_bytes).hexdigest()
    d=pd.read_csv(files['emg']);q=pd.read_csv(files['imu']);q=q[q.source=='Accel'].reset_index(drop=True)
    x=d[CH].to_numpy(float);ts=d.timestamp_us.to_numpy(np.int64)
    t=(ts-ts[0])/1e6;at=(q.timestamp_us.to_numpy(np.int64)-ts[0])/1e6
    xyz=q[['x','y','z']].to_numpy(float)
    an=np.linalg.norm(xyz-np.median(xyz,axis=0),axis=1)
    host=(q.host_rx_time.to_numpy()+8*3600)%86400
    yc=filt(BP,x);live=filt(LIVE,x);yz=signal.sosfiltfilt(BP,x,axis=0)
    envc=envelope(yc);envlive=envelope(live);envz=envelope(yz)
    events=[]; ready=None
    pat=re.compile(r'^\[(\d\d):(\d\d):(\d\d\.\d+)\] RX EVT (READY|SHOT|ABORT) (.*)')
    for line in motor_bytes.decode().splitlines():
        m=pat.match(line)
        if not m:continue
        hh,mm,ss,kind,body=m.groups();wall=int(hh)*3600+int(mm)*60+float(ss)
        if kind=='READY':ready=wall;continue
        fields=dict(re.findall(r'(\w+)=([^ ]+)',body))
        events.append({'shot':int(fields['shot']),'wall':wall,'kind':kind,'fields':fields,'ready':ready})
    # Keep only motor events occurring during this IMU recording.
    events=[e for e in events if host.min()-.1<=e['wall']<=host.max()+.1]
    rows=[];channelrows=[];epochs=[]
    for ev in events:
        ids=np.flatnonzero((host>=ev['wall']-.08)&(host<=ev['wall']+.08))
        if not len(ids):continue
        # SHOT and ABORT timestamps corroborate, but never replace device time.
        peak=ids[np.argmax(an[ids])]
        near=(at>=at[peak]-.06)&(at<at[peak]-.02)
        av=np.linalg.norm(xyz-np.median(xyz[near],axis=0),axis=1)
        base=np.median(av[near]);mad=1.4826*np.median(np.abs(av[near]-base))
        contact_times=[]
        for fraction in [.05,.10,.20]:
            thr=base+max(6*mad,fraction*(av[peak]-base))
            before=np.flatnonzero((at>=at[peak]-.025)&(at<=at[peak])&(av<=thr))
            onset_i=int(before[-1]+1) if len(before) else peak
            contact_times.append(float(at[min(onset_i,peak)]))
        contact=contact_times[1]
        center=np.searchsorted(t,contact);lo=center-1300;hi=center+1801
        if lo<0 or hi>len(x):continue
        rel=t[lo:hi]-contact
        baseline=(rel>=-.5)&(rel<-.2)
        # Raw signal uses a baseline-only linear fit; no acausal filtering.
        coef=np.linalg.lstsq(np.c_[rel[baseline],np.ones(baseline.sum())],x[lo:hi][baseline],rcond=None)[0]
        raw=x[lo:hi]-np.c_[rel,np.ones(len(rel))]@coef
        eraw=envelope(raw)
        on,thr=onset(rel,envc[lo:hi],baseline)
        on_raw,_=onset(rel,eraw,baseline)
        on_live,_=onset(rel,envlive[lo:hi],baseline)
        on_z,_=onset(rel,envz[lo:hi],baseline)
        sensitivity=[med(onset(rel,envc[lo:hi],baseline,k)[0]) for k in [2,3,5]]
        w=(rel>=.005)&(rel<=.08)
        wp=np.flatnonzero(w)[np.argmax(np.abs(np.median(raw[w],axis=1)))]
        peak_delay=rel[wp]*1000
        cc=np.corrcoef(yc[lo:hi][w],rowvar=False)
        common_r=float(np.median(cc[np.triu_indices(48,1)]))
        latency=med(on)
        reasons=['automatic_onset_unvalidated','sync_diagnostics_missing','filter_delay_not_corrected']
        if ev['kind']=='ABORT':reasons.append('motor_abort_'+ev['fields'].get('reason','unknown'))
        if common_r>.95:reasons.append('strong_shared_component')
        if abs(latency-med(on_raw))>5:reasons.append('raw_filter_onset_disagreement_over_5ms')
        if abs(latency-med(on_live))>5:reasons.append('display_detection_filter_disagreement_over_5ms')
        if np.ptp(sensitivity)>5:reasons.append('threshold_sensitivity_over_5ms')
        if latency<=6:reasons.append('onset_at_search_boundary')
        row={'shot':ev['shot'],'motor_event':ev['kind'],'motor_reason':ev['fields'].get('reason'),
             'target_speed_deg_s':float(ev['fields'].get('target_speed',np.nan)),
             'contact_candidate_s':contact,'accel_peak_s':float(at[peak]),
             'contact_threshold_5_20pct_spread_ms':1000*(max(contact_times)-min(contact_times)),
             'accel_peak_counts':float(av[peak]),'accel_host_minus_motor_log_ms':float((host[peak]-ev['wall'])*1000),
             'emg_onset_estimate_ms':latency,'raw_onset_check_ms':med(on_raw),
             'live_filter_onset_ms':med(on_live),'zero_phase_sensitivity_onset_ms':med(on_z),
             'onset_threshold_2sd_ms':sensitivity[0],'onset_threshold_5sd_ms':sensitivity[2],
             'raw_major_peak_delay_ms':float(peak_delay),'detected_channels':int(np.isfinite(on).sum()),
             'onset_channel_q25_ms':float(np.nanquantile(on,.25)),
             'onset_channel_q75_ms':float(np.nanquantile(on,.75)),
             'median_channel_correlation_5_80ms':common_r,
             'include_descriptive_summary':ev['kind']=='SHOT',
             'status':'candidate_not_physiologically_validated','flags':';'.join(reasons)}
        for gi in range(3):row[f'group_{gi+1}_onset_median_ms']=med(on[gi*16:(gi+1)*16])
        rows.append(row)
        for c in range(48):
            channelrows.append({'shot':ev['shot'],'channel':CH[c],'onset_estimate_ms':on[c],
                                'raw_onset_check_ms':on_raw[c],'live_filter_onset_ms':on_live[c],
                                'baseline_env_mean_counts':float(envc[lo:hi][baseline,c].mean()),
                                'threshold_counts':float(thr[c]),'status':row['status']})
        keep=(rel>=-.2)&(rel<=.8)
        epochs.append({'shot':ev['shot'],'rel':rel[keep],'raw':x[lo:hi][keep],
                       'raw_detrended':raw[keep],'live':live[lo:hi][keep],
                       'bp':yc[lo:hi][keep],'env':envc[lo:hi][keep],
                       'onsets':on,'thresholds':thr,'contact':contact,'row':row})
        pd.DataFrame(np.c_[rel[keep]*1000,live[lo:hi][keep]],columns=['relative_ms',*CH]).to_csv(out/'csv'/f'shot_{ev["shot"]:02d}_filtered_counts.csv',index=False,float_format='%.6f')
        print(json.dumps(row,ensure_ascii=False),flush=True)
    summary_df=pd.DataFrame(rows);summary_df.to_csv(out/'latency_summary.csv',index=False)
    pd.DataFrame(channelrows).to_csv(out/'latency_per_channel.csv',index=False)
    # Epoch arrays retain their true per-sample timestamps; rows are not interpolated.
    arrays={}
    for ep in epochs:
        for name in ['rel','raw','raw_detrended','live','bp','onsets','thresholds']:
            arrays[f'shot{ep["shot"]:02d}_{name}']=ep[name]
    np.savez_compressed(out/'reflex_windows.npz',**arrays)
    info={'record':record,'sample_rate_hz':FS,'emg_samples':len(x),'duration_s':float(t[-1]),
          'sequence_breaks':int(np.sum(np.diff(d.sample_seq)!=1)),
          'drdy_breaks':int(np.sum(np.diff(d.drdy_count)!=1)),
          'nonpositive_timestamps':int(np.sum(np.diff(ts)<=0)),
          'large_timestamp_gaps':int(np.sum(np.diff(ts)>2000)),
          'nonfinite_samples':int(np.sum(~np.isfinite(x))),
          'full_scale_samples':int(np.sum(np.abs(x)>=8388607)),
          'emg_nonzero_flag_rows':int(np.sum(d['flags']!=0)),
          'imu_seq_breaks':int(np.sum(np.diff(q.sample_seq)!=1)),
          'imu_link_fault_rows':int(np.sum((q.rx_flags.to_numpy(np.int64)&4)!=0)),
          'sync_diagnostic_rows':len(pd.read_csv(files['sync'])),
          'source_paths':{k:str(f) for k,f in files.items()},'motor_log':str(motor),
          'motor_log_snapshot':str(motor_snapshot),
          'sha256_before':hashes,'method':{'detector':'2nd order causal 20-450 Hz bandpass; no notch; 3 ms trailing RMS; baseline mean+3SD sustained 3 ms',
          'baseline_s':[-.5,-.2],'search_ms':[5,100],'contact':'rising 10% with 6 robust-SD floor; strong accel peak near motor event',
          'filter_delay_compensated':False,'physiological_validation':False}}
    included=summary_df[summary_df.include_descriptive_summary]
    vals=included.emg_onset_estimate_ms.to_numpy()
    info['descriptive_summary']={'motor_shot_events':len(included),'motor_abort_events':len(summary_df)-len(included),
                                'onset_mean_ms':float(np.mean(vals)),'onset_sample_sd_ms':float(np.std(vals,ddof=1)),
                                'onset_median_ms':float(np.median(vals)),'onset_min_ms':float(vals.min()),'onset_max_ms':float(vals.max()),
                                'raw_major_peak_mean_ms':float(included.raw_major_peak_delay_ms.mean())}
    hashes_after={k:digest(f) for k,f in files.items() if f.exists()};hashes_after['motor_snapshot']=digest(motor_snapshot)
    info['source_hashes_unchanged']=hashes==hashes_after
    motor_after=motor.read_bytes()
    info['live_motor_log_append_detected']=motor_after!=motor_bytes
    info['live_motor_log_preserves_snapshot_prefix']=motor_after.startswith(motor_bytes)
    assert info['source_hashes_unchanged']
    (out/'analysis.json').write_text(json.dumps(info,ensure_ascii=False,indent=2))
    plot_all(out,epochs,q,at,av if len(epochs)==1 else an)
    write_report(out,info,summary_df)
    print('FINAL',json.dumps(info['descriptive_summary'],ensure_ascii=False),flush=True)


def plot_all(out,epochs,q,at,an):
    plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
    colors=['#247ba0','#d17b29','#6857a8']
    fig,axs=plt.subplots(7,2,figsize=(15,18),layout='constrained')
    for ax in axs.ravel():ax.set_visible(False)
    overlay,oa=plt.subplots(2,1,figsize=(12,8),layout='constrained')
    for idx,ep in enumerate(epochs):
        row=ep['row'];tt=ep['rel']*1000
        ax=axs.ravel()[idx];ax.set_visible(True)
        for gi in range(3):ax.plot(tt,np.median(ep['live'][:,gi*16:(gi+1)*16],axis=1),lw=.7,color=colors[gi],label=f'CH{gi*16+1:02d}-{(gi+1)*16:02d}')
        ax.axvline(0,color='black',lw=.7);ax.axvline(row['emg_onset_estimate_ms'],color='#bf4036',ls='--',lw=.8)
        ax.set_xlim(-50,500);ax.grid(alpha=.15)
        ax.set_title(f'Shot {ep["shot"]:02d} | {row["motor_event"]} | onset estimate {row["emg_onset_estimate_ms"]:.1f} ms')
        ax.set_ylabel('Filtered ADC counts');ax.set_xlabel('Time from contact candidate (ms)')
        if idx==0:ax.legend(fontsize=7)
        if row['motor_event']=='ABORT':ax.set_facecolor('#fff1e9')
        if row['include_descriptive_summary']:
            oa[0].plot(tt,np.median(ep['live'],axis=1),lw=.7,alpha=.5,label=str(ep['shot']))
            y=np.median(ep['bp'],axis=1);amp=np.max(np.abs(y[(tt>=5)&(tt<80)]))
            oa[1].plot(tt,y/amp,lw=.7,alpha=.5)
        detail,da=plt.subplots(4,1,figsize=(13,11),layout='constrained')
        am=(at>=ep['contact']-.2)&(at<ep['contact']+.8)
        da[0].plot((at[am]-ep['contact'])*1000,an[am],color='#57606b',lw=.8)
        da[0].axvline(0,color='black',lw=.8);da[0].set_xlim(-200,800);da[0].set_ylabel('Accel counts')
        for gi in range(3):da[1].plot(tt,np.median(ep['live'][:,gi*16:(gi+1)*16],axis=1),lw=.8,color=colors[gi],label=f'Group {gi+1}')
        da[1].set_xlim(-100,800);da[1].set_ylabel('Live filtered counts');da[1].legend(fontsize=8,ncol=3)
        for gi in range(3):da[2].plot(tt,np.median(ep['raw_detrended'][:,gi*16:(gi+1)*16],axis=1),lw=.8,color=colors[gi])
        da[2].set_xlim(-20,100);da[2].set_ylabel('Raw detrended counts')
        norm=ep['env']/ep['thresholds']
        for gi in range(3):da[3].plot(tt,np.median(norm[:,gi*16:(gi+1)*16],axis=1),lw=.8,color=colors[gi])
        da[3].axhline(1,color='gray',ls=':');da[3].set_xlim(-20,100);da[3].set_ylabel('Detector RMS / threshold (log)')
        da[3].set_yscale('log');da[3].set_ylim(.05,max(3,float(np.nanmax(norm[(tt>=-20)&(tt<=100)]))*1.2))
        for a in da:
            a.grid(alpha=.15);a.set_xlabel('Time from contact candidate (ms)')
        for a in da[1:]:
            a.axvline(0,color='black',lw=.7);a.axvline(row['emg_onset_estimate_ms'],color='#bf4036',ls='--',label='Onset estimate')
            a.axvline(row['raw_major_peak_delay_ms'],color='#2d8d66',ls=':',lw=.8)
        detail.suptitle(f'Shot {ep["shot"]:02d} | {row["motor_event"]} | contact {ep["contact"]:.6f} s | provisional onset {row["emg_onset_estimate_ms"]:.2f} ms\nRed dashed = detector onset; green dotted = raw main peak; no filter-delay correction',fontsize=11)
        detail.savefig(out/'trials'/f'shot_{ep["shot"]:02d}.png',dpi=145);plt.close(detail)
        grid,ga=plt.subplots(6,8,figsize=(19,13),sharex=True,layout='constrained')
        for c,a in enumerate(ga.ravel()):
            a.plot(tt,ep['live'][:,c],lw=.55,color=colors[c//16]);a.axvline(0,color='black',lw=.5)
            a.axvline(ep['onsets'][c],color='#bf4036',ls='--',lw=.6);a.set_xlim(-30,200)
            a.set_title(CH[c],fontsize=8);a.tick_params(labelsize=6);a.grid(alpha=.12)
        grid.suptitle(f'Shot {ep["shot"]:02d}: all 48 channels | live causal filter | ADC counts | red markers use separate onset detector',fontsize=12)
        grid.savefig(out/'all_channels'/f'shot_{ep["shot"]:02d}_48ch.png',dpi=125);plt.close(grid)
    fig.suptitle('Latest reflex recording 20261026_145447: all 13 motor events\nLive 20-450 Hz + 50 Hz notch. Red onset markers use a separate causal detector. ABORT retained and excluded from summary.',fontsize=13)
    fig.savefig(out/'reflex_all_trials.png',dpi=155);plt.close(fig)
    oa[0].set_xlim(-50,500);oa[0].set_ylabel('Median live-filtered counts');oa[0].legend(ncol=6,fontsize=7,title='Shot')
    oa[1].set_xlim(-20,100);oa[1].set_ylabel('Normalized detector-bandpass waveforms')
    for a in oa:a.axvline(0,color='black',lw=.7);a.grid(alpha=.15);a.set_xlabel('Time from contact candidate (ms)')
    overlay.suptitle('Twelve SHOT events overlaid; ABORT 11 excluded\nOnset estimates require manual and hardware timing validation',fontsize=12)
    overlay.savefig(out/'reflex_overlay.png',dpi=170);plt.close(overlay)


def write_report(out,info,df):
    s=info['descriptive_summary']
    lines=['# 最新叩击附近波形与潜伏期估计','',
           f'来源：`emg48_capture_{info["record"]}.csv`，{info["duration_s"]:.3f} 秒、{info["emg_samples"]:,} 行、48 通道、2 kHz。',
           '最新的 14:59:06 短记录没有明显叩击；本次整理最近包含连续叩击的 14:54:47 记录。',
           '',f'共有 {s["motor_shot_events"]} 次 SHOT 接触候选与 {s["motor_abort_events"]} 次 ABORT。第 11 次为 STRIKE_TRAVEL_LIMIT，保留波形但不加入正常动作的描述统计。',
           '', '**起始延迟结果均为自动候选值，尚未确认是无伪影的生理反射潜伏期。** 接触时间采用加速度设备时间轴上的上升沿候选，不使用电机日志到达时间计算潜伏期。同步诊断无有效行、锤头接触起点未独立标定，滤波延迟也未补偿。',
           '',f'12 次 SHOT 的估计起始延迟：均值 **{s["onset_mean_ms"]:.2f} ± {s["onset_sample_sd_ms"]:.2f} ms**（样本标准差），中位数 {s["onset_median_ms"]:.2f} ms，范围 {s["onset_min_ms"]:.2f}–{s["onset_max_ms"]:.2f} ms。主波峰位置另外列出，不能当作起始潜伏期。','',
           '![全部动作附近的滤波波形](reflex_all_trials.png)','',
           '| 次数 | 电机事件 | 接触候选时刻/s | 起始延迟估计/ms | 原始波形复核/ms | 实时滤波起始/ms | 主波峰延迟/ms |',
           '|---|---|---:|---:|---:|---:|---:|']
    for _,r in df.iterrows():
        lines.append(f'| [{r.shot}](trials/shot_{r.shot:02d}.png) | {r.motor_event} | {r.contact_candidate_s:.6f} | {r.emg_onset_estimate_ms:.2f} | {r.raw_onset_check_ms:.2f} | {r.live_filter_onset_ms:.2f} | {r.raw_major_peak_delay_ms:.2f} |')
    lines += ['', '## 起始点待复核项','',
              '以下标记均保留试次，不据此自动剔除。其余试次也尚未完成生理来源与接触时间验证。', '']
    labels={'motor_abort_':'电机动作中止', 'raw_filter_onset_disagreement_over_5ms':'原始去趋势与检测滤波的起始差超过5 ms',
            'display_detection_filter_disagreement_over_5ms':'界面滤波与检测滤波的起始差超过5 ms',
            'threshold_sensitivity_over_5ms':'2SD至5SD阈值下起始变化超过5 ms',
            'strong_shared_component':'通道间共同成分较强，需核对信号来源'}
    for _,r in df.iterrows():
        notes=[v for k,v in labels.items() if k in r['flags']]
        if notes:lines.append(f'- 第{r.shot}次：'+ '；'.join(notes)+'。')
    lines += ['', '## 定义与方法','',
        '- 接触候选：根据电机 SHOT/ABORT 日志在加速度主机接收时间附近 ±80 ms 内选择强峰，再在设备时间轴取该峰 10% 幅度且高于背景 6 倍稳健标准差的上升沿。日志只帮助区分启动冲击与接触候选。5%/20% 阈值敏感性在表格中保留。',
        '- 起始检测：整段连续的二阶 20–450 Hz 因果带通、不加陷波；3 ms 后向 RMS，以接触前 −500 至 −200 ms 的均值+3SD 为阈值，在接触后 5–100 ms 搜索持续至少 3 ms 的首次越阈。逐通道检测后取 48 通道中位数作为一次动作的估计。48 通道不作为 48 个独立试次。',
        '- 波形展示：按采集界面的四阶 20–450 Hz + 50 Hz/Q30 陷波因果算法重放。展示滤波与起始检测分开，避免把长陷波振铃直接用作起始阈值。单位保留 ADC counts。',
        '- 原始复核：仅从叩击前基线拟合直线并去趋势，再使用同样的 RMS/阈值规则。另存 2SD/5SD 和零相位带通的敏感性结果；零相位结果不作为主要潜伏期。',
        '- 红虚线是自动起始标记；绿点线是原始波形主峰位置。两个标记含义不同，不能把主峰延迟写成潜伏期。',
        '- 没有扣除 ADS 数字滤波、传感器机械传递、模拟前端或检测算法的延迟。EMG 采样间隔为 0.5 ms、加速度约 1 ms，表中的小数用于保留计算值，不代表该量级的测量准确度。',
        '- 多通道共同成分、运动伪迹或接触候选误判均可产生自动越阈。`flags` 保留这些待确认项；不得用数值接近某个典型范围作为生理验证。',
        '- 方法依据：[Hodges 与 Bui 对起始检测阈值/持续时间敏感性的比较](https://pubmed.ncbi.nlm.nih.gov/9020824/)。这里的 3 ms RMS/持续阈值是本次针对短暂脉冲采用的分析参数，不声称是该论文验证的标准腱反射算法。',
        '', '## 文件','',
        '- `reflex_all_trials.png`：13 次动作总览；`reflex_overlay.png`：12 次 SHOT 叠加。',
        '- `trials/shot_XX.png`：每次加速度、滤波波形、原始去趋势波形及起始阈值放大图。',
        '- `all_channels/shot_XX_48ch.png`：逐次 48 通道滤波波形。',
        '- `latency_summary.csv` / `latency_per_channel.csv`：逐次与逐通道结果、阈值敏感性和标记。',
        '- `csv/shot_XX_filtered_counts.csv`：各次 −200 至 +800 ms 的 48 通道滤波窗口及相对时间。',
        '- `reflex_windows.npz`：原始/去趋势/展示滤波/检测滤波窗口、真实相对时间、起始与阈值。',
        '- `analysis.json`：源路径、SHA-256、采样完整性、算法定义和统计。肌电/惯性/标定/同步源文件在分析后重新核对哈希。电机日志仍可能追加，因此本次使用分析开始时的 `motor_log_snapshot.log` 固定输入，并核对原日志前缀。',
        '', '复现：`pixi run python scripts/analyze_reflex_latency.py --record 20261026_145447`。']
    (out/'整理说明.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--record',default='20261026_145447')
    run(parser.parse_args().record)
