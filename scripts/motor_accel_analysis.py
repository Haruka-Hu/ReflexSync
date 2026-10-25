"""Unsmoothed counts analysis and offline report regeneration, no hardware IO."""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import numpy as np

FIELDS=('accel_peak_counts','accel_raw_peak_counts','accel_peak_time_us','accel_peak_seq',
        'accel_threshold','accel_gap_count','accel_link_fault','accel_no_event','accel_multi_event',
        'accel_window_complete','quality_qualified')

def analyze(rows,fire,end=None,min_peak=100.,motor=None):
    motor=motor or {}
    unique={int(r['seq_unwrapped']):r for r in rows}
    ordered=[unique[k] for k in sorted(unique)]
    seq=np.array([int(r['seq_unwrapped']) for r in ordered],dtype=np.int64)
    ts=np.array([int(r['time_unwrapped_us']) for r in ordered],dtype=np.int64)
    v=np.array([[float(r[k]) for k in ('x','y','z')] for r in ordered],dtype=float)
    empty={'accel_window_complete':False,'quality_qualified':False,'accel_no_event':True,
           'accel_gap_count':0,'accel_link_fault':False,'accel_multi_event':False}
    if not len(seq):return empty
    baseline=(seq<=fire['sample_seq'])&(ts>=fire['timestamp_us']-200000)
    action=seq>fire['sample_seq']
    if not baseline.any() or not action.any():return empty
    b=np.median(v[baseline],axis=0)
    base_norm=np.linalg.norm(v[baseline]-b,axis=1)
    med=float(np.median(base_norm));sigma=float(1.4826*np.median(np.abs(base_norm-med)))
    threshold=max(float(min_peak),med+8*sigma)
    dynamic=np.linalg.norm(v-b,axis=1);indices=np.flatnonzero(action)
    peak=int(indices[np.argmax(dynamic[action])]);significant=indices[dynamic[action]>=threshold]
    # Preserve the specified conservative full-window criterion. Startup vibration
    # can also produce a separate event; report this limitation, never hide it.
    clusters=0 if not len(significant) else 1+int((np.diff(ts[significant])>100000).sum())
    gaps=int(np.maximum(np.diff(seq)-1,0).sum())
    timing_gaps=int(((np.diff(ts)<=0)|(np.diff(ts)>3000)).sum())
    flags=np.array([int(r['rx_flags']) for r in ordered])
    link=bool((flags&4).any() or ((flags&2)==0).any())
    complete=bool(end and seq[-1]>=end['sample_seq'] and ts[baseline][-1]-ts[baseline][0]>=180000)
    qualified=bool(complete and motor.get('speed_qualified') and motor.get('contact_candidate') and
        not motor.get('early_drop_suspected') and clusters==1 and not gaps and not timing_gaps and not link)
    result=dict(accel_peak_counts=float(dynamic[peak]),accel_raw_peak_counts=float(np.linalg.norm(v[action],axis=1).max()),
        accel_peak_time_us=int(ts[peak]),accel_peak_seq=int(seq[peak]),accel_threshold=threshold,
        accel_gap_count=gaps,accel_timing_gaps=timing_gaps,accel_link_fault=link,accel_no_event=clusters==0,
        accel_multi_event=clusters>1,accel_window_complete=complete,quality_qualified=qualified,
        baseline_counts=b.tolist(),baseline_norm_median=med,baseline_robust_sigma=sigma,
        event_clusters=clusters,accel_sample_count=len(seq),min_peak_counts=min_peak,
        seq_start=int(seq[0]),seq_end=int(seq[-1]),peak_host_monotonic=float(ordered[peak]['host_monotonic']))
    for i,k in enumerate(('x','y','z')):
        corrected=v[action,i]-b[i]
        for label,func in [('positive',np.argmax),('negative',np.argmin)]:
            ix=int(indices[func(corrected)])
            result[f'{k}_{label}_peak']=float(v[ix,i]-b[i])
            result[f'{k}_{label}_peak_time_us']=int(ts[ix])
    return result

def stats(values):
    values=[float(x) for x in values if x is not None and math.isfinite(float(x))]
    if not values:return dict(n=0,mean=None,sd=None,cv_pct=None,min=None,max=None)
    mean=float(np.mean(values));sd=float(np.std(values,ddof=1)) if len(values)>1 else None
    return dict(n=len(values),mean=mean,sd=sd,cv_pct=sd/mean*100 if sd is not None and mean else None,
                min=min(values),max=max(values))

def report(directory,min_peak=None):
    directory=Path(directory)
    manifest=json.loads((directory/'run.json').read_text())
    config=manifest['config'];min_peak=float(min_peak if min_peak is not None else config.get('accel_min_peak',100))
    entries=[]
    for path in sorted(directory.glob('speed_*/trial_*/result.json')):
        d=json.loads(path.read_text());row=d['row'];accel=path.with_name('accel.csv')
        metrics={};rows=[]
        if accel.exists() and d.get('accel_fire'):
            with accel.open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
            metrics=analyze(rows,d['accel_fire'],d.get('accel_end'),min_peak,row)
            row.update({k:metrics.get(k) for k in FIELDS});d['accel_metrics']=metrics
            path.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        entries.append((row,metrics,rows))
    entries.sort(key=lambda item:item[0]['number'])
    summaries=[]
    for speed in config['speeds']:
        group=[x[0] for x in entries if x[0]['speed_dps']==speed]
        recorded=sum(r.get('accel_peak_counts') is not None for r in group)
        for name,subset in [('all',group),('qualified',[r for r in group if r.get('quality_qualified')])]:
            s=stats([r.get('accel_peak_counts') for r in subset])
            summaries.append(dict(speed_dps=speed,subset=name,planned=config['repeats'],
                attempted=len(group),recorded=recorded,missing=config['repeats']-recorded,**s))
    with (directory/'grade_statistics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=summaries[0]);w.writeheader();w.writerows(summaries)
    if entries:
        fields=list(dict.fromkeys(k for row,_,_ in entries for k in row))
        with (directory/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows([r for r,_,_ in entries])
    (directory/'analysis_parameters.json').write_text(json.dumps(dict(min_peak_counts=min_peak,
        baseline_ms=200,tail_seconds=1,late_grace_seconds=.5,robust_sigma='1.4826*MAD',
        threshold_sigma=8,significant_event_separation_ms=100,units='counts',smoothing=False,
        association='host_monotonic_and_unwrapped_sample_bounds; not shared hardware clocks'),indent=2)+'\n')
    lines=['# 批量叩击与加速度重复性',f'\n运行状态：{manifest["status"]}；{manifest.get("error", "")}\n',
        '|速度(°/s)|记录/计划|分组|有效峰值n|均值(counts)|样本SD|CV(%)|最小|最大|',
        '|---:|---:|---|---:|---:|---:|---:|---:|---:|']
    fmt=lambda v:'—' if v is None else f'{v:.2f}'
    for s in summaries:
        lines.append(f'|{s["speed_dps"]}|{s["recorded"]}/{s["planned"]}|{s["subset"]}|{s["n"]}|'+
                     '|'.join(fmt(s[k]) for k in ('mean','sd','cv_pct','min','max'))+'|')
    lines+=['\n全体组不把缺失峰值当成零；少于2个有效值不计算样本SD和CV。',
        '\n质量子集要求速度目标±10%持续10ms，无提前速降、无明确事件缺失、多事件、样本缺口或链路故障，且窗口完整。',
        '\n多事件按全动作窗口阈值严格标记；启动振动可能造成额外事件，不能直接等同于二次接触。异常记录保留。',
        '\n加速度为各轴去200ms基线中位数后的未经平滑合量。采样约1000点/秒，高速尖峰可能只有1—2点，结果仅描述当前采样链下的峰值重复性。',
        '\n速度来自电机轨迹速降前区间，不是加速度峰值瞬间的精确角速度；未标定为g或接触力。',
        '\n逐击文件：accel.csv、trace.csv、result.json；整段原始：accel_raw.csv；同步、链路及动作标记：capture_events.jsonl。',
        '\n图表：peak_scatter.png、grade_comparison.png、overlay_<速度>.png。']
    (directory/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if not entries:return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,4),layout='constrained')
    for good,color,label in [(True,'tab:blue','Qualified'),(False,'tab:orange','Flagged')]:
        rows=[r for r,_,_ in entries if bool(r.get('quality_qualified'))==good and r.get('accel_peak_counts') is not None]
        ax.scatter([r['number'] for r in rows],[r['accel_peak_counts'] for r in rows],c=color,label=label,s=20)
    ax.set(xlabel='Trial number',ylabel='Unsmoothed peak (counts)');ax.legend();fig.savefig(directory/'peak_scatter.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(12,4),layout='constrained')
    for name,color in [('all','tab:blue'),('qualified','tab:orange')]:
        ss=[s for s in summaries if s['subset']==name]
        for ax,key in zip(axes,('mean','sd','cv_pct')):
            ax.plot([s['speed_dps'] for s in ss],[np.nan if s[key] is None else s[key] for s in ss],'o-',label=name,c=color)
            ax.set(xlabel='Target (deg/s)',ylabel=key+(' (%)' if key=='cv_pct' else ' (counts)'));ax.legend()
    fig.savefig(directory/'grade_comparison.png',dpi=150);plt.close(fig)
    for speed in config['speeds']:
        fig,ax=plt.subplots(figsize=(10,4),layout='constrained');count=0
        for r,m,rows in entries:
            if r['speed_dps']!=speed or 'accel_peak_time_us' not in m:continue
            b=np.array(m['baseline_counts']);t=np.array([int(x['time_unwrapped_us']) for x in rows]);v=np.array([[float(x[k]) for k in ('x','y','z')] for x in rows])
            ax.plot((t-m['accel_peak_time_us'])/1000,np.linalg.norm(v-b,axis=1),alpha=.4,lw=.8,
                    color='tab:blue' if r.get('quality_qualified') else 'tab:orange');count+=1
        ax.set(xlim=(-250,300),xlabel='Time from peak (ms)',ylabel='Counts',title=f'{speed} deg/s, n={count}; blue=qualified, orange=flagged')
        fig.savefig(directory/f'overlay_{speed}.png',dpi=150);plt.close(fig)
    return summaries
