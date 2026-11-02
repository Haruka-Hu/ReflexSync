#!/usr/bin/env python3
"""Read-only September 21/22 vs October 26 EMG audit and impact-aligned analysis.

Run: MPLCONFIGDIR=/tmp/reflex-mpl pixi run python scripts/compare_reflex_emg.py
Raw inputs are never modified. Units remain ADC counts. Impact peaks are
automatically detected candidates, not a measurement of neural response onset.
"""
from pathlib import Path
import gc
import hashlib
import json
import os
import re
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/reflex-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'outputs/reflex_comparison_20261026'
HIST = ROOT.parents[1]/'04-数据'
CH = [f'ch{i:02d}' for i in range(1,49)]
FS = 2000
SOS = np.vstack((signal.tf2sos(*signal.iirnotch(50,30,fs=FS)),
                 signal.butter(4,[20,450],btype='bandpass',fs=FS,output='sos')))
EXAMPLES = ['20260922_153701','20260922_155509','20261026_005026','20261026_005904']


def rms(x):
    return np.sqrt(np.mean(x*x,axis=0))


def medcorr(x):
    x=x[np.all(np.isfinite(x),axis=1)]
    if len(x)<3:
        return float('nan')
    c=np.corrcoef(x,rowvar=False)
    return float(np.nanmedian(c[np.triu_indices(x.shape[1],1)]))


def span_minmax(ax,t,y,color='#2373a8',label=None):
    # Keep each bin's extrema, so overview plots do not hide transient spikes.
    k=max(1,len(t)//16000)
    if k==1:
        ax.plot(t,y,lw=.6,color=color,label=label)
        return
    n=len(t)//k*k
    bins=y[:n].reshape(-1,k)
    tb=t[:n].reshape(-1,k).mean(axis=1)
    ax.fill_between(tb,bins.min(axis=1),bins.max(axis=1),color=color,alpha=.65,label=label)


def run():
    OUT.mkdir(parents=True,exist_ok=True)
    files=sorted(HIST.glob('data_26092*/emg48*.csv'))
    files += sorted((ROOT/'data').glob('emg48_capture_20261026*.csv'))
    records=[]; events=[]; channels=[]; manifest=[]
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    for f in files:
        token=f.stem.replace('emg48_capture_','')
        is_today=token.startswith('20261026')
        # Only the two user-confirmed October reflex recordings are compared.
        if is_today and token not in EXAMPLES:
            continue
        imuf=f.with_name(f.name.replace('emg48_','imu_'))
        if not imuf.exists():
            continue
        q=pd.read_csv(imuf)
        if not len(q) or not np.any(q.source=='Accel'):
            continue
        q=q[q.source=='Accel'].copy()
        d=pd.read_csv(f)
        x=d[CH].to_numpy(float)
        timestamps=d.timestamp_us.to_numpy(np.int64)
        t=(timestamps-timestamps[0])/1e6
        dt=np.diff(timestamps)
        seq=np.diff(d.sample_seq.to_numpy(np.int64))
        drdy=np.diff(d.drdy_count.to_numpy(np.int64))
        a=q[['x','y','z']].to_numpy(float)
        an=np.linalg.norm(a-np.median(a,axis=0),axis=1)
        at=(q.timestamp_us.to_numpy(np.int64)-timestamps[0])/1e6
        aqdt=np.diff(at)
        afs=1/np.median(aqdt[aqdt>0])
        threshold=max(.35*np.max(an),np.median(an)+15*np.median(np.abs(an-np.median(an))))
        peaks,_=signal.find_peaks(an,height=threshold,distance=int(1.5*afs),prominence=threshold*.5)
        peak_times=at[peaks]
        # Full-record filtering; no trial-wise filter reset. Never filter over gaps.
        y=np.empty_like(x)
        breaks=np.flatnonzero((seq!=1)|(dt<=0)|(dt>2000))+1
        edges=np.r_[0,breaks,len(x)]
        for lo,hi in zip(edges[:-1],edges[1:]):
            if hi-lo>100:
                y[lo:hi]=signal.sosfiltfilt(SOS,x[lo:hi],axis=0)
            else:
                y[lo:hi]=np.nan
        freq,power=signal.welch(x,fs=FS,nperseg=4096,axis=0,detrend='linear')
        lf=power[(freq>=.5)&(freq<20)].sum(axis=0)
        total=power[(freq>=.5)&(freq<=450)].sum(axis=0)
        frac=lf/total
        cf=f.with_name(f'mvc_calibration_{token}.json')
        calibration=json.loads(cf.read_text()) if cf.exists() else {}
        r={'id':token,'subject':f.parent.name,'period':'today' if is_today else 'september',
           'path':str(f),'duration_s':float(t[-1]),'samples':len(x),'channels':48,
           'sample_rate_from_timestamps_hz':float(1e6/np.median(dt)),
           'sequence_breaks':int(np.sum(seq!=1)),'sequence_missing_forward':int(np.sum(seq[seq>1]-1)),
           'drdy_breaks':int(np.sum(drdy!=1)),'timestamp_nonpositive':int(np.sum(dt<=0)),
           'timestamp_large_gaps':int(np.sum(dt>2000)), 'timestamp_dt_min_us':int(dt.min()),'timestamp_dt_max_us':int(dt.max()),
           'nonfinite_samples':int(np.sum(~np.isfinite(x))), 'clip_samples':int(np.sum(np.abs(x)>=8388607)),
           'median_channel_raw_range_counts':float(np.median(np.ptp(x,axis=0))),
           'median_channel_lf_power_fraction':float(np.median(frac)),
           'median_channel_filtered_rms_counts':float(np.nanmedian(np.sqrt(np.nanmean(y*y,axis=0)))),
           'filtered_interchannel_median_r':medcorr(y[::10]),
           'accel_threshold_counts':float(threshold),'impact_candidates':len(peaks),
           'host_ads_gain':calibration.get('ads_gain'),
           'emg_flag_nonzero_rows':int(np.sum(d['flags'].to_numpy()!=0)),
           'accel_time_sync_flag_rows':int(np.sum((q.rx_flags.to_numpy(np.int64)&2)!=0)),
           'accel_link_fault_flag_rows':int(np.sum((q.rx_flags.to_numpy(np.int64)&4)!=0)),
           'mvc_calibration_valid':calibration.get('calibration',{}).get('valid')}
        trial_raw=[]; trial_bp=[]; valid_times=[]
        per_events=[]
        for j,p in enumerate(peaks):
            tp=at[p]; center=np.searchsorted(t,tp)
            if tp<1 or tp>t[-1]-1.2:
                continue
            lo=center-2000; hi=center+2400
            if lo<0 or hi>len(x) or np.any((breaks>lo)&(breaks<hi)):
                continue
            rel=(np.arange(lo,hi)-center)/FS
            raw=x[lo:hi]; bp=y[lo:hi]
            pre=(rel>=-.8)&(rel<-.2)
            early=(rel>=.02)&(rel<.10)
            late=(rel>=.2)&(rel<.8)
            instant=(rel>=-.005)&(rel<.010)
            pre_rms=rms(bp[pre])
            base=np.median(raw[pre],axis=0)
            # Late variation after removal of its mean separates settling from offset.
            offset=np.median(raw[late],axis=0)-base
            deriv=np.diff(raw,axis=0)
            di=(rel[1:]>=-.01)&(rel[1:]<.02)
            cmstep=np.max(np.abs(np.median(deriv[di],axis=1)))
            maxstep=np.median(np.max(np.abs(deriv[di]),axis=0))
            ev={'id':token,'candidate_index':j+1,'time_s':float(tp),'accel_peak_counts':float(an[p]),
                'pre_bp_rms_counts':float(np.median(pre_rms)),
                'instant_bp_rms_ratio':float(np.median(rms(bp[instant])/pre_rms)),
                'early_20_100ms_bp_rms_ratio':float(np.median(rms(bp[early])/pre_rms)),
                'late_200_800ms_bp_rms_ratio':float(np.median(rms(bp[late])/pre_rms)),
                'late_raw_offset_abs_counts':float(np.median(np.abs(offset))),
                'instant_channel_median_r':medcorr(raw[instant]),
                'common_derivative_fraction':float(cmstep/maxstep),
                'instant_derivative_median_max_counts':float(maxstep),
                'late_bp_interchannel_median_r':medcorr(bp[late])}
            events.append(ev); per_events.append(ev)
            trial_raw.append(raw-base); trial_bp.append(bp); valid_times.append(tp)
        r['usable_impact_candidates']=len(per_events)
        for key in ['instant_bp_rms_ratio','early_20_100ms_bp_rms_ratio','late_200_800ms_bp_rms_ratio',
                    'late_raw_offset_abs_counts','instant_channel_median_r','common_derivative_fraction','late_bp_interchannel_median_r',
                    'instant_derivative_median_max_counts']:
            r['median_'+key]=float(np.median([e[key] for e in per_events])) if per_events else None
        if per_events:
            v=np.array([e['early_20_100ms_bp_rms_ratio'] for e in per_events])
            r['early_ratio_cv']=float(v.std(ddof=1)/v.mean()) if len(v)>1 else None
        records.append(r)
        for c in range(48):
            channels.append({'id':token,'channel':CH[c],'raw_range_counts':float(np.ptp(x[:,c])),
                             'lf_power_fraction':float(frac[c]),'bp_rms_counts':float(np.sqrt(np.nanmean(y[:,c]**2))),
                             'clip_samples':int(np.sum(np.abs(x[:,c])>=8388607))})
        manifest.append({'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),
                         'imu_path':str(imuf),'imu_sha256':hashlib.sha256(imuf.read_bytes()).hexdigest()})
        if token in EXAMPLES:
            np.savez_compressed(OUT/f'{token}_display.npz',t=t[::4],
                raw=x[::4][:,[0,7,15,23,31,39,47]],
                bp=y[::4][:,[0,7,15,23,31,39,47]],
                accel_t=at,accel=an,impact_times=peak_times,valid_times=valid_times,
                trial_raw=np.asarray(trial_raw),trial_bp=np.asarray(trial_bp),rel=rel)
            fig,axs=plt.subplots(3,1,figsize=(14,8),sharex=True,layout='constrained')
            axs[0].plot(at,an,color='#667085',lw=.6)
            for k,tp in enumerate(peak_times):
                axs[0].axvline(tp,color='#c4503c',alpha=.4,lw=.7)
                axs[0].text(tp,axs[0].get_ylim()[1]*.9,str(k+1),fontsize=8)
            axs[0].set_ylabel('Accel deviation\n(raw counts)')
            span_minmax(axs[1],t,x[:,0]-np.median(x[t<1,0]))
            axs[1].set_ylabel('CH01 raw change\n(ADC counts)')
            span_minmax(axs[2],t,y[:,0]); axs[2].set_ylabel('CH01 20-450 Hz\n+50 Hz notch (counts)')
            axs[2].set_xlabel('Time since first EMG sample (s)')
            fig.suptitle(f'{token}   automatic impact candidates: {len(peaks)}; usable: {len(per_events)}')
            for ax in axs: ax.grid(alpha=.18)
            fig.savefig(OUT/f'{token}_overview.png',dpi=160); plt.close(fig)
        print(json.dumps(r,ensure_ascii=False),flush=True)
        del d,x,y,q,power
        gc.collect()
    pd.DataFrame(records).to_csv(OUT/'record_metrics.csv',index=False)
    pd.DataFrame(events).to_csv(OUT/'impact_metrics.csv',index=False)
    pd.DataFrame(channels).to_csv(OUT/'channel_metrics.csv',index=False)
    (OUT/'source_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    (OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
    plot_comparison()
    refine_contact_comparison()


def plot_comparison():
    fig,axs=plt.subplots(4,3,figsize=(15,13),layout='constrained')
    colors=plt.cm.viridis(np.linspace(.12,.9,12))
    for row,token in enumerate(EXAMPLES):
        d=np.load(OUT/f'{token}_display.npz')
        rel=d['rel']; rr=d['trial_raw']; bb=d['trial_bp']
        # Fixed channel and fixed windows across every file; no latency estimation.
        for j in range(len(rr)):
            axs[row,0].plot(rel,rr[j,:,0],lw=.65,alpha=.5,color=colors[j%12])
            axs[row,1].plot(rel,bb[j,:,0],lw=.65,alpha=.5,color=colors[j%12])
        if len(rr):
            axs[row,0].plot(rel,np.median(rr[:,:,0],axis=0),color='black',lw=1.3)
            axs[row,1].plot(rel,np.median(bb[:,:,0],axis=0),color='black',lw=1.3)
            # Median absolute raw across all channels shows shared transient.
            n=np.median(np.abs(rr),axis=2)
            axs[row,2].plot(rel,np.median(n,axis=0),color='#a94735')
            axs[row,2].fill_between(rel,np.quantile(n,.25,axis=0),np.quantile(n,.75,axis=0),color='#a94735',alpha=.2)
        for col in range(3):
            ax=axs[row,col]; ax.axvline(0,color='gray',ls='--',lw=.8); ax.grid(alpha=.15)
            ax.set_xlim(-.2,.8); ax.ticklabel_format(axis='y',style='sci',scilimits=(-3,4))
        axs[row,0].set_ylabel(token+'\nADC counts')
        axs[row,1].axvspan(.02,.10,color='#f5c364',alpha=.2)
    for ax,title in zip(axs[0],['CH01 raw, baseline subtracted','CH01 bandpass 20-450 Hz + notch','Median absolute raw over 48 channels']):
        ax.set_title(title)
    for ax in axs[-1]: ax.set_xlabel('Time from accelerometer peak (s)')
    fig.suptitle('September vs October: each trace is one automatically detected impact\nBlack = median; raw and filtered panels use independent vertical scales. Yellow = 20-100 ms window.',fontsize=12)
    fig.savefig(OUT/'impact_comparison.png',dpi=170); plt.close(fig)


def refine_contact_comparison():
    """Choose October contact candidates using accel + motor log, never EMG.

    The largest accelerometer peak sometimes belongs to launch. A second large
    peak 80-250 ms later, near the motor SHOT event, is then used. Host log times
    have USB/BLE latency and are corroboration only. No physiological validation.
    """
    pat=re.compile(r'^\[(\d\d):(\d\d):(\d\d\.\d+)\] RX EVT SHOT (.*)')
    shots=[]
    log=ROOT/'outputs/motor_logs/motor_20261026_004714_201031.log'
    for line in log.read_text().splitlines():
        m=pat.match(line)
        if m:
            h,minute,s,body=m.groups()
            shots.append((int(h)*3600+int(minute)*60+float(s),body))
    allrows=[]; summaries=[]; decisions=[]
    fig,axs=plt.subplots(4,3,figsize=(15,12),layout='constrained')
    for row,token in enumerate(EXAMPLES):
        z=np.load(OUT/f'{token}_display.npz')
        rel=z['rel']; raw=z['trial_raw']; bp=z['trial_bp']; at=z['accel_t']; an=z['accel']
        imu=None
        if token.startswith('20261026'):
            imu=pd.read_csv(ROOT/f'data/imu_capture_{token}.csv')
            imu=imu[imu.source=='Accel'].reset_index(drop=True)
        peaks=[]; delays=[]; waves=[]; lateratios=[]; normalized=[]
        changed=0
        for j,tp in enumerate(z['valid_times']):
            shift=0.; ratio=None; shot_dt=None
            if imu is not None:
                w=(at>=tp+.08)&(at<tp+.25)
                si=np.flatnonzero(w)[np.argmax(an[w])]
                pi=np.argmin(np.abs(at-tp))
                ratio=float(an[si]/an[pi])
                ht=pd.to_datetime(imu.host_rx_time.iloc[pi],unit='s',utc=True).tz_convert('Asia/Shanghai')
                wall=ht.hour*3600+ht.minute*60+ht.second+ht.microsecond/1e6
                shot=min(shots,key=lambda s:abs(s[0]-wall))
                shot_dt=shot[0]-wall
                # An independent mechanical/log-based rule, fixed before looking
                # at EMG amplitude. Require both strong second accel and later SHOT.
                if ratio>.25 and .06<shot_dt<.25 and abs((at[si]-tp)-shot_dt)<.08:
                    shift=float(at[si]-tp);changed+=1
                decisions.append({'id':token,'event':j+1,'initial_accel_time_s':float(tp),
                                  'chosen_contact_time_s':float(tp+shift),'shift_ms':shift*1000,
                                  'second_over_first_accel':ratio,'shot_log_after_first_peak_host_ms':shot_dt*1000,
                                  'motor_event':shot[1]})
            tt=rel-shift
            pre=(tt>=-.8)&(tt<-.2); early=(tt>=.02)&(tt<.1); late=(tt>=.2)&(tt<.8)
            pw=(tt>=.015)&(tt<.08)
            pre_r=rms(bp[j,pre]); baseline=np.median(raw[j,pre],axis=0)
            rr=raw[j]-baseline
            pp=np.ptp(bp[j,pw],axis=0)
            peak=float(np.median(pp))
            # Peak timing of the signed spatial median pulse, not response onset.
            aa=np.abs(np.median(rr,axis=1))
            idx=np.flatnonzero(pw)[np.argmax(aa[pw])]
            delay=tt[idx]*1000
            lr=float(np.median(rms(bp[j,late])/pre_r))
            er=float(np.median(rms(bp[j,early])/pre_r))
            norm=bp[j,:,0]/np.max(np.abs(bp[j,pw,0]))
            grid=np.arange(-.1,.401,.0005)
            normalized.append(np.interp(grid,tt,norm))
            peaks.append(peak);delays.append(delay);lateratios.append(lr)
            allrows.append({'id':token,'event':j+1,'contact_candidate_s':float(tp+shift),
                            'contact_shift_from_largest_accel_ms':shift*1000,'pulse_peak_delay_ms':delay,
                            'median_channel_15_80ms_peak_to_peak_counts':peak,
                            'early_20_100ms_bp_rms_ratio':er,'late_200_800ms_bp_rms_ratio':lr})
            axs[row,0].plot(tt,bp[j,:,0],lw=.6,alpha=.45)
            axs[row,1].plot(tt,norm,lw=.6,alpha=.45)
            axs[row,2].plot(tt,rr[:,0],lw=.6,alpha=.45)
        n=np.asarray(normalized); template=np.median(n,axis=0)
        score=(grid>=.01)&(grid<.12)
        correlations=[float(np.corrcoef(w[score],template[score])[0,1]) for w in n]
        summaries.append({'id':token,'events':len(raw),'contact_candidates_shifted':changed,
                          'peak_to_peak_counts_median':float(np.median(peaks)),
                          'peak_to_peak_cv':float(np.std(peaks,ddof=1)/np.mean(peaks)),
                          'median_pulse_peak_delay_ms':float(np.median(delays)),
                          'pulse_peak_delay_min_ms':float(np.min(delays)),
                          'pulse_peak_delay_max_ms':float(np.max(delays)),
                          'median_late_bp_rms_ratio':float(np.median(lateratios)),
                          'median_normalized_waveform_correlation':float(np.median(correlations))})
        axs[row,1].plot(grid,template,color='black',lw=1.3)
        axs[row,0].set_ylabel(token+'\nADC counts')
        for c in range(3):
            axs[row,c].axvline(0,color='gray',ls='--',lw=.8);axs[row,c].grid(alpha=.15)
            axs[row,c].set_xlim(-.1,.4 if c<2 else .8)
    for ax,title in zip(axs[0],['CH01 20-450 Hz + 50 Hz notch','CH01 normalized by its own peak\nBlack = median waveform','CH01 raw, pre-contact baseline subtracted']):
        ax.set_title(title,fontsize=10)
    for ax in axs[-1]:ax.set_xlabel('Time from mechanically selected contact candidate (s)')
    fig.suptitle('Impact-aligned comparison after separating launch acceleration from contact candidates\nIndependent y-scales; raw units remain ADC counts. Candidate timing requires hardware confirmation.',fontsize=12)
    fig.savefig(OUT/'contact_aligned_comparison.png',dpi=170);plt.close(fig)
    pd.DataFrame(allrows).to_csv(OUT/'contact_aligned_impact_metrics.csv',index=False)
    pd.DataFrame(summaries).to_csv(OUT/'contact_aligned_summary.csv',index=False)
    pd.DataFrame(decisions).to_csv(OUT/'contact_selection.csv',index=False)
    print('CONTACT_ALIGNED_SUMMARY',json.dumps(summaries,indent=2),flush=True)


if __name__=='__main__':
    if '--refine-only' in sys.argv:
        refine_contact_comparison()
    else:
        run()
