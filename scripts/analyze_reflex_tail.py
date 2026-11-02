#!/usr/bin/env python3
"""Compare post-pulse tails, reproducing the scope48 MVC causal filter.

pixi run python scripts/analyze_reflex_tail.py
Reads original CSVs; writes only outputs/reflex_comparison_20261026/tail/.
No claim of neural response duration or causation by cable motion.
"""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/reflex-mpl')
import json
import numpy as np
import pandas as pd
from scipy import signal, ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/reflex_comparison_20261026'
OUT=BASE/'tail'
TOKENS=['20260922_153701','20260922_155509','20261026_005026','20261026_005904']
CHANNELS=[f'ch{i:02d}' for i in [1,8,16,24,32,40,48]]
FS=2000
BP=signal.butter(4,[20,450],btype='bandpass',fs=FS,output='sos')
NOTCH=signal.tf2sos(*signal.iirnotch(50,30,fs=FS))
SOS=np.vstack([NOTCH,BP])


def causal(sos,x):
    zi=signal.sosfilt_zi(sos)[:,:,None]*x[0][None,None,:]
    return signal.sosfilt(sos,x,axis=0,zi=zi)[0]


def rms(x):
    return np.sqrt(np.mean(x*x,axis=0))


def settle_time(t,x,pre):
    env=np.median(np.sqrt(ndimage.uniform_filter1d(x*x,40,axis=0)),axis=1)
    floor=2*np.median(rms(x[pre]))
    ok=env<=floor
    # First 100 ms continuous interval below 2x pre-pulse RMS, searched after 80 ms.
    width=200
    runs=np.convolve(ok.astype(int),np.ones(width,dtype=int),mode='valid')
    eligible=np.flatnonzero((t[:len(runs)]>=.08)&(t[:len(runs)]<.8)&(runs==width))
    return float(t[eligible[0]]*1000) if len(eligible) else np.nan


def run():
    OUT.mkdir(parents=True,exist_ok=True)
    records=pd.read_csv(BASE/'record_metrics.csv')
    contacts=pd.read_csv(BASE/'contact_aligned_impact_metrics.csv')
    results=[]; plotdata=[]
    for token in TOKENS:
        source=records.loc[records.id==token,'path'].iloc[0]
        d=pd.read_csv(source,usecols=['timestamp_us',*CHANNELS])
        x=d[CHANNELS].to_numpy(float)
        ts=d.timestamp_us.to_numpy(np.int64)
        t=(ts-ts[0])/1e6
        yc=causal(SOS,x); yb=causal(BP,x)
        # A synthetic input containing only the first 0-80 ms pulse tests how much
        # subsequent oscillation can arise with no further recorded activity.
        events=contacts[contacts.id==token]
        trial=[]
        for _,ev in events.iterrows():
            center=np.searchsorted(t,ev.contact_candidate_s)
            lo=center-2000; hi=center+2000
            if lo<0 or hi>len(x):continue
            tt=np.arange(-2000,2000)/FS
            raw=x[lo:hi].copy(); c=yc[lo:hi].copy(); b=yb[lo:hi].copy()
            pre=(tt>=-.8)&(tt<-.2)
            pulse=(tt>=0)&(tt<.08)
            synthetic=np.zeros_like(raw)
            ip=np.flatnonzero(pulse)
            line=raw[ip[0]]+(raw[ip[-1]]-raw[ip[0]])*np.linspace(0,1,len(ip))[:,None]
            synthetic[pulse]=raw[pulse]-line
            synth_c=signal.sosfilt(SOS,synthetic,axis=0)
            synth_b=signal.sosfilt(BP,synthetic,axis=0)
            # Difference is the isolated pulse's additional notch-related response.
            ring=synth_c-synth_b
            row={'id':token,'event':int(ev.event),'causal_return_2x_baseline_ms':settle_time(tt,c,pre),
                 'bp_only_return_2x_baseline_ms':settle_time(tt,b,pre)}
            for label,left,right in [('80_200',.08,.2),('200_500',.2,.5),('500_800',.5,.8)]:
                w=(tt>=left)&(tt<right)
                row[f'causal_{label}_rms_over_pre']=float(np.median(rms(c[w])/rms(c[pre])))
                row[f'bp_only_{label}_rms_over_pre']=float(np.median(rms(b[w])/rms(b[pre])))
                row[f'notch_pulse_ring_{label}_rms_counts']=float(np.median(rms(ring[w])))
                row[f'causal_{label}_rms_counts']=float(np.median(rms(c[w])))
                row[f'causal_minus_pulse_ring_{label}_rms_counts']=float(np.median(rms(c[w]-ring[w])))
            w=(tt>=.12)&(tt<.5)
            row['ch01_tail_vs_synthetic_ring_r']=float(np.corrcoef(c[w,0],ring[w,0])[0,1])
            # Frequency fit uses a complete 380 ms window (19 cycles at 50 Hz).
            basis=np.c_[np.sin(2*np.pi*50*tt[w]),np.cos(2*np.pi*50*tt[w]),np.ones(w.sum())]
            fit=basis@np.linalg.lstsq(basis,c[w],rcond=None)[0]
            row['tail_50hz_fit_power_fraction']=float(np.median(1-np.sum((c[w]-fit)**2,axis=0)/np.sum((c[w]-np.mean(c[w],axis=0))**2,axis=0)))
            results.append(row)
            trial.append({'t':tt,'c':c,'b':b,'raw':raw-np.median(raw[pre],axis=0),'ring':ring,'event':int(ev.event)})
        plotdata.append((token,trial))
        print(token,'processed',len(trial),flush=True)
        del x,yc,yb,d
    metrics=pd.DataFrame(results)
    metrics.to_csv(OUT/'tail_metrics.csv',index=False)
    summary=metrics.groupby('id').median(numeric_only=True).drop(columns='event')
    summary['events']=metrics.groupby('id').size()
    summary['no_causal_return_by_800ms']=metrics.groupby('id').causal_return_2x_baseline_ms.apply(lambda v:int(v.isna().sum()))
    summary['no_bp_only_return_by_800ms']=metrics.groupby('id').bp_only_return_2x_baseline_ms.apply(lambda v:int(v.isna().sum()))
    summary.to_csv(OUT/'tail_summary.csv')
    print(summary.round(3).to_string(),flush=True)
    fig,axs=plt.subplots(4,3,figsize=(15,12),layout='constrained')
    for row,(token,trials) in enumerate(plotdata):
        for tr in trials:
            tt=tr['t']
            axs[row,0].plot(tt,tr['c'][:,0],lw=.5,alpha=.35,color='#666666')
            axs[row,1].plot(tt,tr['b'][:,0],lw=.5,alpha=.35,color='#317ba7')
        # Median of individual RMS envelopes keeps non-phase-locked disturbances.
        tt=trials[0]['t']
        for key,label,color in [('c','Live filter','#b74a35'),('b','Bandpass only','#317ba7'),('ring','Isolated-pulse notch ringing','#9467bd')]:
            env=np.array([np.sqrt(ndimage.uniform_filter1d(tr[key][:,0]**2,40)) for tr in trials])
            axs[row,2].plot(tt,np.median(env,axis=0),lw=1.3,label=label,color=color)
        # A fixed tail zoom makes the small oscillations readable after the large pulse.
        lim=np.quantile(np.abs(np.concatenate([tr['c'][(tt>=.1)&(tt<.5),0] for tr in trials])),.98)
        for col in range(2):
            axs[row,col].set_ylim(-lim*1.15,lim*1.15)
            axs[row,col].set_xlim(.08,.8)
        axs[row,2].set_xlim(.08,.8)
        axs[row,0].set_ylabel(token+'\nCH01 counts')
        for ax in axs[row]:ax.grid(alpha=.2)
    for ax,title in zip(axs[0],['Tail after live causal filter\n20-450 Hz + 50 Hz notch, Q=30','Same data with 20-450 Hz bandpass only\nDiagnostic comparison','Median 20 ms RMS envelope\nPulse-only simulation is a contribution, not a new recording']):
        ax.set_title(title,fontsize=10)
    for ax in axs[-1]:ax.set_xlabel('Time after contact candidate (s)')
    axs[0,2].legend(fontsize=7)
    fig.suptitle('Post-pulse tail comparison: replay of the actual live filter\nPulse is outside this zoom; y-scales differ across rows but match within each row.',fontsize=12)
    fig.savefig(OUT/'filtered_tail_comparison.png',dpi=170);plt.close(fig)
    # Show every final-session event, with the main pulse hidden in the tail zoom.
    token,trials=plotdata[-1]
    fig,axs=plt.subplots(4,1,figsize=(13,10),sharex=True,layout='constrained')
    for ax,tr in zip(axs,trials):
        tt=tr['t']
        ax.plot(tt,tr['c'][:,0],color='#b74a35',lw=1,label='Live filter')
        ax.plot(tt,tr['b'][:,0],color='#317ba7',lw=.8,label='Bandpass only')
        ax.plot(tt,tr['ring'][:,0],color='#9467bd',lw=.8,ls='--',label='Pulse-only notch ringing')
        ax.set_xlim(.08,.6);ax.grid(alpha=.2);ax.set_ylabel(f'Event {tr["event"]}\nCH01 counts')
        w=(tt>=.1)&(tt<.6)
        lim=np.quantile(np.abs(tr['c'][w,0]),.99)*1.3
        ax.set_ylim(-lim,lim)
    axs[0].legend(ncol=3,fontsize=9);axs[-1].set_xlabel('Time after contact candidate (s)')
    fig.suptitle('October 26, 00:59 recording: each post-pulse tail separately')
    fig.savefig(OUT/'last_four_tail_filter_test.png',dpi=170);plt.close(fig)


if __name__=='__main__':run()
