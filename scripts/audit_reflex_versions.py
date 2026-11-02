#!/usr/bin/env python3
"""Recover local ADS source history without modifying active sources/devices."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import ast
import hashlib
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT.parent/'无线采集系统-v2'
OUT=ROOT/'outputs/reflex_comparison_20261026/version_audit'
HISTORY=Path('/Users/haruka/Library/Application Support/Code/User/History/-7834d348')
TZ=timezone(timedelta(hours=8))
KEYS=['ENABLED_CHANNEL_COUNT','ENABLE_INTERNAL_TEST_SIGNAL','ENABLE_RLD',
      'RLD_SOURCE_CHANNEL_COUNT','RLD_USE_P_INPUTS','RLD_USE_N_INPUTS',
      'RLD_OUTPUT_CHANNEL_BLOCK','NORMAL_CH_CONFIG']


def sha(data):return hashlib.sha256(data).hexdigest()


def config(s):
    out={}
    for key in KEYS:
        m=re.search(r'constexpr\s+\w+\s+'+key+r'\s*=\s*([^;]+);',s)
        out[key]=m.group(1).strip() if m else None
    return out


def class_tree(p,name):
    tree=ast.parse(p.read_text())
    c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
    return ast.dump(c,include_attributes=False)


def run():
    OUT.mkdir(parents=True,exist_ok=True)
    history=json.loads((HISTORY/'entries.json').read_text())
    snapshots=[]
    for e in history['entries']:
        p=HISTORY/e['id']; b=p.read_bytes()
        dt=datetime.fromtimestamp(e['timestamp']/1000,TZ)
        target=OUT/f'ads_{dt:%Y%m%d_%H%M%S}_{e["id"]}'
        target.write_bytes(b)
        snapshots.append({'history_time':dt.isoformat(),'history_source':str(p),
                          'recovered_path':str(target),'sha256':sha(b),'config':config(b.decode())})
    keyfiles=['lib/ads1298_48ch_driver/ads1298_48ch_driver.cpp',
              'lib/ads1298_48ch_driver/ads1298_48ch_driver.h','src/master/main.cpp',
              'lib/master_node/master_node.cpp','tools/scope48_mvc.py','tools/scope48.py']
    files=[]
    for rel in keyfiles:
        old=OLD/rel; new=ROOT/rel
        if not old.exists():continue
        a=old.read_bytes(); b=new.read_bytes() if new.exists() else None
        files.append({'relative_path':rel,'v2_path':str(old),'v2_sha256':sha(a),
                      'v3_sha256':sha(b) if b is not None else None,
                      'identical':a==b,'v2_mtime':datetime.fromtimestamp(old.stat().st_mtime,TZ).isoformat()})
    driver=ROOT/keyfiles[0]
    latest=snapshots[-1]
    oldclass=class_tree(OLD/'tools/scope48_mvc.py','EmgLiveFilter')
    newclass=class_tree(ROOT/'tools/scope48_mvc.py','EmgLiveFilter')
    commits=subprocess.check_output(['git','log','--all','--date=iso-strict','--format=%H %ad %s'],cwd=ROOT,text=True).splitlines()
    artifacts=[]
    for root in [OLD,ROOT]:
        f=root/'.pio/build/master/firmware.bin'
        if f.exists():
            artifacts.append({'path':str(f),'size':f.stat().st_size,
                              'mtime':datetime.fromtimestamp(f.stat().st_mtime,TZ).isoformat(),
                              'sha256':sha(f.read_bytes()),
                              'historical_recording_binding':'unavailable; mtime is not proof of compilation or flashing'})
    result={'history_resource':history['resource'],'snapshots':snapshots,'file_comparison':files,
            'september_13_snapshot_identical_to_current_driver':latest['sha256']==sha(driver.read_bytes()),
            'mvc_live_filter_ast_identical':oldclass==newclass,
            'mvc_live_filter_ast_sha256':sha(oldclass.encode()),'git_commits':commits,
            'firmware_artifacts':artifacts,
            'limitation':'Source history is recoverable. CSVs contain no firmware revision or ADS register dump; the flashed firmware for each recording is unproven.'}
    assert result['september_13_snapshot_identical_to_current_driver']
    assert result['mvc_live_filter_ast_identical']
    assert all(sha(Path(s['history_source']).read_bytes())==sha(Path(s['recovered_path']).read_bytes()) for s in snapshots)
    (OUT/'version_evidence.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({'recovered_snapshots':len(snapshots),'latest_history_time':latest['history_time'],
                      'latest_driver_sha256':latest['sha256'],'current_driver_matches':True,
                      'mvc_filter_class_matches':True,'matching_files':[f['relative_path'] for f in files if f['identical']]},indent=2))


if __name__=='__main__':run()
