#!/usr/bin/env python3
"""Compare frozen and block-pruned spherical maintenance on identical inputs."""
import argparse,csv,hashlib,json,statistics,subprocess,tarfile,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--repeats',type=int,default=7);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
root=Path(__file__).resolve().parents[3];base=root/'performance/six_axis_refinement_20261007/baseline';rows=[];checks=[]
with tempfile.TemporaryDirectory(prefix='p5_memory_ablation_') as temp:
 w=Path(temp)
 with tarfile.open(base/'sources.tar.gz') as archive:
  for member in archive.getmembers():
   if member.isfile() and member.name.startswith('src/pc_gvf/include/'):
    dest=w/member.name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(archive.extractfile(member).read())
 common=['/usr/bin/c++','-O3','-DNDEBUG','-fopenmp','-I',str(w/'src/pc_gvf/include'),'-isystem','/usr/include/eigen3']
 for mode in ['old','new']:
  for check in ['spherical_ingest_check','spherical_memory_check']:
   sources=[str(root/'src/pc_gvf/test/cpp'/f'{check}.cpp')]
   if mode=='new':sources.append(str(root/'src/pc_gvf/src/spherical_memory.cpp'))
   subprocess.run([*common,*sources,str(base/'libpc_gvf_depth_angular_core.a'),'-lcrypto','-o',str(w/f'{mode}_{check}')],check=True)
  run=subprocess.run([str(w/f'{mode}_spherical_memory_check')],text=True,capture_output=True)
  (a.output/f'{mode}_memory_check.csv').write_text(run.stdout);(a.output/f'{mode}_memory_check.stderr').write_text(run.stderr);checks.append(dict(mode=mode,returncode=run.returncode))
 for repeat in range(a.repeats):
  for mode in (['old','new'] if repeat%2==0 else ['new','old']):
   run=subprocess.run([str(w/f'{mode}_spherical_ingest_check')],text=True,capture_output=True)
   (a.output/f'{mode}_ingest_{repeat}.csv').write_text(run.stdout);(a.output/f'{mode}_ingest_{repeat}.stderr').write_text(run.stderr)
   rows.append(dict(mode=mode,repeat=repeat,returncode=run.returncode,**next(csv.DictReader(run.stdout.splitlines()))));print(rows[-1],flush=True)
 medians={mode:statistics.median(float(r['broadphase_ms']) for r in rows if r['mode']==mode) for mode in ['old','new']}
 summary=dict(source_sha256=hashlib.sha256((root/'src/pc_gvf/src/spherical_memory.cpp').read_bytes()).hexdigest(),results=rows,memory_checks=checks,median_ms=medians,speedup=medians['old']/medians['new'])
 (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 assert all(r['returncode']==0 for r in rows+checks)
 assert all(r['same_retained_evidence']=='1' and r['seeded_parity_cases']=='160' and r['large_world_revocations']=='5' for r in rows)
 assert medians['new']<medians['old']
