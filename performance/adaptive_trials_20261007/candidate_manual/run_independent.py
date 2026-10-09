#!/usr/bin/env python3
"""Rebuild archived manual candidate with matching frozen core; never run Isaac."""
import argparse,csv,json,math,subprocess,tarfile,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
folder=Path(__file__).resolve().parent;base=folder.parent/'baseline';sources=folder/'candidate_sources';runs=[]
with tempfile.TemporaryDirectory(prefix='p5_continuous_proposal_') as temp:
 w=Path(temp)
 with tarfile.open(base/'sources.tar.gz') as archive:
  for member in archive.getmembers():
   if member.isfile() and (member.name.startswith('src/pc_gvf/include/') or member.name=='src/pc_gvf/src/omni_depth.cpp'):
    dest=w/'frozen'/member.name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(archive.extractfile(member).read())
 includes=['-I',str(sources/'src/pc_gvf/include'),'-I',str(w/'frozen/src/pc_gvf/include'),'-isystem','/usr/include/eigen3']
 common=['/usr/bin/c++','-O3','-DNDEBUG','-fopenmp',*includes]
 commands=[[*common,'-c',str(sources/'src/pc_gvf/src/omni_depth.cpp'),'-o',str(w/'candidate.o')],
  [*common,'-DomniDepthProposal=baselineOmniDepthProposal','-c',str(w/'frozen/src/pc_gvf/src/omni_depth.cpp'),'-o',str(w/'baseline.o')]]
 for command in commands:subprocess.run(command,check=True)
 for name,src,extra in [('omni_depth_check',sources/'src/pc_gvf/test/cpp/omni_depth_check.cpp',[]),('parity_probe',folder/'parity_probe.cpp',[str(w/'baseline.o')]),('moving_boundary_probe',folder/'moving_boundary_probe.cpp',[])]:
  command=[*common,str(src),str(w/'candidate.o'),*extra,str(base/'libpc_gvf_depth_angular_core.a'),'-lcrypto','-o',str(w/name)]
  subprocess.run(command,check=True);run=subprocess.run([str(w/name)],capture_output=True,text=True)
  (a.output/(name+'.stdout')).write_text(run.stdout);(a.output/(name+'.stderr')).write_text(run.stderr);runs.append({'check':name,'returncode':run.returncode})
  if run.returncode:break
(a.output/'summary.json').write_text(json.dumps(runs,indent=2)+'\n');assert all(r['returncode']==0 for r in runs)
print(json.dumps(runs,indent=2))
