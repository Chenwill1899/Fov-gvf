#!/usr/bin/env python3
"""Frozen-header comparison of the actual old and new OperatorIntent classes."""
import argparse,csv,hashlib,json,math,subprocess,tarfile,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
root=Path(__file__).resolve().parents[3];base=root/'performance/six_axis_refinement_20261007/baseline'
fixture=root/'src/pc_gvf/test/fixtures/paper/isaac_operator_jitter_trace.json'
events=json.loads(fixture.read_text())['events'];results=[];closed=[]
with tempfile.TemporaryDirectory(prefix='p5_operator_ablation_') as temp:
 w=Path(temp)
 with tarfile.open(base/'sources.tar.gz') as archive:
  for member in archive.getmembers():
   if member.isfile() and member.name.startswith('src/pc_gvf/include/'):
    dest=w/'old'/member.name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(archive.extractfile(member).read())
 old=w/'old/src/pc_gvf/include';new=w/'new/pc_gvf';new.mkdir(parents=True)
 (new/'operator_intent.hpp').write_bytes((root/'src/pc_gvf/include/pc_gvf/operator_intent.hpp').read_bytes())
 source=w/'signal.cpp';source.write_text('''#include "pc_gvf/operator_intent.hpp"
#include <iostream>
#include <iomanip>
using namespace pc_gvf::depth_angular;
int main(){OperatorIntent f;double t,x,y,z;std::cout<<std::setprecision(17);while(std::cin>>t>>x>>y>>z){auto q=f.update(Eigen::Vector3d(x,y,z),true);std::cout<<t<<","<<q.x()<<","<<q.y()<<","<<q.z()<<"\\n";}}
''')
 for mode in ['old','new']:
  inc=(['-I',str(new.parent)] if mode=='new' else [])+['-I',str(old),'-isystem','/usr/include/eigen3']
  exe=w/(mode+'_signal');subprocess.run(['/usr/bin/c++','-O3','-DNDEBUG',*inc,str(source),'-o',str(exe)],check=True)
  check=w/(mode+'_closed_loop');subprocess.run(['/usr/bin/c++','-O3','-DNDEBUG','-fopenmp',*inc,str(root/'src/pc_gvf/test/cpp/operator_intent_check.cpp'),str(base/'libpc_gvf_depth_angular_core.a'),'-lcrypto','-o',str(check)],check=True)
  run=subprocess.run([str(check)],text=True,capture_output=True);(a.output/(mode+'_closed_loop.csv')).write_text(run.stdout);(a.output/(mode+'_closed_loop.stderr')).write_text(run.stderr)
  closed.append({'mode':mode,'returncode':run.returncode,**next(csv.DictReader(run.stdout.splitlines()))})
  for hz in [50,60,120]:
   for phase_ticks in [0,.5]:
    rows=[];j=0
    for k in range(34*hz+1):
     t=(k+phase_ticks)/hz
     while j+1<len(events) and events[j+1]['at']<=t+1e-12:j+=1
     rows.append([t,*events[j]['velocity']])
    raw=''.join(' '.join(map(str,r))+'\n' for r in rows)
    run=subprocess.run([str(exe)],input=raw,text=True,capture_output=True,check=True)
    vals=[[float(s) for s in line.split(',')] for line in run.stdout.splitlines()]
    if hz==60 and phase_ticks==0:(a.output/(mode+'_fixture_60hz.csv')).write_text('t,x,y,z\n'+run.stdout)
    max_error=0.;max_norm_error=0.;release_mismatches=0;large_turn_mismatches=0
    for i,(raw,out) in enumerate(zip(rows,vals)):
     norm=math.sqrt(sum(x*x for x in raw[1:]));outnorm=math.sqrt(sum(x*x for x in out[1:]));max_norm_error=max(max_norm_error,abs(norm-outnorm))
     if not norm:release_mismatches+=outnorm!=0;continue
     dot=sum(x*y for x,y in zip(raw[1:],out[1:]));cross=(raw[2]*out[3]-raw[3]*out[2],raw[3]*out[1]-raw[1]*out[3],raw[1]*out[2]-raw[2]*out[1]);max_error=max(max_error,math.atan2(math.sqrt(sum(x*x for x in cross)),dot))
     if i and sum((rows[i][j]-rows[i-1][j])**2 for j in range(1,4))>.1:large_turn_mismatches+=sum((raw[j]-out[j])**2 for j in range(1,4))>1e-20
    for phase,lo,hi in [('all',0,34),('jitter',1.5,15),('slow_turn',17,23)]:
     q=[r[1:] for r in vals if lo<=r[0]<hi];delta=[[b-a for a,b in zip(q[i-1],q[i])] for i in range(1,len(q))];jerk=[[hz*hz*(b-a) for a,b in zip(delta[i-1],delta[i])] for i in range(1,len(delta))]
     results.append(dict(mode=mode,hz=hz,phase_ticks=phase_ticks,phase=phase,total_variation=sum(math.sqrt(sum(z*z for z in d)) for d in delta),intent_jerk_rms=math.sqrt(sum(sum(z*z for z in d) for d in jerk)/len(jerk)),max_angle_error=max_error,max_norm_error=max_norm_error,release_mismatches=release_mismatches,large_turn_mismatches=large_turn_mismatches))
summary=dict(fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),new_header_sha256=hashlib.sha256((root/'src/pc_gvf/include/pc_gvf/operator_intent.hpp').read_bytes()).hexdigest(),closed_loop=closed,signal_metrics=results)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for r in results:
 if r['hz']==60 and r['phase_ticks']==0:print(r)
assert all(r['returncode']==0 for r in closed)
assert all(r['release_mismatches']==0 and r['large_turn_mismatches']==0 and r['max_angle_error']<.018001 and r['max_norm_error']<1e-12 for r in results)
for old in results:
 if old['mode']=='old' and old['phase']=='jitter':
  new=next(r for r in results if r['mode']=='new' and all(r[k]==old[k] for k in ['hz','phase_ticks','phase']))
  assert new['intent_jerk_rms']<old['intent_jerk_rms'] and new['total_variation']<old['total_variation']
