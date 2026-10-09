#!/usr/bin/env python3
"""Bounded offline sweep of proof depth/work limits; never installs binaries."""
from pathlib import Path
import json,os,subprocess,time
out=Path(__file__).resolve().parent;root=out.parents[1]
work=Path('/tmp/ego1p3_union_cert_diagnostic')
source=work/'paper_guidance.cpp';code=source.read_text()
code='#include <cstdlib>\n'+code
code=code.replace('int remaining=8192;','const int limit=std::getenv("EGO_DIAG_LEVEL")?std::atoi(std::getenv("EGO_DIAG_LEVEL")):5;\n    int remaining=std::getenv("EGO_DIAG_BUDGET")?std::atoi(std::getenv("EGO_DIAG_BUDGET")):1024;')
code=code.replace('if(level>=8 || --remaining<=0)return false;','if(level>=limit || --remaining<=0)return false;')
assert 'level>=limit' in code
source=work/'paper_guidance_variable.cpp';source.write_text(code)
exe=work/'variable_replay'
command=['g++','-std=c++17','-O2','-fopenmp','-I/usr/include/eigen3','-I'+str(root/'src/pc_gvf/include'),str(work/'depth_angular_core.cpp'),str(source),str(work/'paper_replay.cpp'),str(work/'replay_main.cpp'),'-o',str(exe)]
with (out/'union_limits_compile.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
snapshot=root/'performance/navigation_benchmark_20260929/omni_final_v17_ego1p3_3_replays/frame_0.bin';rows=[]
for level,budget in [(5,1024),(5,8192),(6,1024),(7,1024),(8,1024),(8,4096)]:
 env=dict(os.environ,EGO_DIAG_LEVEL=str(level),EGO_DIAG_BUDGET=str(budget))
 log=out/f'union_limits_l{level}_b{budget}.txt'
 start=time.monotonic()
 with log.open('w') as stream:r=subprocess.run([str(exe),str(snapshot),'--audit'],stdout=stream,stderr=subprocess.STDOUT,env=env,timeout=120)
 lines=log.read_text().splitlines()
 no_audit=[]
 for repeat in range(5):
  t=time.monotonic();x=subprocess.run([str(exe),str(snapshot)],capture_output=True,text=True,env=env,timeout=60,check=True);no_audit.append(1000*(time.monotonic()-t))
 rows.append(dict(level=level,budget=budget,returncode=r.returncode,audit_wall_s=time.monotonic()-start,known=[line for line in lines if line.startswith('known_body_plus')],decision=json.loads(x.stdout),process_wall_ms=no_audit))
 (out/'union_limits_results.json').write_text(json.dumps(rows,indent=2)+'\n')
 print(json.dumps(rows[-1]),flush=True)
