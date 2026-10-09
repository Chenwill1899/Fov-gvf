#!/usr/bin/env python3
"""Offline shadow experiment; never install or run against live ROS/Isaac."""
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile,time
out=Path(__file__).resolve().parent
root=out.parents[1]
work=Path('/tmp/ego1p3_union_cert_diagnostic')
work.mkdir(exist_ok=False)
files=['src/pc_gvf/src/depth_angular_core.cpp','src/pc_gvf/src/paper_guidance.cpp','src/pc_gvf/src/paper_replay.cpp','src/pc_gvf/tools/paper_replay.cpp']
with tarfile.open(out/'v17_frozen/sources.tar.gz') as tar:
 for name in files:
  dst=work/('replay_main.cpp' if '/tools/' in name else Path(name).name)
  dst.write_bytes(tar.extractfile(name).read())
source=work/'paper_guidance.cpp';code=source.read_text()
assert code.count('int remaining=1024;')==1
assert code.count('if(level>=5 || --remaining<=0)return false;')==1
source.write_text(code.replace('int remaining=1024;','int remaining=8192;').replace('if(level>=5 || --remaining<=0)return false;','if(level>=8 || --remaining<=0)return false;'))
exe=work/'shadow_replay'
command=['g++','-std=c++17','-O2','-fopenmp','-I/usr/include/eigen3','-I'+str(root/'src/pc_gvf/include'),str(work/'depth_angular_core.cpp'),str(source),str(work/'paper_replay.cpp'),str(work/'replay_main.cpp'),'-o',str(exe)]
with (out/'union_refinement_compile.log').open('w') as log:
 subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
snapshot=root/'performance/navigation_benchmark_20260929/omni_final_v17_ego1p3_3_replays/frame_0.bin'
start=time.monotonic()
with (out/'union_refinement_audit.txt').open('w') as log:
 result=subprocess.run([str(exe),str(snapshot),'--audit'],stdout=log,stderr=subprocess.STDOUT,timeout=120)
metadata={'purpose':'Offline refinement diagnostic only, not an installed controller or navigation result','octree_level':8,'box_budget':8192,'returncode':result.returncode,'audit_wall_s':time.monotonic()-start,'shadow_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'shadow_binary_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),'snapshot_sha256':hashlib.sha256(snapshot.read_bytes()).hexdigest(),'compile_command':command}
(out/'union_refinement_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata,indent=2))
