import json,pathlib,subprocess,os
r=pathlib.Path(__file__).resolve().parent;root=r/'pilot';root.mkdir(exist_ok=True)
env=dict(os.environ,LD_LIBRARY_PATH=str(r/'deps/suitesparse/usr/lib/x86_64-linux-gnu'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
source='/home/starry/isaac-data/EGO4P0/docs/performance/reader_jitter_20261102_165205/optimized_20261103_155735/fifo10_200us_r1'
commands=[]
for edge in [3,4]:
 out=root/f'edge{edge}'
 cmd=['taskset','-c','0',str(r/'bin/pipeline_small'),source,str(out),'2','eigen_ldlt_both',str(edge)];commands.append(cmd)
 with (root/f'edge{edge}.log').open('w') as f:p=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
 print(edge,p.returncode,flush=True)
 if p.returncode:raise SystemExit(p.returncode)
(root/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
