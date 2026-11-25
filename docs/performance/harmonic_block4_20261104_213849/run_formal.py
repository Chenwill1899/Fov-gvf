#!/usr/bin/python3
import pathlib,json,hashlib,tarfile,subprocess,os,time
r=pathlib.Path(__file__).resolve().parent;s=json.load(open(r/'session.json'));p=r/'work/EGO4P0/src/new_controller';frozen=r/'frozen';frozen.mkdir(exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p2.relative_to(p)):sha(p2) for p2 in sorted(p.rglob('*')) if p2.is_file()}
(frozen/'source_sha256.json').write_text(json.dumps(files,indent=2)+'\n')
with tarfile.open(frozen/'source.tar.gz','w:gz') as t:t.add(p,arcname='new_controller')
(frozen/'binary_sha256.json').write_text(json.dumps({n:sha(r/'bin'/n) for n in ['pipeline_legacy','pipeline_small','sparse_trial_backend.o']},indent=2)+'\n')
source='/home/starry/isaac-data/EGO4P0/docs/performance/reader_jitter_20261102_165205/optimized_20261103_155735/fifo10_200us_r1'
env=dict(os.environ,LD_LIBRARY_PATH=str(r/'deps/suitesparse/usr/lib/x86_64-linux-gnu'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
commands=[]
for round_id in [1,2,3]:
 d=r/f'round_{round_id}';d.mkdir()
 order=[('legacy',3),('legacy',4),('small',4),('small',3)]
 if round_id==2:order.reverse()
 for policy,edge in order:
  out=d/f'{policy}_edge{edge}';cmd=['taskset','-c','0',str(r/'bin'/('pipeline_'+policy)),source,str(out),'40','eigen_ldlt_both',str(edge)]
  if round_id<3:cmd.append('no_exports')
  start=time.time()
  with (d/f'{policy}_edge{edge}.log').open('w') as log:cp=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
  commands.append({'round':round_id,'policy':policy,'edge':edge,'command':cmd,'exit_code':cp.returncode,'wall_s':time.time()-start})
  (r/'formal_execution.json').write_text(json.dumps({'environment':{k:env[k] for k in ['LD_LIBRARY_PATH','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},'commands':commands},indent=2)+'\n')
  print(round_id,policy,edge,cp.returncode,flush=True)
  if cp.returncode:raise SystemExit(cp.returncode)
assert all(sha(p/rel)==h for rel,h in files.items()),'source changed during timing'
print('formal completed; source frozen',len(files),flush=True)
