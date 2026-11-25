#!/usr/bin/python3
import pathlib,subprocess,json,concurrent.futures
root=pathlib.Path(__file__).resolve().parent;p=root/'work/EGO4P0/src/new_controller';d=json.load(open(root/'backend_build.json'));commands=[]
def build(item):
 name,guard,small=item;cmd=['/usr/bin/g++',*d['flags'],*d['includes'],f'-DEGO4P0_CURVE_GUARD={guard}',f'-DEGO4P0_CORNER_SMALL_STEP={small}','-DEGO4P0_COARSE_PC_MODE=2',str(p/'test/harmonic_fields_benchmark.cpp'),str(root/'bin/sparse_trial_backend.o'),*d['link_flags'],'-o',str(root/'bin'/('pipeline_'+name))];commands.append(cmd)
 with (root/'build_logs'/('pipeline_'+name+'.log')).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 print(name,r.returncode,flush=True);return r.returncode
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:codes=list(pool.map(build,[('legacy',0,0),('small',1,1)]))
(root/'pipeline_build_commands.json').write_text(json.dumps(commands,indent=2)+'\n');raise SystemExit(max(codes))
