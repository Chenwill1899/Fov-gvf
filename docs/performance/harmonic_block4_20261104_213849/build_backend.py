#!/usr/bin/python3
import pathlib,subprocess,json,os,hashlib
root=pathlib.Path(__file__).resolve().parent;p=root/'work/EGO4P0/src/new_controller';deps=root/'deps';lib=deps/'suitesparse/usr/lib/x86_64-linux-gnu'
flags=['-O3','-DNDEBUG','-std=c++17','-Wall','-Wextra','-Wpedantic','-Wno-unknown-pragmas','-fno-fast-math','-pthread','-DAMGCL_NO_BOOST']
incs=['-I'+str(p/'include'),'-I/usr/include/eigen3','-I'+str(deps/'amgcl-1.4.9'),'-I'+str(deps/'suitesparse/usr/include/suitesparse')]
cmd=['/usr/bin/g++',*flags,*incs,'-c',str(p/'src/sparse_trial_backend.cpp'),'-o',str(root/'bin/sparse_trial_backend.o')]
with open(root/'build_logs/backend_final_compile.log','w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
metadata={'flags':flags,'includes':incs,'command':cmd,'exit_code':r.returncode,'backend_cpp_sha256':hashlib.sha256((p/'src/sparse_trial_backend.cpp').read_bytes()).hexdigest(),'link_flags':['-L'+str(lib),'-Wl,--disable-new-dtags,-rpath,'+str(lib),'-lcholmod','-lyaml-cpp','-pthread'],'env':{'LD_LIBRARY_PATH':str(lib),'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}}
(root/'backend_build.json').write_text(json.dumps(metadata,indent=2)+'\n');print('backend compile',r.returncode,flush=True);raise SystemExit(r.returncode)
