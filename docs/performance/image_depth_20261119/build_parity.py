from pathlib import Path
import hashlib
import json
import shlex
import subprocess

out=Path(__file__).resolve().parent
root=out.parents[2]
exp=out.parent/'auto_turn_20261119'
m=json.loads((root/json.loads((exp/'latest_build.json').read_text())['manifest']).read_text())
old=json.loads((root/json.loads((out/'before/latest_build.json').read_text())['manifest']).read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(out/'before/controller.cpp')==old['source_identity']['apps/auto_cruise/controller.cpp']
ref=root/'docs/performance/r05_model_sync_20261117'
b=json.loads((root/json.loads((ref/'latest_build.json').read_text())['manifest']).read_text())
build=(root/b['binary']['path']).parent
cm=build/'CMakeFiles/r05_controller.dir'
flags=[x for l in (cm/'flags.make').read_text().splitlines() if l.startswith('CXX_') for x in shlex.split(l.split(' = ',1)[1])]
dest=out/'parity'
dest.mkdir(exist_ok=True)
objs=[]
for name in ('legacy_surface','parity_controller'):
    src=out/(name+'.cpp'); obj=dest/(name+'.o')
    subprocess.run(['/usr/bin/c++','-I'+str(out.parent/'wp207_fifo_20261119/include'),*flags,'-c',src,'-o',obj],check=True)
    objs.append(str(obj))
cmd=shlex.split((cm/'link.txt').read_text());idx=cmd.index('CMakeFiles/r05_controller.dir/controller.cpp.o')
cmd[idx:idx+1]=objs
exe=dest/'parity-controller';cmd[cmd.index('-o')+1]=str(exe)
subprocess.run(cmd,cwd=build,check=True)
for f in (out/'before/controller.cpp',out/'legacy_surface.cpp',out/'parity_controller.cpp',out/'build_parity.py',out/'run_parity.py'):
    m['source_identity'][str(f.relative_to(root))]=sha(f)
m['binary']={'path':str(exe.relative_to(root)),'sha256':sha(exe)}
(dest/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
(dest/'latest_build.json').write_text(json.dumps({'manifest':str((dest/'manifest.json').relative_to(root))})+'\n')
