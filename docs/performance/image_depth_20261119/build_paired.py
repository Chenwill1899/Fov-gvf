"""Compile the isolated same-frame benchmark against the experiment's measured core."""
import json
import shlex
import subprocess
from pathlib import Path
out=Path(__file__).resolve().parent
root=out.parents[2]
ref=root/'docs/performance/r05_model_sync_20261117'
m=json.loads((root/json.loads((ref/'latest_build.json').read_text())['manifest']).read_text())
build=(root/m['binary']['path']).parent
cm=build/'CMakeFiles/r05_controller.dir'
flags=[a for line in (cm/'flags.make').read_text().splitlines() if line.startswith('CXX_') for a in shlex.split(line.split(' = ',1)[1])]
obj=out/'paired_queries.o'
subprocess.run(['/usr/bin/c++','-I'+str(out.parent/'wp207_fifo_20261119/include'),*flags,'-c',out/'paired_queries.cpp','-o',obj],check=True)
cmd=shlex.split((cm/'link.txt').read_text());cmd[cmd.index('CMakeFiles/r05_controller.dir/controller.cpp.o')]=str(obj)
cmd[cmd.index('-o')+1]=str(out/'paired_queries')
subprocess.run(cmd,cwd=build,check=True)
