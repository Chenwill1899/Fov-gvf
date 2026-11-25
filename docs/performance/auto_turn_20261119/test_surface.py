"""Build and run the exact current controller's depth geometry fixtures on its frozen dependencies."""
from pathlib import Path
import json
import shlex
import subprocess
import tempfile

out=Path(__file__).resolve().parent
root=out.parents[2]
ref=root/'docs/performance/r05_model_sync_20261117'
base=json.loads((root/json.loads((ref/'latest_build.json').read_text())['manifest']).read_text())
build=(root/base['binary']['path']).parent
cm=build/'CMakeFiles/r05_controller.dir'
flags=[a for line in (cm/'flags.make').read_text().splitlines() if line.startswith('CXX_') for a in shlex.split(line.split(' = ',1)[1])]
probe=out.parent/'wp207_fifo_20261119/include'
cal=out.parent/'wp207_fifo_20261119/run_20261119_213433_672565/simulation/simulation.sensors.json'
with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
    obj=Path(tmp)/'test.o';exe=Path(tmp)/'test'
    subprocess.run(['/usr/bin/c++','-I'+str(probe),*flags,'-c',out/'surface_fixture.cpp','-o',obj],check=True)
    cmd=shlex.split((cm/'link.txt').read_text())
    cmd[cmd.index('CMakeFiles/r05_controller.dir/controller.cpp.o')]=str(obj)
    cmd[cmd.index('-o')+1]=str(exe)
    subprocess.run(cmd,cwd=build,check=True)
    check=subprocess.run([exe,cal],text=True,capture_output=True)
    (out/'surface_fixture.log').write_text(check.stdout+check.stderr)
    print(check.stdout+check.stderr,end='')
    check.check_returncode()
