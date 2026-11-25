"""Controlled same-state counterfactual with synthetic raw images, using the measured ELF."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

import numpy as np

out=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=out/'depth_gate_test.json')
report_path=parser.parse_args().output
root=out.parents[2]
reference=root/'docs/performance/wp207_fifo_20261119/run_20261119_213433_672565/simulation'
cfg=json.loads((root/'src/auto_cruise/scenarios.json').read_text())['r05']
with (reference/'trajectory.csv').open() as file:
    states=list(csv.DictReader(file))[:1806]
guide=reference/'intent_waypoints.csv'
points=np.loadtxt(guide,delimiter=',',skiprows=1)
delta=np.diff(points,axis=0)
length=np.linalg.norm(delta,axis=1)
arc=np.r_[0.,np.cumsum(length)]
far=np.full((4,240,320),4.5,'f4').tobytes()
near=np.full((4,240,320),.65,'f4').tobytes()
missing=np.full((4,240,320),np.nan,'f4').tobytes()

def run(replace_step=None, depth=far):
    with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
        folder=Path(tmp)
        shutil.copyfile(guide,folder/guide.name)
        scene=root/cfg['assets']
        manifest=root/json.loads((out/'latest_build.json').read_text())['manifest']
        binary=root/json.loads(manifest.read_text())['binary']['path']
        cmd=[binary,reference/'simulation.sensors.json',scene/cfg['scene'],scene/'occupancy.bin',
             folder/'controller.csv','search','normal',*map(str,cfg['start'])]
        with subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE) as proc:
            offset=0.
            for i,(before,after) in enumerate(zip(states,states[1:]),1):
                pos=np.array([float(before[k]) for k in 'xyz'])
                vel=np.array([float(before[k]) for k in ('vx','vy','vz')])
                idx=int(after['waypoint'])
                frac=np.clip(np.dot(pos-points[idx],delta[idx])/length[idx]**2,0.,1.)
                offset=max(offset,arc[idx]+frac*length[idx])
                aim=[np.interp(min(arc[-1],offset+.55+.08*np.linalg.norm(vel)),arc,axis) for axis in points.T]
                proc.stdin.write(struct.pack('=12d',*pos,*vel,float(before['yaw_rad']),*aim,0.,float(after['guide_cap'])))
                proc.stdin.write(depth if i==replace_step else far)
                proc.stdin.flush()
                reply=struct.unpack('=4d',proc.stdout.read(32))
                if int(reply[-1])==7 or i==replace_step:
                    break
            proc.stdin.close()
            if proc.wait(timeout=8)!=0:
                raise RuntimeError(proc.stderr.read().decode())
        with (folder/'controller.csv').open() as stream:
            row=list(csv.DictReader(stream))[-1]
        return {k:row[k] for k in ('step','mode','turn_requested','turn_check','lateral_demand_m')}

good=run()
assert good['turn_check']=='1'
step=int(good['step'])
result={'observed_clear':good,'obstacle':run(step,near),'unknown':run(step,missing)}
for name in ('obstacle','unknown'):
    r=result[name]
    assert r['step']==good['step'] and r['lateral_demand_m']==good['lateral_demand_m'] and r['turn_requested']=='1'
    assert r['turn_check']=='3' and r['mode']!='7'
assert result['obstacle']['mode'] in ('3','5'), 'known obstacle should brake in this fixture'
report_path.write_text(json.dumps(result,indent=2)+'\n')
print(result)
