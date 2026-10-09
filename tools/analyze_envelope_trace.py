#!/usr/bin/env python3
"""Compare stalls and independently check swept bounding spheres against scene voxels."""
import argparse
import collections
import csv
import json
import hashlib
import math
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('trace',type=Path)
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
rows=list(csv.DictReader(args.trace.open()))
occupancy=root/'scenes/ego_swarm_cloud/occupancy.bin'
scene=root/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'
if hashlib.sha256(scene.read_bytes()).hexdigest()!='43f085ddc46a4391dcd743488af77cf08850a9e279b42252bd744d39057b35b2':
    raise SystemExit('USD differs from the calibrated sphere/voxel geometry')
grid=np.fromfile(occupancy,np.uint8).reshape(400,300,50)
origin=np.array([-80.,-60.,0.]);resolution=.4
minimum_clearance=float('inf');overlap_segments=0
positions=np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
# Occupied USD cubes are centered at origin + index*0.4 with half size 0.2.
# Check a sphere enclosing each complete measured center segment. This is an
# offline oracle, never a source of navigation free-space certificates.
for a,b in zip(positions[:-1],positions[1:]):
    center=(a+b)/2;radius=.48+np.linalg.norm(b-a)/2
    index=np.floor((center-origin)/resolution+.5).astype(int)
    lo=np.maximum(0,index-3);hi=np.minimum(grid.shape,index+4)
    if np.any(lo>=hi):
        minimum_clearance=float('-inf');overlap_segments+=1;continue
    occupied=np.argwhere(grid[lo[0]:hi[0],lo[1]:hi[1],lo[2]:hi[2]])+lo
    clearance=float('inf')
    if len(occupied):
        delta=np.maximum(np.abs(origin+resolution*occupied-center)-resolution/2,0)
        clearance=float(np.min(np.linalg.norm(delta,axis=1)))-radius
    # The ground plane is part of the rendered stage as well.
    clearance=min(clearance,center[2]-radius)
    minimum_clearance=min(minimum_clearance,clearance)
    overlap_segments+=clearance<0
active_time=stopped_time=longest=streak=0.;statuses=collections.Counter();segments=[];start=None
for i,r in enumerate(rows):
    dt=float(rows[i+1]['t'])-float(r['t']) if i+1<len(rows) else 0
    q=math.sqrt(sum(float(r[k])**2 for k in ('qx','qy','qz')))
    v=math.sqrt(sum(float(r[k])**2 for k in ('vx','vy','vz')))
    active=q>.05
    if active:
        active_time+=dt;statuses[r['status']]+=dt
    if active and v<.05:
        if start is None:start=float(r['t'])
        stopped_time+=dt;streak+=dt;longest=max(longest,streak)
    else:
        if start is not None and streak>=1:segments.append({'start_s':start,'duration_s':streak})
        streak=0;start=None
if start is not None and streak>=1:segments.append({'start_s':start,'duration_s':streak})
velocities=np.array([[float(r[k]) for k in ('vx','vy','vz')] for r in rows])
intents=np.array([[float(r[k]) for k in ('qx','qy','qz')] for r in rows])
active_intervals=[];begin=None
for i in range(len(rows)+1):
    active=i<len(rows) and np.linalg.norm(intents[i])>.05
    if active and begin is None:begin=i
    if not active and begin is not None:
        end=min(i,len(rows)-1)
        active_intervals.append({'start_s':float(rows[begin]['t']),
            'end_s':float(rows[end]['t']),
            'displacement_m':float(np.linalg.norm(positions[end]-positions[begin]))})
        begin=None
result={'trace':args.trace.name,'duration_s':float(rows[-1]['t']),'active_input_s':active_time,
    'stopped_with_input_s':stopped_time,'longest_stopped_with_input_s':longest,'stops_over_1s':segments,
    'distance_travelled_m':float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
    'start':positions[0].tolist(),'end':positions[-1].tolist(),
    'controller_status_seconds_with_input':dict(statuses),
    'external_collision_blocks':max(int(r['collision_blocks']) for r in rows),
    'minimum_swept_sphere_clearance_m':minimum_clearance,
    'swept_sphere_overlap_segments':int(overlap_segments),
    'oracle_radius_m':.48,'active_intervals':active_intervals,
    'maximum_measured_speed_mps':float(np.max(np.linalg.norm(velocities,axis=1))),
    'maximum_measured_vertical_speed_mps':float(np.max(np.abs(velocities[:,2]))),
    'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
    'occupancy_sha256':hashlib.sha256(occupancy.read_bytes()).hexdigest()}
print(json.dumps(result,indent=2))
