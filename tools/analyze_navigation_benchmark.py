#!/usr/bin/env python3
"""Independent geometry and timing audit of complete fixed-goal trial groups."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('label')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
folder = root/'performance/navigation_benchmark_20260929'
grid = np.fromfile(root/'scenes/ego_swarm_cloud/occupancy.bin', np.uint8).reshape(400,300,50)
scene = root/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'
assert hashlib.sha256(scene.read_bytes()).hexdigest() == '43f085ddc46a4391dcd743488af77cf08850a9e279b42252bd744d39057b35b2'
origin = np.array([-80.,-60.,0.])
trials = []
trajectories = []
for algorithm in ('user_ego', 'ego1p0', 'ego1p1', 'ego1p2', 'ego1p3', 'ego1p5'):
    for file in sorted(folder.glob(f'{args.label}_{algorithm}_[0-9]*.json')):
        if '.parameters.' in file.name:
            continue
        trial = json.loads(file.read_text())
        trace = file.with_suffix('.csv')
        if not trace.exists():
            trials.append(trial)
            continue
        rows = list(csv.DictReader(trace.open()))
        if not rows:
            trials.append(trial)
            continue
        def matrix(keys):
            return np.array([[float(r[k]) for k in keys] for r in rows])
        times = matrix(['t']).ravel()
        p, v, q = matrix(['x','y','z']), matrix(['vx','vy','vz']), matrix(['qx','qy','qz'])
        commands = matrix(['applied_target_x','applied_target_y','applied_target_z'])
        dt = np.diff(times)
        active = (times[:-1] >= trial['protocol']['warmup_s']) & (np.linalg.norm(q[:-1],axis=1)>.05)
        stopped = active & (np.linalg.norm(v[:-1],axis=1)<.05)
        streak = longest = 0.
        stops = 0
        for flag, delta in zip(stopped,dt):
            if flag:
                streak += delta
                longest = max(longest,streak)
            else:
                stops += streak >= 1.
                streak = 0.
        stops += streak >= 1.
        minimum = float('inf')
        overlaps = 0
        # Conservative sphere encloses each whole center segment; voxel AABBs
        # match the USD, including their half-cell extent. Offline only.
        for a,b in zip(p[:-1],p[1:]):
            center = (a+b)/2
            radius = .48+np.linalg.norm(b-a)/2
            index = np.floor((center-origin)/.4+.5).astype(int)
            reach = int(np.ceil((radius+.4)/.4))+1
            lo,hi = np.maximum(0,index-reach),np.minimum(grid.shape,index+reach+1)
            clearance = center[2]-radius
            if np.any(lo>=hi):
                clearance = float('-inf')
            else:
                occupied = np.argwhere(grid[lo[0]:hi[0],lo[1]:hi[1],lo[2]:hi[2]])+lo
                if len(occupied):
                    delta = np.maximum(np.abs(origin+.4*occupied-center)-.2,0)
                    clearance = min(clearance,float(np.linalg.norm(delta,axis=1).min())-radius)
            minimum = min(minimum,clearance)
            overlaps += clearance < 0
        accel = np.diff(v,axis=0)/dt[:,None]
        jerk = np.linalg.norm(np.diff(accel,axis=0)/dt[1:,None],axis=1)
        moving = (np.linalg.norm(v[1:-1],axis=1)>.05)|(np.linalg.norm(v[2:],axis=1)>.05)
        jumps = np.linalg.norm(np.diff(commands,axis=0),axis=1)
        durations = collections.Counter()
        for r, delta, flag in zip(rows,dt,active):
            if flag:
                durations[r['status']] += delta
        trial.update(safe_arrival=trial['status']=='ARRIVED' and overlaps==0 and trial['collision_blocks']==0,
            distance_travelled_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
            stopped_with_input_s=float(dt[stopped].sum()),longest_stop_s=longest,stops_over_1s=int(stops),
            minimum_swept_clearance_m=minimum,swept_overlap_segments=int(overlaps),
            acceleration_max_mps2=float(np.linalg.norm(accel,axis=1).max()),
            jerk_rms_mps3=float(np.sqrt(np.mean(jerk*jerk))),jerk_p95_mps3=float(np.percentile(jerk,95)),
            jerk_rms_moving_mps3=float(np.sqrt(np.mean(jerk[moving]**2))) if moving.any() else 0.,
            maximum_speed_mps=float(np.linalg.norm(v,axis=1).max()),
            maximum_vertical_speed_mps=float(np.abs(v[:,2]).max()),
            command_jumps_over_025=int((jumps>.25).sum()),status_seconds=dict(durations))
        trials.append(trial)
        trajectories.append((algorithm,file.stem,p))
groups = {}
for algorithm in ('user_ego','ego1p0','ego1p1','ego1p2','ego1p3','ego1p5'):
    group = [t for t in trials if t['algorithm']==algorithm]
    if not group:
        continue
    success = [t for t in group if t.get('safe_arrival',False)]
    def mean(key, data):
        return float(np.mean([t[key] for t in data])) if data else None
    groups[algorithm] = dict(runs=len(group), safe_arrivals=len(success),
        mean_arrival_sim_s=mean('simulation_elapsed_s',success),
        std_arrival_sim_s=float(np.std([t['simulation_elapsed_s'] for t in success],ddof=1)) if len(success)>1 else None,
        mean_arrival_wall_s=mean('wall_elapsed_s',success),
        timeout_penalized_sim_s=float(np.mean([t['simulation_elapsed_s'] if t.get('safe_arrival')
            else t.get('protocol',{}).get('timeout_s',240.) for t in group])),
        mean_stopped_s=mean('stopped_with_input_s',[t for t in group if 'stopped_with_input_s' in t]),
        mean_jerk_rms_mps3=mean('jerk_rms_mps3',[t for t in group if 'jerk_rms_mps3' in t]))
output = dict(groups=groups,trials=trials)
(folder/f'{args.label}_analysis.json').write_text(json.dumps(output,indent=2))
print(json.dumps(groups,indent=2))
if trajectories:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax = plt.subplots(figsize=(12,9))
    # Low-altitude projection is illustrative, not an online navigation map.
    ax.imshow(grid[:,:,:15].max(axis=2).T,origin='lower',extent=(-80,80,-60,60),cmap='Greys',alpha=.4)
    colors = dict(user_ego='tab:blue',ego1p0='tab:orange',ego1p1='tab:green',ego1p2='tab:purple',ego1p3='tab:red',ego1p5='tab:cyan')
    for algorithm,name,p in trajectories:
        ax.plot(p[:,0],p[:,1],color=colors[algorithm],label=name,lw=1.3)
    protocol = json.loads((folder/'protocol.json').read_text())
    for name,marker in [('start','o'),('goal','*')]:
        point = protocol[name]
        ax.scatter(point[0],point[1],s=120,marker=marker,c='red',zorder=10)
        ax.annotate(name,point[:2],xytext=(5,7),textcoords='offset points')
    ax.set(xlabel='World X (m)',ylabel='World Y (m)',title=f'Fixed goal navigation: {args.label}')
    ax.legend(fontsize=7,loc='upper left');ax.set_aspect('equal');fig.tight_layout()
    fig.savefig(folder/f'{args.label}_trajectories.png',dpi=150)
