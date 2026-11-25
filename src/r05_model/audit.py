"""Independently recheck the recorded path and velocity integrator after a run."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser()
p.add_argument("run", type=Path)
p.add_argument("occupancy", type=Path)
a = p.parse_args()
report = json.loads((a.run / "result.json").read_text())
assert hashlib.sha256(a.occupancy.read_bytes()).hexdigest() == report["occupancy_sha256"]
with (a.run / "trajectory.csv").open() as stream:
    rows = list(csv.DictReader(stream))
with (a.run / "controller.csv").open() as stream:
    controller = list(csv.DictReader(stream))
pos = np.array([[float(r[k]) for k in ("x","y","z")] for r in rows])
vel = np.array([[float(r[k]) for k in ("vx","vy","vz")] for r in rows])
target = np.array([[float(r[k]) for k in ("ux","uy","uz")] for r in rows])
t = np.array([float(r["t"]) for r in rows])
kit = np.array([float(r["kit_t"]) for r in rows])
modes = np.array([int(r["mode"]) for r in rows[1:]])
assert np.isfinite(np.c_[pos, vel, target, t, kit]).all()
assert np.allclose(np.diff(t), .005, atol=1e-10, rtol=0)
assert np.allclose(kit-kit[0], t, atol=1e-7, rtol=0)
assert np.allclose(np.diff(pos,axis=0), vel[1:]*.005, atol=1e-10, rtol=0)
assert np.allclose(pos[0], report["start"], atol=1e-10, rtol=0)
assert np.allclose(pos[-1], report["final"], atol=1e-10, rtol=0)
hold = report.get("terminal_hold_steps",0)
assert hold==int(np.count_nonzero(modes==6))
assert len(controller) in {len(rows)-1-hold, len(rows)-hold}
for row, command in zip(rows[1:],controller):
    assert int(row["step"])==int(command["step"])
    assert int(row["mode"])==int(command["mode"])

grid = np.fromfile(a.occupancy, dtype=np.uint8).reshape(1200,300,50)
resolution = .4
origin = np.array([-80.,-60.,0.])
lower = origin-.2
lower[2] = 0.
upper = origin+(np.array(grid.shape)-1)*resolution+.2
upper[2] = 19.8
swept_clearances = []
for begin,end in zip(pos[:-1],pos[1:]):
    middle = (begin+end)*.5
    envelope = .10 + np.linalg.norm(end-begin)*.5
    # Limit this lower bound to 1 m. Every voxel that could make it smaller
    # lies in the enclosing cube after accounting for the voxel half size.
    reach = envelope + 1. + .2
    first = np.maximum(np.floor((middle-reach-origin)/resolution).astype(int), 0)
    last = np.minimum(np.ceil((middle+reach-origin)/resolution).astype(int)+1, grid.shape)
    clearance = min(1., np.min(middle-lower)-envelope, np.min(upper-middle)-envelope)
    if np.all(first<last):
        occupied = np.argwhere(grid[tuple(slice(lo,hi) for lo,hi in zip(first,last))])+first
        if len(occupied):
            delta = np.maximum(np.abs(middle-(origin+occupied*resolution))-.2,0)
            clearance = min(clearance,float(np.min(np.linalg.norm(delta,axis=1)))-envelope)
    swept_clearances.append(clearance)
speed = np.linalg.norm(vel,axis=1)
accel = np.linalg.norm(np.diff(vel,axis=0),axis=1)/.005
mode_counts = {str(int(m)):int(np.count_nonzero(modes==m)) for m in np.unique(modes)}
at_12 = int(np.count_nonzero(np.abs(speed[1:]-12)<.001))
if report["status"]=="ARRIVED":
    assert np.linalg.norm(pos[-1]-np.asarray(report["goal"]))<=1
    assert np.all(speed[-100:]<.1)
    assert np.all(np.linalg.norm(pos[-100:]-np.asarray(report["goal"]),axis=1)<=1)
output = dict(steps=len(rows)-1, rolling_steps=mode_counts.get("1",0),
    rolling_commits=sum(int(r["reference_commit"]) for r in controller),
    field_attempts=sum(int(r["field_attempted"]) for r in controller), modes=mode_counts,
    steps_at_12_mps=at_12, minimum_swept_clearance_lower_bound_m=min(swept_clearances),
    swept_overlap_segments=int(np.count_nonzero(np.asarray(swept_clearances)<=0)),
    speed_violations=int(np.count_nonzero(speed>12+1e-7)),
    acceleration_violations=int(np.count_nonzero(accel>150+1e-7)),
    peak_speed_mps=float(speed.max()), peak_accel_mps2=float(accel.max()),
    at_goal=report["status"]=="ARRIVED", distance_m=float(np.linalg.norm(np.diff(pos,axis=0),axis=1).sum()))
assert output["swept_overlap_segments"] == output["speed_violations"] == output["acceleration_violations"] == 0
(a.run/"audit.json").write_text(json.dumps(output,indent=2)+"\n")
print(json.dumps(output,indent=2))
