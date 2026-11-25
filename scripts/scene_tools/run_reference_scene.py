#!/usr/bin/env python3
"""Run reference maps with the unchanged c36 controller and existing ROS lock.

Examples: lab --mode manual; hangar --mode smoke --headless; forest --mode course.
Smoke = one start-to-goal segment. Course = stop-and-hold waypoints, not racing.
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from reference_scene_geometry import bounds, swept_clearance

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/pc_gvf/tools'))
from sim4_benchmark import audit_sensor_evidence, working_bounds_checks, SENSOR_PROTOCOL_VERSION


def write(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def protocol_for(m,args):
    p=dict(version=SENSOR_PROTOCOL_VERSION,scenario_family='reference_scene_v1',
        start=list(map(float,m['spawn'])),goal=list(map(float,m['goal'])),warmup_s=2.,
        timeout_s=args.timeout,max_speed_mps=args.speed,max_vertical_speed_mps=1.,
        input_deadband_m=.35,arrival_radius_m=.65,arrival_speed_mps=.1,arrival_hold_s=.5,
        body_radius_m=.48,plant_tau_s=.10,plant_accel_mps2=16.,max_depth_m=20.,
        pipeline_delay_s=.20,planning_horizon_s=.5,free_space_floor_enabled=True,
        free_space_floor_z=0.,goal_proposal_cache=True,goal_proposal_cache_ttl_s=.3,
        goal_speed_governor=False,restart_clearance_m=.02,emergency_braking_certificate=True,
        smooth_speed=False,depth_sensor_hfov_deg=None,depth_sensor_camera_offset_m=None,
        working_bounds=m['working_bounds'])
    if args.mode=='course':
        points=[list(map(float,p)) for p in m['waypoints']]
        p.update(waypoints=points,goal=points[-1],leg_timeout_s=120.,between_leg_release_s=.5)
    return p


def audit(folder,m,returncode):
    data=json.loads((folder/'result.json').read_text());p=data['protocol']
    rows=list(csv.DictReader((folder/'trajectory.csv').open()))
    if len(rows)<3:raise ValueError('incomplete trajectory')
    def columns(keys):
        out=np.array([[float(r[k]) for k in keys] for r in rows])
        if not np.isfinite(out).all():raise ValueError('nonfinite trajectory')
        return out
    t=columns(['t']).ravel();pos=columns(['x','y','z']);v=columns(['vx','vy','vz'])
    dt=np.diff(t)
    if np.any(dt<=0):raise ValueError('nonmonotonic timestamps')
    boxes=np.array([bounds(o) for o in m['objects']]);minimum=float('inf');overlaps=0
    for a,b in zip(pos[:-1],pos[1:]):
        lo=np.minimum(a,b)-p['body_radius_m']-.1;hi=np.maximum(a,b)+p['body_radius_m']+.1
        sep=np.maximum(np.maximum(boxes[:,0]-hi,lo-boxes[:,1]),0)
        near=np.all(sep==0,axis=1)
        c=swept_clearance(a,b,[o for o,n in zip(m['objects'],near) if n],p['body_radius_m'])
        if np.any(~near):c=min(c,float(np.linalg.norm(sep[~near],axis=1).min())+.1)
        minimum=min(minimum,c);overlaps+=int(c<0)
    speed=np.linalg.norm(v,axis=1);acc=np.linalg.norm(np.diff(v,axis=0)/dt[:,None],axis=1)
    parameter_record=json.loads((folder/'result.parameters.json').read_text())
    actual=parameter_record['effective']
    sensor_path=folder/'result.sensors.json';sensor=json.loads(sensor_path.read_text())
    # The existing sensor audit expects a frozen evidence file alongside result.
    sensor_digest=digest(sensor_path);frozen=folder/'frozen_sensors.json';frozen.write_bytes(sensor_path.read_bytes())
    from sim4_benchmark import value_sha256,sensor_profile
    identity=dict(sensors_blob=frozen.name,sensors_file_sha256=sensor_digest,
                  sensor_profile_sha256=value_sha256(sensor_profile(sensor)),requested_environment={})
    sensor_checks,_=audit_sensor_evidence(folder/'result.json',p,identity,actual)
    hold=0.;goal=np.array(p['goal'])
    for j in range(len(t)-1,0,-1):
        if np.linalg.norm(pos[j]-goal)<=p['arrival_radius_m'] and speed[j]<=p['arrival_speed_mps']:hold+=dt[j-1]
        else:break
    checks=dict(process_exit=returncode==0,arrived=data['status']=='ARRIVED',
        no_guard=data['collision_blocks']==0,no_swept_overlap=overlaps==0,
        speed_limit=bool(speed.max()<=p['max_speed_mps']+1e-5),
        acceleration_limit=bool(acc.max()<=p['plant_accel_mps2']+1e-4),
        vertical_limit=bool(np.abs(v[:,2]).max()<=p['max_vertical_speed_mps']+1e-5),
        starts_at_spawn=bool(np.linalg.norm(pos[0]-p['start'])<1e-7),
        integration_consistent=bool(np.max(np.abs(np.diff(pos,axis=0)-v[1:]*dt[:,None]))<1e-7),
        final_position_consistent=bool(np.linalg.norm(pos[-1]-data['final_position'])<1e-7),
        independent_arrival_hold=bool(hold+1e-7>=p['arrival_hold_s']),
        manual_policy=actual.get('use_fixed_goal') is False,
        body_parameter=actual.get('body_radius')==.48,
        speed_parameter=actual.get('max_speed')==p['max_speed_mps'],
        delay_parameter=actual.get('paper_pipeline_delay')==.20,
        frame_cadence=bool(np.all(np.abs(dt-1/60)<1e-7)),
        seed_radius_parameter=actual.get('paper_seed_radius')==2.5,
        startup_seed_verified='VERIFIED_FREE_SEED' in (folder/'console.log').read_text(errors='replace'))
    checks.update(working_bounds_checks(p,actual,pos,.48));checks.update(sensor_checks)
    if 'waypoints' in p:checks['all_waypoints']=len(data.get('completed_legs',[]))==len(p['waypoints'])
    before=json.loads((folder/'identity.json').read_text())
    checks['actual_executable_matches']=parameter_record.get('executable_sha256')==before['sha256'].get(parameter_record.get('executable',''))
    checks['frozen_inputs_unchanged']=all(Path(path).is_file() and digest(Path(path))==sha for path,sha in before['sha256'].items())
    out=dict(scene=m['map_identity'],status=data['status'],safe_arrival=all(checks.values()),
        validation_scope='one smoke segment' if 'waypoints' not in p else 'stop-and-hold waypoint course',
        checks=checks,rejected=[k for k,val in checks.items() if not val],
        simulation_elapsed_s=data['simulation_elapsed_s'],wall_elapsed_s=data['wall_elapsed_s'],
        path_m=float(np.linalg.norm(np.diff(pos,axis=0),axis=1).sum()),peak_speed_mps=float(speed.max()),
        minimum_analytic_swept_clearance_m=minimum,swept_overlap_segments=overlaps,
        collision_blocks=data['collision_blocks'],samples=len(rows))
    write(folder/'audit.json',out);return out


def main():
    raise SystemExit("EGO4P0参考场景飞行调度尚待接入新控制器，当前保留离线审计函数。")


if __name__=='__main__':raise SystemExit('EGO4P0参考场景路线控制尚待重写。请使用独立预览脚本查看场景。')
