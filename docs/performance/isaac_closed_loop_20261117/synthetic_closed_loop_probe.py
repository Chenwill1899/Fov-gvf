#!/usr/bin/env python3
"""Ordinary-scheduling correctness probe; analytic depth, NEVER Isaac evidence.

The production app receives only optical-Z images and measured state over its
normal socket. The renderer and independent continuous sweep own all geometry.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time
os.environ.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
import numpy as np

MAGIC=0x45474F3450330001
REQUEST=struct.Struct("<6Q15d")
REPLY=struct.Struct("<6Q12d")

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def render(position, rotation, rays, cameras, scene):
    low=np.asarray(scene["bounds_min"]); high=np.asarray(scene["bounds_max"])
    images=[]
    for ray, camera in zip(rays, cameras):
        direction=ray @ rotation.T
        origin=position + rotation @ np.asarray(camera["camera_position_robot_m"])
        with np.errstate(divide="ignore", invalid="ignore"):
            distances=np.where(direction>0,(high-origin)/direction,(low-origin)/direction)
            distances=np.where(np.abs(direction)<1e-15,np.inf,distances)
            depth=np.min(distances,axis=2)
            for box in scene["boxes"]:
                lo=(np.asarray(box["min"])-origin)/direction
                hi=(np.asarray(box["max"])-origin)/direction
                enter=np.max(np.minimum(lo,hi),axis=2)
                leave=np.min(np.maximum(lo,hi),axis=2)
                hit=(leave>=np.maximum(enter,0)) & (enter>0)
                depth=np.minimum(depth,np.where(hit,enter,np.inf))
        depth=np.where((depth>=camera["near_clip_z_m"])&(depth<=camera["far_clip_z_m"]),depth,np.inf)
        images.append(np.asarray(depth,dtype="<f4").tobytes())
    return b"".join(images)

def segment_box_distance(a,b,low,high):
    # Exact piecewise-quadratic minimization of distance²(segment,AABB).
    d=b-a; cuts=[0.,1.]
    for k in range(3):
        if abs(d[k])>1e-15:
            cuts.extend(t for edge in (low[k],high[k]) if 0<(t:=(edge-a[k])/d[k])<1)
    cuts=sorted(cuts); best=float("inf")
    def squared(t):
        p=a+t*d; return float(np.dot(np.maximum(np.maximum(low-p,p-high),0),np.maximum(np.maximum(low-p,p-high),0)))
    for left,right in zip(cuts,cuts[1:]):
        mid=a+(left+right)*.5*d; active=(mid<low)|(mid>high)
        offset=np.where(mid<low,a-low,a-high)
        denominator=float(np.dot(d[active],d[active]))
        t=np.clip(-float(np.dot(d[active],offset[active]))/denominator,left,right) if denominator else left
        best=min(best,squared(left),squared(right),squared(t))
    return math.sqrt(best)

def clearance(a,b,scene):
    radius=scene["body_radius_m"]
    result=float(min(np.min(np.minimum(a,b)-scene["bounds_min"]),np.min(scene["bounds_max"]-np.maximum(a,b))))-radius
    for box in scene["boxes"]:
        result=min(result,segment_box_distance(a,b,np.array(box["min"]),np.array(box["max"]))-radius)
    return result

def response(velocity,command):
    error=command-velocity; length=float(np.linalg.norm(error))
    scale=1. if np.linalg.norm(command)<1e-8 else .005/.1
    if length>1e-12: scale=min(scale,150*.005/length)
    return velocity+scale*error

def exact(sock,length):
    pieces=[]
    while length:
        piece=sock.recv(length)
        if not piece: raise RuntimeError("truncated controller reply")
        pieces.append(piece);length-=len(piece)
    return b"".join(pieces)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("controller","calibration","scenario","output"): parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--steps",type=int,default=4000)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    scenario=json.loads(args.scenario.read_text());calibration=json.loads(args.calibration.read_text())
    width,height=calibration["render_width"],calibration["render_height"]
    # The original USD helper described aperture-center pixel coordinates.
    # Current C++ tables index integer pixel centers; normalize this synthetic
    # fixture only, preserving the source calibration as a separate artifact.
    for camera in calibration["cameras"]:
        if camera["cx"]==width/2 and camera["cy"]==height/2:
            camera["cx"]-=.5;camera["cy"]-=.5
        if camera["cx"]!=(width-1)/2 or camera["cy"]!=(height-1)/2:
            raise ValueError("unsupported synthetic pixel-center convention")
    calibration["synthetic_probe_integer_pixel_centers"]=True
    effective_calibration=args.output/"synthetic.sensors.json"
    effective_calibration.write_text(json.dumps(calibration,indent=2)+"\n")
    y,x=np.mgrid[:height,:width];rays=[]
    for camera in calibration["cameras"]:
        optical=np.stack(((x-camera["cx"])/camera["fx"],(y-camera["cy"])/camera["fy"],np.ones_like(x)),axis=2)
        rays.append(optical @ np.asarray(camera["robot_from_optical_rotation"]).T)
    result=dict(kind="synthetic_ray_AABB_correctness_only",isaac_sim=False,scheduling="SCHED_OTHER",performance_claim=False,
                controller_sha256=sha(args.controller),input_calibration_sha256=sha(args.calibration),effective_calibration_sha256=sha(effective_calibration),scenario_sha256=sha(args.scenario),
                scenario=scenario["name"],arrived=False,status="running",steps=0,collision_blocks=0,minimum_clearance_m=float("inf"),peak_speed_mps=0.,modes={})
    position=np.asarray(scenario["start"],dtype=float);velocity=np.zeros(3);yaw=0.;hold=0.;process=None
    try:
      with tempfile.TemporaryDirectory(prefix="p3_probe_") as temporary, (args.output/"controller.log").open("w") as log, (args.output/"trace.csv").open("w",newline="") as trace:
        endpoint=str(Path(temporary)/"c.sock")
        process=subprocess.Popen([str(args.controller.resolve()),"--calibration",str(effective_calibration.resolve()),"--scenario",str(args.scenario.resolve()),"--output",str((args.output/"controller").resolve()),"--socket",endpoint,"--correctness-only"],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+60
        while not (args.output/"controller/controller_ready.json").exists():
            if process.poll() is not None: raise RuntimeError("controller failed before ready")
            if time.monotonic()>deadline: raise RuntimeError("controller readiness timeout")
            time.sleep(.02)
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as sock:
          sock.settimeout(30);sock.connect(endpoint)
          writer=csv.writer(trace);writer.writerow(["sequence","simulation_ns","decision","flags","x","y","z","vx","vy","vz","goal_distance","clearance_m","prediction_error"])
          for sequence in range(1,args.steps+1):
            c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            payload=render(position,rotation,rays,calibration["cameras"],scenario)
            stamp=(sequence-1)*5000000
            sock.sendall(REQUEST.pack(MAGIC,1,sequence,stamp,width,height,*position,*velocity,*rotation.reshape(-1))+payload)
            reply=REPLY.unpack(exact(sock,REPLY.size));magic,version,seq,t,decision,flags=reply[:6]
            if (magic,version,seq,t)!=(MAGIC,1,sequence,stamp) or not all(math.isfinite(v) for v in reply[6:]): raise RuntimeError("malformed controller reply")
            result["steps"]=sequence;result["modes"][str(decision)]=result["modes"].get(str(decision),0)+1
            if decision==6: result["status"]="fatal_uncertified";break
            predicted=np.asarray(reply[9:12]);command=np.asarray(reply[6:9]);next_velocity=response(velocity,command)
            error=float(np.max(np.abs(predicted-next_velocity)))
            if error>1e-8: raise RuntimeError("plant prediction mismatch")
            next_position=position+.005*next_velocity;sweep=clearance(position,next_position,scenario)
            if sweep<=1e-10: result["collision_blocks"]+=1;raise RuntimeError("independent continuous sweep rejected")
            result["minimum_clearance_m"]=min(result["minimum_clearance_m"],sweep)
            position,velocity=next_position,next_velocity;speed=float(np.linalg.norm(velocity));distance=float(np.linalg.norm(position-scenario["goal"]))
            result["peak_speed_mps"]=max(result["peak_speed_mps"],speed)
            writer.writerow([sequence,sequence*5000000,decision,flags,*position,*velocity,distance,sweep,error])
            if math.hypot(velocity[0],velocity[1])>.1:
                target=math.atan2(velocity[1],velocity[0]);change=math.atan2(math.sin(target-yaw),math.cos(target-yaw));yaw+=max(-.01,min(.01,change))
            hold=hold+.005 if distance<=scenario["arrival_radius_m"] and speed<=scenario["arrival_speed_mps"] else 0.
            if hold+1e-12>=scenario["arrival_hold_s"]:result["arrived"]=True;result["status"]="arrived";break
          else: result["status"]="step_limit"
        process.wait(timeout=15)
    except Exception as error:
        result["status"]="probe_error";result["error"]=str(error)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        result.update(controller_exit_code=process.returncode if process else None,final_position=position.tolist(),final_velocity=velocity.tolist(),goal_distance_m=float(np.linalg.norm(position-scenario["goal"])))
        if not math.isfinite(result["minimum_clearance_m"]):result["minimum_clearance_m"]=None
        (args.output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result,allow_nan=False))
    return 0 if result["arrived"] and result["controller_exit_code"]==0 else 1

if __name__=="__main__":raise SystemExit(main())
