#!/usr/bin/env python3
"""Compare simulator integration with a 1 ms reference for bounded motion cases.
This is a finite simulation calibration, not a universal model-error proof.
"""
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts/isaac"))
from manual_control_math import velocity_response

def trajectory(initial, target, dt):
    v=tuple(initial);p=np.zeros(3);times=[];positions=[]
    for k in range(round(3/dt)):
        t=(k+1)*dt
        command=target if t<=.4+1e-9 else (0.,0.,0.)
        v=velocity_response(v,command,dt,.22,1.2)
        p=p+dt*np.asarray(v);times.append(t);positions.append(p.copy())
    return np.array(times),np.array(positions)
results=[]
for speed in (.2,1.,2.):
    for angle in (0,45,90,135,180):
        a=np.radians(angle);target=(speed*np.cos(a),speed*np.sin(a),0.)
        t,p=trajectory((speed,0.,0.),target,1/60)
        rt,rp=trajectory((speed,0.,0.),target,.001)
        reference=np.stack([np.interp(t,rt,rp[:,i]) for i in range(3)],axis=1)
        results.append({"speed_mps":speed,"turn_degrees":angle,
                        "max_position_error_m":float(np.max(np.linalg.norm(p-reference,axis=1)))})
print(json.dumps({"cases":results,"maximum_sampled_error_m":max(r["max_position_error_m"] for r in results),
    "scope":"60 Hz kinematic executor vs 1 ms reference; tau .22, acceleration/braking 1.2; 0.4 s response then stop; no sensor or real-hardware bound"},indent=2))
