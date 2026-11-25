#!/usr/bin/python3
"""Plot completed, independently accepted actual Isaac trajectories only."""
import os
from pathlib import Path
ROOT = Path(__file__).resolve().parent
os.environ['MPLCONFIGDIR'] = str(ROOT/'mpl_cache')
import argparse
import csv
import json
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def box_faces(lo, hi):
    x,y,z = [[lo[i],hi[i]] for i in range(3)]
    return [[(a,y[0],z[0]),(a,y[1],z[0]),(a,y[1],z[1]),(a,y[0],z[1])] for a in x] + [
        [(x[0],b,z[0]),(x[1],b,z[0]),(x[1],b,z[1]),(x[0],b,z[1])] for b in y] + [
        [(x[0],y[0],c),(x[1],y[0],c),(x[1],y[1],c),(x[0],y[1],c)] for c in z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    args = parser.parse_args()
    datasets = []
    for folder in args.runs:
        audit = json.loads((folder/'independent_flight_audit.json').read_text())
        execution = json.loads((folder/'independent_execution_audit.json').read_text())
        if not audit['passed'] or not execution['passed']:
            raise ValueError('only accepted actual runs may enter this figure: '+str(folder))
        scenario = json.loads((folder/'scenario.json').read_text())
        rows = list(csv.DictReader((folder/'trajectory.csv').open()))
        datasets.append((folder,scenario,rows,audit['evidence']))
    count = len(datasets)
    fig = plt.figure(figsize=(13, 5*math.ceil(count/2)), constrained_layout=True)
    fig.suptitle('Current EGO4P3 in Isaac Sim: actual 3D closed-loop paths', fontsize=15)
    traces, axes = plt.subplots(count,2,figsize=(12,2.8*count),squeeze=False,constrained_layout=True)
    traces.suptitle('Actual motion: 5 ms physics steps; first-order kinematic plant',fontsize=14)
    for index, (folder,scene,rows,evidence) in enumerate(datasets):
        ax = fig.add_subplot(math.ceil(count/2),2,index+1,projection='3d')
        coordinates = [[float(row[k]) for row in rows] for k in ('x','y','z')]
        ax.plot(*coordinates,color='#087eaf',linewidth=2.2)
        ax.scatter(*scene['start'],color='#168a39',s=45,label='Start')
        ax.scatter(*scene['goal'],color='#cb3437',s=80,marker='*',label='Goal')
        for box in scene['boxes']:
            ax.add_collection3d(Poly3DCollection(box_faces(box['min'],box['max']),
                facecolor='#e58b43',edgecolor='#954d19',alpha=.24,linewidth=.7))
        limits=[]
        for i in range(3):
            values=coordinates[i]+[scene['goal'][i],scene['start'][i]]
            for box in scene['boxes']: values += [box['min'][i],box['max'][i]]
            lo,hi=min(values)-.6,max(values)+.6
            if hi-lo<3:lo-=1;hi+=1
            limits.append((lo,hi))
        ax.set(xlim=limits[0],ylim=limits[1],zlim=limits[2],xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
        ax.set_box_aspect([b-a for a,b in limits]);ax.view_init(elev=24,azim=-58)
        ax.set_title(f"{scene['name']} | goal error {evidence['final_goal_error_m']:.3f} m\n"
                     f"min swept clearance {evidence['minimum_swept_clearance_m']:.3f} m",fontsize=11)
        if index==0:ax.legend(loc='upper left',fontsize=9)
        time=[float(row['t']) for row in rows]
        speed=[math.sqrt(sum(float(row[k])**2 for k in ('vx','vy','vz'))) for row in rows]
        clearance=[float(row['clearance']) for row in rows]
        left,right=axes[index]
        left.plot(time,speed,color='#087eaf');left.axhline(12,color='#cb3437',linestyle='--',linewidth=.8)
        left.set(title=scene['name'],xlabel='Simulation time (s)',ylabel='3D speed (m/s)',ylim=(-.3,13))
        right.plot(time,clearance,color='#168a39');right.axhline(0,color='#cb3437',linewidth=.8)
        right.set(xlabel='Simulation time (s)',ylabel='Swept body clearance (m)')
        for axis in (left,right):axis.grid(alpha=.22)
    out=ROOT/'figures';out.mkdir(exist_ok=True)
    fig.savefig(out/'actual_flight_paths.png',dpi=180)
    traces.savefig(out/'actual_speed_and_clearance.png',dpi=180)
    (out/'source_runs.json').write_text(json.dumps([str(item[0]) for item in datasets],indent=2)+'\n')
    print(out)


if __name__=='__main__':main()
