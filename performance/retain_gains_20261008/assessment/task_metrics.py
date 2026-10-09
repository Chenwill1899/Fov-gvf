"""Artifact parsers only. No historical acceptance policy is imported."""
import csv
import math
import re
from pathlib import Path
from datetime import datetime
from common import finite


def normalize(row, case):
    goal=case=="goal"
    mappings=({"arrival_s":"simulation_elapsed_s", "stop_s":"stopped_with_input_s"} if goal else
        {"angle_mean_deg":"moving_angle_mean_deg", "angle_p95_deg":"moving_angle_p95_deg",
         "progress_m":"active_projected_progress_m", "vector_error_rms_mps":"active_vector_error_rms_mps", "stop_s":"stop_s"})
    mappings.update({"jerk_rms_mps3":"jerk_rms_mps3", "jerk_p95_mps3":"jerk_p95_mps3", "longest_stop_s":"longest_stop_s"})
    metrics={k:row.get(v) for k,v in mappings.items()}
    if goal:
        start=row.get("protocol",{}).get("start");target=row.get("protocol",{}).get("goal")
        length=row.get("distance_travelled_m")
        # A failed partial route must not acquire a favorable path efficiency.
        metrics["path_efficiency"]=(math.dist(start,target)/length
            if row.get("status")=="ARRIVED" and isinstance(start,list) and isinstance(target,list)
            and len(start)==len(target)==3 and all(finite(x) for x in start+target) and finite(length) and length>0 else None)
    safety=row.get("safety",{})
    return metrics,dict(arrived=(row.get("status")=="ARRIVED" if "status" in row else None) if goal else None,
        sweep=row.get("swept_overlap_segments",safety.get("swept_sphere_overlap_segments")),
        external=row.get("collision_blocks",safety.get("external_collision_blocks")),
        clearance=row.get("minimum_swept_clearance_m",safety.get("minimum_swept_sphere_clearance_m")),
        exit_code=row.get("process_exit_code",row.get("returncode")))


def csv_release(path):
    with Path(path).open(newline="") as handle:rows=list(csv.DictReader(handle))
    columns=["t","qx","qy","qz","vx","vy","vz","applied_target_x","applied_target_y","applied_target_z"]
    if len(rows)<2 or any(k not in rows[0] for k in columns):raise ValueError("release columns/samples missing")
    values=[[float(row[k]) for k in columns] for row in rows]
    if any(not math.isfinite(x) for row in values for x in row):raise ValueError("nonfinite release sample")
    times=[r[0] for r in values]
    if any(b<=a for a,b in zip(times,times[1:])):raise ValueError("nonmonotonic CSV time")
    active=[math.sqrt(sum(x*x for x in r[1:4]))>.05 for r in values]
    speeds=[math.sqrt(sum(x*x for x in r[4:7])) for r in values]
    targets=[math.sqrt(sum(x*x for x in r[7:10])) for r in values]
    events=[]
    for i in range(1,len(rows)):
        if active[i] or not active[i-1]:continue
        end=next((j for j in range(i+1,len(rows)) if active[j]),len(rows))
        def settle(series,limit):
            bad=[j for j in range(i,end) if series[j]>limit]
            j=bad[-1]+1 if bad else i
            return times[j]-times[i] if end-j>=2 else None
        events.append(dict(t=times[i],target_s=settle(targets,1e-6),brake_s=settle(speeds,.05),
            observed_interval_s=times[end-1]-times[i],initial_speed_mps=speeds[i]))
    complete=bool(events) and all(e["target_s"] is not None and e["brake_s"] is not None for e in events)
    return dict(events=events,event_count=len(events),complete=complete,
        release_target_s=max(e["target_s"] for e in events) if complete else None,
        release_brake_s=max(e["brake_s"] for e in events) if complete else None,
        scope="Maximum time until settled for >=2 CSV samples within each release interval; no events/censored intervals remain unknown.")


def performance(path):
    content=Path(path).read_text();result={}
    for phrase,prefix in (("Control callback to publish","callback"),("Avoidance compute","core_compute")):
        matches=re.findall(re.escape(phrase)+r" mean / P50 / P95 / max: ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+) / ([0-9.eE+-]+)",content)
        if matches:
            for k,v in zip(("mean","p50","p95","max"),matches[-1]):result[prefix+"_"+k+"_ms"]=float(v)
    return result


def lifecycle(path):
    text=Path(path).read_text(errors="replace")
    return dict(nonzero_child_exits=[int(x) for x in re.findall(r"(?:exit code|exit_code)[ :=]+(-?\d+)",text,re.I) if int(x)!=0],
        compute_deadlines=text.count("COMPUTE_DEADLINE"),
        rosout_invalid_context=text.count("Failed to publish log message to rosout: publisher's context is invalid"))


def launch_time(path):
    matches=re.findall(r"All log files can be found below [^\n]*/(\d{4}-\d\d-\d\d-\d\d-\d\d-\d\d-\d+)-",Path(path).read_text(errors="replace"))
    if len(matches)!=1:raise ValueError("need one actual post-lock ROS launch timestamp")
    return datetime.strptime(matches[0],"%Y-%m-%d-%H-%M-%S-%f").isoformat(timespec="microseconds")
