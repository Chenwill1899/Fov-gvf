#!/usr/bin/python3
"""Independently recompute a simulated flight's motion and swept-sphere clearance.

This module has no Isaac, controller, numpy or ROS imports.  A simulator result
is an assertion to check, not the source of the collision or arrival verdict.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def segment_box_distance(a, b, lo, hi):
    """Exact minimum point-to-AABB distance along a segment, piecewise quadratic."""
    delta = [b[i]-a[i] for i in range(3)]
    cuts = [0., 1.]
    for i in range(3):
        if delta[i]:
            cuts += [t for bound in (lo[i], hi[i])
                     if 0. < (t := (bound-a[i])/delta[i]) < 1.]
    cuts = sorted(set(cuts))

    def squared(t):
        return sum(max(lo[i]-(a[i]+delta[i]*t), 0.,
                       a[i]+delta[i]*t-hi[i])**2 for i in range(3))

    best = min(squared(0.), squared(1.))
    for left, right in zip(cuts[:-1], cuts[1:]):
        middle = (left+right)*.5
        linear = quadratic = 0.
        for i in range(3):
            x = a[i]+delta[i]*middle
            if lo[i] <= x <= hi[i]:
                continue
            bound = lo[i] if x < lo[i] else hi[i]
            linear += delta[i]*(a[i]-bound)
            quadratic += delta[i]*delta[i]
        candidate = max(left, min(right, -linear/quadratic)) if quadratic else middle
        best = min(best, squared(left), squared(right), squared(candidate))
    return math.sqrt(max(0., best))


def swept_clearance(a, b, scenario, radius):
    room = min(min(a[i], b[i])-scenario['bounds_min'][i] for i in range(3))
    room = min(room, min(scenario['bounds_max'][i]-max(a[i], b[i]) for i in range(3)))
    result = room-radius
    for box in scenario['boxes']:
        result = min(result, segment_box_distance(a, b, box['min'], box['max'])-radius)
    return result


def expected_velocity(v, u, dt, tau, acceleration):
    # Reimplemented from the model definition, without importing the simulator.
    difference = [u[i]-v[i] for i in range(3)]
    error = norm(difference)
    multiplier = 1. if norm(u) < 1e-8 else min(1., dt/tau)
    if error > 1e-12:
        multiplier = min(multiplier, acceleration*dt/error)
    return [v[i]+multiplier*difference[i] for i in range(3)]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ray_depth(origin, direction, scenario):
    """Ray parameter is optical Z: do not normalize the pinhole direction."""
    depth = min((scenario['bounds_max'][i]-origin[i])/direction[i] if direction[i]>0
                else (scenario['bounds_min'][i]-origin[i])/direction[i]
                for i in range(3) if abs(direction[i])>1e-12)
    for box in scenario['boxes']:
        near, far = -math.inf, math.inf
        for i in range(3):
            if abs(direction[i]) <= 1e-12:
                if not box['min'][i] <= origin[i] <= box['max'][i]:
                    far = -math.inf
                    break
            else:
                a = (box['min'][i]-origin[i])/direction[i]
                b = (box['max'][i]-origin[i])/direction[i]
                near, far = max(near,min(a,b)), min(far,max(a,b))
        if far >= max(near,0.):
            depth = min(depth,near if near>0. else far)
    return depth


def moving_depth_audit(folder, scenario, trajectory):
    path = folder/'simulation/sensor_samples.csv'
    calibration_path = folder/'simulation/simulation.sensors.json'
    if not path.exists() or not calibration_path.exists():
        return False, {'error':'missing actual moving-depth samples or calibration'}
    cameras = json.loads(calibration_path.read_text())['cameras']
    required = {(camera,u,v) for camera in range(4)
                for u in (40,100,160,220,280) for v in (30,75,120,165,210)}
    groups = {}
    count = invalid = 0
    maximum = 0.
    worst = None
    for sample in csv.DictReader(path.open()):
        sequence, camera, u, v = (int(sample[k]) for k in ('sequence','camera_index','u','v'))
        if sequence < 1 or sequence >= len(trajectory) or (camera,u,v) not in required:
            return False, {'error':'sample identity outside completed trajectory'}
        groups.setdefault(sequence,[]).append((camera,u,v))
        row = trajectory[sequence-1]
        position = [float(row[k]) for k in ('x','y','z')]
        yaw = float(row['yaw']); co, si = math.cos(yaw), math.sin(yaw)
        cal = cameras[camera]
        q = [(u-float(cal['cx']))/float(cal['fx']), (v-float(cal['cy']))/float(cal['fy']),1.]
        body = [sum(cal['robot_from_optical_rotation'][i][j]*q[j] for j in range(3)) for i in range(3)]
        direction = [co*body[0]-si*body[1], si*body[0]+co*body[1],body[2]]
        offset = cal['camera_position_robot_m']
        origin = [position[0]+co*offset[0]-si*offset[1],position[1]+si*offset[0]+co*offset[1],position[2]+offset[2]]
        expected = ray_depth(origin,direction,scenario)
        actual = float(sample['depth_m']); count += 1
        if not math.isfinite(actual) or actual <= 0.:
            invalid += 1
            continue
        error = abs(expected-actual)
        if error > maximum:
            maximum = error
            worst = dict(sequence=sequence,camera=camera,u=u,v=v,expected_m=expected,actual_m=actual)
    complete = (count == 100*(len(trajectory)-1) and set(groups) == set(range(1,len(trajectory)))
                and all(len(points)==100 and set(points)==required for points in groups.values()))
    return complete and invalid==0 and maximum<=.01, dict(samples=count, complete=complete,
        invalid_samples=invalid, maximum_ray_depth_error_m=maximum,tolerance_m=.01,worst=worst,
        source_sha256=digest(path),method='independent pinhole/pose rays against static room and boxes at every motion step')


def audit(folder):
    scenario_path = folder/'scenario.json'
    scenario = json.loads(scenario_path.read_text())
    result_path = folder/'result.json'
    reported = json.loads(result_path.read_text()) if result_path.exists() else {}
    trajectory_path = folder/'trajectory.csv'
    rows = list(csv.DictReader(trajectory_path.open())) if trajectory_path.exists() else []
    base_path = folder/'base_step1.json'
    base = json.loads(base_path.read_text()) if base_path.exists() else {}
    dt = float(scenario['physics_dt_s'])
    radius = float(scenario['body_radius_m'])
    tau = float(base.get('plant_tau_s', .10))
    acceleration = float(base.get('plant_acceleration_mps2', 150.))
    maximum_speed = float(base.get('max_speed_mps', 12.))
    checks = {'trajectory_present': len(rows) >= 2,
              'reported_arrival': reported.get('status') == 'ARRIVED',
              'scenario_static': scenario.get('static_scene') is True,
              'initial_seed_clear': swept_clearance(scenario['start'], scenario['start'],
                  scenario, float(scenario['seed_radius_m'])) > 0.}
    evidence = {'scenario_sha256': digest(scenario_path), 'rows': len(rows),
                'model': 'isotropic first-order kinematic response; not quadrotor rigid-body dynamics',
                'audit_geometry': 'independent exact segment-to-AABB distance plus finite room bounds'}
    if len(rows) < 2:
        return {'passed': False, 'checks': checks, 'failed': [k for k, v in checks.items() if not v],
                'evidence': evidence, 'reported_status': reported.get('status', 'MISSING_RESULT')}
    positions = [[float(r[k]) for k in ('x','y','z')] for r in rows]
    velocities = [[float(r[k]) for k in ('vx','vy','vz')] for r in rows]
    commands = [[float(r[k]) for k in ('ux','uy','uz')] for r in rows]
    times = [float(r['t']) for r in rows]
    kit = [float(r['kit_t']) for r in rows]
    physics = [float(r['physics_t']) for r in rows]
    physics_steps = [int(r['physics_step']) for r in rows]
    steps = [int(r['step']) for r in rows]
    checks['finite_records'] = all(math.isfinite(x) for group in
        (positions, velocities, commands) for row in group for x in row) and all(
        math.isfinite(x) for x in times+kit+physics)
    if not checks['finite_records']:
        return {'passed': False, 'checks': checks, 'failed': [k for k,v in checks.items() if not v],
                'evidence': evidence, 'reported_status': reported.get('status', 'MISSING_RESULT')}
    checks['starts_at_declared_pose'] = norm([positions[0][i]-scenario['start'][i] for i in range(3)]) < 1e-9
    checks['starts_at_rest'] = norm(velocities[0]) < 1e-9
    checks['consecutive_steps'] = steps[0] == 0 and all(b == a+1 for a,b in zip(steps[:-1], steps[1:]))
    max_dt_error = max((abs((b-a)-dt) for a,b in zip(times[:-1],times[1:])), default=math.inf)
    max_kit_error = max((abs((kit[i]-kit[0])-(times[i]-times[0])) for i in range(len(rows))), default=math.inf)
    max_physics_error = max((abs((physics[i]-physics[0])-(times[i]-times[0])) for i in range(len(rows))), default=math.inf)
    checks['simulation_dt'] = max_dt_error <= 1e-8
    checks['kit_time_agrees'] = max_kit_error <= 1e-6
    checks['actual_physics_time_agrees'] = max_physics_error <= 1e-6
    checks['one_actual_physics_step_per_command'] = all(b==a+1 for a,b in zip(physics_steps[:-1],physics_steps[1:]))
    position_error = velocity_error = peak_acceleration = length = 0.
    clearance = swept_clearance(positions[0], positions[0], scenario, radius)
    collisions = 0
    for i in range(1, len(rows)):
        step_dt = times[i]-times[i-1]
        predicted_v = expected_velocity(velocities[i-1], commands[i], step_dt, tau, acceleration)
        velocity_error = max(velocity_error, norm([predicted_v[j]-velocities[i][j] for j in range(3)]))
        position_error = max(position_error, norm([positions[i-1][j]+velocities[i][j]*step_dt-positions[i][j] for j in range(3)]))
        if step_dt > 0:
            peak_acceleration = max(peak_acceleration, norm([velocities[i][j]-velocities[i-1][j] for j in range(3)])/step_dt)
        c = swept_clearance(positions[i-1], positions[i], scenario, radius)
        collisions += c <= 0.
        clearance = min(clearance, c)
        length += norm([positions[i][j]-positions[i-1][j] for j in range(3)])
    speeds = [norm(v) for v in velocities]
    errors = [norm([p[i]-scenario['goal'][i] for i in range(3)]) for p in positions]
    hold = 0.
    for i in range(len(rows)-1, 0, -1):
        if max(errors[i], errors[i-1]) > scenario['arrival_radius_m'] or max(speeds[i], speeds[i-1]) > scenario['arrival_speed_mps']:
            break
        hold += times[i]-times[i-1]
    checks.update(plant_response_matches=velocity_error <= 1e-8,
                  position_integration_matches=position_error <= 1e-8,
                  speed_within_3d_norm_limit=max(speeds) <= maximum_speed+1e-7,
                  acceleration_within_model_limit=peak_acceleration <= acceleration+1e-6,
                  no_continuous_swept_collision=collisions == 0 and clearance > 0.,
                  reaches_goal=errors[-1] <= scenario['arrival_radius_m'],
                  final_velocity_settled=speeds[-1] <= scenario['arrival_speed_mps'],
                  arrival_hold=hold+1e-8 >= scenario['arrival_hold_s'],
                  no_simulator_collision_intervention=int(reported.get('collision_blocks', -1)) == 0)
    if abs(scenario['goal'][2]-scenario['start'][2]) > .5:
        checks['three_dimensional_motion'] = max(p[2] for p in positions)-min(p[2] for p in positions) > .5
    controller_path = folder/'controller.csv'
    controller = list(csv.DictReader(controller_path.open())) if controller_path.exists() else []
    checks['controller_log_present'] = len(controller) >= len(rows)-1
    sensor_path = folder/'simulation/sensor_frames.csv'
    sensor_rows = list(csv.DictReader(sensor_path.open())) if sensor_path.exists() else []
    sensor_groups = {}
    sensor_time_error = 0.
    sensor_absolute_time_error = 0.
    sensor_valid = bool(sensor_rows)
    for frame in sensor_rows:
        sequence = int(frame['sequence'])
        sensor_groups.setdefault(sequence, []).append(frame['camera'])
        expected_stamp = (sequence-1)*dt
        rendered = float(frame['relative_rendered_simulation_s'])
        sensor_valid &= math.isfinite(rendered) and int(frame['reference_denominator']) > 0
        sensor_valid &= int(frame['request_simulation_ns']) == round(expected_stamp*1e9)
        sensor_valid &= 1 <= sequence < len(rows)
        if 1 <= sequence < len(rows):
            actual_render_time = float(frame['rendered_simulation_s'])
            sensor_valid &= math.isfinite(actual_render_time)
            if math.isfinite(actual_render_time):
                sensor_absolute_time_error = max(sensor_absolute_time_error,
                    abs(actual_render_time-physics[sequence-1]))
        if math.isfinite(rendered):
            sensor_time_error = max(sensor_time_error, abs(rendered-expected_stamp))
    checks['four_rendered_camera_frames_each_step'] = (len(sensor_rows) == 4*(len(rows)-1)
        and set(sensor_groups) == set(range(1,len(rows)))
        and all(len(names) == 4 and set(names) == {'Front','Left','Back','Right'} for names in sensor_groups.values()))
    checks['rendered_sensor_times_match_5ms_requests'] = sensor_valid and sensor_time_error <= 1e-7
    checks['no_fixed_sensor_frame_delay'] = sensor_valid and sensor_absolute_time_error <= 1e-7
    checks['moving_depth_matches_actual_pose'], evidence['moving_depth'] = moving_depth_audit(folder,scenario,rows)
    overlay_path = folder/'simulation/overlay_depth_check.json'
    overlay = json.loads(overlay_path.read_text()) if overlay_path.exists() else {}
    checks['overlay_depth_isolation'] = (overlay.get('passed') is True
        and len(overlay.get('checks', [])) == 4
        and {camera.get('camera') for camera in overlay.get('checks', [])} == {'Front','Left','Back','Right'}
        and all(len(camera.get('comparisons', [])) == 2
                and {item.get('mode') for item in camera.get('comparisons', [])} == {'on','off_after'}
                for camera in overlay.get('checks', []))
        and all(comparison.get('passed') is True for camera in overlay.get('checks', [])
                for comparison in camera.get('comparisons', []))
        and all(comparison.get('invalid_mask_equal') is True
                and math.isfinite(float(comparison['max_finite_absolute_difference_m']))
                and 0 <= float(comparison['max_finite_absolute_difference_m']) <= 1e-5
                for camera in overlay.get('checks', []) for comparison in camera.get('comparisons', [])))
    evidence.update(sensor_rows=len(sensor_rows), maximum_rendered_sensor_time_error_s=sensor_time_error,
                    maximum_absolute_sensor_physics_time_error_s=sensor_absolute_time_error)
    checks['one_controller_decision_per_motion_step'] = len(controller) == len(rows)-1
    control_numeric = ('px','py','pz','vx','vy','vz','ux','uy','uz','next_vx','next_vy','next_vz',
                       'requested_speed','goal_distance','cross_track_m','reference_s','certified_ahead_m','controller_compute_ms')
    checks['controller_finite_records'] = bool(controller) and all(
        math.isfinite(float(decision[key])) for decision in controller for key in control_numeric)
    if checks['one_controller_decision_per_motion_step'] and checks['controller_finite_records']:
        state_error = command_error = reply_error = 0.
        protocol_ok = input_ready = reference_only = True
        reference_steps = emergency_steps = inherited_steps = 0
        for i, decision in enumerate(controller, 1):
            state_error = max(state_error,
                norm([float(decision[k])-positions[i-1][j] for j,k in enumerate(('px','py','pz'))]),
                norm([float(decision[k])-velocities[i-1][j] for j,k in enumerate(('vx','vy','vz'))]))
            command_error = max(command_error,
                norm([float(decision[k])-commands[i][j] for j,k in enumerate(('ux','uy','uz'))]))
            reply_error = max(reply_error,
                norm([float(decision[k])-velocities[i][j] for j,k in enumerate(('next_vx','next_vy','next_vz'))]))
            mode, flags = int(decision['decision']), int(decision['flags'])
            protocol_ok &= (int(decision['sequence']) == i and int(decision['simulation_ns']) == round((i-1)*dt*1e9)
                and int(rows[i]['sequence']) == i and int(rows[i]['simulation_ns']) == round(i*dt*1e9)
                and int(rows[i]['reply_simulation_ns']) == int(decision['simulation_ns'])
                and int(rows[i]['mode']) == mode and int(rows[i]['flags']) == flags)
            input_ready &= int(decision['input_ready']) == 1
            if norm(commands[i]) > 1e-8:
                reference_steps += 1
                reference_only &= (mode in (1,2,3) and bool(flags & 2)
                    and decision['tracking_status'] == 'ready'
                    and decision['reason'] == 'reference_command_certified')
            else:
                reference_only &= mode in (0,4,5)
            emergency_steps += mode == 5
            inherited_steps += int(decision['inherited_stop']) != 0
        checks.update(controller_actual_state_matches=state_error <= 1e-8,
            controller_commands_match_execution=command_error <= 1e-8,
            controller_prediction_matches_execution=reply_error <= 1e-8,
            ipc_sequence_and_time_match=protocol_ok, all_controller_inputs_ready=input_ready,
            all_nonzero_commands_from_certified_reference=reference_only and reference_steps > 0)
        evidence.update(controller_sha256=digest(controller_path),
            controller_state_error=state_error, controller_command_error=command_error,
            controller_reply_error=reply_error, certified_reference_command_steps=reference_steps,
            emergency_stop_steps=emergency_steps, inherited_stop_steps=inherited_steps)
    evidence.update(trajectory_sha256=digest(trajectory_path), controller_rows=len(controller),
        maximum_position_model_error_m=position_error, maximum_velocity_model_error_mps=velocity_error,
        maximum_simulation_dt_error_s=max_dt_error, maximum_kit_time_error_s=max_kit_error,
        maximum_actual_physics_time_error_s=max_physics_error,
        minimum_swept_clearance_m=clearance, swept_collision_segments=collisions,
        peak_speed_mps=max(speeds), peak_acceleration_mps2=peak_acceleration, path_length_m=length,
        final_goal_error_m=errors[-1], final_speed_mps=speeds[-1], arrival_hold_s=hold,
        z_range_m=max(p[2] for p in positions)-min(p[2] for p in positions), simulation_s=times[-1]-times[0])
    return {'passed': all(checks.values()), 'checks': checks,
            'failed': [k for k,v in checks.items() if not v], 'evidence': evidence,
            'reported_status': reported.get('status','MISSING_RESULT'),
            'scope': 'motion/geometry correctness; scheduler and algorithm identity require separate execution audit'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir', type=Path)
    args = parser.parse_args()
    try:
        result = audit(args.result_dir.resolve())
    except (ValueError, KeyError, OSError, TypeError) as error:
        result = {'passed': False, 'failed': ['audit_input_or_schema'], 'error': str(error),
                  'scope': 'audit could not validate this incomplete or malformed run'}
    target = args.result_dir/'independent_flight_audit.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'passed': result['passed'], 'failed': result['failed'], 'audit': str(target)}, ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
