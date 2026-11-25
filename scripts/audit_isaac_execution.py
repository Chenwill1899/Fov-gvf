#!/usr/bin/python3
"""Independently verify closed-loop build, process and scheduling evidence.

This audit does not certify the trajectory; audit_isaac_closed_loop.py owns the
separate geometry/plant audit. Saved /proc samples establish observed scheduling,
not continuous absence of unsampled changes or a hard real-time deadline.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shlex

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / 'docs/performance/isaac_closed_loop_20261117'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_run(run):
    errors, checks, counts = [], [], {}

    def check(condition, name):
        (checks if condition else errors).append(name)

    def read(name):
        return json.loads((run / name).read_text())

    def project_file(relative, wanted, label):
        path = (ROOT / relative).resolve()
        check(path.is_relative_to(ROOT) and path.is_file() and sha(path) == wanted, label + ': ' + relative)

    report = dict(schema=1, simulation_only=True, result_dir=str(run), errors=errors,
                  checks=checks, counts=counts, audit_scope='build, identity, saved process samples and IPC frame cadence',
                  limitations=[
                      'Scheduling evidence consists of saved /proc samples; it is not a continuous kernel trace.',
                      'The simulator uses lockstep requests and separately paced wall time, not absolute 200 Hz releases.',
                      'Controller compute excludes IPC, image rendering/copy, plant integration, GUI and log serialization.',
                      'FIFO priority does not prove a 5 ms deadline or successful arrival.',
                      'The plant is a first-order kinematic velocity model, not rigid-body quadrotor physics.',
                      'Isaac launcher/Python/version are hashed; this is not a complete hash of every installed Isaac asset.'])
    try:
        execution, scope, manifest = read('execution.json'), read('scope.json'), read('source_manifest.json')
        check(execution['scope'] == scope, 'execution scope equals copied scope')
        check(execution['status'] == 'COMPLETED', 'supervisor completed without recorded failure')
        check(scope['simulation_only'] is True and scope['simulation_motion_authorized'] is True, 'explicit simulation-only authorization')
        check(scope['world_dimensions'] == 3 and scope['physics_dt_s'] == .005, 'three-dimensional world and 5 ms simulation step')
        check(scope['production_motion_flag'] is False and scope['production_install_modified'] is False, 'production motion remains disabled')
        check(scope['old_controller_bridge_used'] is False, 'old controller bridge not used')
        check(scope['global_rt_settings_modified'] is False and execution['kernel_before'] == execution['kernel_after'], 'global RT settings unchanged')
        check(scope['algorithm_threads'] == 5 and scope['coordinator_cpu'] == 0 and scope['camera_cpus'] == [1, 2, 3, 4], 'five algorithm threads mapped to CPUs 0 through 4')
        check(scope['simulator_policy'] == 'non-real-time' and scope['simulator_main_policy'] == 0 and scope['simulator_worker_policies'] == [0, 3, 5] and scope['simulator_thread_priority'] == 0, 'simulator main OTHER0 and workers non-real-time OTHER/BATCH/IDLE')
        fifo = scope['scheduler'] == 'FIFO5'
        check(fifo or scope['scheduler'] == 'correctness-only SCHED_OTHER', 'explicit supported scheduling mode')
        check(scope['performance_run'] is fifo and scope['ordinary_run_has_performance_conclusion'] is False, 'ordinary mode does not claim performance')
        report['scheduler'] = scope['scheduler']
        report['performance_claim'] = 'FIFO execution scope only; timing and flight outcome require separate analysis' if fifo else 'none: ordinary-scheduling correctness only'
        copied_manifest = Path(scope['manifest']).resolve()
        check(copied_manifest.is_relative_to(EXPERIMENT) and copied_manifest.is_file() and sha(copied_manifest) == sha(run / 'source_manifest.json'), 'copied manifest equals selected isolated build')
        check(manifest['simulation_only'] is True and manifest['world_dimensions'] == 3 and manifest['tests_run'] is True, 'isolated build includes successful tests')
        check(all(command['exit_code'] == 0 for command in manifest['build_commands']), 'all recorded build/test/install commands succeeded')
        check({r['name'] for r in manifest['build_commands']} == {'configure', 'build', 'ctest', 'isolated_install'}, 'build stages are complete')
        for group in ['source_identity', 'production_identity']:
            for relative, wanted in manifest[group].items():
                project_file(relative, wanted, group)
            counts[group] = len(manifest[group])
        for group in ['binary', 'supervisor_check', 'source_archive']:
            project_file(manifest[group]['path'], manifest[group]['sha256'], group)
        for group in ['dependencies', 'simulation_runtime_identity', 'compile_dependency_identity']:
            for filename, wanted in manifest[group].items():
                path = Path(filename)
                check(path.is_absolute() and path.is_file() and sha(path) == wanted, group + ': ' + filename)
            counts[group] = len(manifest[group])
        project_file('docs/performance/isaac_closed_loop_20261117/before/source.tar.gz', manifest['snapshot_sha256'], 'before snapshot')
        active_identity = ROOT / 'docs/performance/runtime/install/new_controller/share/new_controller/build_identity.json'
        production_source = json.loads(active_identity.read_text())
        for relative, wanted in production_source.items():
            project_file(relative, wanted, 'installed production source remains current')
            if relative.startswith('src/new_controller/ldlt/include/') and relative.endswith('.hpp'):
                check(manifest['source_identity'].get(relative) == wanted, 'simulation and production use identical candidate header: ' + relative)
            compiled_path = str((ROOT / relative).resolve())
            if compiled_path in manifest['compile_dependency_identity']:
                check(manifest['compile_dependency_identity'][compiled_path] == wanted, 'actual compiled production dependency matches installed identity: ' + relative)
        counts['active_production_source_files'] = len(production_source)
        expected_translation_units = {
            ROOT / 'src/isaac_closed_loop/controller.cpp',
            ROOT / 'src/isaac_closed_loop/supervisor_check.cpp',
            ROOT / 'src/new_controller/ldlt/src/sparse_trial_backend.cpp'}
        found_translation_units = set()
        for entry in manifest['compile_commands']:
            found_translation_units.add(Path(entry['file']).resolve())
            words = entry.get('arguments') or shlex.split(entry['command'])
            check(all(flag in words for flag in ['-O3', '-DNDEBUG', '-fno-fast-math', '-pthread', '-std=c++17',
                '-DEGO4P2_STEP2=1', '-DEGO4P0_CURVE_GUARD=1', '-DEGO4P0_CORNER_SMALL_STEP=1',
                '-DEGO4P0_COARSE_PC_MODE=2', '-DAMGCL_NO_BOOST']), 'compiler flags for ' + entry['file'])
            check('-I' + str(ROOT / 'src/new_controller/ldlt/include') in words, 'current root candidate includes for ' + entry['file'])
            check(not any('/EGO4P2/' in word or '/EGO2P5/' in word or '/EGO2P6/' in word or 'anchor_overlay' in word or 'verified_start' in word for word in words), 'no legacy overlay in ' + entry['file'])
        check(found_translation_units == expected_translation_units, 'exact three current translation units')
        for relative, wanted in execution['input_sha256'].items():
            check((run / relative).is_file() and sha(run / relative) == wanted, 'immutable copied input: ' + relative)
        check(set(execution['input_sha256']) == {'scenario.json', 'base_step1.json', 'navigation_step2.json'}, 'all three independent input copies recorded')
        check(execution['input_files_unchanged'] is True, 'supervisor rechecked copied inputs')
        cache = execution['cache']
        check(cache['root'] == scope['simulator_cache'] == str(EXPERIMENT / 'shared_cache'), 'shared cache is scoped to this experiment')
        check(cache['mutable_runtime_data'] is True and cache['included_in_algorithm_identity'] is False and cache['previous_caches_deleted'] is False, 'cache scope remains distinct from immutable algorithm identity')
        check(cache['gpu_identity']['exit_code'] == 0 and bool(cache['gpu_identity']['output']) and cache['isaac_version'] == scope['isaac_version'], 'GPU/driver and Isaac cache identity recorded')
        if cache['source_seed_receipt_sha256'] is not None:
            check(sha(Path(cache['source_seed_receipt'])) == cache['source_seed_receipt_sha256'], 'original cache copy receipt remains unchanged')
        for filename, wanted in cache['preparation_receipts_sha256'].items():
            path = Path(filename)
            check(path.is_relative_to(EXPERIMENT) and path.is_file() and sha(path) == wanted, 'cache preparation/activation receipt: ' + filename)
        scenario, base = read('scenario.json'), read('base_step1.json')
        check(sha(run / 'scenario.json') == scope['scenario_sha256'], 'scenario matches selected scope')
        check(base['motion_output_enabled'] is False and base['physics_dt_s'] == scenario['physics_dt_s'] == .005, 'base input unchanged and production output disabled')
        check(0 < scope['initialization_timeout_s'] <= 7200 and 0 < scope['wall_timeout_s'] <= 7200, 'bounded initialization and run times')
        check(scope['total_wall_bound_s'] == scope['initialization_timeout_s'] + scope['wall_timeout_s'], 'separate timeout budgets reported correctly')
        check(0 <= execution['sensors_ready_elapsed_s'] <= scope['initialization_timeout_s'] + .2, 'camera readiness within initialization budget')
        check(0 <= execution['run_wall_s'] <= scope['wall_timeout_s'] + 15, 'run and cleanup within bounded run budget')
        check(execution['started_realtime_ns'] <= execution['ended_realtime_ns'] and execution['started_monotonic_ns'] < execution['sensors_ready_monotonic_ns'] <= execution['ended_monotonic_ns'], 'ordered absolute supervisor clocks')
        uid = scope['algorithm_uid']
        check(uid > 0 and uid == scope['simulator_uid'] == ROOT.stat().st_uid, 'both children use the project owner')
        controller, simulator = execution['processes']['controller'], execution['processes']['simulator']
        check(controller['exit_code'] == simulator['exit_code'] == 0, 'both children exited normally')
        actual = controller['actual']
        check(Path(actual['executable']).resolve() == (ROOT / manifest['binary']['path']).resolve() and actual['executable_sha256'] == manifest['binary']['sha256'], 'actual /proc controller ELF matches build')
        check(actual['mapped_libraries_sha256'] == manifest['dependencies'], 'actual mapped native libraries equal validated dependencies')
        check(actual['limits']['rtprio'] == [str(5 if fifo else 0)] * 2, 'actual controller RT priority limit')
        check(actual['limits']['rttime_us'] == (['5000000', '6000000'] if fifo else ['0', '0']), 'actual controller bounded RT runtime limit')
        sim_actual = simulator['actual']
        sim_python = (ROOT.parent / 'isaacsim/kit/python/bin/python3').resolve()
        check(Path(sim_actual['executable']).resolve() == sim_python and sim_actual['executable_sha256'] == manifest['simulation_runtime_identity'][str(sim_python)], 'actual simulator is the validated Isaac Python')
        check(sim_actual['limits']['rtprio'] == ['0', '0'] and sim_actual['limits']['rttime_us'] == ['0', '0'], 'actual simulator cannot acquire real-time priority')
        ready = read('controller/controller_ready.json')
        expected_ready = dict(simulation_only=True, correctness_only=not fifo, algorithm_threads=5,
            coordinator_policy=1 if fifo else 0, coordinator_priority=5 if fifo else 0, coordinator_cpu=0,
            worker_policies=[1 if fifo else 0] * 4, worker_priorities=[5 if fifo else 0] * 4,
            worker_cpus=[1, 2, 3, 4], ready=True)
        check(all(ready.get(key) == value for key, value in expected_ready.items()) and ready == controller['ready'], 'independent native ready agrees with observed scope')
        camera_ready = read('simulation/sensors_ready.json')
        check(camera_ready == simulator['sensors_ready'], 'sensor ready is preserved exactly')
        check(camera_ready['scenario_sha256'] == sha(run / 'scenario.json') and camera_ready['scene_sha256'] == sha(run / 'simulation/scene.usda'), 'generated scene and immutable scenario SHA')
        check(simulator['calibration_sha256'] == sha(run / 'simulation/simulation.sensors.json'), 'actual calibration SHA unchanged')
        for name in ['supervisor_before', 'supervisor_after']:
            check(bool(execution[name]) and all(t['policy'] == t['priority'] == 0 for t in execution[name]), name + ' uses ordinary scheduling')
        expected_names = {'p3_sim_coord': 0, **{'depth_cam' + str(i): i + 1 for i in range(4)}}
        for kind in ['controller', 'simulator']:
            observations = execution[kind + '_thread_observations']
            check(bool(observations), kind + ' has live /proc observations')
            counts[kind + '_observations'] = len(observations)
            counts[kind + '_thread_samples'] = sum(len(o['threads']) for o in observations)
            previous_elapsed = -1
            for index, observation in enumerate(observations):
                threads = observation['threads']
                elapsed = observation['elapsed_s']
                valid = bool(threads) and previous_elapsed <= elapsed <= execution['wall_s']
                previous_elapsed = elapsed
                if kind == 'controller':
                    valid &= len(threads) == 5 and {t['name'] for t in threads} == set(expected_names)
                else:
                    valid &= sum(t['tid'] == simulator['pid'] for t in threads) == 1
                for thread in threads:
                    valid &= thread['uids'] == [uid] * 4 and thread['uid'] == uid
                    valid &= thread['cap_eff'] == thread['cap_prm'] == 0 and thread['no_new_privs'] == 1
                    if kind == 'controller':
                        valid &= thread['policy'] == (1 if fifo else 0)
                    else:
                        valid &= thread['policy'] in [0, 3, 5]
                        if thread['tid'] == simulator['pid']:
                            valid &= thread['policy'] == 0
                    valid &= thread['priority'] == (5 if fifo and kind == 'controller' else 0)
                    if kind == 'controller':
                        valid &= thread['cpus'] == [expected_names.get(thread['name'])]
                check(valid, kind + ' live sample ' + str(index))
        for source, duplicate in [('simulation/trajectory.csv', 'trajectory.csv'), ('simulation/result.json', 'result.json'), ('controller/controller.csv', 'controller.csv')]:
            check(sha(run / source) == sha(run / duplicate), 'top-level artifact equals child original: ' + duplicate)
        with (run / 'controller.csv').open(newline='') as stream:
            controller_rows = list(csv.DictReader(stream))
        with (run / 'trajectory.csv').open(newline='') as stream:
            trajectory_rows = list(csv.DictReader(stream))
        result, summary = read('result.json'), read('controller/execution_summary.json')
        counts['controller_frames'] = len(controller_rows)
        counts['trajectory_rows_including_initial'] = len(trajectory_rows)
        check(len(controller_rows) == result['completed_steps'] == summary['frames'] and len(trajectory_rows) == len(controller_rows) + 1, 'controller/trajectory/result frame counts agree')
        check(summary['normal_eof'] is True and summary['exit_code'] == 0, 'controller summary records normal EOF')
        valid_cadence = bool(controller_rows) and int(trajectory_rows[0]['sequence']) == 0
        for sequence, (control, trajectory) in enumerate(zip(controller_rows, trajectory_rows[1:]), 1):
            valid_cadence &= int(control['sequence']) == int(trajectory['sequence']) == sequence
            valid_cadence &= int(control['simulation_ns']) == int(trajectory['reply_simulation_ns']) == (sequence - 1) * 5_000_000
            valid_cadence &= int(trajectory['simulation_ns']) == sequence * 5_000_000
        check(valid_cadence, 'every paired IPC sequence and simulation timestamp is exact')
        report['run_status'] = result['status']
        report['arrived'] = result['arrived']
        report['artifact_sha256'] = {name: sha(run / name) for name in ['scope.json', 'source_manifest.json', 'scenario.json', 'base_step1.json', 'navigation_step2.json', 'controller.csv', 'trajectory.csv', 'result.json']}
    except Exception as error:
        errors.append('audit could not complete: ' + repr(error))
    report['passed'] = not errors
    report['checks_passed'] = len(checks)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir', type=Path)
    args = parser.parse_args()
    run = args.result_dir.resolve()
    if not run.is_relative_to(EXPERIMENT) or not run.is_dir():
        parser.error('choose an existing isolated simulation result directory')
    report = audit_run(run)
    target = run / 'independent_execution_audit.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(passed=report['passed'], checks=report['checks_passed'], errors=report['errors'], report=str(target)), ensure_ascii=False))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
