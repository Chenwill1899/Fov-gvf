"""Evidence failures must not become successful high-speed simulation trials."""
import csv
import json

import numpy as np
import pytest

from sim4_benchmark import audit_trial, high_speed_duration, write_json


@pytest.mark.parametrize('flags', [
    ['--pipeline-delay', '-0.1'], ['--pipeline-delay', 'nan'], ['--pipeline-delay', 'inf'],
    ['--tau', '0.024'], ['--tau', 'nan'], ['--tau', 'inf'],
    ['--vertical-speed', '0'], ['--vertical-speed', '-1'], ['--vertical-speed', 'nan'],
    ['--restart-clearance', '-0.01'], ['--restart-clearance', 'nan'],
])
def test_invalid_motion_protocol_options_are_rejected(flags):
    from sim4_benchmark import parser, validate_args
    cli = parser()
    with pytest.raises(SystemExit):
        validate_args(cli, cli.parse_args(['negative'] + flags))


def test_new_motion_options_reach_protocol_and_keep_fixed_body():
    from sim4_benchmark import parser, protocol_for, validate_args
    cli = parser()
    args = cli.parse_args(['protocol', '--tau', '.025', '--vertical-speed', '2',
                          '--restart-clearance', '.1', '--speed', '5', '--accel', '32'])
    validate_args(cli, args)
    protocol = protocol_for(args, 'original')
    assert protocol['version'] == 'ego2p1_sim4_v6'
    assert protocol['plant_tau_s'] == .025
    assert protocol['max_vertical_speed_mps'] == 2.
    assert protocol['restart_clearance_m'] == .1
    assert protocol['max_speed_mps'] == 5.
    assert protocol['plant_accel_mps2'] == 32.
    assert protocol['body_radius_m'] == .48
    assert protocol['free_space_floor_enabled'] is True
    assert protocol['free_space_floor_z'] == 0.


@pytest.mark.parametrize('change', ['none', 'source', 'binary', 'metadata'])
def test_frozen_source_and_actual_execution_evidence(tmp_path, monkeypatch, change):
    import sim4_benchmark as benchmark
    root = tmp_path / 'project'
    source = root / 'src' / 'controller.cpp'
    source.parent.mkdir(parents=True)
    source.write_text('original source')
    install = tmp_path / 'install'
    install.mkdir()
    executable = install / 'controller'
    executable.write_text('original executable')
    executable.chmod(0o755)
    monkeypatch.setattr(benchmark, 'ROOT', root)
    folder = root / 'performance' / 'new'
    folder.mkdir(parents=True)
    result = folder / 'run.json'
    environment = {'FOV_GVF_INSTALL': str(install), 'FOV_GVF_CONTROLLER_EXECUTABLE': str(executable)}
    manifest = benchmark.freeze_trial_evidence(result, {'speed': 4.}, ['bash', 'run.sh'], environment)
    with pytest.raises(RuntimeError, match='refusing to overwrite'):
        benchmark.freeze_trial_evidence(result, {'speed': 4.}, ['bash', 'run.sh'], environment)
    write_json(result.with_suffix('.parameters.json'), dict(algorithm='ego2p1',
        executable=str(executable), executable_sha256=manifest['expected_executable_sha256'],
        effective=dict(max_speed=4., performance_run_id='one', paper_replay_directory='one')))
    if change == 'source':
        source.write_text('edited during run')
    if change == 'binary':
        executable.write_text('rebuilt during run')
    if change == 'metadata':
        parameters = json.loads(result.with_suffix('.parameters.json').read_text())
        parameters['executable_sha256'] = 'wrong'
        write_json(result.with_suffix('.parameters.json'), parameters)
    final = benchmark.finish_trial_evidence(result, manifest)
    assert final['complete']
    assert bool(final['changed_files_during_run']) == (change in ('source', 'binary'))
    assert final['execution_matches_frozen'] == (change != 'metadata')
    original = folder / final['files'][str(source)]['blob']
    assert original.read_text() == 'original source'


def fixture(tmp_path):
    scene = tmp_path / 'scene.usda'
    scene.write_text('# empty fixture')
    shape = (40, 40, 20)
    np.zeros(shape, np.uint8).tofile(scene.with_name('occupancy.bin'))
    result = tmp_path / 'trial.json'
    t = np.arange(211) / 60.
    speed = np.minimum(4., np.maximum(0., np.minimum(4. * t, 4. * (3. - t))))
    x = -78. + np.cumsum(speed / 60.)
    goal = [float(x[-1]), -58., 1.6]
    protocol = dict(start=[-78., -58., 1.6], goal=goal, warmup_s=0., timeout_s=10.,
        max_speed_mps=4., max_vertical_speed_mps=1., body_radius_m=.48,
        plant_tau_s=.22, plant_accel_mps2=4., max_depth_m=10.,
        pipeline_delay_s=.2, arrival_radius_m=1., arrival_speed_mps=.1, arrival_hold_s=.5)
    write_json(result.with_suffix('.protocol.json'), protocol)
    write_json(result, dict(protocol=protocol, status='ARRIVED', collision_blocks=0,
        process_exit_code=0, simulation_elapsed_s=float(t[-1]), wall_elapsed_s=4.,
        goal=goal, final_position=goal, final_distance_m=0.))
    write_json(result.with_suffix('.parameters.json'), dict(effective=dict(max_speed=4.,
        max_vertical_speed=1., command_accel_limit=4., brake_accel_limit=4.,
        body_radius=.48, max_depth=10., paper_pipeline_delay=.2, velocity_response_time=.22),
        runtime_environment=dict(ISAAC_COMMAND_FRAME='world')))
    result.with_suffix('.log').write_text('')
    fields = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz',
              'applied_target_x', 'applied_target_y', 'applied_target_z', 'status']
    rows = []
    for now, position, velocity in zip(t, x, speed):
        row = dict.fromkeys(fields, 0.)
        row.update(t=float(now), x=float(position), y=-58., z=1.6,
            vx=float(velocity), qx=1., applied_target_x=float(velocity), status='FIXTURE')
        rows.append(row)
    save_rows(result, fields, rows)
    return result, scene, shape, fields, rows


def save_rows(result, fields, rows):
    with result.with_suffix('.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


@pytest.mark.parametrize('failure', ['none', 'forged_arrival', 'teleport', 'missing_frame', 'body_frame', 'brake_mismatch'])
def test_rejects_inconsistent_execution(tmp_path, failure):
    result, scene, shape, fields, rows = fixture(tmp_path)
    if failure == 'forged_arrival':
        data = json.loads(result.read_text())
        data['goal'][0] += 3.
        data['protocol']['goal'][0] += 3.
        data['final_position'] = data['goal']
        write_json(result, data)
        write_json(result.with_suffix('.protocol.json'), data['protocol'])
    elif failure == 'teleport':
        rows[len(rows) // 2]['x'] += .1
    elif failure == 'missing_frame':
        del rows[len(rows) // 2]
    elif failure in ('body_frame', 'brake_mismatch'):
        path = result.with_suffix('.parameters.json')
        data = json.loads(path.read_text())
        if failure == 'body_frame':
            data['runtime_environment']['ISAAC_COMMAND_FRAME'] = 'body'
        else:
            data['effective']['brake_accel_limit'] = 2.5
        write_json(path, data)
    save_rows(result, fields, rows)
    report = audit_trial(result, scene, shape)
    assert report['legacy_safe_arrival']
    assert report['accepted'] == (failure == 'none')
    assert report['safe_arrival'] == (failure == 'none')
    assert report['cap_attained']
    assert report['time_ge_3p8_s'] >= 1.


def test_high_speed_uses_executed_end_of_step_velocity():
    cumulative, longest = high_speed_duration(np.array([0., 1., 2., 3.]), np.array([0., 4., 0., 4.]), 3.8)
    assert cumulative == 2.
    assert longest == 1.


@pytest.mark.parametrize('failure', ['none', 'restart', 'proposal_tau', 'rebuilt', 'wrong_binary', 'missing_manifest'])
def test_v4_requires_motion_and_frozen_execution_contract(tmp_path, failure):
    import hashlib
    from sim4_benchmark import value_sha256
    result, scene, shape, _, _ = fixture(tmp_path)
    data = json.loads(result.read_text())
    data['protocol'].update(version='ego2p1_sim4_v4', restart_clearance_m=.1)
    write_json(result, data)
    write_json(result.with_suffix('.protocol.json'), data['protocol'])
    parameters_path = result.with_suffix('.parameters.json')
    parameters = json.loads(parameters_path.read_text())
    parameters['effective'].update(paper_restart_clearance=0. if failure == 'restart' else .1,
        paper_proposal_response_time=.03 if failure == 'proposal_tau' else .22)
    write_json(parameters_path, parameters)
    manifest = dict(complete=True, execution_matches_frozen=failure != 'wrong_binary',
        changed_files_during_run=['controller'] if failure == 'rebuilt' else [],
        prepared_protocol_sha256=value_sha256(data['protocol']),
        parameters_file_sha256=hashlib.sha256(parameters_path.read_bytes()).hexdigest())
    if failure == 'missing_manifest':
        with pytest.raises(FileNotFoundError):
            audit_trial(result, scene, shape)
        return
    write_json(result.with_suffix('.manifest.json'), manifest)
    report = audit_trial(result, scene, shape)
    assert report['accepted'] == (failure == 'none')


@pytest.mark.parametrize('plant_tau,certified_tau,accepted', [
    (.22, .22, True), (.03, .03, True), (.03, .22, False), (.22, .03, False)])
def test_tau_contract_matches_execution_without_rejecting_legacy_tau(tmp_path, plant_tau, certified_tau, accepted):
    result, scene, shape, _, _ = fixture(tmp_path)
    data = json.loads(result.read_text())
    data['protocol']['plant_tau_s'] = plant_tau
    write_json(result, data)
    write_json(result.with_suffix('.protocol.json'), data['protocol'])
    path = result.with_suffix('.parameters.json')
    parameters = json.loads(path.read_text())
    parameters['effective']['velocity_response_time'] = certified_tau
    write_json(path, parameters)
    report = audit_trial(result, scene, shape)
    assert report['legacy_safe_arrival']
    assert report['cap_attained']
    assert report['additional_checks']['tau_parameter_matches'] == accepted
    assert report['safe_arrival'] == accepted
    assert report['accepted'] == accepted
    assert report['plant_tau_s'] == plant_tau
    assert ('tau_parameter_matches' in report['rejection_reasons']) != accepted


def test_safe_arrival_without_reaching_requested_cap_is_not_accepted(tmp_path):
    result, scene, shape, fields, rows = fixture(tmp_path)
    for row in rows:
        row['vx'] *= .5
        row['x'] = -78. + (row['x'] + 78.) * .5
        row['applied_target_x'] *= .5
    data = json.loads(result.read_text())
    goal = [rows[-1]['x'], -58., 1.6]
    data.update(goal=goal, final_position=goal)
    data['protocol']['goal'] = goal
    write_json(result, data)
    write_json(result.with_suffix('.protocol.json'), data['protocol'])
    save_rows(result, fields, rows)
    report = audit_trial(result, scene, shape)
    assert report['safe_arrival']
    assert not report['cap_attained']
    assert not report['accepted']


def test_rejects_horizon_and_smoothing_configuration_mismatch(tmp_path):
    result, scene, shape, _, _ = fixture(tmp_path)
    data = json.loads(result.read_text())
    data['protocol'].update(planning_horizon_s=1., smooth_speed=False)
    write_json(result, data)
    write_json(result.with_suffix('.protocol.json'), data['protocol'])
    path = result.with_suffix('.parameters.json')
    parameters = json.loads(path.read_text())
    parameters['effective'].update(planning_horizon=3., paper_smooth_speed=True)
    write_json(path, parameters)
    report = audit_trial(result, scene, shape)
    assert report['legacy_safe_arrival']
    assert not report['additional_checks']['planning_horizon_parameter_matches']
    assert not report['additional_checks']['smooth_speed_parameter_matches']
    assert not report['accepted']


@pytest.mark.parametrize('mismatch', [None, 'paper_shared_obstacles', 'depth_uncertainty_sigma_range2'])
def test_perception_contract_cannot_be_silently_changed(tmp_path, mismatch):
    result, scene, shape, _, _ = fixture(tmp_path)
    expected = dict(paper_shared_obstacles=False, paper_spherical_memory=False,
        depth_uncertainty_enabled=True, depth_uncertainty_sigma0=.005,
        depth_uncertainty_sigma_range2=0., depth_uncertainty_pose_bound=.01)
    data = json.loads(result.read_text())
    data['protocol']['perception_parameters'] = expected
    write_json(result, data)
    write_json(result.with_suffix('.protocol.json'), data['protocol'])
    path = result.with_suffix('.parameters.json')
    parameters = json.loads(path.read_text())
    parameters['effective'].update(expected)
    if mismatch:
        parameters['effective'][mismatch] = True if mismatch == 'paper_shared_obstacles' else .001
    write_json(path, parameters)
    report = audit_trial(result, scene, shape)
    assert report['legacy_safe_arrival']
    assert report['additional_checks']['perception_parameters_match'] == (mismatch is None)
    assert report['accepted'] == (mismatch is None)


@pytest.mark.parametrize('case', [
    'enabled', 'disabled', 'custom_height', 'legacy_missing', 'legacy_explicit_disabled',
    'silently_disabled', 'silently_moved', 'missing_enabled_parameter', 'missing_height_parameter',
    'partial_protocol', 'protocol_string_bool', 'protocol_numeric_bool', 'protocol_bool_height',
    'protocol_nan_height', 'parameter_string_bool', 'parameter_bool_height', 'parameter_nan_height',
    'legacy_silently_enabled',
])
def test_free_space_floor_contract_is_bound_without_changing_geometry_audit(tmp_path, case):
    result, scene, shape, _, _ = fixture(tmp_path)
    data = json.loads(result.read_text())
    path = result.with_suffix('.parameters.json')
    parameters = json.loads(path.read_text())
    explicit = not case.startswith('legacy_')
    if explicit:
        data['protocol'].update(free_space_floor_enabled=True, free_space_floor_z=0.)
        parameters['effective'].update(paper_free_space_floor_enabled=True, paper_free_space_floor_z=0.)
    if case == 'disabled':
        data['protocol']['free_space_floor_enabled'] = False
        parameters['effective']['paper_free_space_floor_enabled'] = False
    elif case == 'custom_height':
        data['protocol']['free_space_floor_z'] = .15
        parameters['effective']['paper_free_space_floor_z'] = .15
    elif case == 'legacy_explicit_disabled':
        parameters['effective'].update(paper_free_space_floor_enabled=False, paper_free_space_floor_z=0.)
    elif case in ('silently_disabled', 'parameter_string_bool'):
        parameters['effective']['paper_free_space_floor_enabled'] = False if case == 'silently_disabled' else 'true'
    elif case == 'silently_moved':
        parameters['effective']['paper_free_space_floor_z'] = -.2
    elif case == 'missing_enabled_parameter':
        del parameters['effective']['paper_free_space_floor_enabled']
    elif case == 'missing_height_parameter':
        del parameters['effective']['paper_free_space_floor_z']
    elif case == 'partial_protocol':
        del data['protocol']['free_space_floor_z']
    elif case in ('protocol_string_bool', 'protocol_numeric_bool'):
        data['protocol']['free_space_floor_enabled'] = 'true' if case == 'protocol_string_bool' else 1
    elif case in ('protocol_bool_height', 'protocol_nan_height'):
        data['protocol']['free_space_floor_z'] = True if case == 'protocol_bool_height' else float('nan')
    elif case in ('parameter_bool_height', 'parameter_nan_height'):
        parameters['effective']['paper_free_space_floor_z'] = True if case == 'parameter_bool_height' else float('nan')
    elif case == 'legacy_silently_enabled':
        parameters['effective'].update(paper_free_space_floor_enabled=True, paper_free_space_floor_z=0.)
    # Deliberately malformed JSON numbers reach the audit for finite-value tests.
    result.write_text(json.dumps(data))
    result.with_suffix('.protocol.json').write_text(json.dumps(data['protocol']))
    path.write_text(json.dumps(parameters))
    report = audit_trial(result, scene, shape)
    expected = case in ('enabled', 'disabled', 'custom_height', 'legacy_missing', 'legacy_explicit_disabled')
    assert report['legacy_safe_arrival']
    assert report['swept_overlap_segments'] == 0
    assert report['accepted'] == expected
    if not expected:
        assert any(reason.startswith('free_space_floor_') for reason in report['rejection_reasons'])


def sensor_record(hfov=110., requested=110.):
    import math
    ha, va = 24., 18.
    focal = ha/(2.*math.tan(math.radians(hfov/2.)))
    cameras=[]
    for name,suffix,topic in [('Front','',''),('Left','Left','_left'),('Back','Back','_back'),('Right','Right','_right')]:
        cameras.append(dict(name=name,prim_path=f'/World/Robot/CameraMount{suffix}/DepthCamera',
            depth_topic=f'sim/depth{topic}/image_raw',camera_info_topic=f'sim/depth{topic}/camera_info',
            frame_id='camera'+topic,width=320,height=240,focal_length=focal,horizontal_aperture=ha,
            vertical_aperture=va,hfov_deg=hfov,vfov_deg=math.degrees(2.*math.atan(va/(2.*focal))),
            fx=320*focal/ha,fy=240*focal/va,cx=160.,cy=120.,projection='perspective',
            lens_distortion_model=None,horizontal_aperture_offset=0.,vertical_aperture_offset=0.))
    return dict(version='ego2p1_runtime_sensors_v1',run_id='fixture',scene_path='scene.usda',
        requested_hfov_deg=requested,override_applied=requested is not None,
        render_width=320,render_height=240,cameras=cameras)


def v5_fixture(tmp_path, requested=110.):
    import hashlib
    from sim4_benchmark import value_sha256, freeze_sensor_evidence
    result,scene,shape,fields,rows=fixture(tmp_path)
    data=json.loads(result.read_text())
    data['protocol'].update(version='ego2p1_sim4_v5',restart_clearance_m=.1,depth_sensor_hfov_deg=requested,
        free_space_floor_enabled=True,free_space_floor_z=0.)
    write_json(result,data);write_json(result.with_suffix('.protocol.json'),data['protocol'])
    path=result.with_suffix('.parameters.json');parameters=json.loads(path.read_text())
    parameters['effective'].update(paper_restart_clearance=.1,paper_proposal_response_time=.22,
        paper_free_space_floor_enabled=True,paper_free_space_floor_z=0.)
    parameters.update(executable_sha256='controller',common_source_sha256={'script':'source'})
    write_json(path,parameters)
    record=sensor_record(requested if requested is not None else 90.,requested)
    write_json(result.with_suffix('.sensors.json'),record)
    manifest=dict(complete=True,execution_matches_frozen=True,changed_files_during_run=[],
        prepared_protocol_sha256=value_sha256(data['protocol']),parameters_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        requested_environment={'ISAAC_DEPTH_HFOV_DEG':str(requested)} if requested is not None else {})
    freeze_sensor_evidence(result,manifest);write_json(result.with_suffix('.manifest.json'),manifest)
    return result,scene,shape,record,manifest


@pytest.mark.parametrize('failure', ['none','missing_file','missing_blob','changed_after_freeze',
    'request_env','wrong_actual_hfov','missing_camera','duplicate_camera','wrong_resolution',
    'wrong_vertical_aperture','wrong_intrinsics','invalid_focal','distortion','orthographic','nan'])
def test_v5_requires_actual_frozen_sensor_geometry(tmp_path,failure):
    from sim4_benchmark import freeze_sensor_evidence
    result,scene,shape,record,manifest=v5_fixture(tmp_path)
    if failure=='missing_file': result.with_suffix('.sensors.json').unlink()
    elif failure=='missing_blob': (result.parent/manifest['sensors_blob']).unlink()
    elif failure=='request_env': manifest['requested_environment']['ISAAC_DEPTH_HFOV_DEG']='120'
    elif failure!='none':
        if failure=='changed_after_freeze': record['run_id']='edited'
        elif failure=='wrong_actual_hfov': record=sensor_record(120.,110.)
        elif failure=='missing_camera': record['cameras'].pop()
        elif failure=='duplicate_camera': record['cameras'][1]=record['cameras'][0].copy()
        elif failure=='wrong_resolution': record['cameras'][0]['width']=640
        elif failure=='wrong_vertical_aperture': record['cameras'][0]['vertical_aperture']=24.
        elif failure=='wrong_intrinsics': record['cameras'][0]['fx']+=1.
        elif failure=='invalid_focal': record['cameras'][0]['focal_length']=0.
        elif failure=='distortion': record['cameras'][0]['lens_distortion_model']='fisheye'
        elif failure=='orthographic': record['cameras'][0]['projection']='orthographic'
        elif failure=='nan': record['cameras'][0]['hfov_deg']=float('nan')
        result.with_suffix('.sensors.json').write_text(json.dumps(record))
        if failure!='changed_after_freeze': freeze_sensor_evidence(result,manifest)
    write_json(result.with_suffix('.manifest.json'),manifest)
    report=audit_trial(result,scene,shape)
    assert report['audit_version']=='sim4_v5'
    assert report['legacy_safe_arrival']
    assert report['accepted']==(failure=='none')
    assert report['additional_checks']['restart_parameter_matches']
    assert report['additional_checks']['frozen_execution_matches']


def test_v5_default_records_geometry_without_request(tmp_path):
    result,scene,shape,_,_=v5_fixture(tmp_path,requested=None)
    report=audit_trial(result,scene,shape)
    assert report['accepted'] and report['requested_depth_hfov_deg'] is None
    assert report['actual_depth_hfov_deg']==[90.]*4


@pytest.mark.parametrize('raw',['0','180','-1','nan','inf','garbage',''])
def test_invalid_sensor_request_is_rejected_before_launch(monkeypatch,raw):
    from sim4_benchmark import parser,validate_args
    monkeypatch.setenv('ISAAC_DEPTH_HFOV_DEG',raw)
    cli=parser()
    with pytest.raises(SystemExit): validate_args(cli,cli.parse_args(['invalid_sensor']))


def test_sensor_profile_ignores_run_ids_but_not_actual_fov():
    from sim4_benchmark import sensor_profile,value_sha256
    first=sensor_record();second=sensor_record();second.update(run_id='other',scene_path='other')
    assert value_sha256(sensor_profile(first))==value_sha256(sensor_profile(second))
    assert value_sha256(sensor_profile(first))!=value_sha256(sensor_profile(sensor_record(120.,120.)))


def test_v5_freezes_requested_sensor_environment_and_runtime_bytes(tmp_path,monkeypatch):
    import sim4_benchmark as benchmark
    root=tmp_path/'project';root.mkdir();install=tmp_path/'install';install.mkdir()
    executable=install/'controller';executable.write_text('binary');executable.chmod(0o755)
    monkeypatch.setattr(benchmark,'ROOT',root)
    result=root/'run.json';env={'FOV_GVF_INSTALL':str(install),'FOV_GVF_CONTROLLER_EXECUTABLE':str(executable),
        'ISAAC_DEPTH_HFOV_DEG':'120'}
    manifest=benchmark.freeze_trial_evidence(result,{'version':'ego2p1_sim4_v5'},['run'],env)
    write_json(result.with_suffix('.sensors.json'),sensor_record(120.,120.))
    benchmark.finish_trial_evidence(result,manifest)
    assert manifest['requested_environment']['ISAAC_DEPTH_HFOV_DEG']=='120'
    assert (result.parent/manifest['sensors_blob']).read_bytes()==result.with_suffix('.sensors.json').read_bytes()
    assert manifest['sensor_profile_sha256']


def test_summary_separates_actual_sensor_profiles_and_detects_tampering(tmp_path):
    import importlib.util
    from pathlib import Path
    script=Path(__file__).resolve().with_name('summarize_trial_evidence.py')
    spec=importlib.util.spec_from_file_location('sensor_summary_fixture',script)
    summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)
    outputs=[]
    for index,hfov in enumerate((110.,120.)):
        folder=tmp_path/str(index);folder.mkdir()
        result,scene,shape,record,manifest=v5_fixture(folder,requested=hfov)
        audit_trial(result,scene,shape)
        # Same environment deliberately isolates grouping by actual sensor hash.
        manifest['requested_environment']={}
        write_json(result.with_suffix('.manifest.json'),manifest)
        errors=[];row=summary.read_trial(result.with_suffix(''),errors)
        assert not errors and row['sensor_identity_consistent']
        outputs.append((result,row))
    assert outputs[0][1]['group_id']!=outputs[1][1]['group_id']
    assert outputs[0][1]['cross_map_family_id']!=outputs[1][1]['cross_map_family_id']
    result=outputs[0][0]
    record=json.loads(result.with_suffix('.sensors.json').read_text());record['run_id']='tampered'
    write_json(result.with_suffix('.sensors.json'),record)
    row=summary.read_trial(result.with_suffix(''),[])
    assert not row['sensor_identity_consistent'] and not row['identity_complete_and_consistent']


@pytest.mark.parametrize('hfov', [90.,120.,140.])
def test_camera_cli_protocol_preserves_floor_and_requested_fov(monkeypatch,hfov):
    from sim4_benchmark import parser,protocol_for,validate_args
    monkeypatch.delenv('ISAAC_DEPTH_HFOV_DEG',raising=False)
    cli=parser();args=cli.parse_args(['sensor_cli','--camera-hfov',str(hfov)])
    validate_args(cli,args);protocol=protocol_for(args,'original')
    assert protocol['version']=='ego2p1_sim4_v6'
    assert protocol['depth_sensor_hfov_deg']==hfov
    assert protocol['free_space_floor_enabled'] is True and protocol['free_space_floor_z']==0.


@pytest.mark.parametrize('hfov',['0','180','nan','inf','-1'])
def test_camera_cli_rejects_invalid_projection(hfov):
    from sim4_benchmark import parser,validate_args
    cli=parser()
    with pytest.raises(SystemExit):validate_args(cli,cli.parse_args(['sensor_cli','--camera-hfov',hfov]))


def test_unset_camera_cli_preserves_native_90_without_override(monkeypatch):
    from sim4_benchmark import parser,protocol_for,validate_args
    monkeypatch.delenv('ISAAC_DEPTH_HFOV_DEG',raising=False)
    cli=parser();args=cli.parse_args(['sensor_cli']);validate_args(cli,args)
    assert args.camera_hfov is None
    assert protocol_for(args,'original')['depth_sensor_hfov_deg'] is None


def test_no_override_cannot_silently_change_native_camera_fov(tmp_path):
    from sim4_benchmark import freeze_sensor_evidence
    result,scene,shape,_,manifest=v5_fixture(tmp_path,requested=None)
    write_json(result.with_suffix('.sensors.json'),sensor_record(120.,None))
    freeze_sensor_evidence(result,manifest);write_json(result.with_suffix('.manifest.json'),manifest)
    report=audit_trial(result,scene,shape)
    assert not report['accepted'] and not report['additional_checks']['sensor_record_valid']


def test_v5_cannot_omit_floor_contract(tmp_path):
    import hashlib
    from sim4_benchmark import value_sha256
    result,scene,shape,_,manifest=v5_fixture(tmp_path)
    data=json.loads(result.read_text())
    for key in ('free_space_floor_enabled','free_space_floor_z'): del data['protocol'][key]
    write_json(result,data);write_json(result.with_suffix('.protocol.json'),data['protocol'])
    path=result.with_suffix('.parameters.json');parameters=json.loads(path.read_text())
    for key in ('paper_free_space_floor_enabled','paper_free_space_floor_z'): del parameters['effective'][key]
    write_json(path,parameters)
    manifest.update(prepared_protocol_sha256=value_sha256(data['protocol']),parameters_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    write_json(result.with_suffix('.manifest.json'),manifest)
    report=audit_trial(result,scene,shape)
    assert report['additional_checks']['frozen_execution_matches']
    assert not report['additional_checks']['free_space_floor_protocol_complete']
    assert not report['additional_checks']['free_space_floor_parameters_present']
    assert not report['accepted']


def v6_fixture(tmp_path, requested_offset=.6):
    import hashlib
    from sim4_benchmark import freeze_sensor_evidence, value_sha256
    result, scene, shape, record, manifest = v5_fixture(tmp_path, requested=120.)
    data = json.loads(result.read_text())
    data['protocol'].update(version='ego2p1_sim4_v6', depth_sensor_camera_offset_m=requested_offset)
    write_json(result, data); write_json(result.with_suffix('.protocol.json'), data['protocol'])
    offset = .22 if requested_offset is None else requested_offset
    path = result.with_suffix('.parameters.json'); parameters = json.loads(path.read_text())
    parameters['effective']['camera_offset'] = [offset, 0., .02]
    write_json(path, parameters)
    manifest.update(prepared_protocol_sha256=value_sha256(data['protocol']),
                    parameters_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    if requested_offset is not None:
        manifest['requested_environment']['ISAAC_DEPTH_CAMERA_OFFSET_M'] = str(requested_offset)
    record.update(version='ego2p1_runtime_sensors_v2', requested_camera_offset_m=requested_offset,
                  camera_offset_override_applied=requested_offset is not None)
    front = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    for camera, yaw in zip(record['cameras'], (0., 90., 180., -90.)):
        c, s = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
        rz = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        position = (rz @ np.array([offset, 0., .02])).tolist()
        camera.update(mount_path=camera['prim_path'].rsplit('/', 1)[0],
                      mount_translation_robot_m=position, camera_position_robot_m=position.copy(),
                      robot_from_optical_rotation=(rz @ front).tolist(),
                      camera_local_matrix4x4=np.eye(4).tolist())
    write_json(result.with_suffix('.sensors.json'), record)
    freeze_sensor_evidence(result, manifest); write_json(result.with_suffix('.manifest.json'), manifest)
    return result, scene, shape, record, manifest


@pytest.mark.parametrize('failure', ['none', 'legacy_record', 'missing_protocol', 'missing_request',
    'request_env', 'request_false', 'request_numeric_string', 'override_flag', 'mount_path',
    'mount_position', 'camera_position', 'camera_height', 'rotation', 'local_matrix',
    'nonfinite_position', 'bool_position', 'missing_extrinsics', 'missing_effective',
    'effective_offset', 'effective_height', 'effective_type'])
def test_v6_binds_requested_actual_usd_and_effective_camera_offset(tmp_path, failure):
    import hashlib
    from sim4_benchmark import freeze_sensor_evidence, value_sha256
    result, scene, shape, record, manifest = v6_fixture(tmp_path)
    data = json.loads(result.read_text())
    path = result.with_suffix('.parameters.json'); parameters = json.loads(path.read_text())
    if failure == 'legacy_record': record['version'] = 'ego2p1_runtime_sensors_v1'
    elif failure == 'missing_protocol': del data['protocol']['depth_sensor_camera_offset_m']
    elif failure == 'missing_request': del record['requested_camera_offset_m']
    elif failure == 'request_env': manifest['requested_environment']['ISAAC_DEPTH_CAMERA_OFFSET_M'] = '.22'
    elif failure == 'request_false': record['requested_camera_offset_m'] = False
    elif failure == 'request_numeric_string': record['requested_camera_offset_m'] = '.6'
    elif failure == 'override_flag': record['camera_offset_override_applied'] = False
    elif failure == 'mount_path': record['cameras'][0]['mount_path'] = '/other'
    elif failure == 'mount_position': record['cameras'][0]['mount_translation_robot_m'][0] = .22
    elif failure == 'camera_position': record['cameras'][1]['camera_position_robot_m'][1] = .22
    elif failure == 'camera_height': record['cameras'][0]['camera_position_robot_m'][2] = .2
    elif failure == 'rotation': record['cameras'][0]['robot_from_optical_rotation'] = np.eye(3).tolist()
    elif failure == 'local_matrix': record['cameras'][0]['camera_local_matrix4x4'][3][0] = .1
    elif failure == 'nonfinite_position': record['cameras'][0]['camera_position_robot_m'][0] = float('nan')
    elif failure == 'bool_position': record['cameras'][0]['camera_position_robot_m'][1] = False
    elif failure == 'missing_extrinsics': del record['cameras'][0]['robot_from_optical_rotation']
    elif failure == 'missing_effective': del parameters['effective']['camera_offset']
    elif failure == 'effective_offset': parameters['effective']['camera_offset'][0] = .22
    elif failure == 'effective_height': parameters['effective']['camera_offset'][2] = .2
    elif failure == 'effective_type': parameters['effective']['camera_offset'][1] = False
    write_json(result, data); write_json(result.with_suffix('.protocol.json'), data['protocol'])
    write_json(path, parameters)
    manifest.update(prepared_protocol_sha256=value_sha256(data['protocol']),
                    parameters_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    result.with_suffix('.sensors.json').write_text(json.dumps(record))
    freeze_sensor_evidence(result, manifest); write_json(result.with_suffix('.manifest.json'), manifest)
    report = audit_trial(result, scene, shape)
    assert report['audit_version'] == 'sim4_v6'
    assert report['legacy_safe_arrival']
    assert report['accepted'] == (failure == 'none')
    if failure != 'none': assert report['rejection_reasons']


def test_v6_default_keeps_native_offset_and_records_actual_extrinsics(tmp_path):
    result, scene, shape, record, manifest = v6_fixture(tmp_path, requested_offset=None)
    report = audit_trial(result, scene, shape)
    assert report['accepted'] and report['requested_depth_camera_offset_m'] is None
    assert report['actual_camera_positions_robot_m'][0] == [.22, 0., .02]


@pytest.mark.parametrize('raw', ['0', '-1', 'nan', 'inf', '-inf', 'garbage', ''])
def test_camera_offset_env_rejects_invalid_values_before_launch(monkeypatch, raw):
    from sim4_benchmark import parser, validate_args
    monkeypatch.setenv('ISAAC_DEPTH_CAMERA_OFFSET_M', raw)
    cli = parser()
    with pytest.raises(SystemExit): validate_args(cli, cli.parse_args(['invalid_offset']))


@pytest.mark.parametrize('raw', ['0', '-1', 'nan', 'inf'])
def test_camera_offset_cli_rejects_invalid_values_before_launch(monkeypatch, raw):
    from sim4_benchmark import parser, validate_args
    monkeypatch.delenv('ISAAC_DEPTH_CAMERA_OFFSET_M', raising=False)
    cli = parser()
    with pytest.raises(SystemExit): validate_args(cli, cli.parse_args(['invalid_offset', '--camera-offset', raw]))


@pytest.mark.parametrize('env,cli_value,expected', [(None,None,None),('.7',None,.7),(None,'.6',.6),('.7','.6',.6)])
def test_camera_offset_request_honors_cli_then_environment(monkeypatch, env, cli_value, expected):
    from sim4_benchmark import parser, protocol_for, validate_args
    if env is None: monkeypatch.delenv('ISAAC_DEPTH_CAMERA_OFFSET_M', raising=False)
    else: monkeypatch.setenv('ISAAC_DEPTH_CAMERA_OFFSET_M', env)
    cli = parser(); args = cli.parse_args(['offset'] + (['--camera-offset', cli_value] if cli_value else []))
    validate_args(cli, args)
    protocol = protocol_for(args, 'original')
    assert protocol['version'] == 'ego2p1_sim4_v6'
    assert protocol['depth_sensor_camera_offset_m'] == expected
    assert protocol['free_space_floor_enabled'] is True and protocol['free_space_floor_z'] == 0.


def test_sensor_profile_v2_binds_every_extrinsic_field(tmp_path):
    import copy
    from sim4_benchmark import sensor_profile, value_sha256
    result, scene, shape, record, manifest = v6_fixture(tmp_path)
    original = value_sha256(sensor_profile(record))
    for field in ('mount_path', 'mount_translation_robot_m', 'camera_position_robot_m',
                  'robot_from_optical_rotation', 'camera_local_matrix4x4'):
        changed = copy.deepcopy(record); changed['cameras'][0][field] = 'changed'
        assert value_sha256(sensor_profile(changed)) != original
    changed = copy.deepcopy(record); changed.update(run_id='another', scene_path='another')
    assert value_sha256(sensor_profile(changed)) == original


def test_v6_summary_separates_actual_camera_extrinsics(tmp_path):
    import importlib.util
    from pathlib import Path
    script = Path(__file__).resolve().with_name('summarize_trial_evidence.py')
    spec = importlib.util.spec_from_file_location('offset_summary_fixture', script)
    summary = importlib.util.module_from_spec(spec); spec.loader.exec_module(summary)
    outputs = []
    for i, offset in enumerate((.6, .7)):
        folder = tmp_path/str(i); folder.mkdir()
        result, scene, shape, record, manifest = v6_fixture(folder, requested_offset=offset)
        assert audit_trial(result, scene, shape)['accepted']
        errors = []; row = summary.read_trial(result.with_suffix(''), errors)
        assert not errors and row['sensor_identity_consistent']
        outputs.append(row)
    assert outputs[0]['sensor_profile_sha256'] != outputs[1]['sensor_profile_sha256']
    assert outputs[0]['group_id'] != outputs[1]['group_id']
    assert outputs[0]['cross_map_family_id'] != outputs[1]['cross_map_family_id']


def test_v6_freezes_camera_offset_request_and_actual_extrinsics(tmp_path, monkeypatch):
    import sim4_benchmark as benchmark
    root = tmp_path/'project'; root.mkdir(); install = tmp_path/'install'; install.mkdir()
    executable = install/'controller'; executable.write_text('binary'); executable.chmod(0o755)
    monkeypatch.setattr(benchmark, 'ROOT', root)
    result = root/'run.json'
    env = {'FOV_GVF_INSTALL':str(install), 'FOV_GVF_CONTROLLER_EXECUTABLE':str(executable),
           'ISAAC_DEPTH_HFOV_DEG':'120', 'ISAAC_DEPTH_CAMERA_OFFSET_M':'.6'}
    manifest = benchmark.freeze_trial_evidence(result, {'version':'ego2p1_sim4_v6'}, ['run'], env)
    fixture_dir = tmp_path/'fixture'; fixture_dir.mkdir()
    record = v6_fixture(fixture_dir)[3]
    write_json(result.with_suffix('.sensors.json'), record)
    benchmark.finish_trial_evidence(result, manifest)
    assert manifest['requested_environment']['ISAAC_DEPTH_CAMERA_OFFSET_M'] == '.6'
    assert (result.parent/manifest['sensors_blob']).read_bytes() == result.with_suffix('.sensors.json').read_bytes()
    assert manifest['sensor_profile_sha256'] == benchmark.value_sha256(benchmark.sensor_profile(record))


@pytest.mark.parametrize('floor_z', [-.2, 0., 1.4])
def test_planner_floor_cli_reaches_protocol_without_disabling_floor(floor_z):
    from sim4_benchmark import parser, protocol_for, validate_args
    cli = parser(); args = cli.parse_args(['floor_cli', '--floor-z', str(floor_z)])
    validate_args(cli, args)
    protocol = protocol_for(args, 'original')
    assert protocol['version'] == 'ego2p1_sim4_v6'
    assert protocol['free_space_floor_enabled'] is True
    assert protocol['free_space_floor_z'] == floor_z
    assert protocol['body_radius_m'] == .48


def test_planner_floor_cli_default_remains_zero():
    from sim4_benchmark import parser, protocol_for, validate_args
    cli = parser(); args = cli.parse_args(['floor_default'])
    validate_args(cli, args)
    assert args.floor_z == 0.
    assert protocol_for(args, 'original')['free_space_floor_z'] == 0.


@pytest.mark.parametrize('floor_z', ['nan', 'inf', '-inf'])
def test_planner_floor_cli_rejects_nonfinite_height(floor_z):
    from sim4_benchmark import parser, validate_args
    cli = parser()
    with pytest.raises(SystemExit):
        validate_args(cli, cli.parse_args(['invalid_floor', '--floor-z=' + floor_z]))


@pytest.mark.parametrize('height', [.47, 1.6])
def test_planner_floor_height_never_moves_independent_geometry_plane(tmp_path, height):
    result, scene, shape, fields, rows = fixture(tmp_path)
    data = json.loads(result.read_text())
    for key in ('goal', 'final_position'): data[key][2] = height
    for key in ('start', 'goal'): data['protocol'][key][2] = height
    for row in rows: row['z'] = height
    save_rows(result, fields, rows)
    path = result.with_suffix('.parameters.json'); parameters = json.loads(path.read_text())
    geometry = []
    for floor_z in (-.2, 0., 1.4):
        data['protocol'].update(free_space_floor_enabled=True, free_space_floor_z=floor_z)
        parameters['effective'].update(paper_free_space_floor_enabled=True, paper_free_space_floor_z=floor_z)
        write_json(result, data); write_json(result.with_suffix('.protocol.json'), data['protocol'])
        write_json(path, parameters)
        report = audit_trial(result, scene, shape)
        assert report['additional_checks']['free_space_floor_z_parameter_matches']
        assert report['accepted'] == (height == 1.6)
        assert (report['swept_overlap_segments'] > 0) == (height == .47)
        geometry.append((report['swept_overlap_segments'], report['minimum_swept_clearance_m']))
    assert geometry[0] == geometry[1] == geometry[2]


def test_working_bounds_require_actual_parameters_even_for_safe_trace():
    from sim4_benchmark import working_bounds_checks
    bounds = dict(enabled=True, min_x=-80., max_x=399.8, min_y=-60., max_y=59.8, min_z=0., max_z=19.8)
    effective = dict(paper_working_bounds_enabled=True, paper_working_min_x=-80., paper_working_max_x=399.8,
        paper_working_min_y=-60., paper_working_max_y=59.8, paper_free_space_floor_enabled=True,
        paper_free_space_ceiling_enabled=True, paper_free_space_floor_z=0., paper_free_space_ceiling_z=19.8)
    p = np.array([[160., -59., 10.], [160., -58., 10.]])
    assert all(working_bounds_checks({'working_bounds': bounds}, effective, p, .48).values())
    effective['paper_working_bounds_enabled'] = False
    checks = working_bounds_checks({'working_bounds': bounds}, effective, p, .48)
    assert checks['working_bounds_physical_body'] and not checks['working_bounds_parameters_match']


def test_recorded_y_boundary_failure_rejected_before_center_leaves_map():
    from sim4_benchmark import working_bounds_checks
    bounds = dict(enabled=True, min_x=-80., max_x=399.8, min_y=-60., max_y=59.8, min_z=0., max_z=19.8)
    p = np.array([[163.835, -59.98357188539688, 13.509]])
    checks = working_bounds_checks({'working_bounds': bounds}, {}, p, .48)
    assert checks['working_bounds_protocol_valid'] and not checks['working_bounds_physical_body']


def test_working_bounds_contract_rejects_invalid_geometry_and_preserves_legacy():
    from sim4_benchmark import working_bounds_checks
    p = np.zeros((3, 3))
    assert working_bounds_checks({}, {}, p, .48) == {}
    bounds = dict(enabled=True, min_x=2., max_x=2., min_y=-60., max_y=59.8, min_z=0., max_z=19.8)
    assert not working_bounds_checks({'working_bounds': bounds}, {}, p, .48)['working_bounds_protocol_valid']
    bounds['max_x'] = float('nan')
    assert not working_bounds_checks({'working_bounds': bounds}, {}, p, .48)['working_bounds_protocol_valid']
