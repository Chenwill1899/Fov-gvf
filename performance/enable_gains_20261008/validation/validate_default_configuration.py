#!/usr/bin/env python3
"""Construct 16 real ROS launch cases, without executing nodes or simulation.

Source /opt/ros/humble/setup.bash and the P5 install/setup.bash first. The matrix
checks manual/goal defaults, each explicit six-axis env=0, and all-six env=0.
All generated actual Node parameter YAML and launch-recorded metadata are kept.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import tarfile
import traceback

FEATURES={
    'uncertainty':('FOV_GVF_DEPTH_UNCERTAINTY','depth_uncertainty_enabled',True),
    'dynamic':('FOV_GVF_DYNAMIC_OBSTACLES','paper_dynamic_obstacles',True),
    'incremental':('FOV_GVF_INCREMENTAL_FIELD','paper_incremental_field',False),
    'memory':('FOV_GVF_SPHERICAL_MEMORY','paper_spherical_memory',True),
    'operator':('FOV_GVF_OPERATOR_ASSISTANCE','operator_assistance',False),
    'shared':('FOV_GVF_SHARED_OBSTACLES','paper_shared_obstacles',True),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path,data):
    with Path(path).open('x') as stream:
        json.dump(data,stream,indent=2,allow_nan=False);stream.write('\n')


def enabled_features(value):
    names=value.split(',') if value else []
    if len(names)!=len(set(names)) or any(name not in FEATURES for name in names):
        raise ValueError('enabled features must be unique names from '+','.join(FEATURES))
    return set(names)


def cases(enabled=None):
    enabled={name for name,(_,_,value) in FEATURES.items() if value} if enabled is None else set(enabled)
    if not enabled.issubset(FEATURES):raise ValueError('unknown enabled feature')
    result=[];defaults={param:name in enabled for name,(env,param,value) in FEATURES.items()}
    for route,fixed in [('manual',False),('goal',True)]:
        result.append(dict(label=route+'_defaults',fixed_goal=fixed,environment={},expected=dict(defaults)))
        for feature,(env,param,default) in FEATURES.items():
            result.append(dict(label=route+'_disable_'+feature,fixed_goal=fixed,
                environment={env:'0'},expected={**defaults,param:False}))
        result.append(dict(label=route+'_all_disabled',fixed_goal=fixed,
            environment={env:'0' for env,param,value in FEATURES.values()},
            expected={param:False for env,param,value in FEATURES.values()}))
    return result


def feature_checks(parameters,expected):
    # ROS expects booleans. Missing keys, strings or integer 1/0 must not pass.
    return {key:parameters.get(key) is value for key,value in expected.items()}


def read_ros_parameters(path):
    import yaml
    class RosYamlLoader(yaml.SafeLoader):
        pass
    # Humble emits evaluated array parameters with the inert !!python/tuple tag.
    RosYamlLoader.add_constructor('tag:yaml.org,2002:python/tuple',
        lambda loader,node:loader.construct_sequence(node))
    parsed=yaml.load(Path(path).read_text(),Loader=RosYamlLoader)
    if not isinstance(parsed,dict) or len(parsed)!=1:raise ValueError('expected one ROS parameter scope')
    parameters=next(iter(parsed.values()))['ros__parameters']
    if not isinstance(parameters,dict):raise ValueError('ROS parameters must be a mapping')
    return parameters


def source_freeze_check(directory,project):
    directory=Path(directory).resolve();project=Path(project)
    expected=json.loads((directory/'source_sha256.json').read_text());actual={}
    with tarfile.open(directory/'sources.tar.gz') as archive:
        for member in archive.getmembers():
            if not member.isfile():continue
            key=member.name.removeprefix('./')
            if key in actual:raise ValueError('duplicate source in frozen archive')
            actual[key]=hashlib.sha256(archive.extractfile(member).read()).hexdigest()
    archive_mismatches=sorted(k for k in expected.keys()|actual.keys() if expected.get(k)!=actual.get(k))
    runtime={k:v for k,v in expected.items() if not k.endswith('.md')}
    current_mismatches=sorted(k for k,v in runtime.items() if not (project/k).is_file() or sha(project/k)!=v)
    return dict(verified=not archive_mismatches and not current_mismatches,directory=str(directory),
        manifest_sha256=sha(directory/'source_sha256.json'),archive_sha256=sha(directory/'sources.tar.gz'),
        archived_files=len(actual),current_runtime_files=len(runtime),archive_mismatches=archive_mismatches,
        current_runtime_mismatches=current_mismatches,
        scope='All archived sources checked against their frozen hashes. Current runtime/config/script files checked; mutable .md records are archived but excluded from current runtime identity.')


def construct(case,launch_path,output,project,expected):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    label=case['label'];fixed_goal=case['fixed_goal'];directory=output/label
    directory.mkdir();prior_env=dict(os.environ);prior_temp=tempfile.tempdir
    try:
        for key in list(os.environ):
            if key.startswith(('ISAAC_','FOV_GVF_')):del os.environ[key]
        os.environ.update(
            FOV_GVF_RUN_ID='construct_only_'+label,
            FOV_GVF_PERFORMANCE_LOG=str(directory/'unused_performance.md'),
            FOV_GVF_RUNTIME_MANIFEST=str(directory/'runtime.json'),
            FOV_GVF_BENCHMARK_ALGORITHM='ego1p5',
            ISAAC_GOAL_PROTOCOL=str(project/'performance/navigation_benchmark_20260929/protocol.json'),
            ISAAC_BENCHMARK_RESULT=str(directory/'result.json'))
        os.environ.update(case['environment']);tempfile.tempdir=str(directory)
        spec=importlib.util.spec_from_file_location('route_'+label,launch_path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        description=module.generate_launch_description()
        node=next(entity for entity in description.entities if isinstance(entity,Node))
        context=LaunchContext();context.launch_configurations['rviz']='false'
        # Humble's parameter/substitution step only; never call Node.execute().
        node._perform_substitutions(context)
        context.extend_locals({'ros_specific_arguments':{'ns':'__ns:='+node.expanded_node_namespace}})
        command=[perform_substitutions(context,arg) for arg in node.cmd]
        files=[Path(command[i+1]) for i,arg in enumerate(command[:-1]) if arg=='--params-file']
        if len(files)!=1:raise ValueError('expected one actual Node parameter YAML')
        parameters=read_ros_parameters(files[0])
        runtime_path=directory/('result.parameters.json' if fixed_goal else 'runtime.json')
        runtime=json.loads(runtime_path.read_text());executable=Path(command[0]);digest=sha(executable)
        flags=feature_checks(parameters,case['expected'])
        checks={
            'node_command_matches_expected_path':executable.resolve()==Path(expected['path']).resolve(),
            'node_binary_matches_frozen_sha':digest==expected['expected_sha256'],
            'recorded_executable_matches_command':Path(runtime['executable']).resolve()==executable.resolve(),
            'recorded_sha_matches_command':runtime['executable_sha256']==digest,
            'actual_node_use_fixed_goal_matches_route':parameters.get('use_fixed_goal') is fixed_goal,
            'recorded_launch_sha_matches_cloud_source':runtime['launch_sha256']==sha(project/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'),
            'all_six_feature_parameters_match':all(flags.values()),
        }
        if fixed_goal:
            checks['recorded_effective_matches_actual_node_yaml']=runtime['effective']==parameters
            checks['recorded_effective_is_fixed_goal']=runtime['effective'].get('use_fixed_goal') is True
        return dict(route=label,passed=all(checks.values()),checks=checks,feature_checks=flags,
            explicit_feature_overrides=case['environment'],expected_features=case['expected'],
            actual_command=command,executable_sha256=digest,expected_binary=expected,
            actual_node_parameters=parameters,actual_node_parameter_file={'path':str(files[0]),'sha256':sha(files[0])},
            launcher_record={'path':str(runtime_path),'sha256':sha(runtime_path),'data':runtime},
            launch_file={'path':str(launch_path),'sha256':sha(launch_path)},
            process_execution_api_called=False)
    finally:
        os.environ.clear();os.environ.update(prior_env);tempfile.tempdir=prior_temp


def main():
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    parser=argparse.ArgumentParser(description=__doc__)
    project=Path(__file__).resolve().parents[3]
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent/'default_configuration')
    parser.add_argument('--binary-identity',type=Path,default=Path(__file__).with_name('candidate_binary_identity.json'))
    parser.add_argument('--source-freeze',type=Path,required=True,help='Frozen source_sha256.json + sources.tar.gz directory for this build.')
    parser.add_argument('--enabled',default='uncertainty,dynamic,memory,shared',
        help='Comma-separated expected default-on feature names; empty string means all off.')
    parser.add_argument('--launch-location',choices=('source','installed'),default='installed',
        help='Source construction is preparatory; only installed construction requires installed/source identity.')
    args=parser.parse_args()
    try:enabled=enabled_features(args.enabled)
    except ValueError as exc:parser.error(str(exc))
    matrix=cases(enabled);out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    identity_path=args.binary_identity.resolve();identity=json.loads(identity_path.read_text())
    source_identity=source_freeze_check(args.source_freeze,project)
    source=project/'src/pc_gvf/launch';installed=Path(get_package_share_directory('pc_gvf'))/'launch'
    tracked={str(identity_path):sha(identity_path)}
    for name in ('isaac_cloud_navigation.launch.py','navigation_benchmark.launch.py'):
        for directory in (source,installed):tracked[str(directory/name)]=sha(directory/name)
    binary_checks={}
    for key,row in identity.items():
        actual=sha(row['path']);reference=sha(project/row['reference'])
        binary_checks[key]=dict(actual=actual,reference=reference,expected=row['expected_sha256'],
            matches=actual==reference==row['expected_sha256'])
        tracked[row['path']]=actual
    results=[]
    for case in matrix:
        label=case['label'];fixed=case['fixed_goal']
        launch_directory=source if args.launch_location=='source' else installed
        path=launch_directory/('navigation_benchmark.launch.py' if fixed else 'isaac_cloud_navigation.launch.py')
        expected=identity['goal/depth_angular_controller' if fixed else 'manual/depth_angular_controller']
        try:result=construct(case,path,out,project,expected)
        except Exception as exc:result=dict(route=label,passed=False,error=str(exc),traceback=traceback.format_exc(),process_execution_api_called=False)
        write_new(out/(label+'.json'),result);results.append(result)
    launch_identity={name:sha(source/name)==sha(installed/name) for name in ('isaac_cloud_navigation.launch.py','navigation_benchmark.launch.py')}
    unchanged=all(Path(path).is_file() and sha(path)==digest for path,digest in tracked.items())
    require_install_equivalence=args.launch_location=='installed'
    passed=all(r['passed'] for r in results) and all(r['matches'] for r in binary_checks.values()) and (not require_install_equivalence or all(launch_identity.values())) and unchanged and source_identity['verified']
    summary=dict(passed=passed,cases_passed=sum(r['passed'] for r in results),cases_total=len(matrix),launch_location=args.launch_location,installation_equivalence_required=require_install_equivalence,
        install_prefix=get_package_prefix('pc_gvf'),expected_default_features={p:name in enabled for name,(e,p,v) in FEATURES.items()},
        case_matrix=matrix,binary_identity=binary_checks,source_identity=source_identity,installed_matches_source=launch_identity,
        tracked_files_unchanged=unchanged,input_sha256=tracked,script_sha256=sha(__file__),
        cases=[dict(route=r['route'],passed=r['passed'],result_file=str(out/(r['route']+'.json'))) for r in results],
        scope='Main manual/goal launch defaults and explicit feature=0 overrides only. Controller executables bind the supplied current-build frozen identities; the manual/goal core and replay are separately compared to prior mode-isolated kernels. Configuration construction is not evidence that the newly enabled combination improves closed-loop tasks. Legacy demo is outside this contract.',
        process_execution_api_called=False)
    write_new(out/'summary.json',summary)
    print(json.dumps({k:summary[k] for k in ('passed','cases_passed','cases_total','expected_default_features','installed_matches_source','tracked_files_unchanged')},indent=2))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
