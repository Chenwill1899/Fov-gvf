#!/usr/bin/env python3
"""Construct the three installed ROS launch routes without executing any Node.

Run after sourcing /opt/ros/humble/setup.bash and the P5 install/setup.bash.
Only launch-description construction and Node._perform_substitutions are used;
Node.execute, LaunchService.run and subprocess are never invoked. Humble's
substitution step writes the actual ROS parameter YAML, which is retained here.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import traceback


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, data):
    with Path(path).open('x') as out:
        json.dump(data, out, indent=2, allow_nan=False)
        out.write('\n')


def construct(label, launch_path, output, project, expected, fixed_goal, explicit_override):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    import yaml
    directory=output/label
    directory.mkdir()
    before_env=dict(os.environ);previous_temp=tempfile.tempdir
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
        if explicit_override is not None:os.environ['FOV_GVF_CONTROLLER_EXECUTABLE']=str(explicit_override)
        tempfile.tempdir=str(directory)
        spec=importlib.util.spec_from_file_location('route_'+label,launch_path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        description=module.generate_launch_description()
        node=next(entity for entity in description.entities if isinstance(entity,Node))
        context=LaunchContext();context.launch_configurations['rviz']='false'
        # This is only the pre-execution substitution/parameter serialization
        # step of the real Humble Node implementation, not Node.execute().
        node._perform_substitutions(context)
        context.extend_locals({'ros_specific_arguments':{'ns':'__ns:='+node.expanded_node_namespace}})
        command=[perform_substitutions(context,arg) for arg in node.cmd]
        files=[Path(command[i+1]) for i,x in enumerate(command[:-1]) if x=='--params-file']
        if len(files)!=1:raise ValueError('expected exactly one actual Node parameter file')
        class RosYamlLoader(yaml.SafeLoader):
            pass
        # Humble serializes normalized array parameters as !!python/tuple.
        # Accept that one inert sequence tag only; never use unsafe yaml.load.
        RosYamlLoader.add_constructor('tag:yaml.org,2002:python/tuple',
            lambda loader,node:loader.construct_sequence(node))
        parsed=yaml.load(files[0].read_text(),Loader=RosYamlLoader)
        if len(parsed)!=1:raise ValueError('unexpected ROS parameter YAML scope')
        parameters=next(iter(parsed.values()))['ros__parameters']
        runtime_path=directory/('result.parameters.json' if fixed_goal else 'runtime.json')
        runtime=json.loads(runtime_path.read_text())
        executable=Path(command[0]);digest=sha(executable)
        checks={
            'node_command_matches_expected_path':executable.resolve()==Path(expected['path']).resolve(),
            'node_binary_matches_pretested_sha':digest==expected['expected_sha256'],
            'recorded_executable_matches_command':Path(runtime['executable']).resolve()==executable.resolve(),
            'recorded_sha_matches_command':runtime['executable_sha256']==digest,
            'node_use_fixed_goal_matches_route':parameters.get('use_fixed_goal') is fixed_goal,
            'recorded_launch_sha_matches_cloud_source':runtime['launch_sha256']==sha(project/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'),
            'all_six_features_disabled':all(parameters.get(k) is False for k in (
                'depth_uncertainty_enabled','paper_dynamic_obstacles','paper_incremental_field',
                'paper_spherical_memory','operator_assistance','paper_shared_obstacles')),
        }
        if fixed_goal:
            checks['recorded_effective_parameters_equal_actual_node_yaml']=runtime['effective']==parameters
            checks['actual_effective_use_fixed_goal_true']=runtime['effective'].get('use_fixed_goal') is True
        if explicit_override is not None:checks['explicit_override_has_priority']=executable.resolve()==Path(explicit_override).resolve()
        return dict(route=label,passed=all(checks.values()),checks=checks,actual_command=command,
            executable_sha256=digest,expected=expected,actual_node_parameters=parameters,
            actual_node_parameter_file={'path':str(files[0]),'sha256':sha(files[0])},
            launcher_record={'path':str(runtime_path),'sha256':sha(runtime_path),'data':runtime},
            installed_launch={'path':str(launch_path),'sha256':sha(launch_path)},
            process_execution_api_called=False,
            scope='Real installed launch description and ROS Node substitutions constructed; no controller/bridge/simulator process started. Manual launcher records executable and SHA, not effective params; actual Node YAML is independently retained here.')
    finally:
        os.environ.clear();os.environ.update(before_env);tempfile.tempdir=previous_temp


def main():
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent/'launch_routes')
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    project=Path(__file__).resolve().parents[3]
    identity_path=Path(__file__).with_name('binary_identity.json')
    identity=json.loads(identity_path.read_text())
    source=project/'src/pc_gvf/launch';installed=Path(get_package_share_directory('pc_gvf'))/'launch'
    tracked={str(identity_path):sha(identity_path)}
    for name in ('isaac_cloud_navigation.launch.py','navigation_benchmark.launch.py'):
        for directory in (source,installed):tracked[str(directory/name)]=sha(directory/name)
    binary_checks={}
    for key,row in identity.items():
        actual=sha(row['path']);reference=sha(project/row['reference'])
        binary_checks[key]=dict(actual=actual,reference=reference,expected=row['expected_sha256'],matches=actual==reference==row['expected_sha256'])
        tracked[row['path']]=actual
    manual=identity['manual/depth_angular_controller'];goal=identity['goal/depth_angular_controller']
    override=dict(manual,path=str(project/manual['reference']))
    configs=[('default_cloud_manual',installed/'isaac_cloud_navigation.launch.py',manual,False,None),
        ('default_p5_fixed_goal',installed/'navigation_benchmark.launch.py',goal,True,None),
        ('fixed_goal_explicit_baseline_override',installed/'navigation_benchmark.launch.py',override,True,Path(override['path']))]
    results=[]
    for label,path,expected,fixed,explicit in configs:
        try:result=construct(label,path,out,project,expected,fixed,explicit)
        except Exception as exc:result=dict(route=label,passed=False,error=str(exc),traceback=traceback.format_exc(),process_execution_api_called=False)
        write_new(out/(label+'.json'),result);results.append(result)
    launch_identity={name:sha(source/name)==sha(installed/name) for name in ('isaac_cloud_navigation.launch.py','navigation_benchmark.launch.py')}
    unchanged=all(Path(path).is_file() and sha(path)==digest for path,digest in tracked.items())
    passed=all(r['passed'] for r in results) and all(row['matches'] for row in binary_checks.values()) and all(launch_identity.values()) and unchanged
    summary=dict(passed=passed,routes_passed=sum(r['passed'] for r in results),routes_total=3,
        install_prefix=get_package_prefix('pc_gvf'),binary_identity=binary_checks,installed_matches_source=launch_identity,
        tracked_files_unchanged=unchanged,input_sha256=tracked,script_sha256=sha(__file__),
        routes=[dict(route=r['route'],passed=r['passed'],result_file=str(out/(r['route']+'.json'))) for r in results],
        scope='Only the main installed cloud/manual and P5 fixed-goal routes plus explicit override. demo.launch.py intentionally outside this contract. Construction is not Isaac execution validation.',
        process_execution_api_called=False)
    write_new(out/'summary.json',summary)
    print(json.dumps(summary,indent=2))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
