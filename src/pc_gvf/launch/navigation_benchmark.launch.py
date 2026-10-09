"""Run a fixed-goal controller with the shared vehicle and goal protocol."""
import ast
import hashlib
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    roots = {
        'ego1p5': '/home/starry/isaac-data/EGO1P5',
        'ego1p3': '/home/starry/isaac-data/EGO1P3',
        'ego1p2': '/home/starry/isaac-data/EGO1P2',
        'ego1p1': '/home/starry/isaac-data/EGO1P1',
        'ego1p0': '/home/starry/isaac-data/EGO1P0',
        'user_ego': '/home/starry/isaac-data/user_ego/Fov-gvf',
    }
    algorithm = os.environ['FOV_GVF_BENCHMARK_ALGORITHM']
    root = Path(roots[algorithm])
    protocol = json.loads(Path(os.environ['ISAAC_GOAL_PROTOCOL']).read_text())
    source = root / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    tree = ast.parse(source.read_text())
    nodes = sorted((n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == 'Node'), key=lambda n: n.lineno)
    parameter_ast = next(k.value.elts[0] for k in nodes[0].keywords if k.arg == 'parameters')
    context = dict(os=os, float=float, performance_log=os.environ['FOV_GVF_PERFORMANCE_LOG'],
                   performance_run_id=os.environ['FOV_GVF_RUN_ID'])
    native = eval(compile(ast.Expression(parameter_ast), str(source), 'eval'),
                  {'__builtins__': {}}, context)
    overrides = dict(fixed_forward_intent=False, use_fixed_goal=True, stop_at_goal=False,
                     goal_x=protocol['goal'][0], goal_y=protocol['goal'][1], goal_z=protocol['goal'][2],
                     human_intent_topic='/human_intent', human_intent_timeout=.25,
                     max_depth_age=.3, max_depth=10., speed=protocol['max_speed_mps'],
                     max_speed=protocol['max_speed_mps'], max_vertical_speed=protocol['max_vertical_speed_mps'],
                     body_radius=protocol['body_radius_m'], safety_margin=.02, rollout_margin=.02,
                     command_accel_limit=protocol['plant_accel_mps2'])
    effective = dict(native, **overrides)
    # This launcher sets use_fixed_goal=True above. Only P5 fixed-goal runs
    # use the geometry-reuse kernel; the manual launcher keeps the original
    # controller. Explicit frozen executables remain authoritative for ablation.
    default_controller = ('depth_angular_controller_goal' if algorithm == 'ego1p5'
                          and effective['use_fixed_goal'] else 'depth_angular_controller')
    executable = Path(os.environ.get('FOV_GVF_CONTROLLER_EXECUTABLE',
        f'/tmp/fov_gvf_{algorithm}_isaac_install/pc_gvf/lib/pc_gvf/{default_controller}'))
    if not executable.is_absolute() or not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError('controller must be an absolute executable file')
    output = Path(os.environ['ISAAC_BENCHMARK_RESULT']).with_suffix('.parameters.json')
    shared_root=Path('/home/starry/isaac-data/EGO1P5')
    shared_files=['scripts/run_isaac_fov_gvf_navigation.sh','scripts/isaac/run_fov_gvf_navigation.py','scripts/isaac/navigation_benchmark.py',
        'scripts/isaac/manual_control_math.py','src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py',
        'performance/navigation_benchmark_20260929/protocol.json']
    shared_hashes={name:hashlib.sha256((shared_root/name).read_bytes()).hexdigest() for name in shared_files}
    output.write_text(json.dumps(dict(algorithm=algorithm, native=native, overrides=overrides,
        effective=effective, executable=str(executable), common_source_sha256=shared_hashes,
        runtime_environment={'OMP_WAIT_POLICY':os.environ.get('OMP_WAIT_POLICY','')},
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        launch_sha256=hashlib.sha256(source.read_bytes()).hexdigest()), indent=2))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(executable=str(executable), output='screen', parameters=[effective]),
        Node(package='pc_gvf_platforms', executable='position_cmd_to_twist', output='screen',
             parameters=[dict(use_sim_time=True, mode='holonomic', odom_topic='/sim/odom',
                 cmd_in_topic='/position_cmd', cmd_out_topic='/sim/cmd_vel',
                 max_vx=2., max_vy=2., max_vz=1., max_w=0.,
                 performance_log_path=context['performance_log'],
                 performance_run_id=context['performance_run_id'])]),
    ])
