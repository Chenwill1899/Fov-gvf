"""Run a fixed-goal controller with the shared vehicle and goal protocol."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    roots = {
        'ego2p6': '/home/starry/isaac-data/EGO2P6',
        'ego2p5': '/home/starry/isaac-data/EGO2P5',
        'ego2p4': '/home/starry/isaac-data/EGO2P4',
        'ego2p3': '/home/starry/isaac-data/EGO2P3',
        'ego2p2': '/home/starry/isaac-data/EGO2P2',
        'ego2p1': '/home/starry/isaac-data/EGO2P1',
        'ego2p0': '/home/starry/isaac-data/EGO2P0',
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
    for key in ('max_speed_mps', 'max_vertical_speed_mps', 'body_radius_m',
                'plant_accel_mps2', 'plant_tau_s'):
        if not math.isfinite(protocol[key]) or protocol[key] <= 0.:
            raise ValueError(f'{key} must be finite and positive')
    for key, default in (('max_depth_m', 10.), ('goal_proposal_cache_ttl_s', .15)):
        value = protocol.get(key, default)
        if not math.isfinite(value) or value <= 0.:
            raise ValueError(f'{key} must be finite and positive')
    for key, default in (('pipeline_delay_s', .20), ('restart_clearance_m', 0.)):
        value = protocol.get(key, default)
        if not math.isfinite(value) or value < 0.:
            raise ValueError(f'{key} must be finite and nonnegative')
    if protocol.get('goal_proposal_cache_ttl_s', .15) > 10.:
        raise ValueError('goal_proposal_cache_ttl_s must not exceed 10 seconds')
    if os.environ.get('ISAAC_COMMAND_FRAME', 'world') != 'world':
        raise ValueError('benchmark bridge requires ISAAC_COMMAND_FRAME=world')
    if protocol['plant_tau_s'] < .025:
        raise ValueError('plant_tau_s must be at least the .025 second certificate step')
    planning_horizon = protocol.get('planning_horizon_s', 3.)
    if type(planning_horizon) not in (int, float) or not math.isfinite(planning_horizon) or planning_horizon <= 0.:
        raise ValueError('planning_horizon_s must be a finite positive number')
    if type(protocol.get('smooth_speed', True)) is not bool:
        raise ValueError('smooth_speed must be a boolean')
    if type(protocol.get('free_space_floor_enabled', False)) is not bool:
        raise ValueError('free_space_floor_enabled must be a boolean')
    floor_z = protocol.get('free_space_floor_z', 0.)
    if type(floor_z) not in (int, float) or not math.isfinite(floor_z):
        raise ValueError('free_space_floor_z must be a finite number')
    sensor_offset = protocol.get('depth_sensor_camera_offset_m')
    if sensor_offset is not None and (type(sensor_offset) not in (int, float) or
            not math.isfinite(sensor_offset) or sensor_offset <= 0.):
        raise ValueError('depth_sensor_camera_offset_m must be finite and positive or null')
    source = root / 'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    tree = ast.parse(source.read_text())
    nodes = sorted((n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == 'Node'), key=lambda n: n.lineno)
    parameter_ast = next(k.value.elts[0] for k in nodes[0].keywords if k.arg == 'parameters')
    history_max_observations = int(os.environ.get('FOV_GVF_HISTORY_MAX_OBSERVATIONS', '16'))
    if history_max_observations <= 0:
        raise ValueError('FOV_GVF_HISTORY_MAX_OBSERVATIONS must be positive')
    for name, default in (('FOV_GVF_CLEARANCE_REWARD', '.018'),
                          ('FOV_GVF_HISTORY_SAMPLE_INTERVAL', '1.0')):
        value = float(os.environ.get(name, default))
        if not math.isfinite(value) or value <= 0.:
            raise ValueError(f'{name} must be finite and positive')
    context = dict(os=os, float=float, int=int, history_max_observations=history_max_observations,
                   performance_log=os.environ['FOV_GVF_PERFORMANCE_LOG'],
                   performance_run_id=os.environ['FOV_GVF_RUN_ID'], manual_speed=2.0)
    native = eval(compile(ast.Expression(parameter_ast), str(source), 'eval'),
                  {'__builtins__': {}}, context)
    # The simulator's GoalTrial publishes only human-intent messages. This
    # switch exercises the complete manual policy and base kernel while the
    # independent simulator still owns the route and arrival criterion.
    manual_policy=os.environ.get('EGO5090_MANUAL_POLICY','0')=='1'
    affinity=os.environ.get('EGO5090_CONTROLLER_CPU_AFFINITY','')
    if affinity:
        tokens=affinity.split(',')
        if any(not token.isdecimal() for token in tokens):raise ValueError('CPU affinity must be comma-separated CPU IDs')
        requested={int(token) for token in tokens}
        if len(requested)!=len(tokens) or not requested.issubset(os.sched_getaffinity(0)):
            raise ValueError('CPU affinity contains duplicate or unavailable IDs')
    overrides = dict(fixed_forward_intent=False, use_fixed_goal=not manual_policy, stop_at_goal=False,
                     goal_x=protocol['goal'][0], goal_y=protocol['goal'][1], goal_z=protocol['goal'][2],
                     human_intent_topic='/human_intent', human_intent_timeout=.25,
                     max_depth_age=.3, max_depth=protocol.get('max_depth_m', 10.), speed=protocol['max_speed_mps'],
                     max_speed=protocol['max_speed_mps'], max_vertical_speed=protocol['max_vertical_speed_mps'],
                     body_radius=protocol['body_radius_m'], safety_margin=.02, rollout_margin=.02,
                     command_accel_limit=protocol['plant_accel_mps2'],
                     brake_accel_limit=protocol['plant_accel_mps2'],
                     velocity_response_time=protocol['plant_tau_s'],
                     paper_proposal_response_time=protocol['plant_tau_s'],
                     planning_horizon=protocol.get('planning_horizon_s', 3.),
                     paper_smooth_speed=protocol.get('smooth_speed', True),
                     paper_pipeline_delay=protocol.get('pipeline_delay_s', .20))
    overrides['paper_goal_proposal_cache']=protocol.get('goal_proposal_cache',False)
    overrides['paper_goal_speed_governor']=protocol.get('goal_speed_governor',False)
    overrides['paper_goal_proposal_cache_ttl']=protocol.get('goal_proposal_cache_ttl_s',.15)
    overrides['paper_restart_clearance']=protocol.get('restart_clearance_m',0.)
    # Legacy protocols keep their original disabled floor; new sim4 protocols
    # explicitly request the same z=0 half-space as their independent audit.
    overrides['paper_free_space_floor_enabled']=protocol.get('free_space_floor_enabled',False)
    overrides['paper_free_space_floor_z']=float(floor_z)
    if 'depth_sensor_camera_offset_m' in protocol:
        overrides['camera_offset']=[.22 if sensor_offset is None else float(sensor_offset), 0., .02]
    perception = protocol.get('perception_parameters', {})
    boolean_perception = {'paper_shared_obstacles', 'paper_spherical_memory', 'depth_uncertainty_enabled'}
    numeric_perception = {'depth_uncertainty_sigma0', 'depth_uncertainty_sigma_range2', 'depth_uncertainty_pose_bound'}
    if not isinstance(perception, dict) or set(perception) - boolean_perception - numeric_perception:
        raise ValueError('unsupported perception_parameters')
    for key, value in perception.items():
        if key in boolean_perception:
            if type(value) is not bool:
                raise ValueError(f'{key} must be a boolean')
        elif type(value) not in (int, float) or not math.isfinite(value) or value < 0.:
            raise ValueError(f'{key} must be finite and nonnegative')
    overrides.update({key: value if key in boolean_perception else float(value)
                      for key, value in perception.items()})
    # The protocol sets the same response time for certification and execution.
    effective = dict(native, **overrides)
    # The policy switch sets use_fixed_goal independently of the geometry-cache
    # backend. Explicit frozen executables remain authoritative.
    default_controller = ('depth_angular_controller_goal' if algorithm in ('ego1p5', 'ego2p0', 'ego2p1', 'ego2p2', 'ego2p3', 'ego2p4', 'ego2p5', 'ego2p6')
                          and effective['use_fixed_goal'] else 'depth_angular_controller')
    executable = Path(os.environ.get('FOV_GVF_CONTROLLER_EXECUTABLE',
        f'{root}/docs/performance/runtime/install/pc_gvf/lib/pc_gvf/{default_controller}'))
    if not executable.is_absolute() or not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError('controller must be an absolute executable file')
    output = Path(os.environ['ISAAC_BENCHMARK_RESULT']).with_suffix('.parameters.json')
    shared_root=Path('/home/starry/isaac-data/EGO2P6')
    shared_files=['scripts/run_isaac_fov_gvf_navigation.sh','scripts/isaac/run_fov_gvf_navigation.py','scripts/isaac/navigation_benchmark.py',
        'scripts/isaac/manual_control_math.py','src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py',
        ]
    shared_hashes={name:hashlib.sha256((shared_root/name).read_bytes()).hexdigest() for name in shared_files}
    shared_hashes['goal_protocol']=hashlib.sha256(Path(os.environ['ISAAC_GOAL_PROTOCOL']).read_bytes()).hexdigest()
    output.write_text(json.dumps(dict(algorithm=algorithm, native=native, overrides=overrides,
        effective=effective, executable=str(executable), common_source_sha256=shared_hashes,
        runtime_environment={'OMP_WAIT_POLICY':os.environ.get('OMP_WAIT_POLICY',''),
            'ISAAC_COMMAND_FRAME':os.environ.get('ISAAC_COMMAND_FRAME','world'),
            'bridge_mode':'world','controller_cpu_affinity_requested':affinity},
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        launch_sha256=hashlib.sha256(source.read_bytes()).hexdigest()), indent=2))
    rviz_config = str(Path(get_package_share_directory('pc_gvf_platforms')) /
                      'config' / 'isaac_cloud_navigation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(executable=str(executable), output='screen', parameters=[effective], prefix=[f'taskset -c {affinity}'] if affinity else []),
        Node(package='pc_gvf_platforms', executable='position_cmd_to_twist', output='screen',
             parameters=[dict(use_sim_time=True, mode='world', odom_topic='/sim/odom',
                 cmd_in_topic='/position_cmd', cmd_out_topic='/sim/cmd_vel',
                 max_vx=protocol['max_speed_mps'], max_vy=protocol['max_speed_mps'],
                 max_vz=protocol['max_vertical_speed_mps'], max_w=0.,
                 performance_log_path=context['performance_log'],
                 performance_run_id=context['performance_run_id'])]),
        Node(
            package="pc_gvf_platforms", executable="navigation_visualizer", output="screen",
            parameters=[{
                "use_sim_time": True,
                "odom_topic": "/sim/odom",
                "command_topic": "/position_cmd",
                "robot_shape": "quadrotor",
                "display_filter_tau": 0.20,
                "marker_publish_rate": 60.0,
                "path_publish_rate": 10.0,
                "path_min_distance": 0.10,
                "path_max_poses": 1000,
            }],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="pc_gvf_platforms", executable="observed_map_visualizer", output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_frame": "world", "radar_frame": "radar",
                "odom_topic": "/sim/odom",
                "depth_topic": "/sim/depth/image_raw",
                "camera_info_topic": "/sim/depth/camera_info",
                "radar_topic": "/pc_gvf/radar_points",
                "map_topic": "/pc_gvf/observed_map",
                "depth_stride": 4,
                "minimum_depth": 0.3,
                "maximum_depth": 15.0,
                "map_resolution": 0.08,
                "minimum_hits": 2,
                "map_publish_rate": 1.0,
                "publication_resolution": 0.25,
                "maximum_publish_points": 160000,
                "maximum_pose_age": 0.20,
                "camera_offset": [float(os.environ.get("ISAAC_DEPTH_CAMERA_OFFSET_M", "0.22")), 0.0, 0.02],
            }],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
