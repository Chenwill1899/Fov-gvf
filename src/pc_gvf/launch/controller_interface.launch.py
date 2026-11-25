"""Wire an explicitly supplied new controller to the retained simulator bridge."""
import importlib.util
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root=Path(__file__).resolve().parents[3]
    # Installed location is nested deeper; environment.sh supplies the actual project.
    root=Path(os.environ.get('EGO4P0_ROOT','/home/starry/isaac-data/EGO4P4'))
    spec=importlib.util.spec_from_file_location('ego4_new_controller',root/'scripts/require_new_controller.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    executable=module.controller_path()
    params=[{'use_sim_time':True}]
    user_params=os.environ.get('EGO4P0_CONTROLLER_PARAMS','')
    if user_params:
        path=Path(user_params).expanduser()
        if not path.is_absolute() or not path.is_file():raise ValueError('EGO4P0_CONTROLLER_PARAMS must name an existing absolute YAML file')
        params.append(str(path))
    rviz=str(Path(get_package_share_directory('pc_gvf_platforms'))/'config/isaac_cloud_navigation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(executable=executable,output='screen',parameters=params),
        Node(package='pc_gvf_platforms',executable='position_cmd_to_twist',output='screen',parameters=[{
            'use_sim_time':True,'mode':'world','odom_topic':'/sim/odom','cmd_in_topic':'/position_cmd','cmd_out_topic':'/sim/cmd_vel',
            'max_vx':float(os.environ.get('JOYSTICK_HORIZONTAL_SPEED','10')),
            'max_vy':float(os.environ.get('JOYSTICK_HORIZONTAL_SPEED','10')),
            'max_vz':float(os.environ.get('JOYSTICK_VERTICAL_SPEED','1')),'max_w':0.0}]),
        Node(package='pc_gvf_platforms',executable='navigation_visualizer',output='screen',parameters=[{'use_sim_time':True,'robot_shape':'quadrotor','odom_topic':'/sim/odom','command_topic':'/position_cmd'}],condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='pc_gvf_platforms',executable='observed_map_visualizer',output='screen',parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',rviz],output='screen',condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
