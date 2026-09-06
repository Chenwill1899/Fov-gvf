from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    rviz_config = str(Path(get_package_share_directory("pc_gvf_platforms")) / "config" / "ground_demo.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="pc_gvf_platforms", executable="position_cmd_to_twist", output="screen",
             parameters=[{"mode": "diff_drive"}]),
        Node(package="pc_gvf_platforms", executable="kinematic_sim", output="screen",
             parameters=[{"mode": "diff_drive"}]),
        Node(package="pc_gvf_platforms", executable="navigation_visualizer", output="screen",
             parameters=[{"robot_shape": "box"}]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_config], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
