from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    rviz_config = str(Path(get_package_share_directory("pc_gvf_platforms")) / "config" / "depth_demo.rviz")
    controller_parameters = {
        "odom_topic": "/sim/odom",
        "depth_topic": "/sim/depth/image_raw",
        "camera_info_topic": "/sim/depth/camera_info",
        "human_intent_topic": "/human_intent",
        "max_depth_age": 0.3,
        "max_vertical_speed": 0.0,
        "max_speed": 1.0,
        "use_fixed_goal": True,
        "goal_x": 7.2,
        "goal_y": 0.0,
        "goal_z": 1.2,
        "use_reference_line": False,
        "stop_at_goal": True,
        "goal_tolerance": 0.3,
    }
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="single_pillar"),
        Node(
            package="pc_gvf",
            executable="depth_angular_demo_sim",
            output="screen",
            parameters=[{"scenario": LaunchConfiguration("scenario")}],
        ),
        Node(
            package="pc_gvf",
            executable="depth_angular_controller",
            output="screen",
            parameters=[{
                **controller_parameters,
                "cmd_topic": "/position_cmd",
            }],
        ),
        Node(
            package="pc_gvf_platforms",
            executable="navigation_visualizer",
            output="screen",
            parameters=[{"robot_shape": "quadrotor"}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
