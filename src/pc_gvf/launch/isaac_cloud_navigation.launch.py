
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path


def generate_launch_description():
    performance_log = os.environ.get("FOV_GVF_PERFORMANCE_LOG", "")
    performance_run_id = os.environ.get("FOV_GVF_RUN_ID", "unspecified")
    rviz_config = str(
        Path(get_package_share_directory("pc_gvf_platforms")) /
        "config" / "isaac_cloud_navigation.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="pc_gvf", executable="depth_angular_controller", output="screen",
            parameters=[{
                "use_sim_time": True,
                "frame_id": "world",
                "odom_topic": "/sim/odom",
                "depth_topic": "/sim/depth/image_raw",
                "camera_info_topic": "/sim/depth/camera_info",
                "left_depth_topic": "/sim/depth_left/image_raw",
                "left_camera_info_topic": "/sim/depth_left/camera_info",
                "back_depth_topic": "/sim/depth_back/image_raw",
                "back_camera_info_topic": "/sim/depth_back/camera_info",
                "right_depth_topic": "/sim/depth_right/image_raw",
                "right_camera_info_topic": "/sim/depth_right/camera_info",
                "horizontal_360_enabled": True,
                "cmd_topic": "/position_cmd",
                "human_intent_topic": "/human_intent",
                "human_intent_timeout": 0.25,
                "fixed_forward_intent": False,
                "use_fixed_goal": False,
                "stop_at_goal": False,
                "use_reference_line": False,
                "max_depth_age": 0.5,
                "max_depth": 40.0,
                "obstacle_clear_frames": 3,
                "speed": 2.0, "max_speed": 2.0,
                "max_vertical_speed": 1.0,
                "command_accel_limit": 1.2,
                "command_jerk_limit": 8.0,
                "command_response_time": 0.22,
                "speed_recovery_accel": 1.0,
                "speed_brake_accel": 2.5,
                "max_direction_rate": 1.60,
                "continuous_harmonic_guidance": True,
                "harmonic_gradient_radius_cells": 3,
                "harmonic_gradient_lookahead_cells": 2.5,
                "field_publish_rate": 15.0,
                "safe_goal_hysteresis_weight": 0.40,
                "safe_goal_hold_time": 0.35,
                "body_radius": 0.25,
                "safety_margin": 0.20,
                "rollout_margin": 0.10,
                "planning_horizon": 3.0,
                "performance_report_interval": 5.0,
                "performance_log_path": performance_log,
                "performance_run_id": performance_run_id,
            }],
        ),
        Node(
            package="pc_gvf_platforms", executable="position_cmd_to_twist", output="screen",
            parameters=[{
                "use_sim_time": True,
                "mode": "holonomic", "odom_topic": "/sim/odom",
                "cmd_in_topic": "/position_cmd", "cmd_out_topic": "/sim/cmd_vel",
                "max_vx": 2.0, "max_vy": 2.0, "max_vz": 1.0, "max_w": 0.0,
                "performance_report_interval": 5.0,
                "performance_log_path": performance_log,
                "performance_run_id": performance_run_id,
            }],
        ),
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
                "camera_offset": [0.22, 0.0, 0.02],
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
