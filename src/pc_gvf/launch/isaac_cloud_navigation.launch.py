
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json
import hashlib
import math
from pathlib import Path


def controller_cpu_prefix():
    affinity = os.environ.get("EGO5090_CONTROLLER_CPU_AFFINITY", "")
    if not affinity:
        return []
    tokens = affinity.split(",")
    if any(not token.isdecimal() for token in tokens):
        raise ValueError("controller CPU affinity must be comma-separated CPU IDs")
    requested = {int(token) for token in tokens}
    if len(requested) != len(tokens) or not requested.issubset(os.sched_getaffinity(0)):
        raise ValueError("controller CPU affinity contains duplicate or unavailable IDs")
    return [f"taskset -c {affinity}"]


def generate_launch_description():
    cpu_prefix = controller_cpu_prefix()
    performance_log = os.environ.get("FOV_GVF_PERFORMANCE_LOG", "")
    performance_run_id = os.environ.get("FOV_GVF_RUN_ID", "unspecified")
    manual_default = "6.0" if os.environ.get("ISAAC_MANUAL_INPUT_MODE", "joystick") == "joystick" else "2.0"
    manual_speed = float(os.environ.get("JOYSTICK_HORIZONTAL_SPEED", manual_default))
    if not math.isfinite(manual_speed) or manual_speed <= 0.0:
        raise ValueError("JOYSTICK_HORIZONTAL_SPEED must be finite and positive")
    for name, default in (("ISAAC_PLANT_ACCEL", "16.0"), ("ISAAC_PLANT_TAU", "0.10"),
                          ("FOV_GVF_DIRECTION_RATE", "8.0"), ("FOV_GVF_MAX_DEPTH", "10.0"),
                          ("FOV_GVF_GOAL_CACHE_TTL", "0.15"),
                          ("FOV_GVF_HISTORY_SAMPLE_INTERVAL", "1.0"),
                          ("FOV_GVF_CLEARANCE_REWARD", "0.018"),
                          ("ISAAC_DEPTH_CAMERA_OFFSET_M", "0.22"),
                          ("FOV_GVF_PLANNING_HORIZON", "1.0")):
        value = float(os.environ.get(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    history_max_observations = int(os.environ.get("FOV_GVF_HISTORY_MAX_OBSERVATIONS", "16"))
    if os.environ.get("FOV_GVF_FREE_SPACE_FLOOR_ENABLED", "1") not in ("0", "1"):
        raise ValueError("FOV_GVF_FREE_SPACE_FLOOR_ENABLED must be 0 or 1")
    if not math.isfinite(float(os.environ.get("FOV_GVF_FREE_SPACE_FLOOR_Z", "0.0"))):
        raise ValueError("FOV_GVF_FREE_SPACE_FLOOR_Z must be finite")
    if os.environ.get("FOV_GVF_FREE_SPACE_CEILING_ENABLED", "1") not in ("0", "1"):
        raise ValueError("FOV_GVF_FREE_SPACE_CEILING_ENABLED must be 0 or 1")
    if not math.isfinite(float(os.environ.get("FOV_GVF_FREE_SPACE_CEILING_Z", "20.0"))):
        raise ValueError("FOV_GVF_FREE_SPACE_CEILING_Z must be finite")
    delay = float(os.environ.get("FOV_GVF_PIPELINE_DELAY", ".20"))
    if not math.isfinite(delay) or delay < 0.:
        raise ValueError("FOV_GVF_PIPELINE_DELAY must be finite and nonnegative")
    if history_max_observations <= 0:
        raise ValueError("FOV_GVF_HISTORY_MAX_OBSERVATIONS must be positive")
    if float(os.environ.get("FOV_GVF_GOAL_CACHE_TTL", "0.15")) > 10.0:
        raise ValueError("FOV_GVF_GOAL_CACHE_TTL must not exceed 10 seconds")
    if float(os.environ.get("ISAAC_PLANT_TAU", "0.10")) < .025:
        raise ValueError("ISAAC_PLANT_TAU must be at least the .025 second certificate step")
    if os.environ.get("ISAAC_COMMAND_FRAME", "world") != "world":
        raise ValueError("navigation bridge requires ISAAC_COMMAND_FRAME=world")
    # Frozen binaries allow interleaved comparisons without replacing the
    # installed controller. Normal launches keep the package executable.
    controller_executable = os.environ.get("FOV_GVF_CONTROLLER_EXECUTABLE", "depth_angular_controller")
    if "FOV_GVF_CONTROLLER_EXECUTABLE" in os.environ:
        frozen = Path(controller_executable)
        if not frozen.is_absolute() or not frozen.is_file() or not os.access(frozen, os.X_OK):
            raise ValueError("FOV_GVF_CONTROLLER_EXECUTABLE must be an absolute executable file")
    runtime_manifest = os.environ.get("FOV_GVF_RUNTIME_MANIFEST", "")
    if runtime_manifest:
        actual = (Path(controller_executable) if Path(controller_executable).is_absolute()
                  else Path(get_package_prefix("pc_gvf")) / "lib/pc_gvf" / controller_executable)
        if not actual.is_file() or not os.access(actual, os.X_OK):
            raise ValueError("selected controller must be an executable file")
        Path(runtime_manifest).write_text(json.dumps({
            "executable": str(actual),
            "executable_sha256": hashlib.sha256(actual.read_bytes()).hexdigest(),
            "launch_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "controller_cpu_affinity_requested": os.environ.get("EGO5090_CONTROLLER_CPU_AFFINITY", ""),
        }, indent=2))
    rviz_config = str(
        Path(get_package_share_directory("pc_gvf_platforms")) /
        "config" / "isaac_cloud_navigation.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="pc_gvf", executable=controller_executable, output="screen", prefix=cpu_prefix,
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
                "paper_enabled": True,
                "paper_omni_depth": os.environ.get("FOV_GVF_OMNI_DEPTH", "1") == "1",
                "paper_goal_front_only": True,
                "paper_intent_guard": True,
                "paper_smooth_speed": os.environ.get("FOV_GVF_SMOOTH_SPEED", "0") == "1",
                "paper_model_velocity_shaping": False,
                "paper_proposal_feedforward": False,
                "control_rate": 60.0,
                "paper_depth_processing_rate": 10.0,
                "paper_proposal_jerk_limit": 6.0,
                "paper_proposal_command_accel": 2.4,
                "paper_proposal_response_time": float(os.environ.get("ISAAC_PLANT_TAU", "0.10")),
                "paper_response_preview": True,
                "paper_certified_direct": True,
                "paper_emergency_braking_certificate": os.environ.get("FOV_GVF_EMERGENCY_CERTIFICATE", "1") == "1",
                "paper_execution_consistent": os.environ.get("FOV_GVF_EXECUTION_CONSISTENT", "1") == "1",
                "paper_assisted_vertical": os.environ.get("FOV_GVF_ASSISTED_VERTICAL", "1") == "1",
                "paper_bounded_exploration": os.environ.get("FOV_GVF_BOUNDED_EXPLORATION", "1") == "1",
                "paper_execution_dt": float(os.environ.get("FOV_GVF_EXECUTION_DT", "0.016666666666666666")),
                "paper_tracking_margin": float(os.environ.get("FOV_GVF_TRACKING_MARGIN", ".015")),
                "paper_assist_speed": float(os.environ.get("FOV_GVF_ASSIST_SPEED", ".4")),
                "paper_assist_radius": float(os.environ.get("FOV_GVF_ASSIST_RADIUS", ".8")),
                "paper_probe_speed": float(os.environ.get("FOV_GVF_PROBE_SPEED", ".3")),
                "paper_probe_vertical_speed": float(os.environ.get("FOV_GVF_PROBE_VERTICAL_SPEED", ".2")),
                "paper_probe_distance_budget": float(os.environ.get("FOV_GVF_PROBE_DISTANCE_BUDGET", "3.5")),
                "paper_probe_time_budget": float(os.environ.get("FOV_GVF_PROBE_TIME_BUDGET", "12")),
                "paper_probe_volume_budget": float(os.environ.get("FOV_GVF_PROBE_VOLUME_BUDGET", "5")),
                "paper_probe_anchor_range": float(os.environ.get("FOV_GVF_PROBE_ANCHOR_RANGE", "3")),
                "paper_probe_clearance": float(os.environ.get("FOV_GVF_PROBE_CLEARANCE", ".08")),
                "paper_goal_proposal_cache": os.environ.get("FOV_GVF_GOAL_CACHE", "0") == "1",
                "paper_goal_proposal_cache_ttl": float(os.environ.get("FOV_GVF_GOAL_CACHE_TTL", "0.15")),
                "paper_goal_proposal_cache_max_displacement": float(os.environ.get("FOV_GVF_GOAL_CACHE_DISPLACEMENT", ".75")),
                "paper_goal_speed_governor": os.environ.get("FOV_GVF_GOAL_SPEED_GOVERNOR", "0") == "1",
                "paper_depth_proposal": True,
                "paper_angular_goal_cost": False,
                "paper_vehicle_origin_proposal": True,
                "paper_subpixel_proposal": True,
                "paper_local_path_proposal": False,
                "paper_retain_certified_volume": True,
                "paper_shared_obstacles": os.environ.get("FOV_GVF_SHARED_OBSTACLES", "0") == "1",
                "shared_robot_id": os.environ.get("FOV_GVF_ROBOT_ID", "ego2p6"),
                "shared_map_frame": os.environ.get("FOV_GVF_SHARED_FRAME", "world"),
                "operator_assistance": os.environ.get("FOV_GVF_OPERATOR_ASSISTANCE", "0") == "1",
                "paper_spherical_memory": os.environ.get("FOV_GVF_SPHERICAL_MEMORY", "0") == "1",
                "paper_incremental_frontend": os.environ.get("FOV_GVF_INCREMENTAL_FRONTEND", "1") == "1",
                "paper_incremental_field": os.environ.get("FOV_GVF_INCREMENTAL_FIELD", "0") == "1",
                "paper_dynamic_obstacles": os.environ.get("FOV_GVF_DYNAMIC_OBSTACLES", "0") == "1",
                "depth_uncertainty_enabled": os.environ.get("FOV_GVF_DEPTH_UNCERTAINTY", "1") == "1",
                "depth_uncertainty_sigma0": float(os.environ.get("FOV_GVF_DEPTH_SIGMA0", "0.005")),
                "depth_uncertainty_sigma_range2": float(os.environ.get("FOV_GVF_DEPTH_SIGMA_RANGE2", "0.0")),
                "depth_uncertainty_multiplier": 3.0,
                "depth_uncertainty_pose_bound": 0.01,
                "depth_uncertainty_edge_radius": 1,
                "paper_local_history_repair": os.environ.get("FOV_GVF_LOCAL_HISTORY_REPAIR", "1") == "1",
                "paper_goal_projection_clearance": 0.0,
                "paper_intent_corridor": False,
                "paper_native_proposal_stride": 0,
                "angular_width": int(os.environ.get("FOV_GVF_ANGULAR_WIDTH", "64")), "angular_height": int(os.environ.get("FOV_GVF_ANGULAR_HEIGHT", "48")),
                "paper_chart_width": int(os.environ.get("FOV_GVF_CHART_WIDTH", "24")), "paper_chart_height": int(os.environ.get("FOV_GVF_CHART_HEIGHT", "18")),
                "paper_depth_contract": "isaac_rendered_z",
                "paper_observation_history": 60.0,
                "paper_static_verified_travel": True,
                "paper_history_sample_interval": float(os.environ.get("FOV_GVF_HISTORY_SAMPLE_INTERVAL", "1.0")),
                "paper_history_max_observations": history_max_observations,
                "paper_history_uncertainty_rate": 0.0,
                "paper_static_scene": True,
                "paper_observation_uncertainty_rate": 0.1,
                "paper_seed_scene": os.environ.get("FOV_GVF_ACTUAL_SCENE", "") if os.environ.get("FOV_GVF_VERIFIED_START", "1") == "1" else "",
                "paper_seed_occupancy": os.environ.get("FOV_GVF_ESDF_OCCUPANCY", ""),
                "paper_seed_radius": 2.5,
                "paper_field_interval": 0.10,
                "paper_command_change_angle": 0.05,  # radians, relative to active-reference intent
                "paper_pipeline_delay": float(os.environ.get("FOV_GVF_PIPELINE_DELAY", "0.20")),
                "paper_restart_clearance": float(os.environ.get("FOV_GVF_RESTART_CLEARANCE", "0.02")),
                "paper_free_space_floor_enabled": os.environ.get("FOV_GVF_FREE_SPACE_FLOOR_ENABLED", "1") == "1",
                "paper_free_space_floor_z": float(os.environ.get("FOV_GVF_FREE_SPACE_FLOOR_Z", "0.0")),
                "paper_free_space_ceiling_enabled": os.environ.get("FOV_GVF_FREE_SPACE_CEILING_ENABLED", "1") == "1",
                "paper_free_space_ceiling_z": float(os.environ.get("FOV_GVF_FREE_SPACE_CEILING_Z", "19.8")),
                "paper_working_bounds_enabled": os.environ.get("FOV_GVF_WORKING_BOUNDS_ENABLED", "1") == "1",
                "paper_working_min_x": float(os.environ.get("FOV_GVF_WORKING_MIN_X", "-80.0")),
                "paper_working_max_x": float(os.environ.get("FOV_GVF_WORKING_MAX_X", "399.8")),
                "paper_working_min_y": float(os.environ.get("FOV_GVF_WORKING_MIN_Y", "-60.0")),
                "paper_working_max_y": float(os.environ.get("FOV_GVF_WORKING_MAX_Y", "59.8")),
                "paper_rollout_horizon": 0.20,
                "paper_transverse_gain": 1.0,
                "paper_chart_speed": 16.0,
                "paper_minimum_lookahead": float(os.environ.get("FOV_GVF_MIN_LOOKAHEAD", "0.10")),
                "paper_feedback": True,
                "paper_continuation": True,
                "paper_coarse_fine": True,
                "paper_continuous_certificates": os.environ.get("FOV_GVF_CONTINUOUS_CERTIFICATES", "1") == "1",
                "paper_adaptive_lookahead": os.environ.get("FOV_GVF_ADAPTIVE_LOOKAHEAD", "1") == "1",
                "paper_adaptive_grid": True,
                "paper_certificate_resolution": 0.005,
                "paper_replay_directory": os.environ.get("FOV_GVF_REPLAY_DIR", ""),
                "camera_offset": [float(os.environ.get("ISAAC_DEPTH_CAMERA_OFFSET_M", "0.22")), 0.0, 0.02],
                "cmd_topic": "/position_cmd",
                "human_intent_topic": "/human_intent",
                "human_intent_timeout": 0.25,
                "fixed_forward_intent": False,
                "use_fixed_goal": False,
                "stop_at_goal": False,
                "use_reference_line": False,
                "max_depth_age": 0.3,
                "max_depth": float(os.environ.get("FOV_GVF_MAX_DEPTH", "10.0")),
                "obstacle_clear_frames": 3,
                "speed": manual_speed, "max_speed": manual_speed,
                "max_vertical_speed": 1.0,
                "command_accel_limit": float(os.environ.get("ISAAC_PLANT_ACCEL", "16.0")),
                "brake_accel_limit": float(os.environ.get("ISAAC_PLANT_ACCEL", "16.0")),
                "velocity_response_time": float(os.environ.get("ISAAC_PLANT_TAU", "0.10")),
                "max_direction_rate": float(os.environ.get("FOV_GVF_DIRECTION_RATE", "8.0")),
                "continuous_harmonic_guidance": True,
                "harmonic_gradient_radius_cells": 3,
                "harmonic_gradient_lookahead_cells": 2.5,
                "field_publish_rate": 15.0,
                "safe_goal_hysteresis_weight": 0.40,
                "safe_goal_clearance_reward": float(os.environ.get("FOV_GVF_CLEARANCE_REWARD", "0.018")),
                "safe_goal_clearance_cap": 0.0,
                "safe_goal_left_bias": 0.012,
                "safe_goal_hold_time": 0.35,
                "body_radius": float(os.environ.get("FOV_GVF_BODY_RADIUS", "0.48")),
                "safety_margin": float(os.environ.get("FOV_GVF_SAFETY_MARGIN", "0.02")),
                "rollout_margin": float(os.environ.get("FOV_GVF_ROLLOUT_MARGIN", "0.02")),
                "planning_horizon": float(os.environ.get("FOV_GVF_PLANNING_HORIZON", "1.0")),
                "performance_report_interval": 5.0,
                "performance_log_path": performance_log,
                "performance_run_id": performance_run_id,
            }],
        ),
        Node(
            package="pc_gvf_platforms", executable="position_cmd_to_twist", output="screen",
            parameters=[{
                "use_sim_time": True,
                "mode": "world", "odom_topic": "/sim/odom",
                "cmd_in_topic": "/position_cmd", "cmd_out_topic": "/sim/cmd_vel",
                "max_vx": manual_speed, "max_vy": manual_speed, "max_vz": 1.0, "max_w": 0.0,
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
