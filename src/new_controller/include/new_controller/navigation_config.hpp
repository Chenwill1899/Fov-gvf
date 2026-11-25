#pragma once
#include "new_controller/base_config.hpp"
namespace ego4p0 {
struct NavigationConfig {
  static constexpr double reference_turn_acceleration_mps2=80.0;
  static constexpr double reference_max_curvature_per_m=0.5555555555555556;
  static constexpr double reference_min_radius_m=1.8;
  static constexpr double tracking_error_target_m=0.2;
  static constexpr double occupancy_tube_radius_m=0.30000000000000004;
  static constexpr double normal_attach_position_error_m=0.1;
  static constexpr double normal_attach_angle_deg=5.0;
  static constexpr double lateral_gain_per_s=2.0;
  static constexpr double heading_gain_per_s=20.0;
  static constexpr double heading_rate_limit_rad_s=10.0;
  static constexpr double heading_acceleration_limit_rad_s2=100.0;
  static constexpr double compensation_input_limit_mps=17.0;
  static constexpr double compensation_slew_limit_mps2=450.0;
  static constexpr double max_observation_age_sim_s=0.01;
  static constexpr bool motion_output_enabled=false;
  static constexpr uint64_t version=0xeec5374bbd6c08abULL;
  static constexpr const char *sha256="eec5374bbd6c08ab97b4da08faf736277f2d24ac0bde124339b7618a1d26067e";
  static constexpr const char *json=R"nav({"base":{"profile":"ego4p2_base_step1","max_speed_mps":12.0,"velocity_limit_mode":"norm_3d","vertical_speed_limit_mps":null,"local_range_m":4.0,"control_period_s":0.005,"physics_dt_s":0.005,"camera_hz":200,"time_scale":0.02,"plant_tau_s":0.1,"body_radius_m":0.1,"plant_acceleration_mps2":150.0,"reference_command_acceleration_mps2":20.0,"motion_output_enabled":false},"navigation":{"reference_turn_acceleration_mps2":80.0,"reference_max_curvature_per_m":0.5555555555555556,"reference_min_radius_m":1.8,"tracking_error_target_m":0.2,"occupancy_tube_radius_m":0.30000000000000004,"normal_attach_position_error_m":0.1,"normal_attach_angle_deg":5.0,"lateral_gain_per_s":2.0,"heading_gain_per_s":20.0,"heading_rate_limit_rad_s":10.0,"heading_acceleration_limit_rad_s2":100.0,"compensation_input_limit_mps":17.0,"compensation_slew_limit_mps2":450.0,"max_observation_age_sim_s":0.01,"motion_output_enabled":false}})nav";
};
static_assert(!NavigationConfig::motion_output_enabled);
} // namespace ego4p0
