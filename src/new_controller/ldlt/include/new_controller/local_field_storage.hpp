#pragma once
#ifdef EGO4P2_STEP2
#include "new_controller/navigation_config.hpp"
#endif
#include "new_controller/depth_observation_storage.hpp"
#include "new_controller/base_config.hpp"

namespace ego4p0 {
struct ControllerLimits {
#ifdef EGO4P2_STEP2
  // The tube already includes body + tracking error. Never add body again.
  static constexpr double radius_m=NavigationConfig::occupancy_tube_radius_m;
#else
  static constexpr double radius_m=BaseConfig::body_radius_m;
#endif
  static constexpr double local_range_m=BaseConfig::local_range_m,period_s=BaseConfig::control_period_s;
  // This is the legacy REFERENCE COMMAND limiter, not plant capability.
  static constexpr double max_speed_mps=BaseConfig::max_speed_mps;
  static constexpr double max_acceleration_mps2=BaseConfig::reference_command_acceleration_mps2;
};
// A camera-image ray bundle, NOT a body-origin radial free distance or a whole
// angular-cell clearance. Query the projected body envelope before using it.
struct FreeRayEvidence {
  double z_begin=0.,z_end=0.;
  uint32_t raw_valid_pixels=0,total_pixels=0;
  bool begin_known=false,complete_raw=false;
};
enum class CandidateStatus : uint8_t { Unknown,Unavailable,Available };
struct RangeTile {double minimum=std::numeric_limits<double>::quiet_NaN();uint32_t valid=0;};
struct LocalFieldData {
  bool virtual_center=false;
  std::vector<double> min_range;
  std::vector<uint8_t> observation_cameras,complete_cameras,node_state;
  std::vector<uint32_t> missing_prefix;
  std::vector<RangeTile> range_tiles;
  uint32_t available_cells=0,unknown_cells=0;
  std::vector<uint8_t> excluded_camera_mask;
  std::array<uint32_t,4> near_blocks{},complete_far_blocks{},unknown_blocks{};
  std::array<int64_t,4> shared_projection_ns{};
  uint32_t excluded_blocks=0,excluded_cells=0;
  int64_t stamp_ns=0,merge_ns=0;
  uint64_t sequence=0,configuration_id=0,geometry_id=0;
  bool ready=false,panorama_pending=false;
  void allocate(const DirectionGridLayout &grid,bool virtual_model=false) {
    virtual_center=virtual_model;
    if(virtual_model) {
      min_range.assign(grid.cells(),std::numeric_limits<double>::quiet_NaN());
      observation_cameras.assign(grid.cells(),0);complete_cameras.assign(grid.cells(),0);node_state.assign(grid.cells(),0);
      missing_prefix.assign(size_t(grid.azimuth_bins+1)*(grid.elevation_bins+1),0);
      range_tiles.resize(((grid.azimuth_bins+7)/8)*((grid.elevation_bins+7)/8));
    }
    excluded_camera_mask.assign(grid.cells(),0);excluded_cells=excluded_blocks=0;ready=false;panorama_pending=false;
  }
};
} // namespace ego4p0
