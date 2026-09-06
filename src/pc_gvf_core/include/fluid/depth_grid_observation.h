#ifndef FLUID_DEPTH_GRID_OBSERVATION_H
#define FLUID_DEPTH_GRID_OBSERVATION_H

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "fluid/fluid_solver_3d.h"

namespace FLAG_Race {
namespace fluid3d {

enum class CellState : uint8_t { Unknown, Free, Occupied, TrustedFree };

// Depth is axial optical-Z in metres. Zero means no return within max_depth;
// non-finite or negative samples remain invalid/unknown. T_world_camera maps
// optical camera coordinates into world coordinates (camera -> world).
struct DepthFrame3D {
    int width = 0, height = 0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double min_depth = 0.0, max_depth = 0.0;
    std::vector<float> depth_m;
    Eigen::Isometry3d T_world_camera = Eigen::Isometry3d::Identity();
    double stamp_sec = 0.0;
};

struct GridObservation3D {
    std::vector<CellState> state;
    std::vector<uint8_t> solid;  // Free=0; Occupied/Unknown=1.
    bool has_free_boundary = false;
};

bool validDepthFrame3D(const DepthFrame3D& frame);

// Uses the frame's own stamp; callers keep ownership of one immutable frame
// and may use this at command publication without swapping to a newer frame.
bool isDepthFrameFresh(const DepthFrame3D& frame, double now_sec, double max_age_sec);

// Classifies configuration-space grid cells directly in the depth image.
// No points, voxel map, or distance transform are constructed.
bool projectDepthFrameToGrid(const DepthFrame3D& frame, const Grid3D& grid,
                             double robot_radius, double geometry_margin,
                             double surface_band, GridObservation3D* observation);

}  // namespace fluid3d
}  // namespace FLAG_Race

#endif
