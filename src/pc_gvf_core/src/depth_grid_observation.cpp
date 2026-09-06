#include "fluid/depth_grid_observation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace FLAG_Race {
namespace fluid3d {
namespace {

bool validGrid(const Grid3D& grid) {
    return grid.n_fwd > 0 && grid.n_lat > 0 && grid.n_z > 0 &&
           std::isfinite(grid.h) && std::isfinite(grid.h_z) &&
           grid.h > 0.0 && grid.h_z > 0.0 && std::isfinite(grid.z_lo) &&
           grid.origin_xy.allFinite() && grid.e_J.allFinite() && grid.n_J.allFinite();
}

bool isBoundary(const Grid3D& grid, int i, int j, int m) {
    return i == 0 || j == 0 || m == 0 || i == grid.n_fwd - 1 ||
           j == grid.n_lat - 1 || m == grid.n_z - 1;
}

}  // namespace

bool validDepthFrame3D(const DepthFrame3D& frame) {
    if (frame.width <= 0 || frame.height <= 0) return false;
    const size_t pixel_count = static_cast<size_t>(frame.width) * frame.height;
    return std::isfinite(frame.fx) && std::isfinite(frame.fy) &&
           std::isfinite(frame.cx) && std::isfinite(frame.cy) &&
           std::isfinite(frame.min_depth) && std::isfinite(frame.max_depth) &&
           frame.fx > 0.0 && frame.fy > 0.0 &&
           frame.cx >= 0.0 && frame.cx <= frame.width - 1 &&
           frame.cy >= 0.0 && frame.cy <= frame.height - 1 && frame.min_depth > 0.0 &&
           frame.max_depth > frame.min_depth && frame.depth_m.size() == pixel_count &&
           frame.T_world_camera.matrix().allFinite() && std::isfinite(frame.stamp_sec);
}

bool isDepthFrameFresh(const DepthFrame3D& frame, double now_sec, double max_age_sec) {
    return validDepthFrame3D(frame) && std::isfinite(now_sec) &&
           std::isfinite(max_age_sec) && max_age_sec > 0.0 &&
           now_sec >= frame.stamp_sec && now_sec - frame.stamp_sec <= max_age_sec;
}

bool projectDepthFrameToGrid(const DepthFrame3D& frame, const Grid3D& grid,
                             double robot_radius, double geometry_margin,
                             double surface_band, GridObservation3D* observation) {
    if (observation == nullptr) return false;
    *observation = GridObservation3D{};
    if (!validDepthFrame3D(frame) || !validGrid(grid) ||
        !std::isfinite(robot_radius) || !std::isfinite(geometry_margin) ||
        !std::isfinite(surface_band) || robot_radius < 0.0 ||
        geometry_margin < 0.0 || surface_band < 0.0) {
        return false;
    }

    const int grid_size = grid.size();
    observation->state.assign(grid_size, CellState::Unknown);
    const double radius = robot_radius + geometry_margin +
        0.5 * std::sqrt(2.0 * grid.h * grid.h + grid.h_z * grid.h_z);
    const Eigen::Matrix3d R_camera_world = frame.T_world_camera.linear().transpose();
    const Eigen::Vector3d camera_position = frame.T_world_camera.translation();

    for (int i = 0; i < grid.n_fwd; ++i) {
        for (int j = 0; j < grid.n_lat; ++j) {
            for (int m = 0; m < grid.n_z; ++m) {
                const int k = grid.idx(i, j, m);
                const Eigen::Vector3d center_camera = R_camera_world *
                    (grid.cellWorld(i, j, m) - camera_position);
                if (!center_camera.allFinite() || center_camera.z() - radius <= 0.0) {
                    continue;
                }

                double min_u = std::numeric_limits<double>::infinity();
                double max_u = -min_u;
                double min_v = min_u;
                double max_v = -min_u;
                for (int sx : {-1, 1}) {
                    for (int sy : {-1, 1}) {
                        for (int sz : {-1, 1}) {
                            const double z = center_camera.z() + sz * radius;
                            const double u = frame.fx * (center_camera.x() + sx * radius) / z + frame.cx;
                            const double v = frame.fy * (center_camera.y() + sy * radius) / z + frame.cy;
                            min_u = std::min(min_u, u);
                            max_u = std::max(max_u, u);
                            min_v = std::min(min_v, v);
                            max_v = std::max(max_v, v);
                        }
                    }
                }
                if (!std::isfinite(min_u) || !std::isfinite(max_u) ||
                    !std::isfinite(min_v) || !std::isfinite(max_v) ||
                    min_u < 0.0 || min_v < 0.0 ||
                    max_u > frame.width - 1 || max_v > frame.height - 1) {
                    continue;
                }
                const int u_lo = static_cast<int>(std::floor(min_u));
                const int u_hi = static_cast<int>(std::ceil(max_u));
                const int v_lo = static_cast<int>(std::floor(min_v));
                const int v_hi = static_cast<int>(std::ceil(max_v));

                // ponytail: direct bbox scan first; add a min-depth pyramid only
                // if profiling shows this bounded low-resolution image work is too slow.
                double d_min = std::numeric_limits<double>::infinity();
                bool valid_bbox = true;
                for (int v = v_lo; v <= v_hi && valid_bbox; ++v) {
                    for (int u = u_lo; u <= u_hi; ++u) {
                        const float depth = frame.depth_m[static_cast<size_t>(v) * frame.width + u];
                        if (depth == 0.0f) continue;
                        if (!std::isfinite(depth) || depth < frame.min_depth || depth > frame.max_depth) {
                            valid_bbox = false;
                            break;
                        }
                        d_min = std::min(d_min, static_cast<double>(depth));
                    }
                }
                if (!valid_bbox) continue;

                const double z_near = center_camera.z() - radius;
                const double z_far = center_camera.z() + radius;
                if (!std::isfinite(d_min)) {
                    if (z_far <= frame.max_depth) observation->state[k] = CellState::Free;
                } else if (d_min > z_far + surface_band) {
                    observation->state[k] = CellState::Free;
                } else if (d_min >= z_near - surface_band && d_min <= z_far + surface_band) {
                    observation->state[k] = CellState::Occupied;
                }
            }
        }
    }
    observation->solid.resize(grid_size);
    observation->has_free_boundary = false;
    for (int i = 0; i < grid.n_fwd; ++i) {
        for (int j = 0; j < grid.n_lat; ++j) {
            for (int m = 0; m < grid.n_z; ++m) {
                const int k = grid.idx(i, j, m);
                const bool free = observation->state[k] == CellState::Free;
                observation->solid[k] = free ? 0 : 1;
                observation->has_free_boundary = observation->has_free_boundary ||
                    (free && isBoundary(grid, i, j, m));
            }
        }
    }
    return true;
}

}  // namespace fluid3d
}  // namespace FLAG_Race
