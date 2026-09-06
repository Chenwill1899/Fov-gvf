#include "fluid/depth_grid_observation.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

using FLAG_Race::fluid3d::CellState;
using FLAG_Race::fluid3d::DepthFrame3D;
using FLAG_Race::fluid3d::Grid3D;
using FLAG_Race::fluid3d::GridObservation3D;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "depth_grid_observation_check: " << message << std::endl;
        std::exit(1);
    }
}

DepthFrame3D planeFrame() {
    DepthFrame3D frame;
    frame.width = frame.height = 41;
    frame.fx = frame.fy = 10.0;
    frame.cx = frame.cy = 20.0;
    frame.min_depth = 0.1;
    frame.max_depth = 10.0;
    frame.depth_m.assign(static_cast<size_t>(frame.width) * frame.height, 3.0f);
    frame.stamp_sec = 1.0;
    return frame;
}

Grid3D verticalGrid() {
    Grid3D grid;
    grid.origin_xy = Eigen::Vector2d(-0.1, -0.1);
    grid.h = grid.h_z = 0.2;
    grid.n_fwd = grid.n_lat = 1;
    grid.n_z = 20;
    return grid;
}

Grid3D oneCell(const Eigen::Vector2d& xy, double z, double h = 0.1) {
    Grid3D grid;
    grid.origin_xy = xy - Eigen::Vector2d::Constant(0.5 * h);
    grid.z_lo = z - 0.5 * h;
    grid.h = grid.h_z = h;
    grid.n_fwd = grid.n_lat = grid.n_z = 1;
    return grid;
}

Grid3D representativeGrid() {
    Grid3D grid;
    grid.origin_xy = Eigen::Vector2d(-1.5, -1.5);
    grid.z_lo = 2.0;
    grid.h = grid.h_z = 0.1;
    grid.n_fwd = grid.n_lat = 30;
    grid.n_z = 16;
    return grid;
}

}  // namespace

int main() {
    DepthFrame3D frame = planeFrame();
    const std::vector<float> original_depth = frame.depth_m;
    const Grid3D grid = verticalGrid();
    GridObservation3D observation;
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(frame, grid, 0.0, 0.0, 0.01, &observation),
            "constant plane must classify");
    require(observation.state[grid.idx(0, 0, 4)] == CellState::Free,
            "cell in front of plane must be Free");
    require(observation.state[grid.idx(0, 0, 14)] == CellState::Occupied,
            "surface band cell must be Occupied");
    require(observation.state[grid.idx(0, 0, 17)] == CellState::Unknown,
            "cell behind first surface must be Unknown");
    require(observation.solid[grid.idx(0, 0, 4)] == 0 &&
            observation.solid[grid.idx(0, 0, 14)] == 1 &&
            observation.solid[grid.idx(0, 0, 17)] == 1 && observation.has_free_boundary,
            "solid compression and free boundary must match cell states");
    require(frame.depth_m == original_depth, "projection must not modify the input depth frame");
    require(FLAG_Race::fluid3d::isDepthFrameFresh(frame, 1.2, 0.2) &&
            !FLAG_Race::fluid3d::isDepthFrameFresh(frame, 1.201, 0.2) &&
            !FLAG_Race::fluid3d::isDepthFrameFresh(frame, 0.9, 0.2) &&
            !FLAG_Race::fluid3d::isDepthFrameFresh(frame, 1.1, 0.0),
            "freshness must use one frame stamp and reject stale, future, or invalid-age use");

    const Grid3D near_surface = oneCell(Eigen::Vector2d::Zero(), 2.8);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(frame, near_surface, 0.0, 0.0, 0.01, &observation) &&
            observation.state[0] == CellState::Free,
            "small robot should remain Free before the plane");
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(frame, near_surface, 0.2, 0.0, 0.01, &observation) &&
            observation.state[0] == CellState::Occupied,
            "larger robot radius must expand the occupied surface band");

    DepthFrame3D invalid_pixel = frame;
    invalid_pixel.depth_m[20 * invalid_pixel.width + 20] = std::numeric_limits<float>::quiet_NaN();
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(invalid_pixel, near_surface, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Unknown,
            "NaN in a projected bbox must be Unknown");
    DepthFrame3D range_miss = frame;
    range_miss.depth_m.assign(range_miss.depth_m.size(), 0.0f);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(range_miss, near_surface, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Free,
            "zero range misses must certify free space up to max_depth");
    const Grid3D beyond_range = oneCell(Eigen::Vector2d::Zero(), 10.0);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(range_miss, beyond_range, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Unknown,
            "zero range misses must not certify cells extending beyond max_depth");
    const Grid3D out_of_fov = oneCell(Eigen::Vector2d(20.0, 0.0), 1.0);
    const Grid3D behind_camera = oneCell(Eigen::Vector2d::Zero(), -1.0);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(frame, out_of_fov, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Unknown,
            "cell outside the image must be Unknown");
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(frame, behind_camera, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Unknown,
            "cell behind the camera must be Unknown");
    DepthFrame3D bad_size = frame;
    bad_size.depth_m.pop_back();
    require(!FLAG_Race::fluid3d::projectDepthFrameToGrid(bad_size, near_surface, 0.0, 0.0, 0.01,
                                                         &observation),
            "bad depth dimensions must fail the frame contract");
    require(observation.state.empty() && observation.solid.empty() && !observation.has_free_boundary,
            "failed projection must clear a reused observation");
    DepthFrame3D bad_principal_point = frame;
    bad_principal_point.cx = bad_principal_point.width;
    require(!FLAG_Race::fluid3d::validDepthFrame3D(bad_principal_point),
            "principal point outside the image must fail the frame contract");

    DepthFrame3D yawed = planeFrame();
    yawed.depth_m.assign(yawed.depth_m.size(), std::numeric_limits<float>::quiet_NaN());
    for (int v = 16; v <= 18; ++v)
        for (int u = 19; u <= 21; ++u)
            yawed.depth_m[static_cast<size_t>(v) * yawed.width + u] = 3.0f;
    yawed.T_world_camera.linear() =
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Grid3D yaw_target = oneCell(Eigen::Vector2d(1.0, 0.0), 3.0);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(yawed, yaw_target, 0.0, 0.0, 0.01,
                                                        &observation) &&
            observation.state[0] == CellState::Occupied,
            "camera yaw must rotate world cells into the correct image region");
    DepthFrame3D translated = planeFrame();
    translated.T_world_camera.translation().z() = 1.0;
    const Grid3D translation_target = oneCell(Eigen::Vector2d::Zero(), 4.0);
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(translated, translation_target, 0.0, 0.0,
                                                        0.01, &observation) &&
            observation.state[0] == CellState::Occupied,
            "camera translation must be applied world-to-camera");

    DepthFrame3D interior_only = planeFrame();
    interior_only.width = interior_only.height = 101;
    interior_only.fx = interior_only.fy = 100.0;
    interior_only.cx = interior_only.cy = 50.0;
    interior_only.depth_m.assign(static_cast<size_t>(interior_only.width) * interior_only.height,
                                  std::numeric_limits<float>::quiet_NaN());
    for (int v = 47; v <= 53; ++v)
        for (int u = 47; u <= 53; ++u)
            interior_only.depth_m[static_cast<size_t>(v) * interior_only.width + u] = 3.1f;
    Grid3D multidim;
    multidim.origin_xy = Eigen::Vector2d(-0.15, -0.15);
    multidim.z_lo = 2.85;
    multidim.h = multidim.h_z = 0.1;
    multidim.n_fwd = multidim.n_lat = multidim.n_z = 3;
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(interior_only, multidim, 0.0, 0.0, 0.01,
                                                        &observation),
            "multidimensional grid must classify");
    require(observation.state.size() == static_cast<size_t>(multidim.size()) &&
            observation.solid.size() == observation.state.size() &&
            observation.state[multidim.idx(1, 1, 1)] == CellState::Free &&
            observation.solid[multidim.idx(1, 1, 1)] == 0,
            "multidimensional output must preserve Grid3D indexing");
    for (int i = 0; i < multidim.n_fwd; ++i)
        for (int j = 0; j < multidim.n_lat; ++j)
            for (int m = 0; m < multidim.n_z; ++m)
                if (i == 0 || j == 0 || m == 0 || i == multidim.n_fwd - 1 ||
                    j == multidim.n_lat - 1 || m == multidim.n_z - 1)
                    require(observation.state[multidim.idx(i, j, m)] != CellState::Free &&
                            observation.solid[multidim.idx(i, j, m)] == 1,
                            "all multidimensional boundary cells must be non-free solids");
    require(!observation.has_free_boundary,
            "an internal Free cell must not create a free boundary flag");

    DepthFrame3D representative = planeFrame();
    representative.width = 160;
    representative.height = 120;
    representative.fx = representative.fy = 80.0;
    representative.cx = 79.5;
    representative.cy = 59.5;
    representative.depth_m.assign(static_cast<size_t>(representative.width) * representative.height, 5.0f);
    const Grid3D fine_grid = representativeGrid();
    const auto projection_started = std::chrono::steady_clock::now();
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(representative, fine_grid, 0.0, 0.0, 0.01,
                                                        &observation),
            "representative fine grid must classify");
    const double projection_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - projection_started).count();
    size_t known_state_count = 0;
    for (CellState state : observation.state)
        known_state_count += state == CellState::Unknown || state == CellState::Free ||
                             state == CellState::Occupied;
    require(observation.state.size() == static_cast<size_t>(fine_grid.size()) &&
            observation.solid.size() == observation.state.size() &&
            known_state_count == observation.state.size(),
            "representative projection must produce one valid state per cell");
    std::cout << "depth_grid_observation_check: 160x120, 30x30x16 projection "
              << projection_ms << " ms" << std::endl;

    std::cout << "depth_grid_observation_check: passed" << std::endl;
    return 0;
}
