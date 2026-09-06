#include "fluid/fluid_guidance.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace {

using FLAG_Race::fluid::FluidGuidance;
using FLAG_Race::fluid::FluidGuidanceConfig;
using FLAG_Race::fluid3d::CellState;
using FLAG_Race::fluid3d::DepthFrame3D;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "depth_guidance_integration_check: " << message << std::endl;
        std::exit(1);
    }
}

std::shared_ptr<const DepthFrame3D> planeFrame() {
    auto frame = std::make_shared<DepthFrame3D>();
    frame->width = frame->height = 201;
    frame->fx = frame->fy = 40.0;
    frame->cx = frame->cy = 100.0;
    frame->min_depth = 0.1;
    frame->max_depth = 4.0;
    frame->depth_m.assign(static_cast<size_t>(frame->width) * frame->height, 2.0f);
    frame->stamp_sec = 1.0;
    return frame;
}

std::shared_ptr<const DepthFrame3D> forwardLaunchFrame() {
    auto frame = std::make_shared<DepthFrame3D>();
    frame->width = 160;
    frame->height = 120;
    frame->fx = 80.0;
    frame->fy = 60.0 / std::tan(M_PI / 6.0);
    frame->cx = 79.5;
    frame->cy = 59.5;
    frame->min_depth = 0.2;
    frame->max_depth = 5.0;
    frame->depth_m.assign(static_cast<size_t>(frame->width) * frame->height, 3.0f);
    frame->T_world_camera.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
    frame->T_world_camera.linear() =
        (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
    frame->stamp_sec = 1.0;
    return frame;
}

std::shared_ptr<const DepthFrame3D> centeredWallFrame(double wall_depth,
                                                       double wall_width) {
    auto frame = std::make_shared<DepthFrame3D>(*forwardLaunchFrame());
    frame->depth_m.assign(frame->depth_m.size(), 3.0f);
    const int half_width_px = static_cast<int>(std::ceil(
        0.5 * wall_width * frame->fx / wall_depth));
    const int u_lo = std::max(0, static_cast<int>(std::floor(frame->cx)) - half_width_px);
    const int u_hi = std::min(frame->width - 1,
                              static_cast<int>(std::ceil(frame->cx)) + half_width_px);
    for (int v = 0; v < frame->height; ++v)
        for (int u = u_lo; u <= u_hi; ++u)
            frame->depth_m[static_cast<size_t>(v) * frame->width + u] =
                static_cast<float>(wall_depth);
    return frame;
}

FluidGuidanceConfig testConfig() {
    FluidGuidanceConfig cfg;
    cfg.resolve_period = 0.01;
    cfg.window_forward_size = 2.0;
    cfg.window_lateral_size = 1.2;
    cfg.window_rear_margin = 0.3;
    cfg.coarse_grid_resolution = 0.3;
    cfg.coarse_resolution_z = 0.3;
    cfg.fine_forward_size = 2.0;
    cfg.fine_lateral_size = 1.2;
    cfg.fine_resolution = 0.2;
    cfg.grid_resolution_z = 0.2;
    cfg.z_min = 0.2;
    cfg.z_max = 2.2;
    cfg.floor_band = 0.0;
    cfg.ceil_band = 0.0;
    cfg.cg_max_iterations = 300;
    cfg.depth_robot_radius = 0.0;
    cfg.depth_geometry_margin = 0.0;
    cfg.depth_surface_band = 0.02;
    return cfg;
}

FluidGuidanceConfig forwardLaunchConfig() {
    FluidGuidanceConfig cfg;
    cfg.resolve_period = 0.01;
    cfg.window_forward_size = 12.0;
    cfg.window_lateral_size = 10.0;
    cfg.window_rear_margin = 0.3;
    cfg.coarse_grid_resolution = 0.3;
    cfg.coarse_resolution_z = 0.3;
    cfg.fine_forward_size = 6.0;
    cfg.fine_lateral_size = 6.0;
    cfg.fine_resolution = 0.2;
    cfg.grid_resolution_z = 0.15;
    cfg.z_min = 0.2;
    cfg.z_max = 2.6;
    cfg.floor_band = 0.3;
    cfg.ceil_band = 0.3;
    cfg.cg_max_iterations = 200;
    cfg.depth_robot_radius = 0.3;
    cfg.depth_geometry_margin = 0.0;
    cfg.depth_surface_band = 0.02;
    return cfg;
}

FluidGuidanceConfig crossflowLaunchConfig() {
    FluidGuidanceConfig cfg = forwardLaunchConfig();
    cfg.resolve_period = 0.10;
    cfg.crossflow_ratio = 0.5;
    cfg.crossflow_tau = 0.8;
    cfg.crossflow_rate_max = 1.0;
    cfg.stall_speed_ratio = 0.35;
    cfg.stall_latch_time = 0.4;
    cfg.stall_release_ratio = 0.6;
    cfg.stall_release_time = 0.8;
    return cfg;
}

FLAG_Race::fluid3d::Grid3D forwardLaunchFineGrid() {
    FLAG_Race::fluid3d::Grid3D grid;
    grid.e_J = Eigen::Vector2d::UnitX();
    grid.n_J = Eigen::Vector2d::UnitY();
    grid.origin_xy = Eigen::Vector2d(-0.3, -3.0);
    grid.h = 0.2;
    grid.h_z = 0.15;
    grid.z_lo = 0.2;
    grid.n_fwd = 30;
    grid.n_lat = 30;
    grid.n_z = 16;
    return grid;
}

bool finite(const Eigen::Vector3d& v) { return v.allFinite(); }

bool robotSeedReachesObservedFree(const FLAG_Race::fluid::FluidVis3D& vis,
                                  const Eigen::Vector3d& pos) {
    const auto& g = vis.grid;
    const Eigen::Vector2d rel = pos.head<2>() - g.origin_xy;
    const int i0 = static_cast<int>(std::floor(rel.dot(g.e_J) / g.h - 0.5));
    const int j0 = static_cast<int>(std::floor(rel.dot(g.n_J) / g.h - 0.5));
    const int m0 = static_cast<int>(std::floor((pos.z() - g.z_lo) / g.h_z - 0.5));
    if (i0 < 0 || j0 < 0 || m0 < 0 || i0 + 1 >= g.n_fwd ||
        j0 + 1 >= g.n_lat || m0 + 1 >= g.n_z) return false;
    std::vector<uint8_t> seen(g.size(), 0);
    std::queue<int> q;
    for (int di = 0; di < 2; ++di)
        for (int dj = 0; dj < 2; ++dj)
            for (int dm = 0; dm < 2; ++dm) {
                const int k = g.idx(i0 + di, j0 + dj, m0 + dm);
                if (vis.solid[k] || vis.state[k] != CellState::TrustedFree) return false;
                seen[k] = 1;
                q.push(k);
            }
    const int di6[] = {-1, 1, 0, 0, 0, 0};
    const int dj6[] = {0, 0, -1, 1, 0, 0};
    const int dm6[] = {0, 0, 0, 0, -1, 1};
    while (!q.empty()) {
        const int k = q.front(); q.pop();
        if (vis.state[k] == CellState::Free) return true;
        const int i = k / (g.n_lat * g.n_z);
        const int rem = k % (g.n_lat * g.n_z);
        const int j = rem / g.n_z;
        const int m = rem % g.n_z;
        for (int axis = 0; axis < 6; ++axis) {
            const int ni = i + di6[axis], nj = j + dj6[axis], nm = m + dm6[axis];
            if (ni < 0 || nj < 0 || nm < 0 || ni >= g.n_fwd || nj >= g.n_lat || nm >= g.n_z)
                continue;
            const int nk = g.idx(ni, nj, nm);
            if (!vis.solid[nk] && !seen[nk]) { seen[nk] = 1; q.push(nk); }
        }
    }
    return false;
}

FLAG_Race::fluid3d::Grid3D forwardLaunchCoarseGrid() {
    FLAG_Race::fluid3d::Grid3D grid;
    grid.e_J = Eigen::Vector2d::UnitX();
    grid.n_J = Eigen::Vector2d::UnitY();
    grid.origin_xy = Eigen::Vector2d(-0.45, -4.95);
    grid.h = grid.h_z = 0.3;
    grid.z_lo = 0.2;
    grid.n_fwd = 40;
    grid.n_lat = 33;
    grid.n_z = 8;
    return grid;
}

double percentileMs(std::vector<double> samples, double quantile) {
    std::sort(samples.begin(), samples.end());
    const size_t index = static_cast<size_t>(std::ceil(quantile * samples.size())) - 1;
    return samples[std::min(index, samples.size() - 1)];
}

int runBenchmark() {
    constexpr int kWarmup = 5;
    constexpr int kSamples = 40;
    const auto frame = forwardLaunchFrame();
    const auto coarse_grid = forwardLaunchCoarseGrid();
    const auto fine_grid = forwardLaunchFineGrid();
    const auto project_once = [&] {
        FLAG_Race::fluid3d::GridObservation3D coarse, fine;
        require(FLAG_Race::fluid3d::projectDepthFrameToGrid(
                    *frame, coarse_grid, 0.3, 0.0, 0.02, &coarse) &&
                FLAG_Race::fluid3d::projectDepthFrameToGrid(
                    *frame, fine_grid, 0.3, 0.0, 0.02, &fine),
                "benchmark projection must succeed");
    };
    std::vector<double> project_ms, solve_ms;
    for (int i = 0; i < kWarmup + kSamples; ++i) {
        const auto started = std::chrono::steady_clock::now();
        project_once();
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        if (i >= kWarmup) project_ms.push_back(elapsed);
    }
    const Eigen::Vector3d pos(0.0, 0.0, 1.0);
    const Eigen::Vector2d anchor = Eigen::Vector2d::Zero();
    for (int i = 0; i < kWarmup + kSamples; ++i) {
        FluidGuidance guidance;
        guidance.setConfig(forwardLaunchConfig());
        const auto started = std::chrono::steady_clock::now();
        const Eigen::Vector3d cmd = guidance.calcGuidance3D(
            pos, anchor, 0.0, 0.5, 1.0, false, frame);
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        require(finite(cmd) && cmd.x() > 1e-4,
                "benchmark guidance solve must produce forward motion");
        if (i >= kWarmup) solve_ms.push_back(elapsed);
    }
    std::cout << "depth_guidance_integration_check benchmark: samples=" << kSamples
              << " project_coarse_fine_ms median=" << percentileMs(project_ms, 0.50)
              << " p95=" << percentileMs(project_ms, 0.95)
              << " guidance_coarse_fine_ms median=" << percentileMs(solve_ms, 0.50)
              << " p95=" << percentileMs(solve_ms, 0.95) << std::endl;
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--benchmark") return runBenchmark();
    require(argc == 1, "only --benchmark is supported");
    const auto frame = planeFrame();
    FluidGuidance no_query;
    FluidGuidance high_clearance;
    FluidGuidance low_clearance;
    no_query.setConfig(testConfig());
    high_clearance.setConfig(testConfig());
    low_clearance.setConfig(testConfig());
    high_clearance.setDistanceQuery([](const Eigen::Vector3d&) { return 100.0; });
    low_clearance.setDistanceQuery([](const Eigen::Vector3d&) { return -100.0; });

    const Eigen::Vector3d pos(0.0, 0.0, 1.0);
    const Eigen::Vector2d anchor = Eigen::Vector2d::Zero();
    const auto solve_started = std::chrono::steady_clock::now();
    const Eigen::Vector3d no_query_cmd = no_query.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, frame);
    const Eigen::Vector3d high_cmd = high_clearance.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, frame);
    const double solve_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_started).count();
    const Eigen::Vector3d low_cmd = low_clearance.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, frame);
    const auto& no_query_vis = no_query.vis3D();
    const auto& high_vis = high_clearance.vis3D();
    const auto& low_vis = low_clearance.vis3D();

    require(finite(no_query_cmd) && finite(high_cmd) && finite(low_cmd),
            "valid depth must produce finite commands without a distance query");
    require(no_query.fuse3D(Eigen::Vector3d::Zero(), false, 0.0, 0.0, 0.5).isZero(1e-12),
            "invalid 3D field must not fall back to heading motion");
    require(no_query_vis.valid && no_query_vis.field.valid,
            "valid depth must produce a fine field without a distance query");
    require(no_query_vis.state == high_vis.state && high_vis.state == low_vis.state &&
            no_query_vis.solid == high_vis.solid && high_vis.solid == low_vis.solid &&
            (no_query_cmd - high_cmd).norm() < 1e-12 &&
            (high_cmd - low_cmd).norm() < 1e-12,
            "3D state, solid, and command must not depend on the distance query");
    bool saw_free = false, saw_occupied = false, saw_unknown = false;
    for (const CellState state : high_vis.state) {
        saw_free = saw_free || state == CellState::Free;
        saw_occupied = saw_occupied || state == CellState::Occupied;
        saw_unknown = saw_unknown || state == CellState::Unknown;
    }
    require(saw_free && saw_occupied && saw_unknown,
            "plane snapshot must retain Free, Occupied, and Unknown cells");

    Eigen::Vector3d blocked_live_pos;
    bool found_blocked_live_pos = false;
    for (int i = 1; i + 1 < high_vis.grid.n_fwd && !found_blocked_live_pos; ++i)
        for (int j = 1; j + 1 < high_vis.grid.n_lat && !found_blocked_live_pos; ++j)
            for (int m = 1; m + 1 < high_vis.grid.n_z; ++m) {
                const int k = high_vis.grid.idx(i, j, m);
                if (high_vis.solid[k]) {
                    blocked_live_pos = high_vis.grid.cellWorld(i, j, m);
                    found_blocked_live_pos = true;
                    break;
                }
            }
    require(found_blocked_live_pos, "test frame must contain an interior solid cell");
    const Eigen::Vector3d blocked_live_cmd = high_clearance.calcGuidance3D(
        blocked_live_pos, anchor, 0.0, 0.5, 1.001, true, frame);
    require(blocked_live_cmd.isZero(1e-12),
            "cached 3D field must hard-stop when live interpolation corners are solid/Unknown");

    const Eigen::Vector3d invalid_cmd = high_clearance.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 2.0, true, std::shared_ptr<const DepthFrame3D>());
    require(invalid_cmd.isZero(1e-12) && !high_clearance.vis3D().valid,
            "missing snapshot must clear the cached 3D field and fail closed");

    FluidGuidance forward;
    forward.setConfig(forwardLaunchConfig());
    const auto forward_frame = forwardLaunchFrame();
    const Eigen::Vector3d forward_cmd = forward.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, forward_frame);
    std::cout << "depth_guidance_integration_check: forward field="
              << forward.vis3D().field.valid << " command=" << forward_cmd.transpose()
              << std::endl;
    require(forward.vis3D().valid && forward.vis3D().field.valid && finite(forward_cmd) &&
            forward_cmd.x() > 0.49 && forward_cmd.x() <= 0.5 + 1e-9 &&
            std::abs(forward_cmd.y()) < 1e-3 && std::abs(forward_cmd.z()) < 1e-3,
            "open forward view must retain the intent cap without spurious lateral or vertical motion");
    bool saw_trusted = false, saw_forward_occupied = false;
    for (const CellState state : forward.vis3D().state) {
        saw_trusted = saw_trusted || state == CellState::TrustedFree;
        saw_forward_occupied = saw_forward_occupied || state == CellState::Occupied;
    }
    require(saw_trusted && saw_forward_occupied,
            "ego seed must remain distinct from observed occupied cells");
    require(robotSeedReachesObservedFree(forward.vis3D(), pos),
            "robot's TrustedFree sampling component must connect to observed Free cells");

    auto near_plane = std::make_shared<DepthFrame3D>(*forward_frame);
    near_plane->depth_m.assign(near_plane->depth_m.size(), 0.5f);
    FluidGuidance blocked_forward;
    blocked_forward.setConfig(forwardLaunchConfig());
    const Eigen::Vector3d blocked_cmd = blocked_forward.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, near_plane);
    require(blocked_cmd.isZero(1e-12) && !blocked_forward.vis3D().valid,
            "a near plane that removes the observed-free bridge must fail closed");

    // A 3m-wide forward wall is surrounded by measured far free space.  It
    // creates the potential-flow stagnation that should latch crossflow, then
    // introduce a finite lateral component on subsequent 20 ms ticks.
    FluidGuidance crossflow;
    crossflow.setConfig(crossflowLaunchConfig());
    const auto wall = centeredWallFrame(2.0, 3.0);
    Eigen::Vector3d wall_cmd = crossflow.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, wall);
    const double initial_wall_speed = wall_cmd.norm();
    bool saw_lateral_crossflow = std::abs(wall_cmd.y()) > 1e-3;
    for (int tick = 1; tick <= 100; ++tick) {
        wall_cmd = crossflow.calcGuidance3D(
            pos, anchor, 0.0, 0.5, 1.0 + 0.02 * tick, true, wall);
        saw_lateral_crossflow = saw_lateral_crossflow || std::abs(wall_cmd.y()) > 1e-3;
    }
    bool wall_has_projected_free = false;
    for (const CellState state : crossflow.vis3D().state)
        wall_has_projected_free = wall_has_projected_free || state == CellState::Free;
    std::cout << "depth_guidance_integration_check: 3m-wide wall initial="
              << initial_wall_speed << " final=" << wall_cmd.transpose() << std::endl;
    require(crossflow.vis3D().valid && crossflow.vis3D().field.valid && finite(wall_cmd),
            "centered wall must keep a valid finite field");
    require(wall_has_projected_free,
            "centered wall must retain measured Free cells around the obstacle");
    require(initial_wall_speed < 0.35 * 0.5,
            "centered wall must present a near-stagnation first tick");
    require(saw_lateral_crossflow && std::abs(wall_cmd.y()) > 1e-3,
            "centered wall must latch a finite lateral crossflow after repeated ticks");

    // A central near obstacle must block the optical-axis corridor even though
    // the surrounding 3 m wall still gives the projector genuine Free cells.
    auto central_patch = std::make_shared<DepthFrame3D>(*forward_frame);
    for (int v = 56; v <= 64; ++v)
        for (int u = 76; u <= 84; ++u)
            central_patch->depth_m[static_cast<size_t>(v) * central_patch->width + u] = 1.0f;
    FLAG_Race::fluid3d::GridObservation3D patch_observation;
    require(FLAG_Race::fluid3d::projectDepthFrameToGrid(
                *central_patch, forwardLaunchFineGrid(), 0.3, 0.0, 0.02,
                &patch_observation),
            "central-patch frame must project");
    bool patch_has_surrounding_free = false;
    for (CellState state : patch_observation.state)
        patch_has_surrounding_free = patch_has_surrounding_free || state == CellState::Free;
    require(patch_has_surrounding_free,
            "central-patch frame must retain surrounding projected Free cells");
    FluidGuidance patch_blocked_forward;
    patch_blocked_forward.setConfig(forwardLaunchConfig());
    const Eigen::Vector3d patch_blocked_cmd = patch_blocked_forward.calcGuidance3D(
        pos, anchor, 0.0, 0.5, 1.0, true, central_patch);
    require(patch_blocked_cmd.isZero(1e-12) && !patch_blocked_forward.vis3D().valid,
            "central near obstacle must block the trusted optical-axis corridor");

    auto nan_frame = std::make_shared<DepthFrame3D>(*forward_frame);
    nan_frame->depth_m.assign(nan_frame->depth_m.size(), std::numeric_limits<float>::quiet_NaN());
    FluidGuidance nan_guidance;
    nan_guidance.setConfig(forwardLaunchConfig());
    require(nan_guidance.calcGuidance3D(pos, anchor, 0.0, 0.5, 1.0, true, nan_frame).isZero(1e-12) &&
            !nan_guidance.vis3D().valid,
            "all-NaN depth must fail closed");

    auto nan_center = std::make_shared<DepthFrame3D>(*forward_frame);
    for (int v = 50; v <= 70; ++v)
        for (int u = 65; u <= 95; ++u)
            nan_center->depth_m[static_cast<size_t>(v) * nan_center->width + u] =
                std::numeric_limits<float>::quiet_NaN();
    FluidGuidance nan_center_guidance;
    nan_center_guidance.setConfig(forwardLaunchConfig());
    require(nan_center_guidance.calcGuidance3D(
                pos, anchor, 0.0, 0.5, 1.0, true, nan_center).isZero(1e-12) &&
            !nan_center_guidance.vis3D().valid,
            "central invalid depth must block the trusted optical corridor");

    auto bad_frame = std::make_shared<DepthFrame3D>(*forward_frame);
    bad_frame->cx = bad_frame->width;
    FluidGuidance bad_guidance;
    bad_guidance.setConfig(forwardLaunchConfig());
    require(bad_guidance.calcGuidance3D(pos, anchor, 0.0, 0.5, 1.0, true, bad_frame).isZero(1e-12) &&
            !bad_guidance.vis3D().valid,
            "invalid camera intrinsics must fail closed");

    auto bad_size_frame = std::make_shared<DepthFrame3D>(*forward_frame);
    bad_size_frame->depth_m.pop_back();
    FluidGuidance bad_size_guidance;
    bad_size_guidance.setConfig(forwardLaunchConfig());
    require(bad_size_guidance.calcGuidance3D(
                pos, anchor, 0.0, 0.5, 1.0, true, bad_size_frame).isZero(1e-12) &&
            !bad_size_guidance.vis3D().valid,
            "invalid depth image dimensions must fail closed");

    std::cout << "depth_guidance_integration_check: plane project/solve "
              << solve_ms << " ms; passed" << std::endl;
    return 0;
}
