#include "fluid/fluid_guidance.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <queue>

namespace FLAG_Race {
namespace fluid {

using namespace fluid2d;

namespace {

// One solved 2D grid level (the FluidLevel counterpart of fluid_solver_2d.h,
// extended with the timing breakdown the live path logs).
struct FluidLevel2D
{
    Eigen::Vector2d origin{0.0, 0.0};
    double h = 0.0, rear_margin = 0.0;
    int n_forward = 0, n_lateral = 0;
    Eigen::MatrixXd D;
    std::vector<std::vector<bool>> solid;
    fluid2d::SolidComponents comps;
    fluid2d::FluidHarmonicField field;
    double ms_esdf = 0.0, ms_mask = 0.0, ms_label = 0.0, ms_field = 0.0;
    double ms_total() const { return ms_esdf + ms_mask + ms_label + ms_field; }
};

// One solved 3D grid level.
struct FluidLevel3D
{
    fluid3d::Grid3D g;
    std::vector<fluid3d::CellState> state;
    double rear_margin = 0.0;
    int m_lo = 0, m_hi = -1;
    std::vector<uint8_t> solid;
    fluid3d::SolidComponents3D comps;
    fluid3d::SolidComponents3D blocking_comps;
    fluid3d::PotentialField3D field;
    double ms_project = 0.0, ms_mask = 0.0, ms_label = 0.0, ms_field = 0.0;
    double ms_total() const { return ms_project + ms_mask + ms_label + ms_field; }
};

double wallMs(const std::chrono::steady_clock::time_point& a,
              const std::chrono::steady_clock::time_point& b)
{
    return 1e3 * std::chrono::duration<double>(b - a).count();
}

}  // namespace

void FluidGuidance::setConfig(const FluidGuidanceConfig& cfg)
{
    c_ = cfg;
    // Sanity clamps mirroring gvf::init.
    c_.grid_resolution = std::max(1e-3, c_.grid_resolution);
    c_.window_forward_size = std::max(3.0 * c_.grid_resolution, c_.window_forward_size);
    c_.window_lateral_size = std::max(3.0 * c_.grid_resolution, c_.window_lateral_size);
    c_.window_rear_margin = std::max(1.5 * c_.grid_resolution, c_.window_rear_margin);
    c_.window_rear_margin = std::min(
        c_.window_forward_size - 1.5 * c_.grid_resolution, c_.window_rear_margin);
    c_.delta_theta_max_rad = std::max(0.0, c_.delta_theta_max_rad);
    c_.conv_gate_margin = std::max(1e-3, c_.conv_gate_margin);
    c_.conv_length = std::max(1e-3, c_.conv_length);
    c_.bypass_offset = std::max(0.0, c_.bypass_offset);
    c_.default_bias_sign = c_.default_bias_sign < 0.0 ? -1.0 : 1.0;
    c_.side_deadband = std::max(0.0, c_.side_deadband);
    c_.speed_floor = std::max(0.0, std::min(1.0, c_.speed_floor));
    c_.escape_tangent_ratio = std::max(0.0, std::min(1.0, c_.escape_tangent_ratio));
    c_.escape_outward_ratio = std::max(0.0, std::min(1.0, c_.escape_outward_ratio));
    c_.escape_solid_weight = std::max(0.0, std::min(1.0, c_.escape_solid_weight));
    if (!c_.use_coarse && !c_.use_fine) c_.use_fine = true;

    c_.fine_resolution = std::max(1e-3, c_.fine_resolution);
    c_.grid_resolution_z = std::max(1e-3, c_.grid_resolution_z);
    c_.coarse_resolution_z = std::max(1e-3, c_.coarse_resolution_z);
    if (c_.grid_resolution_z > 2.0 * c_.fine_resolution)
        c_.grid_resolution_z = 2.0 * c_.fine_resolution;
    if (c_.coarse_resolution_z > 2.0 * c_.coarse_grid_resolution)
        c_.coarse_resolution_z = 2.0 * c_.coarse_grid_resolution;
    c_.floor_band = std::max(0.0, c_.floor_band);
    c_.ceil_band = std::max(0.0, c_.ceil_band);
    c_.z_max = std::max(c_.z_min + c_.floor_band + c_.ceil_band +
                            3.0 * c_.grid_resolution_z, c_.z_max);
    c_.crossflow_ratio = std::max(0.0, std::min(2.0, c_.crossflow_ratio));
    c_.stall_speed_ratio = std::max(0.0, std::min(1.0, c_.stall_speed_ratio));
    c_.stall_w_veto = std::max(0.0, c_.stall_w_veto);
    c_.stall_release_ratio = std::max(
        c_.stall_speed_ratio + 0.05, std::min(1.0, c_.stall_release_ratio));
    c_.stall_clearance = std::max(0.0, c_.stall_clearance);
    c_.stall_probe = std::max(0.1, c_.stall_probe);
    c_.stall_latch_time = std::max(0.0, c_.stall_latch_time);
    c_.stall_release_time = std::max(0.0, c_.stall_release_time);
    c_.alt_gain = std::max(0.0, c_.alt_gain);
    c_.alt_rate_max = std::max(0.0, c_.alt_rate_max);
    c_.alt_gate_lo = std::max(0.0, c_.alt_gate_lo);
    c_.alt_gate_hi = std::max(c_.alt_gate_lo + 1e-3, c_.alt_gate_hi);
    c_.vertical_preview_len = std::max(0.0, c_.vertical_preview_len);
    c_.vertical_preview_clearance = std::max(0.0, c_.vertical_preview_clearance);
    c_.vertical_preview_rate = std::max(0.0, c_.vertical_preview_rate);
    c_.psi_error_gain = std::max(0.0, c_.psi_error_gain);
    c_.cg_residual_acceptance = std::max(1.0, c_.cg_residual_acceptance);
    c_.side_match_centroid_dist = std::max(1e-3, c_.side_match_centroid_dist);
    c_.side_latch_ttl = std::max(0.1, c_.side_latch_ttl);
    c_.side_match_lateral_tol = std::max(0.0, std::min(1.0, c_.side_match_lateral_tol));
    c_.side_match_forward_tol = std::max(0.0, std::min(1.0, c_.side_match_forward_tol));
    c_.k_n = std::max(0.0, c_.k_n);
    c_.k_n_z_ratio = std::max(0.0, std::min(1.0, c_.k_n_z_ratio));
    c_.streamline_fwd_len = std::max(0.5, c_.streamline_fwd_len);
    c_.streamline_bwd_len = std::max(0.0, c_.streamline_bwd_len);
    c_.streamline_ds = std::max(1e-3, c_.streamline_ds);
    c_.streamline_max_disp = std::max(1e-3, c_.streamline_max_disp);
    c_.streamline_degrade_cooldown = std::max(0.0, c_.streamline_degrade_cooldown);
    c_.d_v_max = std::max(0.0, c_.d_v_max);
    c_.crossflow_tau = std::max(1e-3, c_.crossflow_tau);
    c_.crossflow_rate_max = std::max(0.0, c_.crossflow_rate_max);
    c_.depth_robot_radius = std::max(0.0, c_.depth_robot_radius);
    c_.depth_geometry_margin = std::max(0.0, c_.depth_geometry_margin);
    c_.depth_surface_band = std::max(0.0, c_.depth_surface_band);
    c_.depth_ego_seed_radius = std::max(0.0, c_.depth_ego_seed_radius);
}

void FluidGuidance::reset()
{
    fluid_field_valid_ = false;
    fluid_cached_t_field_valid_ = false;
    fluid_cached_clearance_grad_.setZero();
    fluid_cached_clearance_ = 0.0;
    fluid_cached_clearance_grad_valid_ = false;
    fluid_last_solve_time_ = 0.0;
    fluid3d_field_valid_ = false;
    fluid3d_phi_.clear();
    fluid3d_cached_t_field_valid_ = false;
    fluid3d_cross_latched_ = false;
    fluid3d_cross_beta_ = 0.0;
    fluid3d_stall_accum_ = 0.0;
    fluid3d_release_accum_ = 0.0;
    fluid3d_last_tick_time_ = 0.0;
    fluid3d_coarse_extents_.clear();
    fluid_psi_anchor_ref_valid_ = false;
    streamline_view_.valid = false;
    fluid3d_field_data_valid_ = false;
    fluid3d_track_mode_ = 2;
    fluid3d_streamline_version_ = 0;
    fluid3d_streamline_update_disp_ = 0.0;
    fluid3d_streamline_clearance_min_ = std::numeric_limits<double>::quiet_NaN();
    fluid3d_d_v_current_ = std::numeric_limits<double>::quiet_NaN();
    fluid_diag_field_direction_.setZero();
    fluid_diag_potential_ = 0.0;
    fluid_diag_field_valid_ = false;
    fluid_diag_potential_valid_ = false;
    vis2d_ = FluidVis2D{};
    vis3d_ = FluidVis3D{};
}

void FluidGuidance::invalidate3DField()
{
    fluid3d_field_valid_ = false;
    fluid3d_field_data_valid_ = false;
    fluid3d_Ux_.clear(); fluid3d_Uy_.clear(); fluid3d_Uz_.clear();
    fluid3d_phi_.clear(); fluid3d_state_.clear(); fluid3d_solid_.clear();
    fluid3d_cached_t_field_valid_ = false;
    fluid3d_coarse_extents_.clear();
    streamline_view_ = Streamline3DView{};
    fluid3d_track_mode_ = 2;
    fluid_diag_field_direction_.setZero();
    fluid_diag_potential_ = 0.0;
    fluid_diag_field_valid_ = false;
    fluid_diag_potential_valid_ = false;
    vis3d_ = FluidVis3D{};
}

void FluidGuidance::getDiagnostics(Eigen::Vector3d& field_direction,
                                   double& potential, bool& field_valid,
                                   bool& potential_valid) const
{
    field_direction = fluid_diag_field_direction_;
    potential = fluid_diag_potential_;
    field_valid = fluid_diag_field_valid_;
    potential_valid = fluid_diag_potential_valid_;
}

void FluidGuidance::setDiagnosticDirection(const Eigen::Vector3d& direction)
{
    if (direction.allFinite() && direction.squaredNorm() > 1e-12) {
        fluid_diag_field_direction_ = direction.normalized();
        fluid_diag_field_valid_ = true;
    }
}

void FluidGuidance::getStreamlineDiagnostics(
    const Eigen::Vector3d& pos, Eigen::Vector3d& projection,
    Eigen::Vector3d& e_perp, double& eta_xy, double& eta_3d, bool& valid,
    int& track_mode, bool& end_cap, uint64_t& version, double& update_disp,
    double& streamline_clearance_min, double& projection_clearance,
    double& d_v) const
{
    valid = streamline_view_.valid && streamline_view_.points.size() >= 2;
    track_mode = fluid3d_track_mode_;
    end_cap = valid && streamline_view_.end_cap;
    version = fluid3d_streamline_version_;
    update_disp = fluid3d_streamline_update_disp_;
    streamline_clearance_min = fluid3d_streamline_clearance_min_;
    projection.setConstant(std::numeric_limits<double>::quiet_NaN());
    e_perp.setConstant(std::numeric_limits<double>::quiet_NaN());
    eta_xy = eta_3d = std::numeric_limits<double>::quiet_NaN();
    projection_clearance = std::numeric_limits<double>::quiet_NaN();
    d_v = fluid3d_d_v_current_;
    if (valid) {
        const StreamlineProj3D proj = projectToStreamline3D(pos);
        projection = proj.p;
        e_perp = pos - proj.p;
        eta_xy = e_perp.head<2>().norm();
        eta_3d = e_perp.norm();
        end_cap = proj.end_cap;
    }
}

// ------------------------------------------------------------------ 2D law

Eigen::Vector2d FluidGuidance::fuseLiftedGvfFluid(const Eigen::Vector2d& field_uv,
                                                  bool field_valid,
                                                  double psi_here,
                                                  bool psi_valid,
                                                  double heading_rad,
                                                  double phi, double v_cap,
                                                  double D_local) const
{
    // Primary law (paper eq. 29, normalized): stream-function-error control
    // on the cached harmonic field.  chi = psi / v_s, chi* anchored, k_psi in
    // s^-1; the correction is exactly normal to the flow, so the tangential
    // speed law is untouched and d/dt e_chi = -k_psi*e_chi (exact decay).
    const double v_t = v_cap * std::max(c_.speed_floor,
        fluid2d::fluidSpeedRatio(D_local, c_.d_s, c_.d_drag));
    auto applyRayHoming = [&](const Eigen::Vector2d& v) -> Eigen::Vector2d {
        const double speed = v.norm();
        if (!c_.use_regression || speed <= 1e-9) return v;
        const double theta_ray_target =
            heading_rad - std::atan(phi / c_.conv_length);
        const double theta_field = std::atan2(v.y(), v.x());
        double delta = theta_ray_target - theta_field;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        const double clear_gate = 1.0 - fluid2d::fluidSmoothstepW(
            D_local, c_.d_look, c_.d_look + c_.conv_gate_margin);
        delta = std::max(-c_.delta_theta_max_rad,
                         std::min(c_.delta_theta_max_rad, delta)) * clear_gate;
        const double theta_cmd = theta_field + delta;
        return speed * Eigen::Vector2d(std::cos(theta_cmd), std::sin(theta_cmd));
    };
    if (c_.use_regression && field_valid && psi_valid && fluid_psi_anchor_ref_valid_ &&
        fluid_field_solve_speed_ > 1e-9) {
        const fluid2d::StreamFunctionLawResult r = fluid2d::streamFunctionErrorLaw(
            field_uv, fluid_field_solve_speed_, psi_here / fluid_field_solve_speed_,
            fluid_psi_anchor_ref_, v_t, c_.psi_error_gain);
        if (!r.degraded) return applyRayHoming(r.v);
    }

    // Degenerate branch (no field / psi invalid / chi* not anchored): legacy
    // angle-space law, degraded-mode fallback only.
    double theta_field;
    if (field_valid && field_uv.squaredNorm() > 1e-12) {
        theta_field = std::atan2(field_uv.y(), field_uv.x());
    } else {
        theta_field = heading_rad;
    }
    return applyRayHoming(v_t * Eigen::Vector2d(
        std::cos(theta_field), std::sin(theta_field)));
}

Eigen::Vector2d FluidGuidance::calcGuidance2D(const Eigen::Vector2d& pos_xy,
                                              const Eigen::Vector2d& anchor_xy,
                                              double heading_rad, double v_J,
                                              double now_sec, bool build_viz)
{
    now_sec_ = now_sec;
    if (!distance_) return Eigen::Vector2d::Zero();

    const double v_cap = std::max(0.0, v_J);
    const Eigen::Vector2d e_J(std::cos(heading_rad), std::sin(heading_rad));
    const Eigen::Vector2d n_J(-std::sin(heading_rad), std::cos(heading_rad));
    vis2d_.coverage_valid = true;
    vis2d_.center_xy = pos_xy;
    vis2d_.vis_e_J = e_J;
    vis2d_.vis_n_J = n_J;
    vis2d_.vis_z = c_.cruise_z;

    double D_here = distance_(Eigen::Vector3d(pos_xy.x(), pos_xy.y(), c_.cruise_z));
    // Defensive: a corrupted ESDF read must never read as "clear space".
    // Fall back to the last known-good sampled clearance instead of either
    // extreme (huge bogus value == "go fast"; forced zero == permanent stall).
    if (!std::isfinite(D_here) || std::abs(D_here) > 1e6) {
        D_here = fluid_cached_clearance_grad_valid_ ? fluid_cached_clearance_ : 0.0;
    }

    const bool due = (now_sec - fluid_last_solve_time_) >= c_.resolve_period;

    if (due && v_cap > 1e-9) {
        auto build_level = [&](double forward_size, double lateral_size, double hh,
                               bool centered, double rear_margin_override,
                               const FluidCoarsePrior* prior,
                               const std::function<std::vector<double>(
                                   const fluid2d::SolidComponents&)>& side_fn) {
            FluidLevel2D lv;
            lv.h = hh;
            lv.n_forward = std::max(
                3, static_cast<int>(std::round(forward_size / hh)));
            lv.n_lateral = std::max(
                3, static_cast<int>(std::round(lateral_size / hh)));
            lv.rear_margin = rear_margin_override >= 0.0
                ? std::max(1.5 * hh, std::min((lv.n_forward - 1.5) * hh, rear_margin_override))
                : (centered
                    ? 0.5 * lv.n_forward * hh
                    : std::max(1.5 * hh, std::min((lv.n_forward - 1.5) * hh, c_.window_rear_margin)));
            lv.origin = pos_xy - lv.rear_margin * e_J - 0.5 * lv.n_lateral * hh * n_J;

            const auto t0 = std::chrono::steady_clock::now();
            lv.D.resize(lv.n_forward, lv.n_lateral);
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    const Eigen::Vector2d world =
                        fluid2d::fluidCellWorld(lv.origin, e_J, n_J, hh, i, j);
                    double sample = distance_(
                        Eigen::Vector3d(world.x(), world.y(), c_.cruise_z));
                    if (!std::isfinite(sample) || std::abs(sample) > 1e6) sample = 0.0;
                    lv.D(i, j) = sample;
                }
            }

            const auto t1 = std::chrono::steady_clock::now();
            lv.solid.assign(lv.n_forward, std::vector<bool>(lv.n_lateral, false));
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    lv.solid[i][j] = lv.D(i, j) <= c_.d_s;
                }
            }
            // Fold unreachable free pockets into solid so every surviving free
            // cell reaches the Dirichlet ring (well-posed/SPD solve).
            std::vector<std::vector<bool>> reachable(
                lv.n_forward, std::vector<bool>(lv.n_lateral, false));
            std::queue<std::pair<int, int>> frontier;
            auto push_if_free = [&](int i, int j) {
                if (!lv.solid[i][j] && !reachable[i][j]) {
                    reachable[i][j] = true;
                    frontier.emplace(i, j);
                }
            };
            for (int i = 0; i < lv.n_forward; ++i) {
                push_if_free(i, 0);
                push_if_free(i, lv.n_lateral - 1);
            }
            for (int j = 1; j + 1 < lv.n_lateral; ++j) {
                push_if_free(0, j);
                push_if_free(lv.n_forward - 1, j);
            }
            constexpr int di[4] = {1, -1, 0, 0};
            constexpr int dj[4] = {0, 0, 1, -1};
            while (!frontier.empty()) {
                const auto cell = frontier.front();
                frontier.pop();
                for (int d = 0; d < 4; ++d) {
                    const int ni = cell.first + di[d];
                    const int nj = cell.second + dj[d];
                    if (ni >= 0 && ni < lv.n_forward && nj >= 0 && nj < lv.n_lateral) {
                        push_if_free(ni, nj);
                    }
                }
            }
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    if (!lv.solid[i][j] && !reachable[i][j]) lv.solid[i][j] = true;
                }
            }

            const auto t2 = std::chrono::steady_clock::now();
            lv.comps = labelSolidComponents(lv.solid, lv.origin, e_J, n_J, hh,
                                            anchor_xy, lv.n_forward, lv.n_lateral);
            const auto t3 = std::chrono::steady_clock::now();
            lv.field = solveHarmonicStreamField(
                lv.solid, lv.comps, lv.origin, e_J, n_J, hh,
                lv.n_forward, lv.n_lateral, anchor_xy, v_cap, side_fn(lv.comps),
                c_.bypass_offset, c_.default_bias_sign, c_.side_deadband,
                c_.cg_tolerance, c_.cg_max_iterations,
                c_.cg_residual_acceptance, prior);
            const auto t4 = std::chrono::steady_clock::now();

            lv.ms_esdf  = wallMs(t0, t1);
            lv.ms_mask  = wallMs(t1, t2);
            lv.ms_label = wallMs(t2, t3);
            lv.ms_field = wallMs(t3, t4);
            return lv;
        };

        FluidLevel2D coarse;
        if (c_.use_coarse) {
            coarse = build_level(c_.window_forward_size, c_.window_lateral_size,
                                 c_.coarse_grid_resolution,
                                 /*centered=*/false, /*rear_margin_override=*/-1.0,
                                 nullptr,
                [&](const fluid2d::SolidComponents& comps) {
                    return computeLatchedSides(comps);
                });
        }
        FluidCoarsePrior prior;
        if (coarse.field.valid) {
            prior.psi = &coarse.field.psi;
            prior.origin = coarse.origin;
            prior.e_J = e_J;
            prior.n_J = n_J;
            prior.h = coarse.h;
            prior.n_forward = coarse.n_forward;
            prior.n_lateral = coarse.n_lateral;
            commitSideLatch(coarse.comps, coarse.field.chosen_sides);
        }
        FluidLevel2D fine;
        if (c_.use_fine) {
            fine = build_level(c_.fine_forward_size, c_.fine_lateral_size,
                               c_.grid_resolution,
                               /*centered=*/true, /*rear_margin_override=*/-1.0,
                               coarse.field.valid ? &prior : nullptr,
                [&](const fluid2d::SolidComponents& comps) {
                    std::vector<double> sides(comps.num_components, 0.0);
                    if (!coarse.field.valid) return sides;
                    for (int c = 0; c < comps.num_components; ++c) {
                        int best = -1;
                        double best_d = 1e18;
                        for (int cc = 0; cc < coarse.comps.num_components; ++cc) {
                            const double d = (comps.centroid_world[c] -
                                              coarse.comps.centroid_world[cc]).norm();
                            if (d < best_d) { best_d = d; best = cc; }
                        }
                        if (best >= 0 && best_d <= c_.side_match_centroid_dist &&
                            best < static_cast<int>(coarse.field.chosen_sides.size()) &&
                            std::abs(coarse.field.chosen_sides[best]) == 1.0) {
                            sides[c] = coarse.field.chosen_sides[best];
                        }
                    }
                    return sides;
                });
        }

        const FluidLevel2D& active = fine.field.valid ? fine : coarse;
        const double h = active.h;
        const int n_forward = active.n_forward;
        const int n_lateral = active.n_lateral;
        const double rear_margin = active.rear_margin;
        const Eigen::Vector2d origin = active.origin;
        const Eigen::MatrixXd& D = active.D;
        const std::vector<std::vector<bool>>& solid = active.solid;
        const fluid2d::FluidHarmonicField& field = active.field;

        if (field.valid) {
            // chi* continuity across the rebuild (chi*_new = chi_new(x) - (chi_old(x) - chi*_old)).
            if (fluid_psi_anchor_ref_valid_ && fluid_field_valid_ &&
                fluid_field_solve_speed_ > 1e-9) {
                double chi_old_here = 0.0, chi_new_here = 0.0;
                const bool old_ok = bilinearSamplePsi(
                    fluid_psi_, fluid_field_origin_, fluid_field_e_J_,
                    fluid_field_n_J_, fluid_field_h_, fluid_field_n_forward_,
                    fluid_field_n_lateral_, pos_xy, chi_old_here);
                const bool new_ok = bilinearSamplePsi(
                    field.psi, origin, e_J, n_J, h, n_forward, n_lateral,
                    pos_xy, chi_new_here);
                if (old_ok && new_ok) {
                    fluid_psi_anchor_ref_ = fluid2d::chiStarContinuityUpdate(
                        chi_old_here / fluid_field_solve_speed_,
                        chi_new_here / v_cap, fluid_psi_anchor_ref_);
                }
            } else {
                double chi_anchor = 0.0;
                if (bilinearSamplePsi(field.psi, origin, e_J, n_J, h,
                                      n_forward, n_lateral, anchor_xy,
                                      chi_anchor)) {
                    fluid_psi_anchor_ref_ = chi_anchor / v_cap;
                    fluid_psi_anchor_ref_valid_ = true;
                } else {
                    fluid_psi_anchor_ref_valid_ = false;
                }
            }
            fluid_psi_ = field.psi;
            fluid_Ux_field_ = field.Ux_query;
            fluid_Uy_field_ = field.Uy_query;
            fluid_field_origin_ = origin;
            fluid_field_e_J_ = e_J;
            fluid_field_n_J_ = n_J;
            fluid_field_h_ = h;
            fluid_field_n_forward_ = n_forward;
            fluid_field_n_lateral_ = n_lateral;
            fluid_field_solve_speed_ = v_cap;
            fluid_field_valid_ = true;
        }

        // Refresh the cached center-cell clearance + outward normal.
        {
            const int ci = std::max(1, std::min(
                n_forward - 2, static_cast<int>(std::round(rear_margin / h - 0.5))));
            const int cj = std::max(1, std::min(
                n_lateral - 2, static_cast<int>(std::round(0.5 * n_lateral - 0.5))));
            const double grad_s = (D(ci + 1, cj) - D(ci - 1, cj)) / (2.0 * h);
            const double grad_eta = (D(ci, cj + 1) - D(ci, cj - 1)) / (2.0 * h);
            const Eigen::Vector2d center_grad = grad_s * e_J + grad_eta * n_J;
            const double center_grad_norm = center_grad.norm();
            const double center_clearance = D(ci, cj);
            if (center_grad.allFinite() && center_grad_norm > 1e-6 &&
                center_grad_norm < 50.0 && std::isfinite(center_clearance) &&
                std::abs(center_clearance) < 100.0) {
                fluid_cached_clearance_ = center_clearance;
                fluid_cached_clearance_grad_ = center_grad;
                fluid_cached_clearance_grad_valid_ = true;
            }
        }
        fluid_last_solve_time_ = now_sec;

        // Build the visualization snapshot from the just-solved field (the
        // fused command field per cell, matching what the controller samples).
        if (build_viz) {
            Eigen::MatrixXd Ux_cmd, Uy_cmd;
            if (!c_.use_fine) {
                // Coarse-only ablation: draw the FUSED command field on the
                // full-reach grid.
                Ux_cmd.resize(n_forward, n_lateral);
                Uy_cmd.resize(n_forward, n_lateral);
                for (int qi = 0; qi < n_forward; ++qi) {
                    for (int qj = 0; qj < n_lateral; ++qj) {
                        if (solid[qi][qj]) {
                            Ux_cmd(qi, qj) = 0.0;
                            Uy_cmd(qi, qj) = 0.0;
                            continue;
                        }
                        const Eigen::Vector2d cell_pos = fluid2d::fluidCellWorld(
                            origin, e_J, n_J, h, qi, qj);
                        const double phi_cell = (cell_pos - anchor_xy).dot(n_J);
                        const Eigen::Vector2d v_cell = fuseLiftedGvfFluid(
                            Eigen::Vector2d(field.Ux(qi, qj), field.Uy(qi, qj)),
                            true, field.psi(qi, qj), true,
                            heading_rad, phi_cell, v_cap, D(qi, qj));
                        Ux_cmd(qi, qj) = v_cell.x();
                        Uy_cmd(qi, qj) = v_cell.y();
                    }
                }
                vis2d_.valid = true;
                vis2d_.coarse_style = true;
                vis2d_.origin = origin;
                vis2d_.e_J = e_J;
                vis2d_.n_J = n_J;
                vis2d_.h = h;
                vis2d_.n_forward = n_forward;
                vis2d_.n_lateral = n_lateral;
                vis2d_.Ux = Ux_cmd;
                vis2d_.Uy = Uy_cmd;
                vis2d_.solid = solid;
                vis2d_.coarse_valid = false;
            } else {
                Ux_cmd.resize(n_forward, n_lateral);
                Uy_cmd.resize(n_forward, n_lateral);
                for (int qi = 0; qi < n_forward; ++qi) {
                    for (int qj = 0; qj < n_lateral; ++qj) {
                        if (solid[qi][qj]) {
                            Ux_cmd(qi, qj) = 0.0;
                            Uy_cmd(qi, qj) = 0.0;
                            continue;
                        }
                        const Eigen::Vector2d field_uv(fluid_Ux_field_(qi, qj),
                                                       fluid_Uy_field_(qi, qj));
                        const Eigen::Vector2d cell_pos = fluid2d::fluidCellWorld(
                            origin, e_J, n_J, h, qi, qj);
                        const double phi_cell = (cell_pos - anchor_xy).dot(n_J);
                        const Eigen::Vector2d v_cell = fuseLiftedGvfFluid(
                            field_uv, true, fluid_psi_(qi, qj), true,
                            heading_rad, phi_cell, v_cap, D(qi, qj));
                        Ux_cmd(qi, qj) = v_cell.x();
                        Uy_cmd(qi, qj) = v_cell.y();
                    }
                }
                vis2d_.valid = true;
                vis2d_.coarse_style = false;
                vis2d_.origin = origin;
                vis2d_.e_J = e_J;
                vis2d_.n_J = n_J;
                vis2d_.h = h;
                vis2d_.n_forward = n_forward;
                vis2d_.n_lateral = n_lateral;
                vis2d_.Ux = Ux_cmd;
                vis2d_.Uy = Uy_cmd;
                vis2d_.solid = solid;
                vis2d_.coarse_valid = coarse.field.valid;
                if (coarse.field.valid) {
                    vis2d_.coarse_origin = coarse.origin;
                    vis2d_.coarse_h = coarse.h;
                    vis2d_.coarse_n_forward = coarse.n_forward;
                    vis2d_.coarse_n_lateral = coarse.n_lateral;
                    vis2d_.coarse_Ux = coarse.field.Ux;
                    vis2d_.coarse_Uy = coarse.field.Uy;
                    vis2d_.coarse_solid = coarse.solid;
                }
            }
        }
    } else if (due) {
        // Joystick released: u = 0 analytically; hold the last field.
        fluid_last_solve_time_ = now_sec;
    }

    // Per-tick command: bilinearly sample the cached field at the live pos.
    Eigen::Vector2d field_uv = Eigen::Vector2d::Zero();
    bool field_sample_valid = false;
    if (fluid_field_valid_) {
        field_sample_valid = bilinearSampleField(
            fluid_Ux_field_, fluid_Uy_field_, fluid_field_origin_,
            fluid_field_e_J_, fluid_field_n_J_, fluid_field_h_, fluid_field_n_forward_,
            fluid_field_n_lateral_, pos_xy, field_uv);
    }
    if (field_sample_valid && field_uv.squaredNorm() > 1e-12) {
        fluid_cached_t_field_ = field_uv.normalized();
        fluid_cached_t_field_valid_ = true;
    }
    double psi_here = 0.0;
    const bool psi_sample_valid = fluid_field_valid_ && bilinearSamplePsi(
        fluid_psi_, fluid_field_origin_, fluid_field_e_J_, fluid_field_n_J_,
        fluid_field_h_, fluid_field_n_forward_, fluid_field_n_lateral_,
        pos_xy, psi_here);

    const double phi_here = (pos_xy - anchor_xy).dot(n_J);
    Eigen::Vector2d u_raw = fuseLiftedGvfFluid(
        field_uv, field_sample_valid, psi_here, psi_sample_valid,
        heading_rad, phi_here, v_cap, D_here);

    // Near-wall escape net: keep a useful tangential/outward command at the
    // solid boundary instead of a dead stop.
    const double D_grad = fluid_cached_clearance_;
    const double grad_consistency_tol = std::max(0.50, 2.0 * c_.grid_resolution);
    const bool gradient_valid = fluid_cached_clearance_grad_valid_ &&
                                std::abs(D_grad - D_here) <= grad_consistency_tol;
    const double grad_norm = fluid_cached_clearance_grad_.norm();
    const double raw_speed = u_raw.norm();
    const double stall_lo = std::max(0.0, c_.escape_stall_lo_ratio) * v_cap;
    const double stall_hi = std::max(stall_lo + 1e-6,
                                     c_.escape_stall_hi_ratio * v_cap);
    const double stall_w = fluid2d::fluidSmoothstepW(raw_speed, stall_lo, stall_hi);
    const double near_wall_w =
        fluid2d::fluidSmoothstepW(D_here, c_.d_s, c_.d_drag);
    const bool in_open_near_band = D_here > c_.d_s && D_here < c_.d_turn;
    const double escape_w = D_here <= c_.d_s
        ? std::max(0.0, c_.escape_solid_weight)
        : (in_open_near_band ? stall_w * near_wall_w : 0.0);
    if (v_cap > 1e-9 && escape_w > 1e-6 && gradient_valid) {
        const Eigen::Vector2d outward = fluid_cached_clearance_grad_ / grad_norm;
        const Eigen::Vector2d tangent_ccw(-outward.y(), outward.x());
        const Eigen::Vector2d align_ref =
            fluid_cached_t_field_valid_ ? fluid_cached_t_field_ : e_J;
        double tangent_alignment = tangent_ccw.dot(align_ref);
        if (std::abs(tangent_alignment) <= 1e-4) {
            tangent_alignment = tangent_ccw.dot(e_J);
        }
        const Eigen::Vector2d tangent =
            (tangent_alignment < 0.0 ? -1.0 : 1.0) * tangent_ccw;

        const double inward = u_raw.dot(outward);
        if (inward < 0.0) {
            u_raw -= inward * outward;
        }

        const double tangent_speed = u_raw.dot(tangent);
        if (tangent_speed >= -1e-3) {
            const double tangent_target = c_.escape_tangent_ratio * v_cap;
            const double tangent_supplement = escape_w * std::max(
                0.0, tangent_target - std::max(0.0, tangent_speed));
            u_raw += tangent_supplement * tangent;
        }

        const double outward_speed = u_raw.dot(outward);
        const double outward_target =
            c_.escape_outward_ratio * v_cap * near_wall_w;
        const double outward_supplement = escape_w * std::max(
            0.0, outward_target - std::max(0.0, outward_speed));
        u_raw += outward_supplement * outward;
    }

    if (!u_raw.allFinite()) {
        u_raw.setZero();
        reset();
    }

    if (v_cap > 1e-9) {
        const double speed = u_raw.norm();
        if (speed > v_cap) u_raw *= v_cap / speed;
    } else {
        u_raw.setZero();
    }

    if (D_here <= c_.d_s && !gradient_valid) {
        u_raw.setZero();
    }
    setDiagnosticDirection(Eigen::Vector3d(u_raw.x(), u_raw.y(), 0.0));
    return u_raw;
}

// ------------------------------------------------------------------ 2D side

std::vector<double> FluidGuidance::computeLatchedSides(
    const fluid2d::SolidComponents& comps)
{
    std::vector<double> sides(comps.num_components, 0.0);
    fluid_side_latch_.erase(
        std::remove_if(fluid_side_latch_.begin(), fluid_side_latch_.end(),
                       [&](const FluidSideLatchEntry& e) {
                           return (now_sec_ - e.stamp) > c_.side_latch_ttl;
                       }),
        fluid_side_latch_.end());
    if (comps.num_components == 0 || fluid_side_latch_.empty()) return sides;

    const size_t latch_n = fluid_side_latch_.size();
    std::vector<bool> used(latch_n, false);
    auto overlap_ok = [&](const FluidSideLatchEntry& e, int c) {
        const double lat_lo = std::max(e.eta_min, comps.eta_min[c]);
        const double lat_hi = std::min(e.eta_max, comps.eta_max[c]);
        const double lat_len = std::max(1e-6, std::min(e.eta_max - e.eta_min,
                                                       comps.eta_max[c] - comps.eta_min[c]));
        if ((lat_hi - lat_lo) / lat_len < c_.side_match_lateral_tol) return false;
        const double s_lo = std::max(e.s_min, comps.s_min[c]);
        const double s_hi = std::min(e.s_max, comps.s_max[c]);
        const double s_len = std::max(1e-6, std::min(e.s_max - e.s_min,
                                                     comps.s_max[c] - comps.s_min[c]));
        if ((s_hi - s_lo) / s_len < c_.side_match_forward_tol) return false;
        return true;
    };
    for (int c = 0; c < comps.num_components; ++c) {
        int best = -1;
        double best_d = 1e18;
        for (size_t e = 0; e < latch_n; ++e) {
            if (used[e]) continue;
            const FluidSideLatchEntry& en = fluid_side_latch_[e];
            const double d = (comps.centroid_world[c] - en.centroid).norm();
            if (d > c_.side_match_centroid_dist || d >= best_d) continue;
            if (!overlap_ok(en, c)) continue;
            best_d = d;
            best = static_cast<int>(e);
        }
        if (best >= 0) {
            used[best] = true;
            sides[c] = fluid_side_latch_[best].side;
        }
    }
    return sides;
}

void FluidGuidance::commitSideLatch(const fluid2d::SolidComponents& comps,
                                    const std::vector<double>& chosen_sides)
{
    if (comps.num_components == 0) return;
    const size_t latch_n = fluid_side_latch_.size();
    std::vector<bool> used(latch_n, false);
    auto overlap_ok = [&](const FluidSideLatchEntry& e, int c) {
        const double lat_lo = std::max(e.eta_min, comps.eta_min[c]);
        const double lat_hi = std::min(e.eta_max, comps.eta_max[c]);
        const double lat_len = std::max(1e-6, std::min(e.eta_max - e.eta_min,
                                                       comps.eta_max[c] - comps.eta_min[c]));
        if ((lat_hi - lat_lo) / lat_len < c_.side_match_lateral_tol) return false;
        const double s_lo = std::max(e.s_min, comps.s_min[c]);
        const double s_hi = std::min(e.s_max, comps.s_max[c]);
        const double s_len = std::max(1e-6, std::min(e.s_max - e.s_min,
                                                     comps.s_max[c] - comps.s_min[c]));
        if ((s_hi - s_lo) / s_len < c_.side_match_forward_tol) return false;
        return true;
    };
    for (int c = 0; c < comps.num_components; ++c) {
        const double side = (c < static_cast<int>(chosen_sides.size()) &&
                             std::abs(chosen_sides[c]) == 1.0)
                                ? chosen_sides[c]
                                : 0.0;
        int best = -1;
        double best_d = 1e18;
        for (size_t e = 0; e < latch_n; ++e) {
            if (used[e]) continue;
            const FluidSideLatchEntry& en = fluid_side_latch_[e];
            const double d = (comps.centroid_world[c] - en.centroid).norm();
            if (d > c_.side_match_centroid_dist || d >= best_d) continue;
            if (!overlap_ok(en, c)) continue;
            best_d = d;
            best = static_cast<int>(e);
        }
        FluidSideLatchEntry entry;
        entry.centroid = comps.centroid_world[c];
        entry.side = side;
        entry.eta_min = comps.eta_min[c];
        entry.eta_max = comps.eta_max[c];
        entry.s_min = comps.s_min[c];
        entry.s_max = comps.s_max[c];
        entry.stamp = now_sec_;
        if (best >= 0) {
            used[best] = true;
            fluid_side_latch_[best] = entry;
        } else if (side != 0.0) {
            fluid_side_latch_.push_back(entry);
        }
    }
}

// ------------------------------------------------------------------ 3D law

Eigen::Vector3d FluidGuidance::fuseLiftedGvfFluid3D(const Eigen::Vector3d& field_uvw,
                                                    bool field_valid,
                                                    double heading_rad,
                                                    double /*phi*/, double v_cap) const
{
    // 3D follows depth-derived potential flow directly: no continuous-distance
    // ray turn or speed ramp is available in this mode.
    if (!field_valid) return Eigen::Vector3d::Zero();
    const Eigen::Vector2d uh = field_uvw.head<2>();
    const bool uh_valid = field_valid && uh.squaredNorm() > 1e-12;
    const double uh_norm = uh_valid ? uh.norm() : 0.0;
    Eigen::Vector3d dir(uh_valid ? uh.x() : std::cos(heading_rad) * uh_norm,
                        uh_valid ? uh.y() : std::sin(heading_rad) * uh_norm,
                        field_valid ? field_uvw.z() : 0.0);
    if (dir.squaredNorm() < 1e-12) return Eigen::Vector3d::Zero();
    return v_cap * dir.normalized();
}

Eigen::Vector3d FluidGuidance::calcGuidance3D(const Eigen::Vector3d& pos,
                                              const Eigen::Vector2d& anchor_xy,
                                              double heading_rad, double v_J,
                                              double now_sec, bool build_viz,
                                              const std::shared_ptr<const fluid3d::DepthFrame3D>& depth_frame)
{
    now_sec_ = now_sec;
    if (!depth_frame || !fluid3d::validDepthFrame3D(*depth_frame)) {
        invalidate3DField();
        return Eigen::Vector3d::Zero();
    }

    const double v_cap = std::max(0.0, v_J);
    const Eigen::Vector2d pos_xy = pos.head<2>();
    const Eigen::Vector2d e_J(std::cos(heading_rad), std::sin(heading_rad));
    const Eigen::Vector2d n_J(-std::sin(heading_rad), std::cos(heading_rad));
    vis3d_.pos = pos;
    vis3d_.anchor_xy = anchor_xy;
    vis3d_.heading_rad = heading_rad;
    vis3d_.v_cap = v_cap;

    const double tick_dt = fluid3d_last_tick_time_ <= 0.0
        ? 0.0
        : std::max(0.0, std::min(0.1, now_sec - fluid3d_last_tick_time_));
    fluid3d_last_tick_time_ = now_sec;
    const bool due = (now_sec - fluid_last_solve_time_) >= c_.resolve_period;

    if (due && v_cap > 1e-9) {
        // A failed projection/solve must not fall back to the previous depth
        // snapshot's field.  A new field is installed only after fine succeeds.
        invalidate3DField();
        const double ustar_lat = fluid3d_cross_beta_ * v_cap;

        auto build_level3d = [&](double forward_size, double lateral_size,
                                 double hh, double hz, bool centered,
                                 const fluid3d::CoarsePrior3D* prior) {
            FluidLevel3D lv;
            lv.g.e_J = e_J;
            lv.g.n_J = n_J;
            lv.g.h = hh;
            lv.g.h_z = hz;
            lv.g.z_lo = c_.z_min;
            lv.g.n_fwd = std::max(3, static_cast<int>(std::round(forward_size / hh)));
            lv.g.n_lat = std::max(3, static_cast<int>(std::round(lateral_size / hh)));
            lv.g.n_z = std::max(3, static_cast<int>(std::round(
                (c_.z_max - c_.z_min) / hz)));
            lv.rear_margin = centered
                ? 0.5 * lv.g.n_fwd * hh
                : std::max(1.5 * hh, std::min((lv.g.n_fwd - 1.5) * hh, c_.window_rear_margin));
            lv.g.origin_xy = pos_xy - lv.rear_margin * e_J - 0.5 * lv.g.n_lat * hh * n_J;

            const double z_floor_top = c_.z_min + c_.floor_band;
            const double z_ceil_bot = lv.g.z_lo + lv.g.n_z * hz - c_.ceil_band;
            std::vector<uint8_t> band_solid(lv.g.n_z, 0);
            lv.m_lo = lv.g.n_z;
            lv.m_hi = -1;
            for (int m = 0; m < lv.g.n_z; ++m) {
                const double zc = lv.g.z_lo + (m + 0.5) * hz;
                band_solid[m] = (zc <= z_floor_top || zc >= z_ceil_bot) ? 1 : 0;
                if (!band_solid[m]) {
                    lv.m_lo = std::min(lv.m_lo, m);
                    lv.m_hi = std::max(lv.m_hi, m);
                }
            }

            const auto t0 = std::chrono::steady_clock::now();
            fluid3d::GridObservation3D projected;
            if (!fluid3d::projectDepthFrameToGrid(*depth_frame, lv.g,
                                                  c_.depth_robot_radius, c_.depth_geometry_margin,
                                                  c_.depth_surface_band, &projected)) return lv;
            lv.state = projected.state;
            lv.solid = projected.solid;
            // Keep crossflow geometry independent of coarse solve viability:
            // it describes only measured Occupied cells, never Unknown,
            // TrustedFree, or the later floor/ceiling bands.
            std::vector<uint8_t> occupied_solid(lv.g.size(), 0);
            for (int k = 0; k < lv.g.size(); ++k)
                occupied_solid[k] = lv.state[k] == fluid3d::CellState::Occupied;
            lv.blocking_comps = fluid3d::labelSolidComponents3D(
                occupied_solid, lv.g, anchor_xy, pos_xy, lv.m_lo, lv.m_hi);

            // Ponytail: crop the forward solve box to the depth-confirmed free
            // footprint.  This keeps a Dirichlet shell inside the camera view
            // instead of treating the unobserved far/lateral window faces as
            // free. The robot samples remain in the box; the explicit seed
            // below is the only bridge through the optical near-plane blind spot.
            int observed_i_lo = lv.g.n_fwd, observed_i_hi = -1;
            int observed_j_lo = lv.g.n_lat, observed_j_hi = -1;
            bool have_projected_free = false;
            for (int i = 0; i < lv.g.n_fwd; ++i)
                for (int j = 0; j < lv.g.n_lat; ++j)
                    for (int m = 0; m < lv.g.n_z; ++m) {
                        const fluid3d::CellState state = lv.state[lv.g.idx(i, j, m)];
                        if (state != fluid3d::CellState::Unknown) {
                            observed_i_lo = std::min(observed_i_lo, i); observed_i_hi = std::max(observed_i_hi, i);
                            observed_j_lo = std::min(observed_j_lo, j); observed_j_hi = std::max(observed_j_hi, j);
                        }
                        if (state == fluid3d::CellState::Free) {
                            have_projected_free = true;
                        }
                    }
            if (!have_projected_free) return lv;
            const Eigen::Vector2d rel_robot = pos_xy - lv.g.origin_xy;
            const int robot_i = static_cast<int>(std::floor(rel_robot.dot(lv.g.e_J) / lv.g.h - 0.5));
            const int robot_j = static_cast<int>(std::floor(rel_robot.dot(lv.g.n_J) / lv.g.h - 0.5));
            if (robot_i < 0 || robot_i + 1 >= lv.g.n_fwd ||
                robot_j < 0 || robot_j + 1 >= lv.g.n_lat) return lv;
            const int i_lo = std::min(observed_i_lo, robot_i);
            const int i_hi = std::max(observed_i_hi, robot_i + 1);
            const int j_lo = std::min(observed_j_lo, robot_j);
            const int j_hi = std::max(observed_j_hi, robot_j + 1);
            if (i_hi - i_lo + 1 < 3 || j_hi - j_lo + 1 < 3) return lv;
            if (i_lo != 0 || i_hi != lv.g.n_fwd - 1 || j_lo != 0 || j_hi != lv.g.n_lat - 1) {
                const fluid3d::Grid3D old_g = lv.g;
                const std::vector<fluid3d::CellState> old_state = lv.state;
                const std::vector<uint8_t> old_solid = lv.solid;
                lv.g.origin_xy += i_lo * lv.g.h * lv.g.e_J + j_lo * lv.g.h * lv.g.n_J;
                lv.g.n_fwd = i_hi - i_lo + 1;
                lv.g.n_lat = j_hi - j_lo + 1;
                lv.rear_margin -= i_lo * lv.g.h;
                lv.state.resize(lv.g.size());
                lv.solid.resize(lv.g.size());
                for (int i = 0; i < lv.g.n_fwd; ++i)
                    for (int j = 0; j < lv.g.n_lat; ++j)
                        for (int m = 0; m < lv.g.n_z; ++m) {
                            const int old_k = old_g.idx(i + i_lo, j + j_lo, m);
                            const int new_k = lv.g.idx(i, j, m);
                            lv.state[new_k] = old_state[old_k];
                            lv.solid[new_k] = old_solid[old_k];
                        }
            }
            for (int i = 0; i < lv.g.n_fwd; ++i)
                for (int j = 0; j < lv.g.n_lat; ++j)
                    for (int m = 0; m < lv.g.n_z; ++m)
                        if (band_solid[m]) lv.solid[lv.g.idx(i, j, m)] = 1;
            for (int i = 0; i < lv.g.n_fwd; ++i)
                for (int j = 0; j < lv.g.n_lat; ++j)
                    for (int m = 0; m < lv.g.n_z; ++m) {
                        const int k = lv.g.idx(i, j, m);
                        if (!band_solid[m] && lv.state[k] == fluid3d::CellState::Unknown &&
                            (lv.g.cellWorld(i, j, m) - pos).norm() <= c_.depth_ego_seed_radius) {
                            lv.state[k] = fluid3d::CellState::TrustedFree;
                            lv.solid[k] = 0;
                        }
                    }
            const Eigen::Vector2d cropped_rel_robot = pos_xy - lv.g.origin_xy;
            const int seed_i = static_cast<int>(std::floor(
                cropped_rel_robot.dot(lv.g.e_J) / lv.g.h - 0.5));
            const int seed_j = static_cast<int>(std::floor(
                cropped_rel_robot.dot(lv.g.n_J) / lv.g.h - 0.5));
            const int seed_m = static_cast<int>(std::floor(
                (pos.z() - lv.g.z_lo) / lv.g.h_z - 0.5));
            if (seed_i < 0 || seed_i + 1 >= lv.g.n_fwd || seed_j < 0 ||
                seed_j + 1 >= lv.g.n_lat || seed_m < 0 || seed_m + 1 >= lv.g.n_z) return lv;
            for (int di = 0; di < 2; ++di)
                for (int dj = 0; dj < 2; ++dj)
                    for (int dm = 0; dm < 2; ++dm) {
                        const int k = lv.g.idx(seed_i + di, seed_j + dj, seed_m + dm);
                        if (lv.solid[k]) return lv;  // robot cannot sample this field.
                    }
            const int di6[] = {-1, 1, 0, 0, 0, 0};
            const int dj6[] = {0, 0, -1, 1, 0, 0};
            const int dm6[] = {0, 0, 0, 0, -1, 1};

            const auto seed_reaches_observed_free = [&]() {
                std::queue<int> seed_queue;
                std::vector<uint8_t> seed_seen(lv.g.size(), 0);
                for (int di = 0; di < 2; ++di)
                    for (int dj = 0; dj < 2; ++dj)
                        for (int dm = 0; dm < 2; ++dm) {
                            const int k = lv.g.idx(seed_i + di, seed_j + dj, seed_m + dm);
                            if (lv.solid[k]) return false;
                            seed_seen[k] = 1;
                            seed_queue.push(k);
                        }
                while (!seed_queue.empty()) {
                    const int k = seed_queue.front(); seed_queue.pop();
                    if (lv.state[k] == fluid3d::CellState::Free) return true;
                    const int i = k / (lv.g.n_lat * lv.g.n_z);
                    const int rem = k % (lv.g.n_lat * lv.g.n_z);
                    const int j = rem / lv.g.n_z;
                    const int m = rem % lv.g.n_z;
                    for (int q = 0; q < 6; ++q) {
                        const int ni = i + di6[q], nj = j + dj6[q], nm = m + dm6[q];
                        if (ni < 0 || nj < 0 || nm < 0 || ni >= lv.g.n_fwd ||
                            nj >= lv.g.n_lat || nm >= lv.g.n_z) continue;
                        const int nk = lv.g.idx(ni, nj, nm);
                        if (!lv.solid[nk] && !seed_seen[nk]) {
                            seed_seen[nk] = 1;
                            seed_queue.push(nk);
                        }
                    }
                }
                return false;
            };
            if (!seed_reaches_observed_free()) {
                const Eigen::Vector3d camera_position = depth_frame->T_world_camera.translation();
                const Eigen::Matrix3d R_camera_world = depth_frame->T_world_camera.linear().transpose();
                Eigen::Vector3d optical_forward = depth_frame->T_world_camera.linear() *
                    Eigen::Vector3d::UnitZ();
                if (!camera_position.allFinite() || !R_camera_world.allFinite() ||
                    !optical_forward.allFinite() || optical_forward.squaredNorm() < 1e-12) return lv;
                optical_forward.normalize();
                const double capsule_radius = 0.5 * std::sqrt(
                    2.0 * lv.g.h * lv.g.h + lv.g.h_z * lv.g.h_z);
                const auto center_depth_is_free = [&](const Eigen::Vector3d& p) {
                    const Eigen::Vector3d p_camera = R_camera_world * (p - camera_position);
                    if (!p_camera.allFinite() || p_camera.z() <= 0.0) return false;
                    const double u = depth_frame->fx * p_camera.x() / p_camera.z() + depth_frame->cx;
                    const double v = depth_frame->fy * p_camera.y() / p_camera.z() + depth_frame->cy;
                    if (!std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0 ||
                        u > depth_frame->width - 1 || v > depth_frame->height - 1) return false;
                    const int ui = static_cast<int>(std::lround(u));
                    const int vi = static_cast<int>(std::lround(v));
                    const float depth = depth_frame->depth_m[static_cast<size_t>(vi) *
                        depth_frame->width + ui];
                    const double clear_depth = depth == 0.0f
                        ? depth_frame->max_depth : static_cast<double>(depth);
                    return std::isfinite(clear_depth) &&
                        (depth == 0.0f || (clear_depth >= depth_frame->min_depth &&
                                          clear_depth <= depth_frame->max_depth)) &&
                        clear_depth > p_camera.z() + c_.depth_robot_radius +
                            c_.depth_geometry_margin + c_.depth_surface_band;
                };
                int target_k = -1;
                double target_axial = std::numeric_limits<double>::infinity();
                for (int i = 0; i < lv.g.n_fwd; ++i)
                    for (int j = 0; j < lv.g.n_lat; ++j)
                        for (int m = 0; m < lv.g.n_z; ++m) {
                            const int k = lv.g.idx(i, j, m);
                            if (lv.state[k] != fluid3d::CellState::Free) continue;
                            const Eigen::Vector3d delta = lv.g.cellWorld(i, j, m) - pos;
                            const double axial = delta.dot(optical_forward);
                            if (axial <= c_.depth_ego_seed_radius || axial >= target_axial ||
                                (delta - axial * optical_forward).norm() > capsule_radius) continue;
                            target_axial = axial;
                            target_k = k;
                        }
                if (target_k < 0) return lv;
                const int target_i = target_k / (lv.g.n_lat * lv.g.n_z);
                const int target_rem = target_k % (lv.g.n_lat * lv.g.n_z);
                const int target_j = target_rem / lv.g.n_z;
                const int target_m = target_rem % lv.g.n_z;
                const Eigen::Vector3d capsule_end = lv.g.cellWorld(target_i, target_j, target_m);
                const Eigen::Vector3d capsule_delta = capsule_end - pos;
                const double capsule_len2 = capsule_delta.squaredNorm();
                for (int i = 0; i < lv.g.n_fwd; ++i)
                    for (int j = 0; j < lv.g.n_lat; ++j)
                        for (int m = 0; m < lv.g.n_z; ++m) {
                            const int k = lv.g.idx(i, j, m);
                            const Eigen::Vector3d to_cell = lv.g.cellWorld(i, j, m) - pos;
                            const double t = std::max(0.0, std::min(1.0,
                                to_cell.dot(capsule_delta) / capsule_len2));
                            if ((to_cell - t * capsule_delta).norm() > capsule_radius ||
                                to_cell.norm() <= c_.depth_ego_seed_radius) continue;
                            if (lv.state[k] == fluid3d::CellState::Occupied || band_solid[m] ||
                                (lv.state[k] == fluid3d::CellState::Unknown &&
                                 !center_depth_is_free(lv.g.cellWorld(i, j, m)))) return lv;
                            if (lv.state[k] == fluid3d::CellState::Unknown) {
                                lv.state[k] = fluid3d::CellState::TrustedFree;
                                lv.solid[k] = 0;
                            }
                        }
            }
            if (!seed_reaches_observed_free()) return lv;
            bool has_free_boundary = false;
            for (int i = 0; i < lv.g.n_fwd; ++i)
                for (int j = 0; j < lv.g.n_lat; ++j)
                    for (int m = 0; m < lv.g.n_z; ++m) {
                        const bool edge = i == 0 || j == 0 || m == 0 || i == lv.g.n_fwd - 1 ||
                                          j == lv.g.n_lat - 1 || m == lv.g.n_z - 1;
                        has_free_boundary = has_free_boundary || (edge && !lv.solid[lv.g.idx(i,j,m)]);
                    }
            if (!has_free_boundary) return lv;
            const auto t1 = std::chrono::steady_clock::now();
            fluid3d::foldUnreachablePockets3D(lv.solid, lv.g);

            const auto t2 = std::chrono::steady_clock::now();
            lv.comps = fluid3d::labelSolidComponents3D(
                lv.solid, lv.g, anchor_xy, pos_xy, lv.m_lo, lv.m_hi);
            const auto t3 = std::chrono::steady_clock::now();
            lv.field = fluid3d::solvePotentialFlow3D(
                lv.solid, lv.g, v_cap, ustar_lat, prior,
                c_.cg_tolerance, c_.cg_max_iterations,
                c_.cg_residual_acceptance);
            const auto t4 = std::chrono::steady_clock::now();

            lv.ms_project = wallMs(t0, t1);
            lv.ms_mask  = wallMs(t1, t2);
            lv.ms_label = wallMs(t2, t3);
            lv.ms_field = wallMs(t3, t4);
            return lv;
        };

        const FluidLevel3D coarse = build_level3d(
            c_.window_forward_size, c_.window_lateral_size,
            c_.coarse_grid_resolution, c_.coarse_resolution_z,
            /*centered=*/false, nullptr);
        fluid3d::CoarsePrior3D prior;
        if (coarse.field.valid) {
            prior.phi = &coarse.field.phi;
            prior.grid = coarse.g;
        }
        const FluidLevel3D fine = build_level3d(
            c_.fine_forward_size, c_.fine_lateral_size,
            c_.fine_resolution, c_.grid_resolution_z,
            /*centered=*/false, coarse.field.valid ? &prior : nullptr);

        if (!fine.field.valid) {
            fluid_last_solve_time_ = now_sec;
            return Eigen::Vector3d::Zero();
        }
        fluid3d_field_grid_ = fine.g;
        fluid3d_Ux_ = fine.field.Ux;
        fluid3d_Uy_ = fine.field.Uy;
        fluid3d_Uz_ = fine.field.Uz;
        fluid3d_phi_ = fine.field.phi;
        fluid3d_field_valid_ = true;
        fluid3d_state_ = fine.state;
        fluid3d_solid_ = fine.solid;
        fluid3d_field_data_valid_ = true;
        fluid3d_field_solve_speed_ = v_cap;
        updateStreamline3D(pos, fine.g, v_cap);

        fluid3d_coarse_extents_.clear();
        for (int c = 0; c < coarse.blocking_comps.num_components; ++c) {
            fluid3d_coarse_extents_.push_back({coarse.blocking_comps.eta_min[c],
                                               coarse.blocking_comps.eta_max[c],
                                               coarse.blocking_comps.s_min[c],
                                               coarse.blocking_comps.z_max[c]});
        }

        fluid_last_solve_time_ = now_sec;

        // Visualization snapshot: expose the solved fine/coarse fields so the
        // ROS adapter can build the quiver frame (subscriber-gated upstream).
        if (build_viz) {
            vis3d_.valid = true;
            vis3d_.grid = fine.g;
            vis3d_.solid = fine.solid;
            vis3d_.state = fine.state;
            vis3d_.field = fine.field;
            vis3d_.coarse_valid = coarse.field.valid;
            if (coarse.field.valid) {
                vis3d_.grid_coarse = coarse.g;
                vis3d_.solid_coarse = coarse.solid;
                vis3d_.field_coarse = coarse.field;
            }
        }
    } else if (due) {
        fluid_last_solve_time_ = now_sec;
    }

    // Per-tick command: trilinearly sample the cached field at the live 3D pos.
    Eigen::Vector3d field_uvw = Eigen::Vector3d::Zero();
    bool field_sample_valid = false;
    if (fluid3d_field_valid_) {
        const fluid3d::Grid3D& g = fluid3d_field_grid_;
        const Eigen::Vector2d rel = pos.head<2>() - g.origin_xy;
        const int i0 = static_cast<int>(std::floor(rel.dot(g.e_J) / g.h - 0.5));
        const int j0 = static_cast<int>(std::floor(rel.dot(g.n_J) / g.h - 0.5));
        const int m0 = static_cast<int>(std::floor((pos.z() - g.z_lo) / g.h_z - 0.5));
        bool passable = i0 >= 0 && j0 >= 0 && m0 >= 0 &&
            i0 + 1 < g.n_fwd && j0 + 1 < g.n_lat && m0 + 1 < g.n_z;
        for (int di = 0; passable && di < 2; ++di)
            for (int dj = 0; passable && dj < 2; ++dj)
                for (int dm = 0; dm < 2; ++dm)
                    passable = fluid3d_solid_[g.idx(i0 + di, j0 + dj, m0 + dm)] == 0;
        if (passable) field_sample_valid = fluid3d::trilinearSampleField3D(
            fluid3d_Ux_, fluid3d_Uy_, fluid3d_Uz_, g, pos, field_uvw);
    }
    if (field_sample_valid && field_uvw.squaredNorm() > 1e-12) {
        fluid_diag_field_direction_ = field_uvw.normalized();
        fluid_diag_field_valid_ = true;
    } else {
        fluid_diag_field_direction_.setZero();
        fluid_diag_field_valid_ = false;
    }
    fluid_diag_potential_valid_ = false;
    if (fluid3d_field_data_valid_ && !fluid3d_phi_.empty()) {
        const Eigen::Vector2d rel = pos_xy - fluid3d_field_grid_.origin_xy;
        const double gi = rel.dot(fluid3d_field_grid_.e_J) /
                          fluid3d_field_grid_.h - 0.5;
        const double gj = rel.dot(fluid3d_field_grid_.n_J) /
                          fluid3d_field_grid_.h - 0.5;
        const double gm = (pos.z() - fluid3d_field_grid_.z_lo) /
                          fluid3d_field_grid_.h_z - 0.5;
        const int i0 = static_cast<int>(std::floor(gi));
        const int j0 = static_cast<int>(std::floor(gj));
        const int m0 = static_cast<int>(std::floor(gm));
        if (i0 >= 0 && j0 >= 0 && m0 >= 0 &&
            i0 + 1 < fluid3d_field_grid_.n_fwd &&
            j0 + 1 < fluid3d_field_grid_.n_lat &&
            m0 + 1 < fluid3d_field_grid_.n_z) {
            const double tx = gi - i0, ty = gj - j0, tz = gm - m0;
            auto at = [&](int i, int j, int m) {
                return fluid3d_phi_[fluid3d_field_grid_.idx(i, j, m)];
            };
            auto lerp3 = [&](int i, int j) {
                const double a = at(i, j, m0) * (1.0 - tz) + at(i, j, m0 + 1) * tz;
                const double b = at(i, j + 1, m0) * (1.0 - tz) + at(i, j + 1, m0 + 1) * tz;
                return a * (1.0 - ty) + b * ty;
            };
            const double a = lerp3(i0, j0);
            const double b = lerp3(i0 + 1, j0);
            fluid_diag_potential_ = a * (1.0 - tx) + b * tx;
            fluid_diag_potential_valid_ = std::isfinite(fluid_diag_potential_);
        }
    }
    if (field_sample_valid && field_uvw.squaredNorm() > 1e-12) {
        fluid3d_cached_t_field_ = field_uvw.normalized();
        fluid3d_cached_t_field_valid_ = true;
    }
    if (!field_sample_valid) {
        fluid3d_track_mode_ = 2;
        fluid3d_d_v_current_ = std::numeric_limits<double>::quiet_NaN();
        return Eigen::Vector3d::Zero();
    }

    const double phi_here = (pos_xy - anchor_xy).dot(n_J);
    const bool streamline_ok = field_sample_valid && streamline_view_.valid &&
                               fluid3d_field_data_valid_ &&
                               fluid3d_field_valid_;
    Eigen::Vector3d v_nominal = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_raw;
    if (streamline_ok) {
        v_nominal = calcStreamlineGuidance3D(pos, v_cap,
                                             field_sample_valid ? field_uvw.z() : 0.0,
                                             heading_rad);
        u_raw = v_nominal;
        fluid3d_track_mode_ = 0;
    } else {
        u_raw = fuseLiftedGvfFluid3D(
            field_uvw, field_sample_valid, heading_rad, phi_here, v_cap);
        v_nominal = u_raw;
        fluid3d_track_mode_ = 2;
    }
    setDiagnosticDirection(v_nominal);

    // The joystick cap is a PLANAR authority limit; z is capped separately by
    // the manager's actuator clamp before publish.
    if (v_cap > 1e-9) {
        const double horizontal_speed = u_raw.head<2>().norm();
        if (horizontal_speed > v_cap) {
            u_raw.head<2>() *= v_cap / horizontal_speed;
        }
    } else {
        u_raw.setZero();
    }


    // Record the SAFETY-layer modification d_v = v_final - v_nominal.
    {
        const Eigen::Vector3d d_v = u_raw - v_nominal;
        const double d_norm = d_v.norm();
        fluid3d_d_v_observed_ = std::max(fluid3d_d_v_observed_, d_norm);
        fluid3d_d_v_current_ = d_norm;
        if (fluid3d_track_mode_ == 0 && d_norm > 0.05) {
            fluid3d_track_mode_ = 1;
        }
    }

    // Crossflow latch update, last: it consumes this tick's field sample and
    // only influences the NEXT solve's u*.
    if (v_cap > 1e-9) {
        if (!fluid3d_cross_latched_) {
            const bool stalled = field_sample_valid &&
                field_uvw.norm() < c_.stall_speed_ratio * v_cap &&
                std::abs(field_uvw.z()) < c_.stall_w_veto;
            const double eta_robot = (pos_xy - anchor_xy).dot(n_J);
            const CompExtent* blocking_component = nullptr;
            for (const auto& c : fluid3d_coarse_extents_) {
                if (c.s_min < 0.0 || c.s_min > c_.stall_probe + 1.0 ||
                    eta_robot < c.eta_min - c_.side_deadband ||
                    eta_robot > c.eta_max + c_.side_deadband) continue;
                if (!blocking_component || c.s_min < blocking_component->s_min)
                    blocking_component = &c;
            }
            const bool blocked = stalled && blocking_component;
            fluid3d_stall_accum_ = (stalled && blocked)
                ? fluid3d_stall_accum_ + tick_dt : 0.0;
            if (fluid3d_stall_accum_ >= c_.stall_latch_time) {
                double side = c_.default_bias_sign;
                if (blocking_component) {
                    const double cost_plus = std::max(0.0, blocking_component->eta_max - eta_robot);
                    const double cost_minus = std::max(0.0, eta_robot - blocking_component->eta_min);
                    if (std::abs(cost_plus - cost_minus) < c_.side_deadband) {
                        side = c_.default_bias_sign;
                    } else {
                        side = (cost_plus <= cost_minus) ? 1.0 : -1.0;
                    }
                }
                fluid3d_cross_side_ = side;
                fluid3d_cross_latched_ = true;
                fluid3d_release_accum_ = 0.0;
            }
        } else {
            const bool recovered = field_sample_valid &&
                field_uvw.norm() >= c_.stall_release_ratio * v_cap;
            fluid3d_release_accum_ = recovered
                ? fluid3d_release_accum_ + tick_dt : 0.0;
            if (fluid3d_release_accum_ >= c_.stall_release_time) {
                fluid3d_cross_latched_ = false;
                fluid3d_stall_accum_ = 0.0;
            }
        }
        const double beta_des = (fluid3d_cross_latched_ ? fluid3d_cross_side_ : 0.0) *
                                c_.crossflow_ratio;
        const double beta_err = beta_des - fluid3d_cross_beta_;
        const double beta_rate = std::max(-c_.crossflow_rate_max,
            std::min(c_.crossflow_rate_max,
                     beta_err / std::max(1e-6, c_.crossflow_tau)));
        fluid3d_cross_beta_ += beta_rate * tick_dt;
    }

    return u_raw;
}

// ---- 3D local reference streamline ----

void FluidGuidance::updateStreamline3D(const Eigen::Vector3d& pos,
                                       const fluid3d::Grid3D& g, double /*v_cap*/)
{
    Streamline3DView next;
    if (!fluid3d_field_valid_ || !fluid3d_field_data_valid_) {
        streamline_view_.valid = false;
        return;
    }
    if (c_.freeze_streamline && streamline_view_.valid &&
        streamline_view_.points.size() >= 2) {
        return;
    }
    if (now_sec_ < fluid3d_streamline_degrade_until_) {
        streamline_view_.valid = false;
        return;
    }
    const double ds = std::max(1e-3, c_.streamline_ds);
    const double u_min = 0.02 * std::max(0.1, fluid3d_field_solve_speed_);
    Eigen::Vector3d seed = pos;
    if (streamline_view_.valid && streamline_view_.points.size() >= 2) {
        const StreamlineProj3D proj = projectToStreamline3D(pos);
        seed = proj.p;
    }

    auto solid_at = [&](const Eigen::Vector3d& p) {
        const Eigen::Vector2d rel = p.head<2>() - g.origin_xy;
        const int i = std::max(0, std::min(g.n_fwd - 1,
            static_cast<int>(std::floor(rel.dot(g.e_J) / g.h))));
        const int j = std::max(0, std::min(g.n_lat - 1,
            static_cast<int>(std::floor(rel.dot(g.n_J) / g.h))));
        const int m = std::max(0, std::min(g.n_z - 1,
            static_cast<int>(std::floor((p.z() - g.z_lo) / g.h_z))));
        return fluid3d_solid_[g.idx(i, j, m)] != 0;
    };
    auto sample_u = [&](const Eigen::Vector3d& p, Eigen::Vector3d& u) {
        return fluid3d::trilinearSampleField3D(
            fluid3d_Ux_, fluid3d_Uy_, fluid3d_Uz_, g, p, u);
    };
    auto step = [&](const Eigen::Vector3d& p, double dir, Eigen::Vector3d& p_next) {
        Eigen::Vector3d u, u_mid;
        if (!sample_u(p, u) || solid_at(p)) return false;
        const double un = u.norm();
        if (un < u_min) return false;
        const Eigen::Vector3d p_mid = p + 0.5 * ds * dir * (u / un);
        if (solid_at(p_mid)) return false;
        if (!sample_u(p_mid, u_mid)) return false;
        const double unm = u_mid.norm();
        if (unm < u_min) return false;
        p_next = p + ds * dir * (u_mid / unm);
        return !solid_at(p_next);
    };

    std::vector<Eigen::Vector3d> pts, tans;
    pts.push_back(seed);
    {
        Eigen::Vector3d u0;
        Eigen::Vector3d t0 = (sample_u(seed, u0) && u0.norm() > u_min)
                                 ? u0.normalized()
                                 : Eigen::Vector3d::UnitX();
        tans.push_back(t0);
    }
    {
        Eigen::Vector3d p = seed;
        double s = 0.0;
        while (s < c_.streamline_bwd_len) {
            Eigen::Vector3d p_next;
            if (!step(p, -1.0, p_next)) break;
            pts.insert(pts.begin(), p_next);
            tans.insert(tans.begin(), p_next - p);
            p = p_next;
            s += ds;
        }
    }
    {
        Eigen::Vector3d p = seed;
        double s = 0.0;
        while (s < c_.streamline_fwd_len) {
            Eigen::Vector3d p_next;
            if (!step(p, 1.0, p_next)) break;
            pts.push_back(p_next);
            tans.push_back(p_next - p);
            p = p_next;
            s += ds;
        }
    }
    for (size_t k = 0; k < tans.size(); ++k) {
        if (tans[k].norm() > 1e-6) tans[k].normalize();
    }
    if (pts.size() < 2) {
        streamline_view_.valid = false;
        return;
    }

    double max_disp = 0.0;
    if (streamline_view_.valid && streamline_view_.points.size() >= 2) {
        const Streamline3DView& old = streamline_view_;
        for (const Eigen::Vector3d& p : pts) {
            double best = 1e18;
            for (size_t k = 0; k + 1 < old.points.size(); ++k) {
                const Eigen::Vector3d d = old.points[k + 1] - old.points[k];
                const double L2 = d.squaredNorm();
                const double s = (L2 > 1e-12)
                    ? std::min(1.0, std::max(0.0, (p - old.points[k]).dot(d) / L2))
                    : 0.0;
                best = std::min(best, (p - (old.points[k] + s * d)).squaredNorm());
            }
            max_disp = std::max(max_disp, std::sqrt(best));
        }
        if (max_disp > c_.streamline_max_disp) {
            streamline_view_.valid = false;
            fluid3d_streamline_degrade_until_ =
                now_sec_ + c_.streamline_degrade_cooldown;
            return;
        }
    }

    next.arc.resize(pts.size(), 0.0);
    for (size_t k = 1; k < pts.size(); ++k) {
        const Eigen::Vector3d d = pts[k] - pts[k - 1];
        next.arc[k] = next.arc[k - 1] + (k > 0 && pts[k] == pts[k - 1] ? 0.0 : d.norm());
    }
    const size_t seed_k = std::distance(pts.begin(), std::find(pts.begin(), pts.end(), seed));
    for (double& a : next.arc) a -= next.arc[seed_k];
    next.points = pts;
    next.tangents = tans;
    next.valid = true;
    next.end_cap = false;
    streamline_view_ = next;
    fluid3d_streamline_update_disp_ = max_disp;
    ++fluid3d_streamline_version_;
    fluid3d_streamline_clearance_min_ = std::numeric_limits<double>::quiet_NaN();
}

FluidGuidance::StreamlineProj3D FluidGuidance::projectToStreamline3D(
    const Eigen::Vector3d& x) const
{
    StreamlineProj3D out;
    const Streamline3DView& sl = streamline_view_;
    if (!sl.valid || sl.points.size() < 2) return out;
    double best_dist2 = 1e18;
    int best_k = 0;
    double best_s = 0.0;
    Eigen::Vector3d best_p = sl.points.front();
    Eigen::Vector3d best_t = sl.tangents.empty() ? Eigen::Vector3d::UnitX()
                                                 : sl.tangents.front();
    for (size_t k = 0; k + 1 < sl.points.size(); ++k) {
        const Eigen::Vector3d d = sl.points[k + 1] - sl.points[k];
        const double L2 = d.squaredNorm();
        const double s = (L2 > 1e-12)
            ? std::min(1.0, std::max(0.0, (x - sl.points[k]).dot(d) / L2))
            : 0.0;
        const Eigen::Vector3d cand = sl.points[k] + s * d;
        const double dist2 = (x - cand).squaredNorm();
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            best_k = static_cast<int>(k);
            best_s = s;
            best_p = cand;
            if (!sl.tangents.empty()) {
                best_t = sl.tangents[k] * (1.0 - s) + sl.tangents[k + 1] * s;
                if (best_t.norm() > 1e-9) best_t.normalize();
            }
        }
    }
    out.s = best_s;
    out.p = best_p;
    out.t = best_t;
    out.end_cap = (best_k == 0) || (best_k == static_cast<int>(sl.points.size()) - 2);
    return out;
}

Eigen::Vector3d FluidGuidance::calcStreamlineGuidance3D(const Eigen::Vector3d& pos,
                                                        double v_cap,
                                                        double w_robot,
                                                        double heading_rad)
{
    const StreamlineProj3D proj = projectToStreamline3D(pos);
    streamline_view_.end_cap = proj.end_cap;
    const double v_t = v_cap;
    const Eigen::Vector3d e_perp = pos - proj.p;
    const Eigen::Vector2d t_xy(proj.t.x(), proj.t.y());
    const double t_n = t_xy.norm();
    Eigen::Vector2d dir;
    if (t_n > 1e-6) {
        dir = t_xy / t_n;
    } else {
        dir = Eigen::Vector2d(std::cos(heading_rad), std::sin(heading_rad));
    }
    return Eigen::Vector3d(
        v_t * dir.x() - c_.k_n * e_perp.x(),
        v_t * dir.y() - c_.k_n * e_perp.y(),
        w_robot - (c_.k_n * c_.k_n_z_ratio) * e_perp.z());
}

}  // namespace fluid
}  // namespace FLAG_Race
