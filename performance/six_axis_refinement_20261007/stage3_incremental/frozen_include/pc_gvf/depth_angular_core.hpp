#ifndef PC_GVF_DEPTH_ANGULAR_CORE_HPP_
#define PC_GVF_DEPTH_ANGULAR_CORE_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>

namespace pc_gvf {
namespace depth_angular {

constexpr double kEpsilon = 1.0e-9;

using BinaryMask = std::vector<std::uint8_t>;

// Exact Euclidean pixel clearance in O(width*height). Nonzero cells are blocked;
// an all-free mask preserves the legacy hypot(x, y+1) boundary convention.
std::vector<double> euclideanDistanceToBlocked(
    const BinaryMask& blocked_mask, int width, int height);

void applyObstacleReleaseHysteresis(
    const BinaryMask& observed_mask,
    int clear_frames,
    bool advance_frame,
    BinaryMask* stable_mask,
    std::vector<std::uint8_t>* clear_counts);

struct SimConfig
{
    double body_radius = 0.25;
    double safety_margin = 0.12;
    double reference_speed = 1.2;
    double max_accel = 3.0;
    double brake_accel = 2.5;
    double velocity_tau = 0.22;
    double delay = 0.10;
    double planning_horizon = 0.90;
    double control_dt = 0.05;
    double dynamics_dt = 0.01;
    double rollout_dt = 0.05;
    double rollout_horizon = 0.80;
    double rollout_margin = 0.10;
    double max_direction_rate = 1.60;
    bool horizontal_only = false;
    bool continuous_harmonic_guidance = false;
    bool recapture_reference = false;
    bool angular_goal_cost = false;
    bool subpixel_goal = false;
    bool finite_goal = false;
    int harmonic_gradient_radius_cells = 3;
    double harmonic_gradient_lookahead_cells = 2.5;
    double goal_tolerance = 0.28;
    double clearance_reward = 0.018;
    // Proposal-only reward cap in angular pixels; zero retains legacy cost.
    double goal_clearance_cap = 0.;
    double goal_projection_clearance = 0.;
    double hysteresis_weight = 0.20;
    double deterministic_left_bias = 0.012;
    double convergence_distance = 2.5;
    double convergence_margin = 1.0;
    double convergence_length = 1.5;
    double convergence_max_angle = 0.6;
    int source_radius_cells = 1;
    int goal_radius_cells = 1;
    double field_tolerance = 1.0e-7;
    int field_max_iterations = 1200;
    int depth_point_stride = 2;
    int cone_chunk_size = 256;
};

Eigen::Vector3d normalize(
    const Eigen::Vector3d& value,
    const Eigen::Vector3d* fallback = nullptr);

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double limit);

std::size_t selectClosestYaw(
    double desired_yaw,
    const std::vector<double>& camera_yaws);

Eigen::Matrix3d fixedCameraRotation();

bool quaternionMatrix(
    double x, double y, double z, double w, Eigen::Matrix3d* rotation);

class Camera
{
public:
    Camera(
        int width = 48,
        int height = 36,
        double horizontal_fov_deg = 90.0,
        double vertical_fov_deg = 68.0,
        double max_depth = 8.0);

    void setIntrinsics(double fx, double fy, double cx, double cy);

    Eigen::Vector3d rayFromPixel(const Eigen::Vector2d& pixel) const;

    bool pixelFromDirection(
        const Eigen::Vector3d& direction_camera,
        Eigen::Vector2d* pixel) const;

    bool inside(const Eigen::Vector2d& pixel, double margin = 0.0) const;

    const Eigen::Vector3d& ray(int u, int v) const;
    const std::vector<Eigen::Vector3d>& rays() const { return rays_camera_; }

    int width() const { return width_; }
    int height() const { return height_; }
    double horizontalFovDeg() const { return horizontal_fov_deg_; }
    double verticalFovDeg() const { return vertical_fov_deg_; }
    double maxDepth() const { return max_depth_; }
    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }

private:
    void rebuildRays();

    int width_;
    int height_;
    double horizontal_fov_deg_;
    double vertical_fov_deg_;
    double max_depth_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    std::vector<Eigen::Vector3d> rays_camera_;
};

std::vector<Eigen::Vector3d> backprojectObstaclePoints(
    const std::vector<double>& depth,
    const Camera& camera,
    int stride);

std::vector<double> collisionConeFreeDistance(
    const std::vector<Eigen::Vector3d>& points_camera,
    const Camera& camera,
    double effective_radius,
    int chunk_size);

bool nearestFreePixel(
    const Eigen::Vector2d& query,
    const BinaryMask& free_mask,
    int width,
    int height,
    const Eigen::Vector2d* history,
    double left_bias,
    Eigen::Vector2d* result);

std::vector<int> labelFreeComponents(
    const BinaryMask& free_mask,
    int width,
    int height,
    int* component_count = nullptr);

struct GoalSelection
{
    bool valid = false;
    Eigen::Vector2d source = Eigen::Vector2d::Zero();
    Eigen::Vector2d goal = Eigen::Vector2d::Zero();
    std::vector<int> labels;
};

GoalSelection chooseSafeGoal(
    const Eigen::Vector2d& reference,
    const Eigen::Vector2d& source,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d* previous_goal,
    const SimConfig& config,
    const Camera* angular_camera = nullptr);

BinaryMask diskMask(
    int width,
    int height,
    const Eigen::Vector2d& center,
    int radius);

struct HarmonicSolution
{
    std::vector<double> potential;
    bool valid = false;
};

HarmonicSolution solveAngularHarmonic(
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source,
    const Eigen::Vector2d& goal,
    const SimConfig& config,
    const std::vector<double>* initial_potential = nullptr);

double bilinearSample(
    const std::vector<double>& image,
    int width,
    int height,
    const Eigen::Vector2d& pixel,
    double default_value);

std::vector<Eigen::Vector2d> discreteHarmonicPath(
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source,
    const Eigen::Vector2d& goal,
    int max_steps = 500);

Eigen::Vector2d continuousHarmonicTarget(
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source,
    const Eigen::Vector2d& goal,
    int gradient_radius_cells,
    double lookahead_cells);

double angularDistance(
    const Camera& camera,
    const Eigen::Vector2d& first,
    const Eigen::Vector2d& second);

Eigen::Vector2d angularRateLimit(
    const Camera& camera,
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to,
    double max_angle);

Eigen::Vector3d referenceConvergedDirection(
    const Eigen::Vector3d& field_direction_world,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& reference_origin_world,
    const Eigen::Vector3d& reference_direction_world,
    double observed_clearance,
    const SimConfig& config);

Eigen::Vector2d selectCommandDirection(
    const Camera& camera,
    const Eigen::Vector2d& previous,
    const Eigen::Vector2d& safe_source,
    const Eigen::Vector2d& goal,
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    const SimConfig& config);

double brakingSpeed(
    double distance,
    double reference_speed,
    const SimConfig& config);

bool rolloutIsSafe(
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const Eigen::Vector3d& command_world,
    const std::vector<Eigen::Vector3d>& points_world,
    const Eigen::Vector3d& camera_position_world,
    const Eigen::Matrix3d& rotation_camera_from_world,
    const Camera& camera,
    const SimConfig& config);

double depthRolloutLimit(
    double desired_speed,
    const Eigen::Vector3d& ray_world,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const std::vector<Eigen::Vector3d>& points_world,
    const Eigen::Matrix3d& rotation_camera_from_world,
    const Camera& camera,
    const SimConfig& config);

struct AngularSolution
{
    std::vector<double> depth;
    std::vector<double> free_distance;
    BinaryMask planning_mask;
    std::vector<double> potential;
    Eigen::Vector2d reference_pixel = Eigen::Vector2d::Zero();
    Eigen::Vector2d source_pixel = Eigen::Vector2d::Zero();
    Eigen::Vector2d goal_pixel = Eigen::Vector2d::Zero();
    Eigen::Vector2d command_pixel = Eigen::Vector2d::Zero();
    bool field_valid = false;
};

struct GuidanceResult
{
    Eigen::Vector3d command_world = Eigen::Vector3d::Zero();
    Eigen::Vector2d returned_goal_pixel = Eigen::Vector2d::Zero();
    double selected_free_distance = 0.0;
    double braking_limited_speed = 0.0;
    double rollout_limited_speed = 0.0;
    bool goal_reanchored = false;
    bool goal_valid = false;
    bool safety_limited = false;
    bool emergency_stop = false;
    AngularSolution solution;
};

// direction_proposal_only omits the legacy speed/rollout limiter. Its output
// is an UNVALIDATED proposal and requires an external full motion certificate.
GuidanceResult computeGuidance(
    const std::vector<double>& depth,
    const Camera& camera,
    const SimConfig& config,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const Eigen::Vector3d& goal_world,
    const Eigen::Vector2d& previous_pixel,
    const Eigen::Vector2d* previous_goal_pixel,
    const Eigen::Matrix3d& rotation_world_from_camera,
    const Eigen::Vector3d* reference_origin_world = nullptr,
    const Eigen::Vector3d* reference_direction_world = nullptr,
    BinaryMask* planning_mask_state = nullptr,
    std::vector<std::uint8_t>* planning_clear_counts = nullptr,
    int obstacle_clear_frames = 1,
    bool advance_obstacle_frame = true,
    bool retain_previous_goal = false,
    const std::vector<double>* initial_potential = nullptr,
    const std::vector<Eigen::Vector3d>* native_points_camera = nullptr,
    bool direction_proposal_only = false,
    const std::function<bool(const Eigen::Vector2d&)>& goal_admissible = {},
    bool target_proposal_only = false,
    const Eigen::Vector3d* camera_origin_world = nullptr);

}  // namespace depth_angular
}  // namespace pc_gvf

#endif  // PC_GVF_DEPTH_ANGULAR_CORE_HPP_
