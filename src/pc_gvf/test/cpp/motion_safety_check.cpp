#include "pc_gvf/depth_angular_core.hpp"

#include "fixture_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using pc_gvf::depth_angular::BinaryMask;
using pc_gvf::depth_angular::Camera;
using pc_gvf::depth_angular::SimConfig;
using pc_gvf::depth_angular::angularDistance;
using pc_gvf::depth_angular::angularRateLimit;
using pc_gvf::depth_angular::bilinearSample;
using pc_gvf::depth_angular::brakingSpeed;
using pc_gvf::depth_angular::depthRolloutLimit;
using pc_gvf::depth_angular::discreteHarmonicPath;
using pc_gvf::depth_angular::referenceConvergedDirection;
using pc_gvf::depth_angular::rolloutIsSafe;
using pc_gvf::depth_angular::selectCommandDirection;
using pc_gvf_test::Fixture;

constexpr double kPi = 3.141592653589793238462643383279502884;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "motion_safety_check: " << message << '\n';
        std::exit(1);
    }
}

bool close(double actual, double expected, double absolute, double relative)
{
    return std::abs(actual - expected) <=
        absolute + relative * std::abs(expected);
}

Eigen::Vector2d vector2(const std::vector<double>& values)
{
    if (values.size() != 2) {
        throw std::runtime_error("expected two fixture values");
    }
    return Eigen::Vector2d(values[0], values[1]);
}

Eigen::Vector3d vector3(const std::vector<double>& values)
{
    if (values.size() != 3) {
        throw std::runtime_error("expected three fixture values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Matrix3d matrix3(const std::vector<double>& values)
{
    if (values.size() != 9) {
        throw std::runtime_error("expected nine fixture values");
    }
    Eigen::Matrix3d result;
    result <<
        values[0], values[1], values[2],
        values[3], values[4], values[5],
        values[6], values[7], values[8];
    return result;
}

BinaryMask binaryMask(const std::vector<double>& values)
{
    BinaryMask result;
    result.reserve(values.size());
    for (const double value : values) {
        require(value == 0.0 || value == 1.0,
                "fixture mask must contain only zero and one");
        result.push_back(value == 0.0 ? 0 : 1);
    }
    return result;
}

Camera fixtureCamera(const Fixture& fixture)
{
    Camera camera(
        fixture.integer("camera.width"),
        fixture.integer("camera.height"),
        fixture.scalar("camera.hfov_deg"),
        fixture.scalar("camera.vfov_deg"),
        fixture.scalar("camera.max_depth"));
    camera.setIntrinsics(
        fixture.scalar("camera.fx"), fixture.scalar("camera.fy"),
        fixture.scalar("camera.cx"), fixture.scalar("camera.cy"));
    return camera;
}

SimConfig fixtureConfig(const Fixture& fixture)
{
    SimConfig config;
    config.body_radius = fixture.scalar("cfg.body_radius");
    config.safety_margin = fixture.scalar("cfg.safety_margin");
    config.reference_speed = fixture.scalar("cfg.reference_speed");
    config.max_accel = fixture.scalar("cfg.max_accel");
    config.brake_accel = fixture.scalar("cfg.brake_accel");
    config.velocity_tau = fixture.scalar("cfg.velocity_tau");
    config.delay = fixture.scalar("cfg.delay");
    config.rollout_dt = fixture.scalar("cfg.rollout_dt");
    config.rollout_horizon = fixture.scalar("cfg.rollout_horizon");
    config.rollout_margin = fixture.scalar("cfg.rollout_margin");
    config.max_direction_rate = fixture.scalar("cfg.max_direction_rate");
    config.convergence_distance = fixture.scalar("cfg.convergence_distance");
    config.convergence_margin = fixture.scalar("cfg.convergence_margin");
    config.convergence_length = fixture.scalar("cfg.convergence_length");
    config.convergence_max_angle = fixture.scalar("cfg.convergence_max_angle");
    config.control_dt = fixture.scalar("cfg.control_dt");
    return config;
}

std::vector<Eigen::Vector3d> fixturePoints(const Fixture& fixture)
{
    const std::vector<double> values = fixture.numbers("expected.obstacle_points");
    require(values.size() % 3 == 0, "obstacle point data must have three columns");
    std::vector<Eigen::Vector3d> result;
    result.reserve(values.size() / 3);
    for (std::size_t index = 0; index < values.size(); index += 3) {
        result.emplace_back(values[index], values[index + 1], values[index + 2]);
    }
    return result;
}

void checkFixture(const std::string& directory, const std::string& name)
{
    const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
    const Camera camera = fixtureCamera(fixture);
    const SimConfig config = fixtureConfig(fixture);
    const int width = camera.width();
    const int height = camera.height();
    const BinaryMask blocked =
        binaryMask(fixture.numbers("expected.planning_mask"));
    const Eigen::Vector2d previous = vector2(fixture.numbers("input.q_previous"));
    Eigen::Vector2d pre_reference = previous;

    const bool selection_valid =
        fixture.value("expected.selected_q_source") != "none";
    if (!selection_valid) {
        require(fixture.integer("expected.harmonic_path_count") == 0 &&
                    fixture.value("expected.harmonic_path") == "none",
                "invalid selection must not have a path: " + name);
    } else {
        const Eigen::Vector2d source =
            vector2(fixture.numbers("expected.selected_q_source"));
        const Eigen::Vector2d goal =
            vector2(fixture.numbers("expected.selected_q_goal"));
        const std::vector<double> potential =
            fixture.numbers("expected.harmonic_potential");
        const std::vector<Eigen::Vector2d> path = discreteHarmonicPath(
            potential, blocked, width, height, source, goal);
        const std::vector<double> expected_path =
            fixture.numbers("expected.harmonic_path");
        require(static_cast<int>(path.size()) ==
                    fixture.integer("expected.harmonic_path_count") &&
                    expected_path.size() == path.size() * 2,
                "harmonic path shape mismatch: " + name);
        for (std::size_t index = 0; index < path.size(); ++index) {
            require(path[index] == Eigen::Vector2d(
                        expected_path[index * 2], expected_path[index * 2 + 1]),
                    "harmonic path mismatch: " + name + " step " +
                        std::to_string(index));
        }

        require(close(
                    angularDistance(camera, previous, source),
                    fixture.scalar("expected.angular_distance_previous_source"),
                    1.0e-12, 1.0e-12),
                "previous/source angular distance mismatch: " + name);
        const Eigen::Vector2d limited_goal = angularRateLimit(
            camera, previous, goal,
            config.max_direction_rate * config.control_dt);
        require((limited_goal -
                 vector2(fixture.numbers("expected.rate_limited_goal"))).norm() <=
                    1.0e-10,
                "rate-limited goal mismatch: " + name);
        const Eigen::Vector2d direct_command = selectCommandDirection(
            camera, previous, source, goal, potential, blocked, config);
        require((direct_command -
                 vector2(fixture.numbers("expected.direct_selected_q_cmd"))).norm() <=
                    1.0e-9,
                "selected command mismatch: " + name);
        pre_reference = angularDistance(camera, source, goal) < 0.4 * kPi / 180.0
            ? limited_goal
            : direct_command;
    }
    require((pre_reference -
             vector2(fixture.numbers("expected.pre_reference_q_cmd"))).norm() <= 1.0e-9,
            "pre-reference command mismatch: " + name);

    const Eigen::Matrix3d rotation_world_from_camera =
        matrix3(fixture.numbers("input.R_wc"));
    const Eigen::Vector3d raw_field_direction = rotation_world_from_camera *
        camera.rayFromPixel(pre_reference);
    const std::vector<Eigen::Vector3d> points_camera = fixturePoints(fixture);
    double observed_clearance = camera.maxDepth();
    for (const Eigen::Vector3d& point : points_camera) {
        observed_clearance = std::min(observed_clearance, point.norm());
    }
    require(close(
                observed_clearance, fixture.scalar("expected.observed_clearance"),
                1.0e-12, 1.0e-12),
            "observed clearance mismatch: " + name);

    if (fixture.value("input.reference_origin_w") == "none") {
        require(fixture.value("expected.reference_corrected_direction_w") == "none",
                "case without a reference must have no correction fixture: " + name);
    } else {
        const Eigen::Vector3d corrected = referenceConvergedDirection(
            raw_field_direction,
            vector3(fixture.numbers("input.position_w")),
            vector3(fixture.numbers("input.reference_origin_w")),
            vector3(fixture.numbers("input.reference_direction_w")),
            observed_clearance,
            config);
        require((corrected - vector3(fixture.numbers(
                    "expected.reference_corrected_direction_w"))).norm() <= 1.0e-12,
                "reference correction mismatch: " + name);
    }

    const std::vector<double> free_distance =
        fixture.numbers("expected.free_distance");
    const Eigen::Vector2d final_command_pixel =
        vector2(fixture.numbers("expected.q_cmd"));
    const double command_free_distance = bilinearSample(
        free_distance, width, height, final_command_pixel, 0.0);
    require(close(
                command_free_distance,
                fixture.scalar("expected.command_free_distance"),
                1.0e-12, 1.0e-12),
            "bilinear free-distance mismatch: " + name);

    const Eigen::Vector3d position = vector3(fixture.numbers("input.position_w"));
    const Eigen::Vector3d velocity = vector3(fixture.numbers("input.velocity_w"));
    const double goal_distance =
        (vector3(fixture.numbers("input.goal_w")) - position).norm();
    const double reference_speed = std::min(
        config.reference_speed,
        std::sqrt(std::max(0.0, 2.0 * config.brake_accel * goal_distance)));
    require(close(reference_speed, fixture.scalar("expected.reference_speed"),
                  1.0e-12, 1.0e-12),
            "reference speed mismatch: " + name);
    const double braking = brakingSpeed(
        command_free_distance, reference_speed, config);
    require(close(braking, fixture.scalar("expected.braking_speed"),
                  1.0e-12, 1.0e-12),
            "braking speed mismatch: " + name);

    const Eigen::Vector3d final_ray_world = rotation_world_from_camera *
        camera.rayFromPixel(final_command_pixel);
    std::vector<Eigen::Vector3d> points_world;
    points_world.reserve(points_camera.size());
    for (const Eigen::Vector3d& point_camera : points_camera) {
        points_world.push_back(position + rotation_world_from_camera * point_camera);
    }
    const bool full_speed_safe = rolloutIsSafe(
        position, velocity, braking * final_ray_world, points_world, position,
        rotation_world_from_camera.transpose(), camera, config);
    require(full_speed_safe ==
                static_cast<bool>(fixture.integer("expected.full_speed_rollout_safe")),
            "full-speed rollout decision mismatch: " + name);
    const double limited_speed = depthRolloutLimit(
        braking, final_ray_world, position, velocity, points_world,
        rotation_world_from_camera.transpose(), camera, config);
    require(close(
                limited_speed, fixture.scalar("expected.rollout_limited_speed"),
                1.0e-9, 1.0e-9),
            "rollout-limited speed mismatch: " + name);
}

void checkIndependentBranches()
{
    const Camera camera;
    const SimConfig config;
    const std::vector<double> image = {
        1.0, 2.0,
        3.0, 5.0,
    };
    require(close(
                bilinearSample(image, 2, 2, Eigen::Vector2d(0.5, 0.5), -1.0),
                2.75, 1.0e-15, 0.0),
            "bilinear center sample must use all four pixels");
    require(bilinearSample(
                image, 2, 2, Eigen::Vector2d(-0.1, 0.5), -7.0) == -7.0,
            "out-of-bounds bilinear sample must return its default");
    std::vector<double> image_with_nan = image;
    image_with_nan[3] = std::numeric_limits<double>::quiet_NaN();
    require(bilinearSample(
                image_with_nan, 2, 2, Eigen::Vector2d(0.5, 0.5), -9.0) == -9.0,
            "nonfinite bilinear neighborhood must return its default");

    require(brakingSpeed(0.0, 1.2, config) == 0.0,
            "zero usable clearance must command zero braking speed");
    require(brakingSpeed(100.0, 1.2, config) == 1.2,
            "large clearance must preserve reference speed");
    require(depthRolloutLimit(
                1.0e-6, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero(), {}, Eigen::Matrix3d::Identity(),
                camera, config) == 0.0,
            "tiny desired speed must take the zero-speed branch");

    const Eigen::Vector3d field = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d unchanged = referenceConvergedDirection(
        field, Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitX(), 0.0, config);
    require(unchanged == field,
            "low-clearance convergence gate must leave direction unchanged");
    const Eigen::Vector3d no_reference = referenceConvergedDirection(
        field, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), 8.0, config);
    require(no_reference == field,
            "zero reference direction must leave field direction unchanged");

    require(rolloutIsSafe(
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                Eigen::Vector3d::UnitX(), {}, Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Identity(), camera, config),
            "empty obstacle points must make rollout immediately safe");
    const std::vector<Eigen::Vector3d> colliding_point = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
    };
    require(!rolloutIsSafe(
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                Eigen::Vector3d::UnitX(), colliding_point,
                Eigen::Vector3d::Zero(), pc_gvf::depth_angular::fixedCameraRotation().transpose(),
                camera, config),
            "a point at the initial swept segment must be unsafe");
}

}  // namespace

int main()
{
    const std::string directory = PC_GVF_FIXTURE_DIR;
    const Fixture manifest = Fixture::load(directory + "/MANIFEST.txt");
    const std::vector<std::string> cases = manifest.strings("cases");
    for (const std::string& name : cases) {
        checkFixture(directory, name);
    }
    checkIndependentBranches();
    std::cout << "motion_safety_check: passed " << cases.size()
              << " Python fixtures\n";
    return 0;
}
