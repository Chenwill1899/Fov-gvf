#include "pc_gvf/depth_angular_core.hpp"

#include "fixture_reader.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using pc_gvf::depth_angular::AngularSolution;
using pc_gvf::depth_angular::Camera;
using pc_gvf::depth_angular::GuidanceResult;
using pc_gvf::depth_angular::SimConfig;
using pc_gvf::depth_angular::computeGuidance;
using pc_gvf_test::Fixture;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "guidance_composition_check: " << message << '\n';
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
    config.planning_horizon = fixture.scalar("cfg.planning_horizon");
    config.control_dt = fixture.scalar("cfg.control_dt");
    config.dynamics_dt = fixture.scalar("cfg.dynamics_dt");
    config.rollout_dt = fixture.scalar("cfg.rollout_dt");
    config.rollout_horizon = fixture.scalar("cfg.rollout_horizon");
    config.rollout_margin = fixture.scalar("cfg.rollout_margin");
    config.max_direction_rate = fixture.scalar("cfg.max_direction_rate");
    config.goal_tolerance = fixture.scalar("cfg.goal_tolerance");
    config.clearance_reward = fixture.scalar("cfg.clearance_reward");
    config.hysteresis_weight = fixture.scalar("cfg.hysteresis_weight");
    config.deterministic_left_bias =
        fixture.scalar("cfg.deterministic_left_bias");
    config.convergence_distance = fixture.scalar("cfg.convergence_distance");
    config.convergence_margin = fixture.scalar("cfg.convergence_margin");
    config.convergence_length = fixture.scalar("cfg.convergence_length");
    config.convergence_max_angle = fixture.scalar("cfg.convergence_max_angle");
    config.source_radius_cells = fixture.integer("cfg.source_radius_cells");
    config.goal_radius_cells = fixture.integer("cfg.goal_radius_cells");
    config.field_tolerance = fixture.scalar("cfg.field_tolerance");
    config.field_max_iterations = fixture.integer("cfg.field_max_iterations");
    config.depth_point_stride = fixture.integer("cfg.depth_point_stride");
    config.cone_chunk_size = fixture.integer("cfg.cone_chunk_size");
    return config;
}

void compareVector(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double absolute,
    double relative,
    bool allow_nan,
    const std::string& context)
{
    require(actual.size() == expected.size(), "shape mismatch: " + context);
    for (std::size_t index = 0; index < actual.size(); ++index) {
        if (allow_nan && std::isnan(expected[index])) {
            require(std::isnan(actual[index]),
                    "NaN layout mismatch: " + context + " index " +
                        std::to_string(index));
        } else {
            require(close(actual[index], expected[index], absolute, relative),
                    "numeric mismatch: " + context + " index " +
                        std::to_string(index));
        }
    }
}

void checkFixture(const std::string& directory, const std::string& name)
{
    const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
    const Camera camera = fixtureCamera(fixture);
    const std::vector<double> depth = fixture.numbers("input.depth");
    const Eigen::Vector3d position = vector3(fixture.numbers("input.position_w"));
    const Eigen::Vector3d velocity = vector3(fixture.numbers("input.velocity_w"));
    const Eigen::Vector3d goal = vector3(fixture.numbers("input.goal_w"));
    const Eigen::Vector2d previous = vector2(fixture.numbers("input.q_previous"));
    const Eigen::Matrix3d rotation = matrix3(fixture.numbers("input.R_wc"));

    Eigen::Vector2d previous_goal = Eigen::Vector2d::Zero();
    const Eigen::Vector2d* previous_goal_pointer = nullptr;
    if (fixture.value("input.q_goal_previous") != "none") {
        previous_goal = vector2(fixture.numbers("input.q_goal_previous"));
        previous_goal_pointer = &previous_goal;
    }
    Eigen::Vector3d reference_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_direction = Eigen::Vector3d::Zero();
    const Eigen::Vector3d* reference_origin_pointer = nullptr;
    const Eigen::Vector3d* reference_direction_pointer = nullptr;
    if (fixture.value("input.reference_origin_w") != "none") {
        reference_origin = vector3(fixture.numbers("input.reference_origin_w"));
        reference_direction = vector3(
            fixture.numbers("input.reference_direction_w"));
        reference_origin_pointer = &reference_origin;
        reference_direction_pointer = &reference_direction;
    }

    const GuidanceResult result = computeGuidance(
        depth, camera, fixtureConfig(fixture), position, velocity, goal,
        previous, previous_goal_pointer, rotation, reference_origin_pointer,
        reference_direction_pointer);
    const AngularSolution& solution = result.solution;

    compareVector(solution.depth, depth, 0.0, 0.0, true, name + " depth");
    compareVector(
        solution.free_distance, fixture.numbers("expected.free_distance"),
        1.0e-9, 1.0e-9, false, name + " free distance");
    const std::vector<double> expected_mask =
        fixture.numbers("expected.planning_mask");
    require(solution.planning_mask.size() == expected_mask.size(),
            "planning-mask shape mismatch: " + name);
    for (std::size_t index = 0; index < expected_mask.size(); ++index) {
        require(solution.planning_mask[index] ==
                    static_cast<std::uint8_t>(expected_mask[index]),
                "planning-mask mismatch: " + name + " pixel " +
                    std::to_string(index));
    }
    compareVector(
        solution.potential, fixture.numbers("expected.potential"),
        1.0e-6, 1.0e-6, true, name + " potential");

    require((solution.reference_pixel -
             vector2(fixture.numbers("expected.q_ref"))).norm() <= 1.0e-6,
            "reference pixel mismatch: " + name);
    require((solution.source_pixel -
             vector2(fixture.numbers("expected.q_source"))).norm() <= 1.0e-6,
            "source pixel mismatch: " + name);
    require((solution.goal_pixel -
             vector2(fixture.numbers("expected.q_goal"))).norm() <= 1.0e-6,
            "goal pixel mismatch: " + name);
    require((solution.command_pixel -
             vector2(fixture.numbers("expected.q_cmd"))).norm() <= 1.0e-6,
            "command pixel mismatch: " + name);
    require(solution.field_valid ==
                static_cast<bool>(fixture.integer("expected.field_valid")),
            "field validity mismatch: " + name);
    require((result.returned_goal_pixel -
             vector2(fixture.numbers("expected.q_goal_returned"))).norm() <= 1.0e-6,
            "returned goal mismatch: " + name);
    require((result.command_world -
             vector3(fixture.numbers("expected.command_w"))).norm() <= 1.0e-6,
            "world command mismatch: " + name);
}

}  // namespace

int main()
{
    const std::string directory = PC_GVF_FIXTURE_DIR;
    const Fixture manifest = Fixture::load(directory + "/MANIFEST.txt");
    const std::vector<std::string> cases = manifest.strings("cases");
    require(static_cast<int>(cases.size()) == manifest.integer("case_count"),
            "manifest case count must match its case list");
    for (const std::string& name : cases) {
        checkFixture(directory, name);
    }
    std::cout << "guidance_composition_check: passed " << cases.size()
              << " complete Python fixtures\n";
    return 0;
}
