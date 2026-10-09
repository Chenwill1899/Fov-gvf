#include "pc_gvf/depth_angular_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace {

using pc_gvf::depth_angular::Camera;
using pc_gvf::depth_angular::SimConfig;
using pc_gvf::depth_angular::computeGuidance;
using pc_gvf::depth_angular::fixedCameraRotation;
using pc_gvf::depth_angular::selectClosestYaw;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "horizontal_360_check: " << message << '\n';
        std::exit(1);
    }
}

Eigen::Vector2d directionPixel(
    const Camera& camera, const Eigen::Vector3d& direction_camera)
{
    Eigen::Vector2d pixel;
    if (!camera.pixelFromDirection(direction_camera, &pixel)) {
        if (direction_camera.z() > pc_gvf::depth_angular::kEpsilon) {
            pixel = Eigen::Vector2d(
                camera.fx() * direction_camera.x() / direction_camera.z() +
                    camera.cx(),
                camera.fy() * direction_camera.y() / direction_camera.z() +
                    camera.cy());
        } else {
            pixel = Eigen::Vector2d(
                direction_camera.x() >= 0.0 ? camera.width() - 2.0 : 1.0,
                camera.cy());
        }
        pixel.x() = std::max(
            1.0, std::min(pixel.x(), camera.width() - 2.0));
        pixel.y() = std::max(
            1.0, std::min(pixel.y(), camera.height() - 2.0));
    }
    return pixel;
}

}  // namespace

int main()
{
    const std::vector<double> camera_yaws{
        0.0, 0.5 * kPi, kPi, -0.5 * kPi};
    const std::vector<double> desired_yaws{
        0.0, 0.25 * kPi, 0.5 * kPi, 0.75 * kPi,
        kPi, -0.75 * kPi, -0.5 * kPi, -0.25 * kPi};
    for (std::size_t index = 0; index < desired_yaws.size(); ++index) {
        const std::size_t selected = selectClosestYaw(
            desired_yaws[index], camera_yaws);
        const double error = std::abs(std::atan2(
            std::sin(desired_yaws[index] - camera_yaws[selected]),
            std::cos(desired_yaws[index] - camera_yaws[selected])));
        require(error <= 0.25 * kPi + 1.0e-12,
                "selected view does not cover direction index " +
                    std::to_string(index));
    }
    require(selectClosestYaw(0.0, camera_yaws) == 0, "front cardinal view");
    require(selectClosestYaw(0.5 * kPi, camera_yaws) == 1, "left cardinal view");
    require(selectClosestYaw(kPi, camera_yaws) == 2, "back cardinal view");
    require(selectClosestYaw(-0.5 * kPi, camera_yaws) == 3, "right cardinal view");

    Camera camera(48, 36, 90.0, 68.0, 10.0);
    const std::vector<double> depth(48 * 36, 0.0);
    SimConfig config;
    config.horizontal_only = true;
    config.reference_speed = 2.0;
    config.max_direction_rate = 20.0;
    config.control_dt = 0.1;
    config.planning_horizon = 3.0;
    const Eigen::Vector3d position(0.0, 0.0, 1.0);
    const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    for (double desired_yaw : desired_yaws) {
        const std::size_t selected = selectClosestYaw(
            desired_yaw, camera_yaws);
        const double camera_yaw = camera_yaws[selected];
        const Eigen::Matrix3d yaw_rotation = Eigen::AngleAxisd(
            camera_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d rotation = yaw_rotation * fixedCameraRotation();
        const Eigen::Vector3d requested(
            std::cos(desired_yaw), std::sin(desired_yaw), 0.0);
        const Eigen::Vector2d requested_pixel = directionPixel(
            camera, rotation.transpose() * requested);
        const auto result = computeGuidance(
            depth, camera, config, position, velocity,
            position + 8.0 * requested, requested_pixel, nullptr, rotation);
        require(result.command_world.head<2>().norm() > 0.1,
                "clear horizontal view produced no command");
        require(std::abs(result.command_world.z()) < 1.0e-10,
                "horizontal guidance produced vertical velocity");
        const double clear_alignment =
            result.command_world.normalized().dot(requested);
        require(clear_alignment > 0.97,
                "clear horizontal command did not follow requested direction: " +
                    std::to_string(desired_yaw) + " alignment=" +
                    std::to_string(clear_alignment));
    }

    for (double desired_yaw : desired_yaws) {
        const std::size_t selected = selectClosestYaw(
            desired_yaw, camera_yaws);
        const double camera_yaw = camera_yaws[selected];
        const Eigen::Matrix3d yaw_rotation = Eigen::AngleAxisd(
            camera_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d rotation = yaw_rotation * fixedCameraRotation();
        const Eigen::Vector3d requested(
            std::cos(desired_yaw), std::sin(desired_yaw), 0.0);
        const Eigen::Vector2d obstacle_pixel = directionPixel(
            camera, rotation.transpose() * requested);
        const int obstacle_u = static_cast<int>(std::nearbyint(obstacle_pixel.x()));
        std::vector<double> center_obstacle(48 * 36, 0.0);
        for (int v = 8; v < 28; ++v) {
            for (int u = std::max(1, obstacle_u - 2);
                 u <= std::min(46, obstacle_u + 2); ++u) {
                center_obstacle[static_cast<std::size_t>(v) * 48 + u] = 2.0;
            }
        }
        const auto result = computeGuidance(
            center_obstacle, camera, config, position, velocity,
            position + 8.0 * requested, obstacle_pixel, nullptr, rotation);
        require(result.command_world.head<2>().norm() > 0.05,
                "horizontal obstacle produced no avoidance command");
        require(std::abs(result.command_world.z()) < 1.0e-10,
                "obstacle avoidance leaked into vertical velocity");
        require(result.command_world.normalized().dot(requested) < 0.995,
                "center obstacle did not deflect horizontal command");
    }
    std::cout << "horizontal_360_check: passed view selection and horizontal-only guidance\n";
    return 0;
}
