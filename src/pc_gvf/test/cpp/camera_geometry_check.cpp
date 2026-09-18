#include "pc_gvf/depth_angular_core.hpp"

#include "fixture_reader.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using pc_gvf::depth_angular::Camera;
using pc_gvf_test::Fixture;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "camera_geometry_check: " << message << '\n';
        std::exit(1);
    }
}

bool close(double actual, double expected, double tolerance = 1.0e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

Eigen::Vector2d vector2(const std::vector<double>& values)
{
    if (values.size() != 2) {
        throw std::runtime_error("expected two values");
    }
    return Eigen::Vector2d(values[0], values[1]);
}

void checkMathHelpers()
{
    const Eigen::Vector3d input(3.0, 4.0, 0.0);
    require((pc_gvf::depth_angular::normalize(input) -
             Eigen::Vector3d(0.6, 0.8, 0.0)).norm() < 1.0e-15,
            "normalize must match the Python helper");
    require((pc_gvf::depth_angular::clampNorm(input, 2.5) -
             Eigen::Vector3d(1.5, 2.0, 0.0)).norm() < 1.0e-15,
            "clampNorm must preserve direction");
    const Eigen::Vector3d fallback(1.0, 2.0, 3.0);
    const Eigen::Vector3d invalid(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    require(pc_gvf::depth_angular::normalize(invalid, &fallback) == fallback,
            "invalid normalization must return the supplied fallback");

    const Eigen::Matrix3d fixed = pc_gvf::depth_angular::fixedCameraRotation();
    require((fixed * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitX()).norm() < 1.0e-15,
            "camera optical z must map to body/world x");
    require((fixed * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitY()).norm() < 1.0e-15,
            "camera x must map to negative body/world y");
    require((fixed * Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitZ()).norm() < 1.0e-15,
            "camera y must map to negative body/world z");

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    require(pc_gvf::depth_angular::quaternionMatrix(0.0, 0.0, 0.0, 1.0, &rotation) &&
            rotation.isApprox(Eigen::Matrix3d::Identity(), 1.0e-15),
            "identity quaternion must produce identity rotation");
    require(!pc_gvf::depth_angular::quaternionMatrix(0.0, 0.0, 0.0, 0.0, &rotation),
            "zero quaternion must be rejected");
}

void checkPythonReferenceValues()
{
    Camera camera;
    require(close(camera.fx(), 24.000000000000004) &&
            close(camera.fy(), 26.68609743322932) &&
            close(camera.cx(), 23.5) && close(camera.cy(), 17.5),
            "default intrinsics must match Python");
    const Eigen::Vector3d corner = camera.rayFromPixel(Eigen::Vector2d(0.0, 0.0));
    require((corner - Eigen::Vector3d(
                -0.6335287326942062,
                -0.42428988383920835,
                0.6470080674323808)).norm() < 1.0e-14,
            "default corner ray must match Python");

    Camera alternate(32, 24, 70.0, 55.0, 6.0);
    const Eigen::Vector3d alternate_corner =
        alternate.rayFromPixel(Eigen::Vector2d(0.0, 0.0));
    require((alternate_corner - Eigen::Vector3d(
                -0.5188799933967922,
                -0.3816117149790563,
                0.7649418614792161)).norm() < 1.0e-14,
            "alternate corner ray must match Python");
}

void checkFixtures()
{
    const std::string directory = PC_GVF_FIXTURE_DIR;
    const Fixture manifest = Fixture::load(directory + "/MANIFEST.txt");
    const std::vector<std::string> cases = manifest.strings("cases");
    require(manifest.integer("format_version") == 1,
            "unsupported fixture format");
    require(static_cast<int>(cases.size()) == manifest.integer("case_count"),
            "manifest case count must match its case list");

    for (const std::string& name : cases) {
        const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
        require(fixture.value("name") == name, "fixture name mismatch: " + name);
        Camera camera(
            fixture.integer("camera.width"),
            fixture.integer("camera.height"),
            fixture.scalar("camera.hfov_deg"),
            fixture.scalar("camera.vfov_deg"),
            fixture.scalar("camera.max_depth"));
        camera.setIntrinsics(
            fixture.scalar("camera.fx"), fixture.scalar("camera.fy"),
            fixture.scalar("camera.cx"), fixture.scalar("camera.cy"));

        const std::size_t pixel_count =
            static_cast<std::size_t>(camera.width()) * camera.height();
        require(fixture.numbers("input.depth").size() == pixel_count,
                "depth shape mismatch: " + name);
        require(fixture.numbers("expected.free_distance").size() == pixel_count,
                "free-distance shape mismatch: " + name);
        require(fixture.numbers("expected.planning_mask").size() == pixel_count,
                "planning-mask shape mismatch: " + name);
        require(fixture.numbers("expected.potential").size() == pixel_count,
                "potential shape mismatch: " + name);

        for (int v = 0; v < camera.height(); ++v) {
            for (int u = 0; u < camera.width(); ++u) {
                const Eigen::Vector3d computed =
                    camera.rayFromPixel(Eigen::Vector2d(u, v));
                require((camera.ray(u, v) - computed).norm() < 1.0e-15,
                        "cached ray mismatch: " + name);
            }
        }

        const Eigen::Vector2d q_ref = vector2(fixture.numbers("expected.q_ref"));
        Eigen::Vector2d round_trip;
        require(camera.pixelFromDirection(camera.rayFromPixel(q_ref), &round_trip) &&
                (round_trip - q_ref).norm() < 1.0e-12,
                "pixel/ray round trip mismatch: " + name);
    }
    require(cases.size() == 16, "the frozen migration suite must contain 16 cases");
}

}  // namespace

int main()
{
    checkMathHelpers();
    checkPythonReferenceValues();
    checkFixtures();
    std::cout << "camera_geometry_check: passed\n";
    return 0;
}
