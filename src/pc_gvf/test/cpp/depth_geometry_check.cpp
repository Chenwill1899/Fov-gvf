#include "pc_gvf/depth_angular_core.hpp"

#include "fixture_reader.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using pc_gvf::depth_angular::Camera;
using pc_gvf::depth_angular::BinaryMask;
using pc_gvf::depth_angular::applyObstacleReleaseHysteresis;
using pc_gvf::depth_angular::backprojectObstaclePoints;
using pc_gvf::depth_angular::collisionConeFreeDistance;
using pc_gvf_test::Fixture;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "depth_geometry_check: " << message << '\n';
        std::exit(1);
    }
}

bool close(double actual, double expected, double absolute, double relative)
{
    return std::abs(actual - expected) <=
        absolute + relative * std::abs(expected);
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

void checkFixture(const std::string& directory, const std::string& name)
{
    const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
    const Camera camera = fixtureCamera(fixture);
    const std::vector<double> depth = fixture.numbers("input.depth");
    const int stride = fixture.integer("cfg.depth_point_stride");
    const std::vector<Eigen::Vector3d> points =
        backprojectObstaclePoints(depth, camera, stride);

    const std::vector<double> expected_points =
        fixture.numbers("expected.obstacle_points");
    require(
        static_cast<int>(points.size()) ==
            fixture.integer("expected.obstacle_point_count"),
        "obstacle point count mismatch: " + name);
    require(expected_points.size() == points.size() * 3,
            "obstacle point fixture shape mismatch: " + name);
    for (std::size_t index = 0; index < points.size(); ++index) {
        for (int axis = 0; axis < 3; ++axis) {
            require(
                close(points[index][axis], expected_points[index * 3 + axis],
                      1.0e-12, 1.0e-12),
                "backprojected point mismatch: " + name + " index " +
                    std::to_string(index) + " axis " + std::to_string(axis));
        }
    }

    const double effective_radius =
        fixture.scalar("cfg.body_radius") +
        fixture.scalar("cfg.safety_margin");
    const int chunk_size = fixture.integer("cfg.cone_chunk_size");
    const std::vector<double> free = collisionConeFreeDistance(
        points, camera, effective_radius, chunk_size);
    const std::vector<double> expected_free =
        fixture.numbers("expected.free_distance");
    require(free.size() == expected_free.size(),
            "free-distance fixture shape mismatch: " + name);
    for (std::size_t index = 0; index < free.size(); ++index) {
        require(close(free[index], expected_free[index], 1.0e-9, 1.0e-9),
                "collision-cone free distance mismatch: " + name +
                    " pixel " + std::to_string(index));
    }

    const std::vector<double> one_ray_chunks = collisionConeFreeDistance(
        points, camera, effective_radius, 1);
    require(one_ray_chunks == free,
            "chunk size must not change collision-cone results: " + name);
}

template<typename Callable>
void requireInvalidArgument(Callable callable, const std::string& message)
{
    try {
        callable();
    } catch (const std::invalid_argument&) {
        return;
    }
    require(false, message);
}

void checkInputValidation()
{
    const Camera camera;
    requireInvalidArgument(
        [&camera]() {
            backprojectObstaclePoints(std::vector<double>(1, 1.0), camera, 2);
        },
        "a depth image with the wrong size must be rejected");
    requireInvalidArgument(
        [&camera]() {
            backprojectObstaclePoints(
                std::vector<double>(
                    static_cast<std::size_t>(camera.width()) * camera.height(),
                    camera.maxDepth()),
                camera, 0);
        },
        "a nonpositive depth stride must be rejected");
    requireInvalidArgument(
        [&camera]() {
            collisionConeFreeDistance({}, camera, -0.1, 256);
        },
        "a negative effective radius must be rejected");
    requireInvalidArgument(
        [&camera]() {
            collisionConeFreeDistance({}, camera, 0.5, 0);
        },
        "a nonpositive chunk size must be rejected");
    requireInvalidArgument(
        [&camera]() {
            collisionConeFreeDistance(
                {Eigen::Vector3d(
                    std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)},
                camera, 0.5, 256);
        },
        "a nonfinite obstacle point must be rejected");
}

void checkObstacleReleaseHysteresis()
{
    BinaryMask state;
    std::vector<std::uint8_t> clear_counts;
    applyObstacleReleaseHysteresis(
        BinaryMask{0, 1, 0}, 3, true, &state, &clear_counts);
    require(state == BinaryMask({0, 1, 0}), "initial mask must be accepted");

    const BinaryMask clear{0, 0, 0};
    applyObstacleReleaseHysteresis(clear, 3, true, &state, &clear_counts);
    require(state[1] == 1, "obstacle must remain after one clear frame");
    applyObstacleReleaseHysteresis(clear, 3, false, &state, &clear_counts);
    require(clear_counts[1] == 1, "reused depth must not advance clear count");
    applyObstacleReleaseHysteresis(clear, 3, true, &state, &clear_counts);
    require(state[1] == 1, "obstacle must remain after two clear frames");
    applyObstacleReleaseHysteresis(clear, 3, true, &state, &clear_counts);
    require(state[1] == 0, "obstacle must clear on the third clear frame");

    applyObstacleReleaseHysteresis(
        BinaryMask{1, 0, 0}, 3, true, &state, &clear_counts);
    require(state[0] == 1, "new obstacle must be occupied immediately");
}

void checkEdgeCaseCounts(const std::string& directory)
{
    const Fixture all_zero = Fixture::load(directory + "/all_zero_depth.fixture");
    const Fixture all_nan = Fixture::load(directory + "/all_nan_depth.fixture");
    const Fixture kept = Fixture::load(directory + "/single_kept_near_pixel.fixture");
    const Fixture skipped =
        Fixture::load(directory + "/single_skipped_near_pixel.fixture");
    require(all_zero.integer("expected.obstacle_point_count") == 0,
            "all-zero depth must yield no obstacle points");
    require(all_nan.integer("expected.obstacle_point_count") == 0,
            "all-NaN depth must yield no obstacle points");
    require(kept.integer("expected.obstacle_point_count") == 1,
            "the stride-aligned near pixel must be retained");
    require(skipped.integer("expected.obstacle_point_count") == 0,
            "the non-stride-aligned near pixel must be skipped");
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
    // Compare projected broad-phase bounds against an independent exhaustive
    // ray/sphere oracle, including near-camera, off-screen and grazing cases.
    std::mt19937 random(42949);std::uniform_real_distribution<double> xy(-15.,15.),z(-2.,20.);
    for(double radius:{0.,.01,.58,3.})for(int variant=0;variant<2;++variant) {
        Camera camera(64,48,90,68,20);
        if(variant)camera.setIntrinsics(45.2,39.7,27.3,20.1);
        std::vector<Eigen::Vector3d> points;
        for(int i=0;i<500;++i)points.emplace_back(xy(random),xy(random),z(random));
        points.emplace_back(.1,.1,radius+1e-8);
        auto actual=collisionConeFreeDistance(points,camera,radius,17);
        for(std::size_t i=0;i<actual.size();++i) {
            double expected=std::max(0.,camera.maxDepth()-radius);
            for(const auto& p:points) {
                const double longitudinal=camera.rays()[i].dot(p);
                const double lateral=std::max(0.,p.squaredNorm()-longitudinal*longitudinal);
                if(longitudinal>0&&lateral<radius*radius)
                    expected=std::min(expected,longitudinal-std::sqrt(std::max(0.,radius*radius-lateral)));
            }
            require(std::abs(actual[i]-expected)<1e-11,"projected bounds skipped a ray/sphere intersection");
        }
    }
    checkEdgeCaseCounts(directory);
    checkInputValidation();
    checkObstacleReleaseHysteresis();
    std::cout << "depth_geometry_check: passed " << cases.size()
              << " Python fixtures\n";
    return 0;
}
