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
using pc_gvf::depth_angular::GoalSelection;
using pc_gvf::depth_angular::HarmonicSolution;
using pc_gvf::depth_angular::SimConfig;
using pc_gvf::depth_angular::chooseSafeGoal;
using pc_gvf::depth_angular::continuousHarmonicTarget;
using pc_gvf::depth_angular::diskMask;
using pc_gvf::depth_angular::labelFreeComponents;
using pc_gvf::depth_angular::nearestFreePixel;
using pc_gvf::depth_angular::solveAngularHarmonic;
using pc_gvf_test::Fixture;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "angular_field_check: " << message << '\n';
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

SimConfig fixtureConfig(const Fixture& fixture)
{
    SimConfig config;
    config.clearance_reward = fixture.scalar("cfg.clearance_reward");
    config.hysteresis_weight = fixture.scalar("cfg.hysteresis_weight");
    config.deterministic_left_bias =
        fixture.scalar("cfg.deterministic_left_bias");
    config.source_radius_cells = fixture.integer("cfg.source_radius_cells");
    config.goal_radius_cells = fixture.integer("cfg.goal_radius_cells");
    config.field_tolerance = fixture.scalar("cfg.field_tolerance");
    config.field_max_iterations = fixture.integer("cfg.field_max_iterations");
    return config;
}

void comparePotential(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    const std::string& context)
{
    require(actual.size() == expected.size(),
            "potential shape mismatch: " + context);
    for (std::size_t index = 0; index < actual.size(); ++index) {
        if (std::isnan(expected[index])) {
            require(std::isnan(actual[index]),
                    "potential NaN layout mismatch: " + context + " pixel " +
                        std::to_string(index));
        } else {
            require(close(actual[index], expected[index], 1.0e-6, 1.0e-6),
                    "potential mismatch: " + context + " pixel " +
                        std::to_string(index));
        }
    }
}

void checkFixture(const std::string& directory, const std::string& name)
{
    const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
    const int width = fixture.integer("camera.width");
    const int height = fixture.integer("camera.height");
    const BinaryMask blocked =
        binaryMask(fixture.numbers("expected.planning_mask"));
    const Eigen::Vector2d reference = vector2(fixture.numbers("expected.q_ref"));
    const Eigen::Vector2d source = vector2(fixture.numbers("input.q_previous"));
    const std::vector<double> previous_values =
        fixture.value("input.q_goal_previous") == "none"
            ? std::vector<double>()
            : fixture.numbers("input.q_goal_previous");
    const Eigen::Vector2d previous = previous_values.empty()
        ? Eigen::Vector2d::Zero()
        : vector2(previous_values);
    const Eigen::Vector2d* previous_pointer =
        previous_values.empty() ? nullptr : &previous;

    const GoalSelection selection = chooseSafeGoal(
        reference, source, blocked, width, height, previous_pointer,
        fixtureConfig(fixture));
    const bool expected_valid =
        fixture.value("expected.selected_q_source") != "none";
    require(selection.valid == expected_valid,
            "safe-goal validity mismatch: " + name);

    const std::vector<double> expected_labels =
        fixture.numbers("expected.component_labels");
    require(selection.labels.size() == expected_labels.size(),
            "component-label shape mismatch: " + name);
    int maximum_label = 0;
    for (std::size_t index = 0; index < selection.labels.size(); ++index) {
        require(selection.labels[index] == static_cast<int>(expected_labels[index]),
                "component label mismatch: " + name + " pixel " +
                    std::to_string(index));
        maximum_label = std::max(maximum_label, selection.labels[index]);
    }
    require(maximum_label == fixture.integer("expected.component_count"),
            "component count mismatch: " + name);

    if (!expected_valid) {
        require(fixture.value("expected.selected_q_goal") == "none",
                "invalid fixture selection must have no goal: " + name);
        require(fixture.value("expected.harmonic_potential") == "none",
                "invalid fixture selection must not invoke the solver: " + name);
        require(fixture.integer("expected.harmonic_valid") == 0,
                "invalid fixture selection cannot have a valid field: " + name);
        return;
    }

    require((selection.source -
             vector2(fixture.numbers("expected.selected_q_source"))).norm() <= 1.0e-12,
            "selected source mismatch: " + name);
    require((selection.goal -
             vector2(fixture.numbers("expected.selected_q_goal"))).norm() <= 1.0e-12,
            "selected goal mismatch: " + name);

    const HarmonicSolution harmonic = solveAngularHarmonic(
        blocked, width, height, selection.source, selection.goal,
        fixtureConfig(fixture));
    require(harmonic.valid ==
                static_cast<bool>(fixture.integer("expected.harmonic_valid")),
            "harmonic validity mismatch: " + name);
    comparePotential(
        harmonic.potential,
        fixture.numbers("expected.harmonic_potential"),
        name);
}

void checkConnectivityAndTieBreaking()
{
    const BinaryMask diagonal_free = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };
    int component_count = 0;
    const std::vector<int> labels =
        labelFreeComponents(diagonal_free, 3, 3, &component_count);
    require(component_count == 1 && labels[0] == 1 && labels[4] == 1 &&
                labels[8] == 1,
            "diagonal cells must connect under the Python 8-neighbor rule");

    const BinaryMask symmetric_free = {
        0, 0, 0,
        1, 0, 1,
        0, 0, 0,
    };
    Eigen::Vector2d selected;
    require(nearestFreePixel(
                Eigen::Vector2d(1.0, 1.0), symmetric_free, 3, 3,
                nullptr, 0.012, &selected) &&
                selected == Eigen::Vector2d(0.0, 1.0),
            "positive deterministic bias must favor negative image-u");
}

void checkHarmonicBranches()
{
    const BinaryMask open(25, 0);
    SimConfig config;
    config.source_radius_cells = 0;
    config.goal_radius_cells = 0;

    HarmonicSolution overlap = solveAngularHarmonic(
        open, 5, 5, Eigen::Vector2d(2.0, 2.0),
        Eigen::Vector2d(2.0, 2.0), config);
    require(overlap.valid && overlap.potential[12] == 1.0,
            "overlapping source and goal must take the Python source-first branch");
    require(overlap.potential[0] == 0.0,
            "overlap branch must set other free cells to zero");

    const BinaryMask two_cells(2, 0);
    HarmonicSolution no_unknown = solveAngularHarmonic(
        two_cells, 2, 1, Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0), config);
    require(no_unknown.valid && no_unknown.potential[0] == 1.0 &&
                no_unknown.potential[1] == 0.0,
            "a fully Dirichlet grid must not require CG iterations");

    const BinaryMask blocked(9, 1);
    HarmonicSolution unavailable = solveAngularHarmonic(
        blocked, 3, 3, Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(2.0, 2.0), config);
    require(!unavailable.valid,
            "masked source and goal regions must make the field invalid");
    require(std::all_of(
                unavailable.potential.begin(), unavailable.potential.end(),
                [](double value) { return std::isnan(value); }),
            "an unavailable field must remain entirely NaN");

    const BinaryMask disk = diskMask(5, 5, Eigen::Vector2d(2.0, 2.0), 1);
    require(std::count(disk.begin(), disk.end(), static_cast<std::uint8_t>(1)) == 5,
            "a radius-one disk must contain its center and four axial neighbors");

    const HarmonicSolution cold = solveAngularHarmonic(
        open, 5, 5, Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(3.0, 2.0), config);
    require(cold.valid, "cold harmonic solve must succeed");
    const HarmonicSolution warm = solveAngularHarmonic(
        open, 5, 5, Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(3.0, 2.0), config, &cold.potential);
    require(warm.valid, "warm-started harmonic solve must succeed");
    comparePotential(warm.potential, cold.potential, "warm start");
}

void checkContinuousGradient()
{
    constexpr int width = 9;
    constexpr int height = 7;
    BinaryMask blocked(width * height, 0);
    std::vector<double> potential(width * height, 0.0);
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            potential[static_cast<std::size_t>(v) * width + u] =
                1.0 - static_cast<double>(u) / (width - 1);
        }
    }
    const Eigen::Vector2d source(3.25, 3.1);
    const Eigen::Vector2d target = continuousHarmonicTarget(
        potential, blocked, width, height, source,
        Eigen::Vector2d(8.0, 3.0), 3, 2.5);
    require(target.x() > source.x() + 2.4,
            "continuous harmonic descent must follow the fitted x-gradient");
    require(std::abs(target.y() - source.y()) < 1.0e-10,
            "a horizontal potential must not create vertical quantization");
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
    checkConnectivityAndTieBreaking();
    checkHarmonicBranches();
    checkContinuousGradient();
    std::cout << "angular_field_check: passed " << cases.size()
              << " Python fixtures\n";
    return 0;
}
