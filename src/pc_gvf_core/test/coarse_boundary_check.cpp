#include "fluid/fluid_solver_2d.h"
#include "fluid/fluid_solver_3d.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "coarse_boundary_check: " << message << '\n';
        std::exit(1);
    }
}

void check3D() {
    using namespace FLAG_Race::fluid3d;
    Grid3D grid;
    grid.origin_xy = Eigen::Vector2d(2.0, -3.0);
    grid.e_J = Eigen::Vector2d(0.6, 0.8);
    grid.n_J = Eigen::Vector2d(-0.8, 0.6);
    grid.z_lo = 1.0;
    grid.h = 0.4; grid.h_z = 0.2;
    grid.n_fwd = 5; grid.n_lat = 4; grid.n_z = 3;
    std::vector<double> phi(grid.size());
    for (int i = 0; i < grid.n_fwd; ++i)
        for (int j = 0; j < grid.n_lat; ++j)
            for (int m = 0; m < grid.n_z; ++m)
                phi[grid.idx(i, j, m)] = 1.0 + i + 2.0 * j + 3.0 * m;
    CoarsePrior3D prior;
    prior.grid = grid; prior.phi = &phi;
    auto world = [&](double s, double t, double z) {
        const Eigen::Vector2d xy = grid.origin_xy + s * grid.e_J + t * grid.n_J;
        return Eigen::Vector3d(xy.x(), xy.y(), grid.z_lo + z);
    };
    double value = -123.0;
    require(prior.sample(world(1.0 * grid.h, 1.0 * grid.h, 1.0 * grid.h_z), value),
            "interior interpolation must succeed");
    require(std::abs(value - 4.0) < 1e-10, "trilinear interpolation of affine field");
    const double limits[] = {grid.n_fwd * grid.h, grid.n_lat * grid.h, grid.n_z * grid.h_z};
    for (int axis = 0; axis < 3; ++axis) {
        for (int side = 0; side < 2; ++side) {
            Eigen::Vector3d local(grid.h, grid.h, grid.h_z);
            local[axis] = side ? limits[axis] : 0.0;
            require(prior.sample(world(local.x(), local.y(), local.z()), value),
                    "shared half-cell boundary fringe must remain valid");
            local[axis] += side ? 0.01 : -0.01;
            value = -123.0;
            require(!prior.sample(world(local.x(), local.y(), local.z()), value),
                    "genuinely outside coarse domain must be rejected on every face");
            require(value == -123.0, "rejected sample must leave output untouched");
        }
    }
    Grid3D fine = grid;
    fine.origin_xy -= 2.0 * grid.h * grid.e_J;
    const std::vector<uint8_t> solid(fine.size(), 0);
    const auto system = buildPoissonSystem3D(solid, fine, 1.0, 0.0, &prior);
    require(system.phi_bc[fine.idx(0, 1, 1)] == 0.0,
            "out-of-domain fine boundary must use far-field fallback, not coarse edge");
}

void check2D() {
    using namespace FLAG_Race::fluid2d;
    constexpr int n = 12;
    const double h = 0.25, speed = 2.0;
    const Eigen::Vector2d origin(0.0, -1.5), e(1.0, 0.0), normal(0.0, 1.0), anchor(0.0, 0.0);
    std::vector<std::vector<bool>> solid(n, std::vector<bool>(n, false));
    for (int i = 0; i < n; ++i) solid[i][1] = true;
    auto components = labelSolidComponents(solid, origin, e, normal, h, anchor, n, n);
    auto solve = [&](const std::vector<std::vector<bool>>& mask, const SolidComponents& comps,
                     const FluidCoarsePrior* prior) {
        return solveHarmonicStreamField(mask, comps, origin, e, normal, h, n, n,
            anchor, speed, std::vector<double>(comps.num_components, 1.0),
            3.0, 1.0, 0.1, 1e-8, 1000, 10.0, prior);
    };
    const auto wall = solve(solid, components, nullptr);
    require(wall.valid, "wall solve must converge");
    require(wall.chosen_sides[0] == 0.0, "spanning wall must not receive a bypass side");
    const double ambient = speed * components.centroid_world[0].y();
    require(std::abs(wall.psi(6, 1) - ambient) < 1e-10, "wall must receive ambient stream function");
    require(std::abs(wall.Ux(6, 6) - speed) < 1e-5 && std::abs(wall.Uy(6, 6)) < 1e-5,
            "parallel wall must preserve forward ambient flow away from the wall");

    solid.assign(n, std::vector<bool>(n, false)); solid[6][7] = true;
    components = labelSolidComponents(solid, origin, e, normal, h, anchor, n, n);
    const auto closed = solve(solid, components, nullptr);
    require(closed.valid && closed.chosen_sides[0] == 1.0,
            "closed obstacle must retain requested bypass side");
    require(std::abs(closed.psi(6, 7) + speed * 3.0) < 1e-10,
            "closed obstacle must retain bypass-offset boundary value");
    Eigen::MatrixXd constant = Eigen::MatrixXd::Constant(n, n, 0.75);
    FluidCoarsePrior prior;
    prior.psi = &constant; prior.origin = origin; prior.e_J = e; prior.n_J = normal;
    prior.h = h; prior.n_forward = prior.n_lateral = n;
    const auto nested = solve(solid, components, &prior);
    require(nested.valid && std::abs(nested.psi(6, 7) - 0.75) < 1e-10,
            "closed obstacle must still inherit a valid coarse prior");
}
}  // namespace

int main(int argc, char** argv) {
    const bool only2D = argc > 1 && std::string(argv[1]) == "2d";
    const bool only3D = argc > 1 && std::string(argv[1]) == "3d";
    if (!only2D) check3D();
    if (!only3D) check2D();
    std::cout << "coarse_boundary_check: passed\n";
}
