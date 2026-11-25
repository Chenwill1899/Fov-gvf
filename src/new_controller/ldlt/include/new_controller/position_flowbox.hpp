#pragma once

#include "new_controller/spherical_geometry.hpp"

namespace ego4p0 {

// Local harmonic field of the osculating circle, or of parallel lines at
// zero curvature. Inputs are expressed as bearings and metric radii.
class PositionFlowBox {
  using Bearing = spherical::Bearing;
  using Radial = spherical::Radial;
  using Tangent = spherical::Tangent;

 public:
  struct Coordinates {
    double phase_m = 0., normal_m = 0., binormal_m = 0.;
    double cos_phase = 1., sin_phase = 0., radius_ratio = 1.;
  };
  struct Guidance {
    Bearing desired;
    Radial direction_rate;
    Radial angular_velocity(Bearing current, double heading_gain) const {
      const auto feedforward = spherical::scale(spherical::cross(desired, direction_rate.dir), direction_rate.r);
      return spherical::add(feedforward, spherical::cross(current, desired), heading_gain);
    }
  };

  PositionFlowBox(Bearing heading, Radial curvature)
      : heading_(heading), curvature_(curvature.r) {
    if (curvature_ > 0.) {
      auto n = spherical::separation(heading_, curvature.dir).toward;
      double length = std::hypot(n.east, n.north);
      if (length > 0.) normal_ = {n.east / length, n.north / length};
    }
  }

  bool coordinates(Radial offset, Coordinates &out) const {
    const auto local = components(offset);
    const double x = local.along, y = local.normal;
    const double a = 1. - curvature_ * y, b = curvature_ * x;
    // Select the local branch of atan2; stay away from the circle axis.
    if (!(a > .5)) return false;
    const double d = std::hypot(a, b);
    out.cos_phase = a / d;
    out.sin_phase = b / d;
    out.radius_ratio = d;
    out.phase_m = curvature_ > 0. ? std::atan2(b, a) / curvature_ : x;
    // Stable form of (1 - d)/curvature, including the straight-line limit.
    out.normal_m = (2. * y - curvature_ * (x*x + y*y)) / (1. + d);
    out.binormal_m = local.binormal;
    return std::isfinite(out.phase_m) && std::isfinite(out.normal_m) &&
           std::isfinite(out.binormal_m);
  }

  // The two rows of the transverse Jacobian are orthonormal, so its
  // right pseudoinverse maps the two feedback components onto n and b.
  Guidance guidance(const Coordinates &p, Radial velocity, double speed, double gain,
                    bool with_rate = true) const {
    if (speed <= 0.) return {heading_, {}};
    double wn = -gain*p.normal_m, wb = -gain*p.binormal_m;
    const double magnitude = std::hypot(wn, wb);
    // At low speed reserve a forward component and bound J*v by the available speed.
    const double factor = magnitude > .95*speed ? .95*speed/magnitude : 1.;
    wn *= factor;
    wb *= factor;
    const double vt = std::sqrt(speed*speed - wn*wn - wb*wb);
    const auto desired = from_rotated(p, vt, wn, wb).dir;
    if (!with_rate) return {desired, {}};

    const auto v = components(velocity);
    const double v_along = p.cos_phase*v.along + p.sin_phase*v.normal;
    const double v_normal = -p.sin_phase*v.along + p.cos_phase*v.normal;
    const double omega = curvature_ * v_along / p.radius_ratio;
    double dwn = -gain*v_normal, dwb = -gain*v.binormal;
    if (factor < 1.) {
      const double n = wn/(.95*speed), b = wb/(.95*speed);
      const double parallel = n*dwn + b*dwb;
      dwn = factor*(dwn - parallel*n);
      dwb = factor*(dwb - parallel*b);
    }
    const double dvt = -(wn*dwn + wb*dwb) / vt;
    const auto derivative = from_rotated(p, (dvt - omega*wn)/speed,
                                         (dwn + omega*vt)/speed, dwb/speed);
    return {desired, derivative};
  }

 private:
  struct Components { double along, normal, binormal; };
  Components components(Radial v) const {
    const auto s = spherical::separation(heading_, v.dir);
    return {v.r*s.cos_arc,
            v.r*(normal_.east*s.toward.east + normal_.north*s.toward.north),
            v.r*(normal_.east*s.toward.north - normal_.north*s.toward.east)};
  }
  Radial from_rotated(const Coordinates &p, double t, double n, double b) const {
    const double along = p.cos_phase*t - p.sin_phase*n;
    const double normal = p.sin_phase*t + p.cos_phase*n;
    const double lateral = std::hypot(normal, b);
    if (lateral == 0.) return spherical::scale({heading_, 1.}, along);
    const Tangent axis{(normal*normal_.east - b*normal_.north)/lateral,
                       (normal*normal_.north + b*normal_.east)/lateral};
    return {spherical::advance(heading_, axis, std::atan2(lateral, along)),
            std::hypot(along, lateral)};
  }
  Bearing heading_;
  double curvature_;
  Tangent normal_{1., 0.};
};
} // namespace ego4p0
