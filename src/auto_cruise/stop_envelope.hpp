#pragma once
#include <algorithm>
#include <cmath>

namespace cruise_model {
// Bounds for the synchronous simulation plant, in metres, seconds and m/s.
// At most one extra application of the last acknowledged input before braking.
// One extra tick of the same input changes velocity by at most velocity_change.
constexpr double position_error=1e-8, velocity_error=1e-8;

struct StopEnvelope { double length, extra_radius; };

inline StopEnvelope stop_envelope(double speed,double held_speed,double velocity_change,
                                  double dt,double guaranteed_deceleration) {
  const double worst_speed=std::max(speed,held_speed)+velocity_error;
  const double stopping_time=worst_speed/guaranteed_deceleration+dt;
  // A non-expanding straight brake maps v to v*max(0,1-b*dt/|v|).
  // The two stop rays can separate by at most |delta_v| per second.
  const double horizon=dt+stopping_time;
  return {worst_speed*dt + worst_speed*worst_speed/(2*guaranteed_deceleration),
          position_error+velocity_error*(dt+horizon)+velocity_change*horizon};
}
}
