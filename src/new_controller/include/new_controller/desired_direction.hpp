#pragma once
#include "new_controller/depth_input.hpp"

namespace ego4p0 {
enum class DesiredInputMode { None,BodyVelocity,WorldVelocity,WorldGoal };
struct DesiredInput {
  DesiredInputMode mode=DesiredInputMode::None;
  std::array<double,3> vector{}; // u_B, u_W, or g_W, according to mode
  double goal_speed_mps=10.;
  int64_t stamp_ns=0;
};
struct DesiredMotion {
  std::array<double,3> d_desired{};
  double v_desired=0.;
  bool has_direction=false,valid=true;
};
inline DesiredMotion desired_motion(const DesiredInput &in,const BodyState &state) noexcept {
  DesiredMotion out;
  if(in.mode==DesiredInputMode::None) return out;
  std::array<double,3> h{},source=in.vector;
  for(size_t k=0;k<3;++k) if(!std::isfinite(source[k])) {out.valid=false;return out;}
  if(in.mode==DesiredInputMode::WorldGoal) {
    if(!std::isfinite(in.goal_speed_mps)) {out.valid=false;return out;}
    for(size_t k=0;k<3;++k) {if(!std::isfinite(state.p_W[k])) {out.valid=false;return out;}source[k]-=state.p_W[k];}
  }
  if(in.mode==DesiredInputMode::BodyVelocity) h=source;
  else if(in.mode==DesiredInputMode::WorldVelocity || in.mode==DesiredInputMode::WorldGoal) {
    for(size_t k=0;k<3;++k) for(size_t j=0;j<3;++j) {
      if(!std::isfinite(state.R_WB[j*3+k])) {out.valid=false;return out;}
      h[k]+=state.R_WB[j*3+k]*source[j];
    }
  } else {out.valid=false;return out;}
  const double length=std::hypot(h[0],h[1],h[2]);
  if(!std::isfinite(length)) {out.valid=false;return out;}
  if(length<=1e-9) return out; // zero target speed immediately, no normalization
  for(size_t k=0;k<3;++k) out.d_desired[k]=h[k]/length;
  out.v_desired=std::clamp(in.mode==DesiredInputMode::WorldGoal ? in.goal_speed_mps : length,0.,ControllerLimits::max_speed_mps);
  out.has_direction=true;return out;
}
// For the future gradient-integrated reference, AFTER its direction is known.
// Vectors must be in one fixed frame. This is not applied to d_desired/v_desired.
inline std::array<double,3> acceleration_limited_reference(const std::array<double,3> &previous,
                                                         std::array<double,3> target) {
  for(size_t k=0;k<3;++k) if(!std::isfinite(previous[k]) || !std::isfinite(target[k]))
    throw std::invalid_argument("nonfinite reference velocity");
  if(std::hypot(previous[0],previous[1],previous[2])>ControllerLimits::max_speed_mps+1e-12)
    throw std::invalid_argument("previous reference exceeds speed limit");
  const double speed=std::hypot(target[0],target[1],target[2]);
  if(!std::isfinite(speed)) throw std::invalid_argument("reference speed overflow");
  if(speed>ControllerLimits::max_speed_mps) for(auto &x:target) x*=ControllerLimits::max_speed_mps/speed;
  std::array<double,3> delta{};for(size_t k=0;k<3;++k) delta[k]=target[k]-previous[k];
  const double change=std::hypot(delta[0],delta[1],delta[2]);
  const double bound=ControllerLimits::max_acceleration_mps2*ControllerLimits::period_s;
  if(change>bound) for(size_t k=0;k<3;++k) target[k]=previous[k]+delta[k]*(bound/change);
  return target;
}
} // namespace ego4p0
