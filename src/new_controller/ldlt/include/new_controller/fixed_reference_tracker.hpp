#pragma once
#include "new_controller/navigation_pipeline.hpp"

namespace ego4p0 {

enum class TrackingStatus {
  Ready, NoReference, InvalidStep, SpeedEntryRequired, OutsideAttachment,
  EndOfReference, ProjectionLost, OutsideTrackingTube, PlantAcceleration, CompensationLimit
};

inline const char *tracking_status_name(TrackingStatus status) {
  switch(status) {
    case TrackingStatus::Ready:return "ready";
    case TrackingStatus::NoReference:return "no_reference";
    case TrackingStatus::InvalidStep:return "invalid_step";
    case TrackingStatus::SpeedEntryRequired:return "speed_entry_required";
    case TrackingStatus::OutsideAttachment:return "outside_attachment";
    case TrackingStatus::EndOfReference:return "end_of_reference";
    case TrackingStatus::ProjectionLost:return "projection_lost";
    case TrackingStatus::OutsideTrackingTube:return "outside_tracking_tube";
    case TrackingStatus::PlantAcceleration:return "plant_acceleration";
    case TrackingStatus::CompensationLimit:return "compensation_limit";
  }
  return "invalid";
}

struct TrackingCommand {
  TrackingStatus status=TrackingStatus::NoReference;
  Direction3 desired_W{},omega_W{},next_velocity_W{},plant_input_W{};
  double s=0.,cross_track_m=0.;
  bool outer_updated=false;
};

// Two intrinsic coordinates in the plane normal to a unit 3D tangent.  The
// signed reflection keeps |sign + tangent.z| >= 1, including both poles.  This
// is a moving local frame, not a fixed world plane or an altitude constraint.
// No frame state is differentiated or carried across updates: after the 2D
// feedback is lifted, the existing world-space derivative remains valid even
// when the frame's sign branch changes at tangent.z == 0.
struct TrackingNormalPlane2D {
  double x,y,sign,a;

  explicit TrackingNormalPlane2D(const Direction3 &unit_tangent)
    :x(unit_tangent[0]),y(unit_tangent[1]),sign(std::copysign(1.,unit_tangent[2])),
     a(-1./(sign+unit_tangent[2])){}

  std::array<double,2> coordinates(const Direction3 &world)const {
    // Apply E^T without materializing its six basis coefficients.
    const double common=a*(x*world[0]+y*world[1])-world[2];
    return {world[0]+sign*x*common,sign*world[1]+y*common};
  }

  Direction3 lift(const std::array<double,2> &local)const {
    const double common=sign*x*local[0]+y*local[1];
    return {local[0]+a*x*common,sign*local[1]+a*y*common,-common};
  }

  static double squared_norm(const std::array<double,2> &local) {
    return local[0]*local[0]+local[1]*local[1];
  }
};

class FixedReferenceTracker {
 public:
  static constexpr double attachment_speed_tolerance_mps=.001;
  static constexpr double max_step_s=NavigationConfig::max_observation_age_sim_s;

  // previous_input and previous_omega are the actual preceding actuator state.
  TrackingStatus attach(const SpatialReference &reference,const Direction3 &position,
                        const Direction3 &velocity,double s,double horizon,
                        const Direction3 &previous_input,const Direction3 &previous_omega) {
    attached_=false;
    if(!reference.valid()||!finite(position)||!std::isfinite(s)||s<0.||
       !std::isfinite(horizon)||horizon<=s)
      return TrackingStatus::NoReference;
    const double speed=nav_norm(velocity);
    if(!std::isfinite(speed)||std::abs(speed-BaseConfig::max_speed_mps)>attachment_speed_tolerance_mps)
      return TrackingStatus::SpeedEntryRequired;
    if(!reference.normal_attachment(position,velocity,s))return TrackingStatus::OutsideAttachment;
    if(!NavigationLimits::compensation_input(previous_input,previous_input,BaseConfig::physics_dt_s)||
       !NavigationLimits::heading_input(previous_omega,previous_omega,BaseConfig::physics_dt_s))
      return TrackingStatus::CompensationLimit;
    reference_=reference;
    s_=s;
    horizon_=horizon;
    last_input_=previous_input;
    last_omega_=previous_omega;
    outer_elapsed_=BaseConfig::control_period_s;
    attached_=true;
    return TrackingStatus::Ready;
  }

  // Call before the physical advance, with its actual step size and measured
  // world state. Invalid steps produce no command and require explicit re-entry.
  TrackingCommand step(const Direction3 &position,const Direction3 &velocity,double dt) {
    if(!attached_)return {};
    if(!std::isfinite(dt)||dt<=0.||dt>max_step_s||!finite(position))
      return fail(TrackingStatus::InvalidStep);
    auto heading=velocity;
    if(!unit_direction(heading))return fail(TrackingStatus::SpeedEntryRequired);

    const bool outer=outer_elapsed_+1e-12>=BaseConfig::control_period_s;
    if(outer) {
      const auto status=update_outer(position,velocity,heading);
      if(status!=TrackingStatus::Ready)return fail(status);
      outer_elapsed_=0.;
    }
    const double next_distance=BaseConfig::max_speed_mps*(outer_elapsed_+dt);
    if(s_+next_distance>horizon_)return fail(TrackingStatus::EndOfReference);

    const auto omega=nav_add(last_omega_,
      limit(nav_add(omega_target_,last_omega_,-1.),NavigationConfig::heading_acceleration_limit_rad_s2*dt));
    const auto next=nav_scale(rotate_heading(heading,omega,dt),BaseConfig::max_speed_mps);
    const auto delta=nav_add(next,velocity,-1.);
    if(nav_norm(delta)>BaseConfig::plant_acceleration_mps2*dt+1e-12)
      return fail(TrackingStatus::PlantAcceleration);
    const double beta=std::min(1.,dt/BaseConfig::plant_tau_s);
    const auto input=nav_add(velocity,delta,1./beta);
    // The existing plant has a distinct zero-target braking branch.
    if(nav_norm(input)<1e-8||!NavigationLimits::compensation_input(input,last_input_,dt))
      return fail(TrackingStatus::CompensationLimit);

    last_omega_=omega;
    last_input_=input;
    outer_elapsed_+=dt;
    return {TrackingStatus::Ready,desired_,omega,next,input,s_,cross_track_,outer};
  }

  const SpatialReference &reference()const{return reference_;}
  double progress()const{return s_;}
  void set_horizon(double horizon){horizon_=horizon;}

  // Commit a compatible splice in one coordinator operation. Keep the actuator
  // state and force the next outer update to use the new geometry.
  void replace(const SpatialReference &reference,double s,double horizon){
    reference_=reference;
    s_=s;
    horizon_=horizon;
    outer_elapsed_=BaseConfig::control_period_s;
  }

  void detach(){attached_=false;}
  bool attached()const{return attached_;}

 private:
  static bool finite(const Direction3 &v) {
    return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});
  }

  static Direction3 limit(Direction3 v,double maximum) {
    const double length=nav_norm(v);
    return length>maximum?nav_scale(v,maximum/length):v;
  }

  static Direction3 rotate_heading(Direction3 h,const Direction3 &omega,double dt) {
    const double rate=nav_norm(omega);
    if(rate==0.)return h;
    const auto axis=nav_scale(omega,1./rate);
    const auto first=nav_cross(axis,h);
    const double angle=rate*dt;
    h=nav_add(nav_add(h,first,std::sin(angle)),nav_cross(axis,first),1.-std::cos(angle));
    unit_direction(h);
    return h;
  }

  TrackingCommand fail(TrackingStatus why) {
    detach();
    TrackingCommand result;
    result.status=why;
    result.s=s_;
    result.cross_track_m=cross_track_;
    return result;
  }

  TrackingStatus update_outer(const Direction3 &position,const Direction3 &velocity,
                              const Direction3 &heading) {
    const double lo=std::max(0.,s_-.05),hi=std::min(horizon_,s_+.25);
    if(hi<=lo)return TrackingStatus::EndOfReference;
    double projected=reference_.project_chords(position,lo,hi);
    auto sample=reference_.query(projected);
    auto error=nav_add(position,sample.position_W,-1.);
    double along=direction_dot(error,sample.tangent_W);
    double denominator=1.-direction_dot(error,sample.curvature_W);
    if(!(denominator>.5))return TrackingStatus::ProjectionLost;
    const double refined=std::clamp(projected+along/denominator,lo,hi);
    if(std::abs(refined-projected)>1e-5) {
      projected=refined;
      sample=reference_.query(projected);
      error=nav_add(position,sample.position_W,-1.);
      along=direction_dot(error,sample.tangent_W);
      denominator=1.-direction_dot(error,sample.curvature_W);
    }

    constexpr double residual_limit=.001;
    if(!(denominator>.5)||(std::abs(along)>residual_limit&&
       !(projected==0.&&along<0.)))return TrackingStatus::ProjectionLost;
    auto tangent=sample.tangent_W;
    if(!unit_direction(tangent))return TrackingStatus::ProjectionLost;
    const TrackingNormalPlane2D normal_plane(tangent);
    const auto transverse=normal_plane.coordinates(error);
    s_=projected;
    cross_track_=std::hypot(transverse[0],transverse[1])+sample.position_error_bound_m;
    if(!(cross_track_<=NavigationConfig::tracking_error_target_m))
      return TrackingStatus::OutsideTrackingTube;

    constexpr double speed=BaseConfig::max_speed_mps;
    constexpr double lateral_gain=NavigationConfig::lateral_gain_per_s;
    const std::array<double,2> correction_local{-lateral_gain*transverse[0],-lateral_gain*transverse[1]};
    const auto correction=normal_plane.lift(correction_local);
    const double tangential_speed=std::sqrt(speed*speed-TrackingNormalPlane2D::squared_norm(correction_local));
    desired_=nav_add(nav_scale(tangent,tangential_speed),correction);

    // Differentiate r, its tangential part, and the constant-speed feedback.
    // The implicit projection equation gives ds/dt=(v·T)/(1-r·curvature).
    const double s_rate=projected==0.&&along<0.?0.:direction_dot(velocity,tangent)/denominator;
    const auto tangent_rate=nav_scale(sample.curvature_W,s_rate);
    const auto error_rate=nav_add(velocity,tangent,-s_rate);
    const double along_rate=direction_dot(error_rate,tangent)+direction_dot(error,tangent_rate);
    const auto normal_rate=nav_add(nav_add(error_rate,tangent,-along_rate),tangent_rate,-along);
    const auto correction_rate=nav_scale(normal_rate,-lateral_gain);
    const double tangential_rate=-direction_dot(correction,correction_rate)/tangential_speed;
    const auto direction=nav_scale(desired_,1./speed);
    const auto direction_rate=nav_scale(nav_add(
      nav_add(correction_rate,tangent,tangential_rate),tangent_rate,tangential_speed),1./speed);
    omega_target_=limit(nav_add(nav_cross(direction,direction_rate),nav_cross(heading,direction),
                               NavigationConfig::heading_gain_per_s),NavigationConfig::heading_rate_limit_rad_s);
    return finite(omega_target_)?TrackingStatus::Ready:TrackingStatus::ProjectionLost;
  }

  SpatialReference reference_;
  Direction3 desired_{},omega_target_{},last_input_{},last_omega_{};
  double s_=0.,horizon_=0.,cross_track_=0.,outer_elapsed_=0.;
  bool attached_=false;
};

} // namespace ego4p0
