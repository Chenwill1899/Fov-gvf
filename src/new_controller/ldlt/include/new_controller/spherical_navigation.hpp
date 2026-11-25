#pragma once

#include "new_controller/rolling_navigation.hpp" // public status codes and timing constants
#include "new_controller/spherical_reference.hpp"
#include "new_controller/position_flowbox.hpp"

namespace ego4p0 {

struct SphericalTick {
  RollingMode mode=RollingMode::Protection;
  RollingReason reason=RollingReason::Input;
  bool committed=false,flowbox_valid=false;
  double flowbox_normal_m=0.,flowbox_binormal_m=0.;
  int64_t planning_ns=0,certification_ns=0,control_ns=0;
  spherical::Radial next_velocity{},plant_input{},omega{};
};

// Synchronous four-camera Isaac protocol. Its only rotational degree of freedom
// is yaw. Validate the protocol orientation before using longitude addition.
class SphericalNavigation {
  using Radial=spherical::Radial;
  using Bearing=spherical::Bearing;
  using Sample=SphericalReference::Sample;
  static constexpr double speed=BaseConfig::max_speed_mps;
  static constexpr double dt=BaseConfig::physics_dt_s;
 public:
  void configure(const std::array<CameraCalibration,4>&c,uint32_t w,uint32_t h,uint64_t id,bool defer_unused_panorama=false){
    defer_unused_panorama_=defer_unused_panorama;
    pipeline_.configure(c,w,h,id,true);reset_epoch();
  }
  void allocate(NavigationObservation &o)const{pipeline_.allocate(o);}
  void enable_fifo5(){pipeline_.enable_fifo5();}
  FrameObservationBuilder &builder(){return pipeline_.builder();}
  const NavigationOutput& output()const{return out_;}
  const HarmonicResult& field()const{return pipeline_.field();}
  Bearing reference_heading(double s)const{return candidate_.heading(s);}
  void interlock(){attached_=false;prefix_={};}
  void reset_epoch(){
    interlock();pipeline_.reset_epoch();out_={};last_clock_=last_state_=-1;have_epoch_=false;
    previous_velocity_.reset();
  }

  const NavigationOutput &process(NavigationObservation &o,int64_t now,Radial position,Radial velocity,
                                  Bearing desired_body,bool has_direction,bool request_plan) {
    return plan(o,now,position,velocity,desired_body,has_direction,request_plan,nullptr,true);
  }

  const NavigationOutput &angular_step(NavigationObservation &o,int64_t now,Radial position,Radial velocity,
                                       Bearing desired_body,bool valid_goal,bool request_grid=true) {
    return plan(o,now,position,velocity,desired_body,valid_goal,request_grid,nullptr,false);
  }

  template<class Oracle>
  SphericalTick rolling_tick(NavigationObservation &o,int64_t now,Radial position,Radial velocity,
                             Bearing desired_body,Radial old_input,Radial old_omega,Oracle &&oracle) {
    SphericalTick tick;const int64_t stamp=o.observation_pose.t_ns;
    auto reject=[&](RollingReason why){interlock();tick.reason=why;return tick;};
    if((last_clock_>=0&&now<last_clock_) || (last_state_>=0&&stamp<last_state_) ||
       (have_epoch_ && epoch_!=o.receive_epoch)){
      reset_epoch();tick.mode=RollingMode::TimeReset;return reject(RollingReason::Timing);
    }
    epoch_=o.receive_epoch;have_epoch_=true;last_clock_=now;
    if(last_state_>=0 && stamp!=last_state_ && stamp-last_state_!=BaseConfig::period_ns){
      last_state_=stamp;return reject(RollingReason::Timing);
    }
    const bool repeated=stamp==last_state_;last_state_=stamp;
    if(!valid(position)||!valid(velocity)||!valid(old_input)||!valid(old_omega)||
       std::abs(velocity.r-speed)>FixedReferenceTracker::attachment_speed_tolerance_mps+1e-12||
       old_input.r>NavigationConfig::compensation_input_limit_mps+1e-12||
       old_omega.r>NavigationConfig::heading_rate_limit_rad_s+1e-12)return reject(RollingReason::Input);
    const double yaw=observer_yaw(o);
    const Bearing world_intent{spherical::wrap(desired_body.az+yaw),desired_body.el};
    const auto begin=field_now_ns();
    // Preprocess the incoming group before any new or continuing proof reads it.
    process(o,now,position,velocity,desired_body,true,false);
    if(!out_.input_ready){tick.planning_ns=field_now_ns()-begin;return reject(RollingReason::Input);}

    Sample join{},anchor{};bool needs_plan=!attached_;
    if(attached_){
      const auto cert=field_now_ns();
      prefix_=certify(active_,progress_,oracle);
      tick.certification_ns+=field_now_ns()-cert;
      if(prefix_.end<progress_+speed*dt || spherical::angle(world_intent,active_intent_)>RollingNavigator::intent_change_rad){
        join=active_.sample(progress_);
        needs_plan=join.curvature.r<=RollingReference::connection_tolerance;
      }
    }
    if(needs_plan){
      const auto pre_ns=out_.preprocessing_ns;
      plan(o,now,position,velocity,desired_body,true,true,attached_?&join:nullptr,true);
      out_.preprocessing_ns+=pre_ns;
      if(out_.spatial_ready){
        const auto cert=field_now_ns();
        const auto offer=certify(candidate_,0.,oracle);
        const auto entry=candidate_.sample(0.);
        const bool splice_ok=!attached_ ||
          (spherical::distance(join.position,entry.position)<=RollingReference::connection_tolerance &&
           spherical::angle(join.heading,entry.heading)<=RollingReference::connection_tolerance &&
           spherical::distance(join.curvature,entry.curvature)<=RollingReference::connection_tolerance);
        if(offer.end>=speed*dt && splice_ok &&
           spherical::distance(position,entry.position)<=NavigationConfig::normal_attach_position_error_m &&
           spherical::angle(velocity.dir,entry.heading)<=NavigationConfig::normal_attach_angle_deg*direction_pi/180.){
          active_=candidate_;active_intent_=world_intent;attached_=true;
          prefix_=offer;progress_=0.;tick.committed=true;anchor=entry;
        }
        tick.certification_ns+=field_now_ns()-cert;
      }
    }
    tick.planning_ns=field_now_ns()-begin-tick.certification_ns;
    if(!attached_ || prefix_.end<progress_+speed*dt)return reject(RollingReason::Prefix);
    if(repeated){tick.reason=RollingReason::None;tick.mode=RollingMode::RepeatedState;return tick;}
    const auto control=field_now_ns();
    const double anchor_s=tick.committed?0.:std::min(progress_+speed*dt,prefix_.end);
    if(!tick.committed)anchor=active_.sample(anchor_s);
    const auto reason=track(anchor,anchor_s,position,velocity,old_input,old_omega,tick);
    tick.control_ns=field_now_ns()-control;
    if(reason!=RollingReason::None)return reject(reason);
    tick.reason=RollingReason::None;tick.mode=RollingMode::Tracking;return tick;
  }

 private:
  bool defer_unused_panorama_=false;
  static bool valid(Radial p){return std::isfinite(p.r)&&p.r>=0.&&std::isfinite(p.dir.az)&&
    std::isfinite(p.dir.el)&&std::abs(p.dir.el)<=direction_pi/2;}
  static double observer_yaw(const NavigationObservation &o){
    return std::atan2(o.observation_pose.R_WB[3],o.observation_pose.R_WB[0]);
  }
  const NavigationOutput &plan(NavigationObservation &o,int64_t now,Radial position,Radial velocity,
                               Bearing goal,bool has_direction,bool request,const Sample *connection,bool spatial) {
    const auto &r=o.observation_pose.R_WB;
    const bool yaw_only=std::abs(r[2])+std::abs(r[5])+std::abs(r[6])+std::abs(r[7])+std::abs(r[8]-1.)<1e-10;
    NavigationState state;state.actual=o.observation_pose;state.actual_valid=o.observation_pose_valid&&
      valid(position)&&valid(velocity)&&yaw_only;state.current_simulation_ns=now;
    const double yaw=observer_yaw(o);
    const auto previous=previous_velocity_.value_or(Bearing{spherical::wrap(goal.az+yaw),goal.el});
    const auto heading=connection?connection->heading:(velocity.r>1e-9?velocity.dir:previous);
    const AngularIntent intent{{spherical::wrap(heading.az-yaw),heading.el},{spherical::wrap(goal.az),goal.el}};
    const DesiredMotion flags{{},speed,true,has_direction};
    out_=pipeline_.process_if(o,state,flags,false,false,[&]{return NavigationPlan{request,false,nullptr};},intent,
                              defer_unused_panorama_ && !request);
    out_.spatial_enabled=request && spatial;
    if(out_.input_ready && velocity.r>1e-9)previous_velocity_=velocity.dir;
    if(spatial && out_.angular_ready){
      const auto start=field_now_ns();
      out_.spatial_ready=candidate_.build(pipeline_.field(),connection?connection->position:position,yaw);
      if(out_.spatial_ready)out_.turn_length_m=candidate_.length();
      else out_.invalid=NavigationInvalid::InvalidSpatialCurve;
      out_.spatial_ns=field_now_ns()-start;out_.total_ns+=out_.spatial_ns;
    }
    return out_;
  }

  struct Ball {Radial center;double radius;};
  struct Prefix {double end=0.;std::array<Ball,11> balls{};size_t count=0;};
  template<class Oracle>
  static Prefix certify(const SphericalReference &ref,double s,Oracle &oracle) {
    constexpr double step=SpatialReferenceConfig::lookup_step_m;
    const double start=std::max(0.,std::floor((s-.05)/step)*step),end=s+.35;
    Prefix result;result.end=start;auto left=ref.sample(start);
    for(size_t i=1;i<=result.balls.size() && result.end<end-1e-12;++i){
      const double next=std::min(end,start+i*step);const auto right=ref.sample(next);
      const double h=next-result.end;const auto chord=spherical::add(right.position,left.position,-1.);
      const auto middle=spherical::add(left.position,chord,.5);
      // Linear interpolation error <= max|gamma''| h²/8, plus position
      // quadrature errors at the two ends (convex interpolation).
      const double bend=result.end<ref.length()?NavigationConfig::reference_max_curvature_per_m*h*h/8.:0.;
      const double radius=NavigationConfig::occupancy_tube_radius_m+chord.r*.5+bend+
        std::max(left.error,right.error)+128*std::numeric_limits<double>::epsilon()*(1.+middle.r);
      if(oracle(middle,radius)!=TubeStatus::ObservedClear)break;
      result.balls[result.count++]={middle,radius};result.end=next;left=right;
    }
    return result;
  }
  static Radial limit(Radial p,double cap){p.r=std::min(p.r,cap);return p;}
  RollingReason track(const Sample &anchor,double anchor_s,Radial position,Radial velocity,
                      Radial old_input,Radial old_omega,SphericalTick &tick){
    const PositionFlowBox box(anchor.heading,anchor.curvature);
    const auto offset=spherical::add(position,anchor.position,-1.);
    PositionFlowBox::Coordinates coords;
    if(!box.coordinates(offset,coords))return RollingReason::Controller;
    tick.flowbox_valid=true;
    tick.flowbox_normal_m=coords.normal_m;tick.flowbox_binormal_m=coords.binormal_m;
    const double station=anchor_s+coords.phase_m;
    const double projected=std::max(0.,station);
    if(projected+speed*dt>prefix_.end)return RollingReason::Prefix;
    if(projected<std::max(0.,progress_-.05) || projected>progress_+.25)return RollingReason::Controller;

    // The exact reference and the local circle share position and tangent at
    // the anchor. Integrating their curvature bounds twice encloses their gap.
    const double bend=.5*(NavigationConfig::reference_max_curvature_per_m+anchor.curvature.r)*
                      coords.phase_m*coords.phase_m;
    const double error=std::hypot(coords.normal_m,coords.binormal_m)+bend+anchor.error+std::max(0.,-station);
    if(error>NavigationConfig::tracking_error_target_m)return RollingReason::Controller;
    const auto command=box.guidance(coords,velocity,speed,NavigationConfig::lateral_gain_per_s);
    const auto target=limit(command.angular_velocity(velocity.dir,NavigationConfig::heading_gain_per_s),
                            NavigationConfig::heading_rate_limit_rad_s);
    const auto omega=spherical::add(old_omega,limit(spherical::add(target,old_omega,-1.),NavigationConfig::heading_acceleration_limit_rad_s2*dt));
    const Radial next{spherical::rotate_about(omega.dir,velocity.dir,omega.r*dt),speed};
    const auto delta=spherical::add(next,velocity,-1.);
    const auto input=spherical::add(velocity,delta,BaseConfig::plant_tau_s/dt);
    if(!valid(input)||delta.r>BaseConfig::plant_acceleration_mps2*dt+1e-12 ||input.r<1e-8 ||
       input.r>NavigationConfig::compensation_input_limit_mps+1e-12 ||
       spherical::distance(input,old_input)>NavigationConfig::compensation_slew_limit_mps2*dt+1e-12)return RollingReason::Controller;
    const auto end=spherical::add(position,next,dt);
    for(size_t i=0;i<prefix_.count;++i){
      const auto &ball=prefix_.balls[i];const double margin=ball.radius-BaseConfig::body_radius_m;
      if(spherical::distance(position,ball.center)<=margin && spherical::distance(end,ball.center)<=margin){
        progress_=projected;tick.omega=omega;tick.next_velocity=next;tick.plant_input=input;return RollingReason::None;
      }
    }
    return RollingReason::SweptStep;
  }
  NavigationPipeline pipeline_;
  NavigationOutput out_;
  SphericalReference candidate_,active_;
  std::optional<Bearing> previous_velocity_;
  Bearing active_intent_{};
  Prefix prefix_;
  double progress_=0.;
  bool attached_=false,have_epoch_=false;
  uint64_t epoch_=0;
  int64_t last_clock_=-1,last_state_=-1;
};
} // namespace ego4p0
