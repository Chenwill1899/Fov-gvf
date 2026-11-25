#pragma once
#include "new_controller/fixed_reference_tracker.hpp"

namespace ego4p0 {

enum class RollingMode { Tracking, Protection, RepeatedState, TimeReset };
enum class RollingReason { None, Input, Timing, Prefix, Controller, SweptStep };

inline const char *rolling_mode_name(RollingMode mode) {
  switch(mode) {
    case RollingMode::Tracking:return "tracking";
    case RollingMode::Protection:return "protection";
    case RollingMode::RepeatedState:return "repeated_state";
    case RollingMode::TimeReset:return "time_reset";
  }
  return "invalid";
}

struct RollingActuatorState {
  Direction3 input_W{},omega_W{};
};

struct RollingPrefix {
  double begin=0.,end=0.;
  unsigned balls=0,clear_count=0;
  std::array<TubeSection,11> clear_balls{};
};

template<class Oracle>
RollingPrefix certify_rolling_prefix(const SpatialReference &reference,double s,Oracle &&oracle) {
  constexpr double step=SpatialReferenceConfig::lookup_step_m;
  constexpr double ahead=.35;
  const double first=std::max(0.,std::floor((s-.05)/step)*step);
  const double last=s+ahead;
  const double curvature=NavigationConfig::reference_max_curvature_per_m*reference.tangent_norm_bound();
  const double curved_through=CurvatureSchedule::make(reference.profile().zeta_end+
    reference.angular_path().length_error()).length;
  RollingPrefix result{first,first,0};
  auto left=reference.position(first);
  const double first_index=std::round(first/step);
  // Include the last partial section, and stop at the first unproven ball.
  for(unsigned index=1;index<=result.clear_balls.size()&&left.s<last-1e-12;++index){
    auto right=reference.position(std::min(last,(first_index+index)*step));
    if(right.s<=left.s)break;
    const auto ball=reference_cover(left,right,curvature,curved_through);
    ++result.balls;
    if(oracle(ball.cover_center_W,ball.cover_radius_m)!=TubeStatus::ObservedClear)break;
    result.clear_balls[result.clear_count++]=ball;
    result.end=right.s;
    left=right;
  }
  return result;
}

class RollingReference {
 public:
  static constexpr double connection_tolerance=1e-7;

  template<class Oracle>
  bool renew(Oracle &&oracle) {
    if(!tracker_.attached())return false;
    prefix_=certify_rolling_prefix(tracker_.reference(),tracker_.progress(),oracle);
    tracker_.set_horizon(prefix_.end);
    return prefix_.end>tracker_.progress();
  }

  template<class Oracle>
  bool install(const SpatialReference &candidate,double join_s,const Direction3 &position,
               const Direction3 &velocity,const RollingActuatorState &actuator,double dt,Oracle &&oracle) {
    last_offer_balls_=0;
    if(!candidate.valid()||!std::isfinite(join_s)||join_s<0.)return false;
    const bool replacing=tracker_.attached();
    if(replacing){
      const auto old=connection();
      const auto next=candidate.query(join_s);
      if(nav_norm(nav_add(old.position_W,next.position_W,-1.))>connection_tolerance||
         nav_angle(old.tangent_W,next.tangent_W)>connection_tolerance||
         nav_norm(nav_add(old.curvature_W,next.curvature_W,-1.))>connection_tolerance||
         !candidate.normal_attachment(position,velocity,join_s))return false;
    }
    const auto proof=certify_rolling_prefix(candidate,join_s,oracle);
    last_offer_balls_=proof.balls;
    if(proof.end<join_s+BaseConfig::max_speed_mps*dt)return false;
    if(replacing){
      const double global=progress();
      tracker_.replace(candidate,join_s,proof.end);
      offset_=global-join_s;
    }else{
      if(tracker_.attach(candidate,position,velocity,join_s,proof.end,actuator.input_W,actuator.omega_W)
         !=TrackingStatus::Ready)return false;
      // A recovery joins a newly certified path at the current physical state.
      offset_=last_progress_-join_s;
    }
    prefix_=proof;
    ++version_;
    return true;
  }

  bool can_step(double dt)const {
    return tracker_.attached()&&prefix_.begin<=tracker_.progress()&&
      prefix_.end>=tracker_.progress()+BaseConfig::max_speed_mps*dt;
  }

  TrackingCommand step(const Direction3 &position,const Direction3 &velocity,double dt) {
    auto command=tracker_.step(position,velocity,dt);
    if(command.status==TrackingStatus::Ready)
      last_progress_=std::max(last_progress_,offset_+command.s);
    return command;
  }

  // A single observed sphere that contains both vehicle endpoint balls
  // contains the swept straight step by convexity.
  bool swept_step_clear(const Direction3 &position,const Direction3 &next,double dt)const {
    const auto endpoint=nav_add(position,next,dt);
    for(unsigned i=0;i<prefix_.clear_count;++i){
      const auto &ball=prefix_.clear_balls[i];
      const double margin=ball.cover_radius_m-BaseConfig::body_radius_m;
      const auto a=nav_add(position,ball.cover_center_W,-1.);
      const auto b=nav_add(endpoint,ball.cover_center_W,-1.);
      if(direction_dot(a,a)<=margin*margin&&direction_dot(b,b)<=margin*margin)return true;
    }
    return false;
  }

  SpatialSample connection()const{return tracker_.reference().query(tracker_.progress());}
  bool attached()const{return tracker_.attached();}
  uint64_t version()const{return version_;}
  double progress()const{return last_progress_;}
  const RollingPrefix &prefix()const{return prefix_;}
  unsigned last_offer_balls()const{return last_offer_balls_;}
  void invalidate(){tracker_.detach();prefix_={};}
  void reset(){invalidate();last_progress_=0.;offset_=0.;}

 private:
  FixedReferenceTracker tracker_;
  RollingPrefix prefix_;
  double offset_=0.,last_progress_=0.;
  uint64_t version_=0;
  unsigned last_offer_balls_=0;
};

struct RollingTick {
  RollingMode mode=RollingMode::Protection;
  RollingReason reason=RollingReason::Input;
  TrackingCommand command;
  uint64_t reference_version=0;
  double global_s=0.,certified_end=0.;
  bool new_observation=false,planned=false,continued=false,committed=false;
  unsigned cover_balls=0;
  int64_t planning_ns=0,certification_ns=0,control_ns=0;
};

class RollingNavigator {
 public:
  static constexpr int64_t max_age_ns=int64_t(NavigationConfig::max_observation_age_sim_s*1e9);
  static constexpr double intent_change_rad=direction_pi/180.;

  explicit RollingNavigator(NavigationPipeline &pipeline):pipeline_(pipeline){}

  RollingTick tick(NavigationObservation *observation,const NavigationState &state,
                   const DesiredMotion &desired,const RollingActuatorState &actuator) {
    return tick_with_oracle(observation,state,desired,actuator,[&]{
      return RawDepthBallOracle(observation->depth,pipeline_.builder().local_geometry(),observation->observation_pose);
    });
  }

  // A deterministic scene replay can supply its own observed-ball predicate.
  // Production calls tick(), which only uses the current raw-depth group.
  template<class OracleFactory>
  RollingTick tick_with_oracle(NavigationObservation *o,const NavigationState &state,
                              const DesiredMotion &desired,const RollingActuatorState &actuator,
                              OracleFactory make_oracle) {
    RollingTick result;
    const int64_t now=state.current_simulation_ns,stamp=state.actual.t_ns;
    const bool epoch_changed=o&&have_epoch_&&o->receive_epoch!=epoch_;
    if((last_clock_>=0&&now<last_clock_)||
       (state.actual_valid&&last_state_>=0&&stamp<last_state_)||epoch_changed){
      reset_epoch();
      last_clock_=now;
      if(o){epoch_=o->receive_epoch;have_epoch_=true;}
      result.mode=RollingMode::TimeReset;
      result.reason=RollingReason::Timing;
      return result;
    }
    last_clock_=now;
    if(!state.actual_valid||!NavigationPipeline::valid_state(state.actual)||state.configuration_version!=NavigationConfig::version||
       stamp<0||stamp>now||now-stamp>max_age_ns)
      return protect(result,RollingReason::Input);
    if(o&&!have_epoch_){epoch_=o->receive_epoch;have_epoch_=true;}

    const bool new_state=stamp!=last_state_;
    const double dt=last_state_<0||!new_state?BaseConfig::physics_dt_s:(stamp-last_state_)*1e-9;
    if(new_state)last_state_=stamp;
    if(dt<=0.||dt>BaseConfig::physics_dt_s+1e-12)
      return protect(result,RollingReason::Timing);
    const bool fresh=o&&o->identity.simulation_stamp_ns>=0&&o->identity.simulation_stamp_ns<=now&&
      now-o->identity.simulation_stamp_ns<=max_age_ns;
    auto desired_direction=desired.d_desired;
    const bool intent_ok=desired.valid&&desired.has_direction&&std::isfinite(desired.v_desired)&&
      std::abs(desired.v_desired-BaseConfig::max_speed_mps)<=FixedReferenceTracker::attachment_speed_tolerance_mps&&
      unit_direction(desired_direction);
    if(!intent_ok)return protect(result,RollingReason::Input);
    auto world_intent=fresh?nav_rotate(o->observation_pose.R_WB,desired_direction):Direction3{};
    if(fresh&&!unit_direction(world_intent))return protect(result,RollingReason::Input);
    const auto velocity=nav_rotate(state.actual.R_WB,state.actual.v_B);

    if(fresh&&(!have_observation_||!(o->identity==observation_))){
      result.new_observation=true;
      SpatialSample join;
      const auto begin=field_now_ns();
      const auto &plan=pipeline_.process_if(*o,state,desired,false,true,[&]{
        auto oracle=make_oracle();
        if(reference_.attached()){
          const auto cert_begin=field_now_ns();
          result.continued=reference_.renew(oracle);
          result.cover_balls+=reference_.prefix().balls;
          result.certification_ns+=field_now_ns()-cert_begin;
          const bool change=nav_angle(active_intent_,world_intent)>intent_change_rad;
          if(!change&&reference_.can_step(dt))return NavigationPlan{false,false,nullptr};
          join=reference_.connection();
          // New Step3 curves begin at zero curvature. Keep the current ramp
          // until a geometrically compatible replacement can join it.
          if(nav_norm(join.curvature_W)>RollingReference::connection_tolerance)
            return NavigationPlan{false,false,nullptr};
          return NavigationPlan{true,false,&join};
        }
        return NavigationPlan{true,false,nullptr};
      });
      result.planning_ns=field_now_ns()-begin-result.certification_ns;
      result.planned=plan.field_attempted;
      observation_=o->identity;
      have_observation_=true;
      if(!plan.input_ready)return protect(result,RollingReason::Input);
      if(plan.spatial_ready){
        auto oracle=make_oracle();
        const auto cert_begin=field_now_ns();
        result.committed=reference_.install(pipeline_.reference(),0.,state.actual.p_W,velocity,actuator,dt,oracle);
        result.cover_balls+=reference_.last_offer_balls();
        result.certification_ns+=field_now_ns()-cert_begin;
        if(result.committed)active_intent_=world_intent;
      }
      proof_stamp_=o->identity.simulation_stamp_ns;
    }

    if(proof_stamp_<0||now-proof_stamp_>max_age_ns)return protect(result,RollingReason::Input);
    if(!reference_.can_step(dt))return protect(result,RollingReason::Prefix);
    result.reference_version=reference_.version();
    result.global_s=reference_.progress();
    result.certified_end=reference_.prefix().end;
    if(!new_state){
      result.mode=RollingMode::RepeatedState;
      result.reason=RollingReason::None;
      return result;
    }

    const auto begin=field_now_ns();
    result.command=reference_.step(state.actual.p_W,velocity,dt);
    if(result.command.status!=TrackingStatus::Ready){
      result.control_ns=field_now_ns()-begin;
      return protect(result,RollingReason::Controller);
    }
    if(!reference_.swept_step_clear(state.actual.p_W,result.command.next_velocity_W,dt)){
      result.control_ns=field_now_ns()-begin;
      return protect(result,RollingReason::SweptStep);
    }
    result.control_ns=field_now_ns()-begin;
    result.mode=RollingMode::Tracking;
    result.reason=RollingReason::None;
    result.global_s=reference_.progress();
    return result;
  }

  // External collision protection owns the actuator whenever this mode is active.
  void interlock(){reference_.invalidate();proof_stamp_=-1;}
  void reset_epoch(){
    interlock();
    reference_.reset();
    pipeline_.reset_epoch();
    last_clock_=last_state_=-1;
    have_observation_=have_epoch_=false;
  }

 private:
  RollingTick protect(RollingTick result,RollingReason reason){
    result.reference_version=reference_.version();
    result.global_s=reference_.progress();
    interlock();
    const auto status=result.command.status;
    result.command={};
    result.command.status=status;
    result.mode=RollingMode::Protection;
    result.reason=reason;
    return result;
  }

  NavigationPipeline &pipeline_;
  RollingReference reference_;
  ObservationIdentity observation_;
  Direction3 active_intent_{};
  int64_t last_clock_=-1,last_state_=-1,proof_stamp_=-1;
  uint64_t epoch_=0;
  bool have_observation_=false,have_epoch_=false;
};

} // namespace ego4p0
