#pragma once
#include "new_controller/navigation_math.hpp"
#include "new_controller/reference_occupancy.hpp"
#include "new_controller/depth_observations.hpp"
#include "new_controller/harmonic_planner.hpp"
#include <cstring>
namespace ego4p0 {
struct ObservationIdentity {
  uint64_t group=0,calibration_version=0,payload_version=0;
  int64_t simulation_stamp_ns=0;std::array<int64_t,4>camera_stamp_ns{};
  uint8_t camera_mask=0;
  bool operator==(const ObservationIdentity&o)const{return group==o.group&&calibration_version==o.calibration_version&&payload_version==o.payload_version&&simulation_stamp_ns==o.simulation_stamp_ns&&camera_stamp_ns==o.camera_stamp_ns&&camera_mask==o.camera_mask;}
};
struct NavigationObservation {DepthFrame depth;ObservationIdentity identity;BodyState observation_pose;bool observation_pose_valid=false;uint64_t receive_epoch=0;};
struct NavigationState {BodyState actual;int64_t current_simulation_ns=0;uint64_t configuration_version=NavigationConfig::version;bool actual_valid=false;};
// Validation interfaces for the later execution stage. They do not emit or
// clamp commands, and use the caller's actual elapsed time.
struct NavigationLimits {
  static bool compensation_input(const Direction3&u,const Direction3&previous,double dt){
    if(!std::isfinite(dt)||dt<=0)return false;
    for(double x:u)if(!std::isfinite(x))return false;
    for(double x:previous)if(!std::isfinite(x))return false;
    return nav_norm(u)<=NavigationConfig::compensation_input_limit_mps+1e-12&&
      nav_norm(nav_add(u,previous,-1))<=NavigationConfig::compensation_slew_limit_mps2*dt+1e-12;
  }
  static bool heading_input(const Direction3&w,const Direction3&previous,double dt){
    if(!std::isfinite(dt)||dt<=0)return false;
    for(double x:w)if(!std::isfinite(x))return false;
    for(double x:previous)if(!std::isfinite(x))return false;
    return nav_norm(w)<=NavigationConfig::heading_rate_limit_rad_s+1e-12&&
      nav_norm(nav_add(w,previous,-1))<=NavigationConfig::heading_acceleration_limit_rad_s2*dt+1e-12;
  }
};
enum class NavigationInvalid:uint8_t {None,MissingCamera,GroupMismatch,CalibrationMismatch,MissingObservationPose,MissingActualState,InvalidState,FutureObservation,StaleObservation,ConfigurationMismatch,GroupRegression,MissingDesired,NoAngularCurve,InvalidSpatialCurve,AngularInterpolationOutsideOmega};
inline const char*navigation_invalid_name(NavigationInvalid r){switch(r){case NavigationInvalid::MissingCamera:return "missing_camera";case NavigationInvalid::GroupMismatch:return "group_identity_mismatch";case NavigationInvalid::CalibrationMismatch:return "calibration_version_mismatch";case NavigationInvalid::MissingObservationPose:return "missing_observation_pose";case NavigationInvalid::MissingActualState:return "missing_actual_state";case NavigationInvalid::InvalidState:return "invalid_state";case NavigationInvalid::FutureObservation:return "future_observation";case NavigationInvalid::StaleObservation:return "stale_observation";case NavigationInvalid::ConfigurationMismatch:return "configuration_version_mismatch";case NavigationInvalid::GroupRegression:return "group_regression";case NavigationInvalid::MissingDesired:return "missing_desired_direction";case NavigationInvalid::NoAngularCurve:return "no_accepted_angular_curve";case NavigationInvalid::InvalidSpatialCurve:return "invalid_spatial_curve";case NavigationInvalid::AngularInterpolationOutsideOmega:return "angular_interpolation_outside_omega";default:return "none";}}
struct NavigationOutput {
  ObservationIdentity observation;BodyState observation_pose,actual;int64_t current_simulation_ns=0;
  uint64_t configuration_version=NavigationConfig::version,reference_version=0,field_version=0,curve_version=0,geometry_version=0;
  NavigationInvalid invalid=NavigationInvalid::MissingCamera;
  bool input_ready=false,angular_ready=false,motion_output_enabled=false,field_attempted=false;
  const char*q0_source="none",*cache_event="none";
  bool spatial_enabled=false,spatial_ready=false,spatial_reused=false,occupancy_reused=false;
  double turn_length_m=0,checked_horizon_m=0;
  uint32_t observed_sections=0,unknown_sections=0,obstacle_sections=0,outside_sections=0;
  int64_t coordinator_cpu_ns=0,preprocessing_ns=0,capture_ns=0,field_ns=0,spatial_ns=0,occupancy_ns=0,total_ns=0;
};
struct NavigationPlan {
  bool requested=true,full_tube=true;
  const SpatialSample *connection=nullptr;
};
class NavigationPipeline {
 public:
  void configure(const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h,uint64_t calibration_version=1,bool angles_only=false){
    if(!calibration_version)throw std::invalid_argument("calibration version must be explicit");
    for(const auto&camera:cal){BodyState pose;for(int i=0;i<3;++i)for(int j=0;j<3;++j)pose.R_WB[3*i+j]=camera.T_BC[4*i+j];
      for(double x:camera.T_BC)if(!std::isfinite(x))throw std::invalid_argument("nonfinite extrinsics");
      if(!valid_state(pose)||camera.T_BC[12]!=0||camera.T_BC[13]!=0||camera.T_BC[14]!=0||camera.T_BC[15]!=1)throw std::invalid_argument("invalid calibration rotation/homogeneous transform");}
    width_=w;height_=h;calibration_=calibration_version;rays_.configure(cal,w,h);
    builder_.configure(cal,w,h,1,1,16,4,0,true,true,true,true,false,true,true);
    planner_.configure(cal,w,h,HarmonicConfig{},4,angles_only);planner_.set_trial_solvers(SparseTrialMode::EigenLDLT,true);planner_.allocate(input_,w,h);planner_.allocate(field_);
    had_group_=false;have_velocity_=false;capture_valid_=false;output_={};
    invalidate_spatial();
    Q_.reserve(8192);cached_Q_.reserve(8192);
    checked_domain_.reserve(input_.fine_state.size());
  }
  void allocate(NavigationObservation&o)const{o.depth.allocate(size_t(width_)*height_);builder_.allocate(o.depth);}
  void enable_fifo5(){builder_.enable_fifo5_workers();sched_param p{};p.sched_priority=5;if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&coordinator_mask()))throw std::runtime_error("coordinator CPU0 affinity failed");if(pthread_setschedparam(pthread_self(),SCHED_FIFO,&p))throw std::runtime_error("coordinator FIFO5 requires verified launcher");builder_.refresh_worker_schedulers();}
  const NavigationOutput&process(NavigationObservation&o,const NavigationState&s,const DesiredMotion&desired,bool cold=false,bool spatial=false){
    return process_if(o,s,desired,cold,spatial,[]{return NavigationPlan{};});
  }
  template<class Decide>
  const NavigationOutput&process_if(NavigationObservation&o,const NavigationState&s,const DesiredMotion&desired,
                                    bool cold,bool spatial,Decide decide,std::optional<AngularIntent> bearings=std::nullopt,
                                    bool defer_panorama=false){
    const auto cpu_begin=field_thread_cpu_ns();
    const auto begin=field_now_ns();output_={};output_.observation=o.identity;output_.observation_pose=o.observation_pose;output_.actual=s.actual;output_.current_simulation_ns=s.current_simulation_ns;output_.reference_version=++reference_version_;output_.spatial_enabled=spatial;
    auto fail=[&](NavigationInvalid why)->const NavigationOutput&{capture_valid_=false;output_.invalid=why;output_.coordinator_cpu_ns=field_thread_cpu_ns()-cpu_begin;output_.total_ns=field_now_ns()-begin;planner_.invalidate_cache();invalidate_spatial();if(!output_.input_ready){field_.status=FieldStatus::NoInput;field_.termination=CurveEnd::NoField;field_.curve_size=0;}return output_;};
    if(o.identity.camera_mask!=15)return fail(NavigationInvalid::MissingCamera);
    if(!o.identity.group||!o.identity.payload_version||o.identity.group!=o.depth.sequence||o.identity.simulation_stamp_ns!=o.depth.t_ns||std::any_of(o.identity.camera_stamp_ns.begin(),o.identity.camera_stamp_ns.end(),[&](int64_t t){return t!=o.depth.t_ns;}))return fail(NavigationInvalid::GroupMismatch);
    if(o.identity.calibration_version!=calibration_)return fail(NavigationInvalid::CalibrationMismatch);
    if(!o.observation_pose_valid||o.observation_pose.t_ns!=o.depth.t_ns)return fail(NavigationInvalid::MissingObservationPose);
    if(!s.actual_valid)return fail(NavigationInvalid::MissingActualState);
    if(!valid_state(s.actual)||!valid_state(o.observation_pose)||s.actual.t_ns>s.current_simulation_ns)return fail(NavigationInvalid::InvalidState);
    if(s.configuration_version!=NavigationConfig::version)return fail(NavigationInvalid::ConfigurationMismatch);
    if(s.current_simulation_ns<o.depth.t_ns)return fail(NavigationInvalid::FutureObservation);
    if(s.current_simulation_ns-o.depth.t_ns>int64_t(NavigationConfig::max_observation_age_sim_s*1e9)||s.current_simulation_ns-s.actual.t_ns>int64_t(NavigationConfig::max_observation_age_sim_s*1e9))return fail(NavigationInvalid::StaleObservation);
    if(had_group_&&(o.depth.sequence<last_group_.group||o.depth.t_ns<last_group_.simulation_stamp_ns))return fail(NavigationInvalid::GroupRegression);
    auto goal=desired.d_desired;if(!desired.valid||!desired.has_direction||!std::isfinite(desired.v_desired)||desired.v_desired<0||desired.v_desired>BaseConfig::max_speed_mps||(!bearings&&!unit_direction(goal)))return fail(NavigationInvalid::MissingDesired);
    if(bearings&&(!builder_.layout().cell(bearings->goal.az,bearings->goal.el)||
                 !builder_.layout().cell(bearings->start.az,bearings->start.el)))return fail(NavigationInvalid::MissingDesired);
    for(const auto&raw:o.depth.D)if(raw.size()!=size_t(width_)*height_)return fail(NavigationInvalid::MissingCamera);
    const bool same=had_group_&&o.identity==last_group_;
    if(!same){o.depth.observations.ready=false;o.depth.local.ready=false;output_.cache_event=had_group_?"new_depth_group_strict_matrix_checks":"initial_group";}else output_.cache_event="same_group";
    const auto pre=field_now_ns();builder_.process(o.depth,rays_,defer_panorama);output_.preprocessing_ns=field_now_ns()-pre;
    const LocalFieldView evidence(o.depth,builder_.local_geometry());
    if(!evidence.image_evidence_ready())return fail(NavigationInvalid::GroupMismatch);
    const auto plan=decide();
    if(!plan.requested){
      last_group_=o.identity;had_group_=true;output_.input_ready=true;
      output_.invalid=NavigationInvalid::None;
      output_.coordinator_cpu_ns=field_thread_cpu_ns()-cpu_begin;output_.total_ns=field_now_ns()-begin;
      return output_;
    }
    if(!evidence.ready())return fail(NavigationInvalid::GroupMismatch);
    // The protected payload is immutable for its complete identity. A changed
    // payload revision, calibration, group, or a failed input requires capture.
    // Skipped rolling plans must not mark an uncaptured new group as reusable.
    if(cold||!capture_valid_||!(capture_identity_==o.identity)){
      input_.capture(o.depth);output_.capture_ns=input_.capture_ns;
      if(!input_.ready)return fail(NavigationInvalid::GroupMismatch);
      capture_identity_=o.identity;capture_valid_=true;
    }else input_.capture_ns=0;
    last_group_=o.identity;had_group_=true;output_.input_ready=true;
    intent_={};intent_.desired=desired;intent_.desired.d_desired=goal;intent_.R_WB=o.observation_pose.R_WB;intent_.p_W=s.actual.p_W;intent_.state_stamp_ns=s.actual.t_ns;intent_.coordinate_revision=calibration_;
    if(bearings){
      intent_.bearings=bearings;output_.q0_source="spherical_observation_bearing";
    }else{
      auto velocity_W=nav_rotate(s.actual.R_WB,s.actual.v_B);intent_.q0=nav_unrotate(o.observation_pose.R_WB,velocity_W);
      if(unit_direction(intent_.q0)){have_velocity_=true;last_velocity_W_=velocity_W;output_.q0_source="actual_velocity_in_observation_frame";}
      else if(have_velocity_){intent_.q0=nav_unrotate(o.observation_pose.R_WB,last_velocity_W_);unit_direction(intent_.q0);output_.q0_source="last_actual_velocity_in_world";}
      else{intent_.q0=goal;output_.q0_source="stationary_initial_desired";}
    }
    if(plan.connection){
      intent_.p_W=plan.connection->position_W;
      intent_.q0=nav_unrotate(o.observation_pose.R_WB,plan.connection->tangent_W);
      if(!unit_direction(intent_.q0))return fail(NavigationInvalid::InvalidState);
      output_.q0_source="reference_connection";
    }
    output_.field_attempted=true;
    if(cold){planner_.invalidate_cache();invalidate_spatial();}
    const auto f=field_now_ns();planner_.build(input_,intent_,field_);output_.field_ns=field_now_ns()-f;output_.field_version=field_.field_version;output_.curve_version=field_.curve_version;
    const bool coarse_ready=field_.coarse_topology_only?
      field_.fine_neumann_roi&&field_.coarse.status==FieldStatus::TopologyReady:
      field_.coarse.status==FieldStatus::Valid&&std::isfinite(field_.coarse.residual)&&field_.coarse.residual<=1e-7;
    if(field_.status!=FieldStatus::Valid||field_.termination!=CurveEnd::Exit||!field_.curve_size||!coarse_ready||
      field_.fine.status!=FieldStatus::Valid||!std::isfinite(field_.fine.residual)||
      field_.fine.residual>1e-7)return fail(NavigationInvalid::NoAngularCurve);
    output_.angular_ready=true;output_.invalid=NavigationInvalid::None;
    if(spatial)update_spatial(o,intent_.p_W,plan.full_tube);
    else invalidate_spatial();
    output_.coordinator_cpu_ns=field_thread_cpu_ns()-cpu_begin;output_.total_ns=field_now_ns()-begin;
    return output_;
  }
  void reset_epoch(){
    had_group_=false;have_velocity_=false;capture_valid_=false;
    planner_.invalidate_cache();invalidate_spatial();
    output_={};field_.status=FieldStatus::NoInput;field_.termination=CurveEnd::NoField;field_.curve_size=0;
  }
  const NavigationOutput&output()const{return output_;}const HarmonicResult&field()const{return field_;}const HarmonicIntent&intent()const{return intent_;}
  const SpatialReference &reference()const{return reference_;}
  const ContinuousTubeChecker &tube()const{return tube_;}
  FrameObservationBuilder&builder(){return builder_;}const PixelRayTables&rays()const{return rays_;}HarmonicPlanner&planner(){return planner_;}
  const HarmonicPlanner&planner()const{return planner_;}
  static bool valid_state(const BodyState&s){
    for(double x:s.p_W)if(!std::isfinite(x))return false;
    for(double x:s.v_B)if(!std::isfinite(x))return false;
    for(double x:s.R_WB)if(!std::isfinite(x))return false;
    for(int a=0;a<3;++a)for(int b=0;b<3;++b){double q=0;for(int j=0;j<3;++j)q+=s.R_WB[3*j+a]*s.R_WB[3*j+b];if(std::abs(q-(a==b?1.:0.))>1e-6)return false;}
    const auto&R=s.R_WB;return std::abs(R[0]*(R[4]*R[8]-R[5]*R[7])-R[1]*(R[3]*R[8]-R[5]*R[6])+R[2]*(R[3]*R[7]-R[4]*R[6])-1)<1e-6;
  }
 private:
  void invalidate_spatial(){
    reference_.invalidate();
    tube_.invalidate();
    domain_valid_=false;
    occupancy_valid_=false;
  }

  void update_spatial(const NavigationObservation &o,const Direction3 &anchor,bool full_tube){
    const auto start=field_now_ns();
    Q_.clear();
    for(uint32_t i=0;i<field_.curve_size;++i)Q_.push_back(field_.curve[i].direction);
    const auto &rotation=o.observation_pose.R_WB;
    const bool same_shape=reference_.valid()&&Q_==cached_Q_;
    const bool same_pose=same_shape&&reference_.anchor()==anchor&&reference_.rotation()==rotation;
    if(!same_shape){
      domain_valid_=false;
      if(!reference_.build(Q_,anchor,rotation)){
        invalidate_spatial();
        output_.invalid=NavigationInvalid::InvalidSpatialCurve;
        output_.spatial_ns=field_now_ns()-start;
        return;
      }
      cached_Q_=Q_;
    }else if(!same_pose&&!reference_.place(anchor,rotation)){
      invalidate_spatial();
      output_.invalid=NavigationInvalid::InvalidSpatialCurve;
      output_.spatial_ns=field_now_ns()-start;
      return;
    }
    if(!domain_valid_||checked_domain_!=input_.fine_state){
      if(!domain_.check(reference_.angular_path(),builder_.layout(),input_.fine_state)){
        invalidate_spatial();
        output_.invalid=NavigationInvalid::AngularInterpolationOutsideOmega;
        output_.spatial_ns=field_now_ns()-start;
        return;
      }
      checked_domain_=input_.fine_state;
      domain_valid_=true;
    }
    if(!same_pose)++geometry_version_;
    output_.geometry_version=geometry_version_;
    output_.spatial_ready=true;
    output_.spatial_reused=same_shape;
    output_.turn_length_m=reference_.profile().length;
    output_.spatial_ns=field_now_ns()-start;
    if(!full_tube){
      tube_.invalidate();
      occupancy_valid_=false;
      return;
    }
    const auto check_start=field_now_ns();
    // process() has already validated observation identity, pose, calibration
    // and freshness. A new depth group always gets a new occupancy proof.
    output_.occupancy_reused=occupancy_valid_&&same_pose&&occupancy_identity_==o.identity&&
      occupancy_observer_==o.observation_pose.p_W;
    if(!output_.occupancy_reused){
      checked_horizon_=observation_check_horizon(reference_,o.observation_pose);
      tube_.check(reference_,checked_horizon_,RawDepthBallOracle(o.depth,builder_.local_geometry(),o.observation_pose));
      occupancy_identity_=o.identity;
      occupancy_observer_=o.observation_pose.p_W;
      occupancy_valid_=true;
    }
    output_.checked_horizon_m=checked_horizon_;
    for(const auto &section:tube_.sections()){
      switch(section.status){
        case TubeStatus::ObservedClear:++output_.observed_sections;break;
        case TubeStatus::Unknown:++output_.unknown_sections;break;
        case TubeStatus::PotentialObstacle:++output_.obstacle_sections;break;
        case TubeStatus::OutsideObservationRange:++output_.outside_sections;break;
      }
    }
    output_.occupancy_ns=field_now_ns()-check_start;
  }

  static const cpu_set_t&coordinator_mask(){static const cpu_set_t mask=[](){cpu_set_t s;CPU_ZERO(&s);CPU_SET(0,&s);return s;}();return mask;}
  uint32_t width_=0,height_=0;uint64_t calibration_=0,reference_version_=0;bool had_group_=false,have_velocity_=false;
  PixelRayTables rays_;FrameObservationBuilder builder_;HarmonicPlanner planner_;HarmonicDepthInput input_;HarmonicResult field_;HarmonicIntent intent_;
  ObservationIdentity last_group_;NavigationOutput output_;Direction3 last_velocity_W_{};
  ObservationIdentity capture_identity_;bool capture_valid_=false;
  SpatialReference reference_;
  AngularDomainCertificate domain_;
  ContinuousTubeChecker tube_;
  std::vector<Direction3> Q_,cached_Q_;
  std::vector<uint8_t> checked_domain_;
  ObservationIdentity occupancy_identity_;
  Direction3 occupancy_observer_{};
  uint64_t geometry_version_=0;
  double checked_horizon_=0;
  bool domain_valid_=false,occupancy_valid_=false;
};
} // namespace ego4p0
