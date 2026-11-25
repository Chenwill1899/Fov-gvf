#pragma once
#include "new_controller/fixed_reference_tracker.hpp"
#include "new_controller/navigation_io.hpp"
#include <deque>
#include <memory>
#include <string>

// Explicit simulation application. No production flags or core algorithms are
// changed. Scene geometry is consumed once, to certify the initial free ball.
namespace ego4p3_sim {
using namespace ego4p0;
using Vec=Direction3;
inline bool finite(const Vec&v){return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}
inline Vec limit(Vec v,double maximum){const double n=nav_norm(v);return n>maximum?nav_scale(v,maximum/n):v;}
inline Vec plant_response(const Vec&velocity,const Vec&input,double dt){
  const Vec error=nav_add(input,velocity,-1);const double length=nav_norm(error);
  double scale=nav_norm(input)<1e-8?1.:std::min(1.,dt/std::max(BaseConfig::plant_tau_s,1e-6));
  if(length>1e-12)scale=std::min(scale,BaseConfig::plant_acceleration_mps2*dt/length);
  return nav_add(velocity,error,scale);
}
inline Vec read_vec(const YAML::Node&n){if(!n.IsSequence()||n.size()!=3)throw std::invalid_argument("three finite coordinates required");Vec v{n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};if(!finite(v))throw std::invalid_argument("nonfinite scenario vector");return v;}
inline bool simulation_calibration_matches(const std::array<CameraCalibration,4>&cal){
  // USD stores clipping planes as float. Validate the model within its storage
  // precision; retain the ACTUAL near value in all raw-depth oracle queries.
  for(const auto&c:cal)if(!std::isfinite(c.near_clip_z_m)||std::abs(c.near_clip_z_m-.1)>1e-7||
      std::abs(c.T_BC[3])>1e-9||std::abs(c.T_BC[7])>1e-9||std::abs(c.T_BC[11])>1e-9)return false;
  return true;
}
struct GoalIntent {
  DesiredMotion preprocessing_input;
  bool reference_allowed=false;
};
inline GoalIntent goal_intent(const Vec&goal,const BodyState&state,double cruise){
  DesiredInput input;input.mode=DesiredInputMode::WorldGoal;input.vector=goal;input.goal_speed_mps=cruise;
  GoalIntent result;result.preprocessing_input=desired_motion(input,state);
  result.reference_allowed=result.preprocessing_input.valid&&result.preprocessing_input.has_direction;
  if(!result.reference_allowed){
    // Input validation still runs on every frame. This placeholder is NEVER a
    // planning or command direction: both actions require reference_allowed.
    result.preprocessing_input.has_direction=true;result.preprocessing_input.d_desired={1,0,0};result.preprocessing_input.v_desired=0;
  }
  return result;
}
struct Scenario {
  Vec start{},goal{};double seed_radius=0,cruise=12,arrival_radius=.25,arrival_speed=.1;
  uint32_t width=0,height=0;std::string identity;bool seed_certified=false;
  static Scenario parse(const YAML::Node&root){
    Scenario s;
    if(root["schema"].as<int>()!=1||!root["static_scene"].as<bool>())throw std::invalid_argument("only fixed static scene schema 1 supported");
    for(const auto&p:std::vector<std::pair<const char*,double>>{{"physics_dt_s",.005},{"body_radius_m",.1},{"camera_offset_m",0},{"camera_near_m",.1},{"camera_far_m",50}})
      if(root[p.first].as<double>()!=p.second)throw std::invalid_argument(std::string("incompatible scenario ")+p.first);
    s.start=read_vec(root["start"]);s.goal=read_vec(root["goal"]);s.seed_radius=root["seed_radius_m"].as<double>();
    s.cruise=root["cruise_speed_mps"].as<double>();s.arrival_radius=root["arrival_radius_m"].as<double>();s.arrival_speed=root["arrival_speed_mps"].as<double>();
    s.width=root["image_width"].as<uint32_t>();s.height=root["image_height"].as<uint32_t>();
    s.identity=root["name"].as<std::string>();
    if(!std::isfinite(s.seed_radius)||s.seed_radius<=NavigationConfig::occupancy_tube_radius_m||s.seed_radius>4||
       !std::isfinite(s.cruise)||s.cruise<=0||s.cruise>12||!std::isfinite(s.arrival_radius)||s.arrival_radius<=0||s.arrival_radius>.25||
       !std::isfinite(s.arrival_speed)||s.arrival_speed<=0||s.arrival_speed>.1||s.width<16||s.height<16||s.width>4096||s.height>4096)
      throw std::invalid_argument("invalid simulation limits");
    const Vec low=read_vec(root["bounds_min"]),high=read_vec(root["bounds_max"]);
    for(int k=0;k<3;++k)if(!(low[k]<s.start[k]-s.seed_radius&&high[k]>s.start[k]+s.seed_radius))throw std::invalid_argument("initial seed intersects room boundary");
    if(!root["boxes"].IsSequence())throw std::invalid_argument("box list required for independent spawn check");
    for(const auto&box:root["boxes"]){const auto a=read_vec(box["min"]),b=read_vec(box["max"]);Vec d{};
      for(int k=0;k<3;++k){if(!(a[k]<b[k]))throw std::invalid_argument("invalid scene box");d[k]=std::max({a[k]-s.start[k],0.,s.start[k]-b[k]});}
      if(!(nav_norm(d)>s.seed_radius))throw std::invalid_argument("initial seed intersects obstacle");
    }
    // Do not retain bounds, boxes or any waypoint/route in the controller.
    s.seed_certified=true;return s;
  }
  bool contains(const Vec&center,double radius)const{return seed_certified&&finite(center)&&std::isfinite(radius)&&radius>0&&nav_norm(nav_add(center,start,-1))+radius<=seed_radius;}
};

struct WireRequest {uint64_t magic,version,sequence,simulation_ns,width,height;double values[15];};
struct WireReply {uint64_t magic=0x45474F3450330001ULL,version=1,sequence=0,simulation_ns=0,decision=6,flags=0;double values[12]{};};
static_assert(sizeof(WireRequest)==168&&sizeof(WireReply)==144,"IPC layout mismatch");
inline BodyState validate_request(const WireRequest&r,uint64_t previous,uint32_t w,uint32_t h){
  if(r.magic!=0x45474F3450330001ULL||r.version!=1||previous==UINT64_MAX||r.sequence==0||r.sequence>uint64_t(INT64_MAX/BaseConfig::period_ns)||r.sequence!=previous+1||r.width!=w||r.height!=h||
     r.simulation_ns!=(r.sequence-1)*uint64_t(BaseConfig::period_ns)||r.simulation_ns>uint64_t(INT64_MAX))throw std::runtime_error("IPC identity, size, sequence or time mismatch");
  BodyState s;s.t_ns=int64_t(r.simulation_ns);Vec velocity{};
  for(int k=0;k<3;++k){s.p_W[k]=r.values[k];velocity[k]=r.values[3+k];}for(int k=0;k<9;++k)s.R_WB[k]=r.values[6+k];
  s.v_B=nav_unrotate(s.R_WB,velocity);
  if(!finite(velocity)||!NavigationPipeline::valid_state(s)||nav_norm(velocity)>12+1e-6)throw std::runtime_error("IPC invalid actual state");
  return s;
}

// Same implicit projection and intrinsic normal-plane feedback as the fixed
// 12 m/s tracker. This app adds a scalar speed state and its derivative. The
// compensation and heading limits remain explicit rejection conditions.
class VariableReferenceTracker {
 public:
  bool attach(const SpatialReference&r,const Vec&p,const Vec&v){
    if(!r.valid()||nav_norm(nav_add(p,r.query(0).position_W,-1))>.1)return false;
    if(nav_norm(v)>1e-8&&!r.normal_attachment(p,v,0))return false;
    reference_=r;s_=0;active_=true;return true;
  }
  bool project(const Vec&p){
    if(!active_)return false;
    const double lo=std::max(0.,s_-.05),hi=s_+.25;
    double x=reference_.project_chords(p,lo,hi);auto a=reference_.query(x);auto error=nav_add(p,a.position_W,-1);
    double along=direction_dot(error,a.tangent_W),den=1-direction_dot(error,a.curvature_W);
    if(!(den>.5))return false;
    const double refined=std::clamp(x+along/den,lo,hi);
    if(std::abs(refined-x)>1e-5){x=refined;a=reference_.query(x);error=nav_add(p,a.position_W,-1);along=direction_dot(error,a.tangent_W);den=1-direction_dot(error,a.curvature_W);}
    if(!(den>.5)||(std::abs(along)>.001&&!(x==0&&along<0)))return false;
    const auto local=TrackingNormalPlane2D(a.tangent_W).coordinates(error);
    cross_track_=std::hypot(local[0],local[1])+a.position_error_bound_m;
    if(cross_track_>NavigationConfig::tracking_error_target_m)return false;
    s_=x;return true;
  }
  TrackingCommand step(const Vec&p,const Vec&v,double target,double dt){
    TrackingCommand result;result.s=s_;result.cross_track_m=cross_track_;
    auto fail=[&](TrackingStatus why){result.status=why;return result;};
    if(!active_)return fail(TrackingStatus::NoReference);
    if(!finite(p)||!finite(v)||!std::isfinite(target)||target<0||target>12||dt!=BaseConfig::physics_dt_s)return fail(TrackingStatus::InvalidStep);
    if(!project(p))return fail(TrackingStatus::ProjectionLost);
    const auto a=reference_.query(s_);auto error=nav_add(p,a.position_W,-1);const auto T=a.tangent_W;
    const double speed=nav_norm(v),next_speed=std::clamp(target,std::max(0.,speed-20*dt),std::min(12.,speed+20*dt));
    if(next_speed<1e-8){result.status=TrackingStatus::Ready;result.plant_input_W={};result.next_velocity_W=plant_response(v,{},dt);result.s=s_;result.cross_track_m=cross_track_;return result;}
    Vec heading=speed>1e-8?nav_scale(v,1/speed):T;
    const TrackingNormalPlane2D normal(T);const auto local=normal.coordinates(error);const double transverse=std::hypot(local[0],local[1]);
    const double gain=transverse>0?std::min(NavigationConfig::lateral_gain_per_s,.5*next_speed/transverse):NavigationConfig::lateral_gain_per_s;
    const std::array<double,2> correction2{-gain*local[0],-gain*local[1]};const auto correction=normal.lift(correction2);
    const double tangential=std::sqrt(next_speed*next_speed-TrackingNormalPlane2D::squared_norm(correction2));
    const auto desired=nav_add(nav_scale(T,tangential),correction);const auto direction=nav_scale(desired,1/next_speed);
    const double along=direction_dot(error,T),den=1-direction_dot(error,a.curvature_W);
    const double s_rate=s_==0&&along<0?0:direction_dot(v,T)/den;
    const auto T_rate=nav_scale(a.curvature_W,s_rate),error_rate=nav_add(v,T,-s_rate);
    const double along_rate=direction_dot(error_rate,T)+direction_dot(error,T_rate);
    const auto normal_rate=nav_add(nav_add(error_rate,T,-along_rate),T_rate,-along);
    const auto correction_rate=nav_scale(normal_rate,-gain);const double speed_rate=(next_speed-speed)/dt;
    const double tangential_rate=(next_speed*speed_rate-direction_dot(correction,correction_rate))/tangential;
    const auto direction_rate=nav_scale(nav_add(nav_add(nav_add(correction_rate,T,tangential_rate),T_rate,tangential),direction,-speed_rate),1/next_speed);
    const auto omega_target=limit(nav_add(nav_cross(direction,direction_rate),nav_cross(heading,direction),NavigationConfig::heading_gain_per_s),NavigationConfig::heading_rate_limit_rad_s);
    const auto omega=nav_add(last_omega_,limit(nav_add(omega_target,last_omega_,-1),NavigationConfig::heading_acceleration_limit_rad_s2*dt));
    const double rate=nav_norm(omega);if(rate>0){const auto axis=nav_scale(omega,1/rate),first=nav_cross(axis,heading);heading=nav_add(nav_add(heading,first,std::sin(rate*dt)),nav_cross(axis,first),1-std::cos(rate*dt));unit_direction(heading);}
    const auto next=nav_scale(heading,next_speed),delta=nav_add(next,v,-1);
    if(nav_norm(delta)>BaseConfig::plant_acceleration_mps2*dt+1e-12)return fail(TrackingStatus::PlantAcceleration);
    const auto input=nav_add(v,delta,1/std::min(1.,dt/BaseConfig::plant_tau_s));
    if(!NavigationLimits::compensation_input(input,last_input_,dt))return fail(TrackingStatus::CompensationLimit);
    result={TrackingStatus::Ready,desired,omega,plant_response(v,input,dt),input,s_,cross_track_,true};return result;
  }
  void commit(const TrackingCommand&c){last_input_=c.plant_input_W;last_omega_=c.omega_W;}
  void emergency(){last_input_={};last_omega_={};active_=false;}
  bool active()const{return active_;}double progress()const{return s_;}double cross_track()const{return cross_track_;}
  const SpatialReference&reference()const{return reference_;}
 private:SpatialReference reference_;Vec last_input_{},last_omega_{};double s_=0,cross_track_=0;bool active_=false;
};

struct EvidenceCounts {uint32_t current=0,history=0,seed=0,unknown=0,obstacle=0,outside=0,current_obstacle=0,history_obstacle=0;};
enum class EvidenceSource {None,Current,Seed,History};
struct EvidenceDecision {TubeStatus status;EvidenceSource source;};
template<class HistoryQuery>
EvidenceDecision static_evidence_union(TubeStatus current,bool seed,size_t count,HistoryQuery history){
  // A CURRENT raw veto is never reversed. Historical PotentialObstacle is a
  // conservative ROI test, not an occupied point: in one immutable static
  // scene, ANY complete raw ObservedClear ball is a valid geometric proof.
  if(current!=TubeStatus::Unknown)return {current,EvidenceSource::Current};
  if(seed)return {TubeStatus::ObservedClear,EvidenceSource::Seed};
  bool potential=false;
  for(size_t i=0;i<count;++i){const auto status=history(i);
    if(status==TubeStatus::ObservedClear)return {status,EvidenceSource::History};
    potential|=status==TubeStatus::PotentialObstacle;
  }
  return {potential?TubeStatus::PotentialObstacle:TubeStatus::Unknown,EvidenceSource::None};
}
struct MotionProof {bool clear=true,obstacle=false,outside=false,current_obstacle=false;double distance=0;};
struct FlightResult {
  uint64_t decision=6,flags=0;Vec input{},next_velocity{};double requested_speed=0,goal_distance=0,cross_track=0,progress=0,certified_ahead=0;
  int64_t control_ns=0,proof_ns=0,history_ns=0,total_ns=0;EvidenceCounts evidence;bool inherited=false,replanned=false,bootstrap=false;
  const char*reason="uncertified";TrackingStatus tracking_status=TrackingStatus::NoReference;
  uint64_t active_field_version=0,active_curve_version=0,active_geometry_version=0;
};
class FlightSupervisor {
 public:
  FlightSupervisor(const Scenario&scenario,const std::array<CameraCalibration,4>&cal,uint64_t version):scenario_(scenario),calibration_(version){
    if(!scenario.seed_certified)throw std::invalid_argument("certified spawn required");pipeline_.configure(cal,scenario.width,scenario.height,version);
    current_=std::make_unique<NavigationObservation>();pipeline_.allocate(*current_);
    // Allocate retained-frame buffers before reporting ready, never on a flight
    // tick. All original depth/ROI metadata remain owned with their raw image.
    for(unsigned i=0;i<16;++i){auto frame=std::make_unique<NavigationObservation>();pipeline_.allocate(*frame);spares_.push_back(std::move(frame));}
  }
  NavigationObservation&observation(){return *current_;}NavigationPipeline&pipeline(){return pipeline_;}
  const Scenario&scenario()const{return scenario_;}
  void identify(uint64_t sequence,const BodyState&s){auto&o=*current_;navigation_identity(o,sequence,calibration_);o.depth.t_ns=s.t_ns;o.identity.simulation_stamp_ns=s.t_ns;o.identity.camera_stamp_ns.fill(s.t_ns);o.observation_pose=s;o.observation_pose_valid=true;}
  FlightResult step(const BodyState&s){
    const int64_t start=field_now_ns();FlightResult result;counts_={};const Vec v=nav_rotate(s.R_WB,s.v_B);const double speed=nav_norm(v);
    result.goal_distance=nav_norm(nav_add(scenario_.goal,s.p_W,-1));
    const bool state_match=!expected_?nav_norm(nav_add(s.p_W,scenario_.start,-1))<1e-7&&speed<1e-8:
      nav_norm(nav_add(s.p_W,expected_p_,-1))<1e-6&&nav_norm(nav_add(v,expected_v_,-1))<1e-6;
    if(!state_match){result.reason="plant_state_mismatch";result.total_ns=field_now_ns()-start;return result;}
    if(tracker_.active()&&!tracker_.project(s.p_W))tracker_.emergency();
    const auto goal=goal_intent(scenario_.goal,s,scenario_.cruise);const auto&desired=goal.preprocessing_input;
    NavigationState state;state.actual=s;state.actual_valid=true;state.current_simulation_ns=s.t_ns;
    SpatialSample connection;bool use_connection=false;
    bool request=!tracker_.active();
    if(tracker_.active()&&s.t_ns-last_plan_ns_>=50000000){connection=tracker_.reference().query(tracker_.progress());if(nav_norm(connection.curvature_W)<1e-10){request=true;use_connection=true;}}
    const bool hold=result.goal_distance<=scenario_.arrival_radius&&speed<=scenario_.arrival_speed;
    if(hold||!goal.reference_allowed)request=false;
    const auto&o=pipeline_.process_if(*current_,state,desired,false,true,[&]{
      if(request&&speed<1e-8&&!use_connection){
        const auto&stencil=pipeline_.planner().fine_stencil();const auto&mask=current_->depth.local.node_state;
        Vec start_direction=desired.d_desired;bool available=mask[stencil.cell(start_direction)]==uint8_t(CandidateStatus::Available);
        if(!available){
          double score=-2;uint32_t best=UINT32_MAX;
          for(uint32_t j=0;j<mask.size();++j)if(mask[j]==uint8_t(CandidateStatus::Available)){const double q=direction_dot(stencil.nodes[j].direction,desired.d_desired);if(q>score){score=q;best=j;}}
          if(best!=UINT32_MAX){start_direction=stencil.nodes[best].direction;available=true;}
        }
        // An explicit stationary connection also prevents the core's retained
        // last-velocity q0 from silently steering a restart after braking.
        if(available){connection={};connection.position_W=s.p_W;connection.tangent_W=nav_rotate(s.R_WB,start_direction);use_connection=true;result.bootstrap=true;}
      }
      return NavigationPlan{request,false,use_connection?&connection:nullptr};
    });
    if(!o.input_ready){result.reason="invalid_observation";result.total_ns=field_now_ns()-start;return result;}
    if(request){last_plan_ns_=s.t_ns;result.replanned=true;
      if(o.spatial_ready&&(!tracker_.active()||pipeline_.reference().continuous_connection(connection))&&tracker_.attach(pipeline_.reference(),s.p_W,v)){
        active_field_version_=o.field_version;active_curve_version_=o.curve_version;active_geometry_version_=o.geometry_version;
      }
    }
    if(o.spatial_ready)result.flags|=1;
    const int64_t control_start=field_now_ns();
    const double goal_cap=std::sqrt(2*20*std::max(0.,result.goal_distance-.12));
    result.requested_speed=hold||!goal.reference_allowed?0:std::min(scenario_.cruise,goal_cap);
    TrackingCommand command;
    if(!hold&&goal.reference_allowed&&tracker_.active())command=tracker_.step(s.p_W,v,result.requested_speed,BaseConfig::physics_dt_s);
    result.tracking_status=command.status;result.cross_track=tracker_.cross_track();result.progress=tracker_.progress();
    if(tracker_.active()){result.active_field_version=active_field_version_;result.active_curve_version=active_curve_version_;result.active_geometry_version=active_geometry_version_;}
    result.control_ns=field_now_ns()-control_start;
    const int64_t proof_start=field_now_ns();
    bool accepted=false;
    if(!hold&&command.status==TrackingStatus::Ready){
      const auto proof=certify(s.p_W,command.next_velocity_W);
      if(proof.clear){result.input=command.plant_input_W;result.next_velocity=command.next_velocity_W;result.certified_ahead=proof.distance;
        result.decision=result.requested_speed<scenario_.cruise-1e-8?3:speed<scenario_.cruise-1e-6?1:2;result.flags|=2;result.reason="reference_command_certified";tracker_.commit(command);accepted=true;}
    }
    if(!accepted){
      const Vec brake=plant_response(v,{},BaseConfig::physics_dt_s);const auto proof=certify(s.p_W,brake);
      const bool inherited=brake_certificate_&&state_match&&!proof.current_obstacle&&!proof.outside;
      if(proof.clear||inherited){result.input={};result.next_velocity=brake;result.certified_ahead=proof.distance;result.inherited=!proof.clear&&inherited;
        result.decision=hold?4:speed>1e-8?5:0;result.reason=hold?"terminal_hold":speed>1e-8?"certified_emergency_stop":"stationary_protection";
        if(hold)result.flags|=4;tracker_.emergency();accepted=true;}
      else{result.reason="no_current_or_inherited_stop_certificate";result.decision=6;}
    }
    result.proof_ns=field_now_ns()-proof_start;result.evidence=counts_;
    if(counts_.obstacle)result.flags|=8;if(counts_.seed)result.flags|=16;
    if(accepted){expected_=true;expected_v_=result.next_velocity;expected_p_=nav_add(s.p_W,expected_v_,BaseConfig::physics_dt_s);brake_certificate_=true;}
    else brake_certificate_=false;
    result.total_ns=field_now_ns()-start;return result;
  }
  // Retain only actual processed raw frames at separated poses, bounded memory.
  // Identity is immutable: a supervisor serves exactly one static-scene run.
  int64_t retain(){const auto begin=field_now_ns();if(!pipeline_.output().input_ready)return 0;
    if(history_.empty()||nav_norm(nav_add(current_->observation_pose.p_W,history_.back()->observation_pose.p_W,-1))>=.20){
      std::unique_ptr<NavigationObservation>replacement;
      if(history_.size()==16){replacement=std::move(history_.front());history_.pop_front();}
      else{replacement=std::move(spares_.back());spares_.pop_back();}
      history_.push_back(std::move(current_));current_=std::move(replacement);
    }
    return field_now_ns()-begin;
  }
  TubeStatus ball(const Vec&center,double radius){
    const auto current=RawDepthBallOracle(current_->depth,pipeline_.builder().local_geometry(),current_->observation_pose)(center,radius);
    const auto decision=static_evidence_union(current,scenario_.contains(center,radius),history_.size(),[&](size_t i){
      const auto&frame=*history_[history_.size()-1-i];
      const auto status=RawDepthBallOracle(frame.depth,pipeline_.builder().local_geometry(),frame.observation_pose)(center,radius);
      if(status==TubeStatus::PotentialObstacle)++counts_.history_obstacle;
      return status;
    });
    if(decision.status==TubeStatus::PotentialObstacle){++counts_.obstacle;if(decision.source==EvidenceSource::Current)++counts_.current_obstacle;}
    else if(decision.status==TubeStatus::OutsideObservationRange)++counts_.outside;
    else if(decision.status==TubeStatus::Unknown)++counts_.unknown;
    else if(decision.source==EvidenceSource::Current)++counts_.current;
    else if(decision.source==EvidenceSource::Seed)++counts_.seed;
    else if(decision.source==EvidenceSource::History)++counts_.history;
    return decision.status;
  }
  MotionProof certify(const Vec&position,const Vec&next_velocity){
    MotionProof proof;const double speed=nav_norm(next_velocity),dt=BaseConfig::physics_dt_s;
    // Semi-implicit 150 m/s² zero-input braking travels no more than this
    // continuous stopping distance plus one discrete step. Includes THIS step.
    const double distance=speed*dt+speed*speed/(2*BaseConfig::plant_acceleration_mps2)+speed*dt;
    proof.distance=distance;const Vec direction=speed>0?nav_scale(next_velocity,1/speed):Vec{};
    const unsigned count=std::max(1u,unsigned(std::ceil(distance/.12)));
    for(unsigned i=0;i<count;++i){const double a=distance*i/count,b=distance*(i+1)/count;
      const Vec c=nav_add(position,direction,(a+b)*.5);const auto previous_current_vetoes=counts_.current_obstacle;
      const auto status=ball(c,NavigationConfig::occupancy_tube_radius_m+(b-a)*.5+1e-12);
      proof.clear&=status==TubeStatus::ObservedClear;proof.obstacle|=status==TubeStatus::PotentialObstacle;proof.outside|=status==TubeStatus::OutsideObservationRange;
      proof.current_obstacle|=counts_.current_obstacle!=previous_current_vetoes;
      // Keep checking after Unknown: a later current obstacle must veto an
      // inherited stop proof, regardless of evaluation order.
    }
    return proof;
  }
 private:
  Scenario scenario_;uint64_t calibration_;NavigationPipeline pipeline_;VariableReferenceTracker tracker_;
  std::unique_ptr<NavigationObservation>current_;std::deque<std::unique_ptr<NavigationObservation>>history_,spares_;
  EvidenceCounts counts_;Vec expected_p_{},expected_v_{};bool expected_=false,brake_certificate_=false;int64_t last_plan_ns_=-50000000;
  uint64_t active_field_version_=0,active_curve_version_=0,active_geometry_version_=0;
};
} // namespace ego4p3_sim
