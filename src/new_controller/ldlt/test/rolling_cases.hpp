#pragma once
#include "fixed_tracking_cases.hpp"
#include "new_controller/navigation_io.hpp"
#include "new_controller/rolling_navigation.hpp"

namespace ego4p0::rolling_fixture {
using tracking_fixture::require;

struct ObservedScene {
  Direction3 observer{};
  double wall_x=std::numeric_limits<double>::infinity();

  TubeStatus operator()(const Direction3 &center,double radius)const {
    if(nav_norm(nav_add(center,observer,-1.))+radius>BaseConfig::local_range_m)
      return TubeStatus::OutsideObservationRange;
    return center[0]+radius>=wall_x?TubeStatus::PotentialObstacle:TubeStatus::ObservedClear;
  }

  bool swept_clear(const Direction3 &start,const Direction3 &end)const {
    return std::max(start[0],end[0])+BaseConfig::body_radius_m<wall_x;
  }
};

inline void set_observation(NavigationObservation &o,uint64_t group,const NavigationState &state) {
  o.depth.sequence=group;
  o.depth.t_ns=state.actual.t_ns;
  o.identity={group,1,group,o.depth.t_ns,{o.depth.t_ns,o.depth.t_ns,o.depth.t_ns,o.depth.t_ns},15};
  o.observation_pose=state.actual;
  o.observation_pose_valid=true;
}

inline void set_state(NavigationState &s,const Direction3 &p,const Direction3 &v,int64_t stamp) {
  s.actual.p_W=p;
  s.actual.v_B=v;
  s.actual.t_ns=stamp;
  s.actual_valid=true;
  s.current_simulation_ns=stamp;
}

inline constexpr int steps_per_case=140;
inline constexpr int case_count=5;
inline void run(NavigationPipeline &pipeline,NavigationObservation &packet,
                const std::filesystem::path &csv_path,int repeats,bool periodic) {
  std::ofstream csv;
  if(!csv_path.empty()){
    csv.open(csv_path);
    if(!csv)throw std::runtime_error("cannot open rolling CSV");
    csv<<std::setprecision(17)<<"case,repeat,step,wall_ns,release_lateness_ns,response_ns,skipped_releases,planning_ns,certification_ns,control_ns,planned,continued,committed,cover_balls,version,s_m,velocity_mps,error_bound_m\n";
  }
  constexpr double dt=BaseConfig::physics_dt_s;
  constexpr double speed=BaseConfig::max_speed_mps;
  RollingNavigator rolling(pipeline);
  uint64_t group=0;
  for(const std::string name:{"straight","wall90","turn180","spatial","switch"}){
    const double wall=name=="wall90"?3.8:std::numeric_limits<double>::infinity();
    const auto turn=sphere_point(direction_pi/2.,0.);
    Direction3 desired=name=="wall90"?turn:name=="turn180"?Direction3{-1,0,0}:
      name=="spatial"?sphere_point(direction_pi/9.,direction_pi/20.):Direction3{1,0,0};
    for(auto &image:packet.depth.D)std::fill(image.begin(),image.end(),10.f);
    for(int repeat=0;repeat<repeats;++repeat){
      rolling.reset_epoch();
      auto release=field_now_ns()+BaseConfig::period_ns;
      Direction3 position{},velocity{speed,0,0};
      RollingActuatorState actuator{velocity,{}};
      uint64_t commits=0,plans=0;
      bool held_in_turn=false;
      double progress=0.;
      for(int step=0;step<steps_per_case;++step){
        if(periodic){
          const timespec t{release/1000000000LL,release%1000000000LL};
          int error;
          do{error=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);}while(error==EINTR);
          require(error==0,"rolling periodic release");
        }
        NavigationState state;
        if(name=="spatial"&&step==20)position[2]+=.03; // fixture disturbance
        set_state(state,position,velocity,int64_t(step+1)*BaseConfig::period_ns);
        set_observation(packet,++group,state);
        Direction3 goal=desired;
        if(name=="switch"&&step>=12)
          goal=(step>=22&&step<=25)?sphere_point(direction_pi/2.,.15):turn;
        const ObservedScene observed{position,wall};
        const auto begin=field_now_ns();
        const auto result=rolling.tick_with_oracle(&packet,state,{goal,speed,true,true},actuator,
                                                   [&]{return observed;});
        const auto end=field_now_ns();
        const auto response=end-release;
        const auto skipped=periodic?std::max<int64_t>(0,response/BaseConfig::period_ns):0;
        if(result.mode!=RollingMode::Tracking)
          throw std::runtime_error(name+" step "+std::to_string(step)+" protection "+std::to_string(int(result.reason))+" "+tracking_status_name(result.command.status));
        require(result.global_s>=progress-1e-12,name+" monotonic global progress");
        require(result.cover_balls<=22,name+" bounded proof work");
        progress=result.global_s;
        commits+=result.committed;
        plans+=result.planned;
        if(name=="switch"&&step>=22&&step<=25)held_in_turn|=result.continued&&!result.planned;
        const auto next=tracking_fixture::model_velocity(velocity,result.command.plant_input_W,dt);
        tracking_fixture::verify_step(result.command,velocity,next,actuator.input_W,actuator.omega_W,dt);
        const auto endpoint=nav_add(position,next,dt);
        require(observed.swept_clear(position,endpoint),name+" independent wall collision");
        if(csv.is_open())csv<<name<<','<<repeat<<','<<step<<','<<end-begin<<','
          <<(periodic?begin-release:0)<<','<<(periodic?response:end-begin)<<','<<skipped<<','
          <<result.planning_ns<<','<<result.certification_ns<<','<<result.control_ns<<','
          <<result.planned<<','<<result.continued<<','<<result.committed<<','<<result.cover_balls<<','
          <<result.reference_version<<','<<progress<<','<<nav_norm(next)<<','<<result.command.cross_track_m<<'\n';
        actuator={result.command.plant_input_W,result.command.omega_W};
        position=endpoint;
        velocity=next;
        release+=(skipped+1)*BaseConfig::period_ns;
      }
      require(progress>6.,name+" renewal beyond initial observation");
      require(commits==(name=="switch"?2u:1u)&&plans==commits,name+" event-driven solve and commit");
      if(name=="switch")require(held_in_turn,"retain old ramp during incompatible request");
    }
  }
  if(csv.is_open()&&!csv)throw std::runtime_error("rolling CSV write failed");
}

inline void verify_faults(NavigationPipeline &pipeline,NavigationObservation &packet) {
  RollingNavigator rolling(pipeline);
  rolling.reset_epoch();
  constexpr double speed=BaseConfig::max_speed_mps;
  constexpr auto period=BaseConfig::period_ns;
  constexpr double dt=BaseConfig::physics_dt_s;
  Direction3 position{},velocity{speed,0,0};
  RollingActuatorState actuator{velocity,{}};
  const DesiredMotion forward{{1,0,0},speed,true,true};
  for(auto &image:packet.depth.D)std::fill(image.begin(),image.end(),10.f);
  NavigationState state;
  set_state(state,position,velocity,period);
  set_observation(packet,1,state);
  auto cycle=[&](NavigationObservation *observation,double wall=std::numeric_limits<double>::infinity()){
    return rolling.tick_with_oracle(observation,state,forward,actuator,[&]{return ObservedScene{position,wall};});
  };
  auto first=cycle(&packet);
  require(first.mode==RollingMode::Tracking&&first.committed,"start certified rolling reference");
  const auto duplicate=cycle(&packet);
  require(duplicate.mode==RollingMode::RepeatedState&&!duplicate.new_observation&&!duplicate.planned&&
          duplicate.cover_balls==0&&duplicate.control_ns==0,"same group and stamp must not advance");
  actuator={first.command.plant_input_W,first.command.omega_W};
  for(int k=2;k<=4;++k){
    position=nav_add(position,velocity,dt);
    set_state(state,position,velocity,k*period);
    const auto missing=cycle(k==2?&packet:nullptr);
    require(!missing.new_observation&&!missing.planned&&missing.cover_balls==0,"same observation across state ticks");
    require(missing.mode==(k<4?RollingMode::Tracking:RollingMode::Protection),"interrupt and age-out");
  }
  position=nav_add(position,velocity,dt);
  set_state(state,position,velocity,5*period);
  set_observation(packet,2,state);
  const auto recovery=cycle(&packet);
  require(recovery.mode==RollingMode::Tracking&&recovery.committed&&
          recovery.reference_version>first.reference_version,"recover with new proof and measured entry speed");

  position=nav_add(position,velocity,dt);
  set_state(state,position,velocity,6*period);
  set_observation(packet,3,state);
  auto blocked=cycle(&packet,position[0]+.12);
  require(blocked.mode==RollingMode::Protection,"new wall invalidates prefix");
  const auto braking=tracking_fixture::model_velocity(velocity,{},dt);
  const ObservedScene wall{position,position[0]+.12};
  require(!wall.swept_clear(position,nav_add(position,braking,dt)),"external collision interlock vetoes brake step");

  set_state(state,{},velocity,0);
  ++packet.receive_epoch;
  set_observation(packet,4,state);
  require(cycle(&packet).mode==RollingMode::TimeReset,"explicit epoch reset");
  require(cycle(&packet).mode==RollingMode::Tracking,"re-enter after clock reset");
  set_state(state,{.06,0,0},velocity,period);
  state.configuration_version=NavigationConfig::version+1;
  require(cycle(&packet).mode==RollingMode::Protection,"configuration mismatch");
  state.configuration_version=NavigationConfig::version;
  set_state(state,{.18,0,0},velocity,3*period);
  require(cycle(&packet).mode==RollingMode::Protection,"missed physical sample");
}

inline int verify_steep_spatial_protection(NavigationPipeline &pipeline,NavigationObservation &packet){
  RollingNavigator rolling(pipeline);
  rolling.reset_epoch();
  Direction3 position{},velocity{12.,0.,0.};
  RollingActuatorState actuator{velocity,{}};
  for(int k=0;k<100;++k){
    if(k==20)position[2]+=.03;
    NavigationState state;
    set_state(state,position,velocity,(k+1)*BaseConfig::period_ns);
    set_observation(packet,k+1,state);
    const DesiredMotion desired{sphere_point(direction_pi/4.,direction_pi/9.),12.,true,true};
    const auto result=rolling.tick_with_oracle(&packet,state,desired,actuator,[&]{return ObservedScene{position};});
    if(result.mode==RollingMode::Protection){
      require(result.reason==RollingReason::Controller&&result.command.status==TrackingStatus::OutsideTrackingTube&&
              result.command.plant_input_W==Direction3{},"steep three-dimensional case must reject excessive error");
      return k;
    }
    require(result.mode==RollingMode::Tracking,"steep test entry");
    velocity=tracking_fixture::model_velocity(velocity,result.command.plant_input_W,BaseConfig::physics_dt_s);
    actuator={result.command.plant_input_W,result.command.omega_W};
    position=nav_add(position,velocity,BaseConfig::physics_dt_s);
  }
  throw std::runtime_error("steep three-dimensional test missed its protection condition");
}

inline void verify_mid_ramp_splice(){
  SpatialReference a,b;
  tracking_fixture::build(a,{"a",90,0,0,.005});
  tracking_fixture::build(b,{"b",180,0,0,.005});
  constexpr double at=.7,dt=.005;
  const auto join=a.query(at);
  const auto velocity=nav_scale(join.tangent_W,12.);
  const auto omega=nav_scale(nav_cross(join.tangent_W,join.curvature_W),12.);
  const auto input=nav_add(velocity,join.curvature_W,14.4);
  const RollingActuatorState previous{input,omega};
  RollingReference reference;
  const ObservedScene open{join.position_W};
  require(reference.install(a,at,join.position_W,velocity,previous,dt,open),"attach mid-ramp fixture");
  require(!reference.install(b,0.,join.position_W,velocity,previous,dt,open)&&
          reference.version()==1&&reference.can_step(dt),"retain old certified path after rejected splice");
  require(reference.install(b,at,join.position_W,velocity,previous,dt,open),"commit compatible nonzero-curvature splice");
  const auto next=reference.step(join.position_W,velocity,dt);
  tracking_fixture::verify_step(next,velocity,tracking_fixture::model_velocity(velocity,next.plant_input_W,dt),input,omega,dt);
  require(nav_norm(next.omega_W)>1.&&reference.version()==2,"preserve progress and actuator direction through splice");
}
} // namespace ego4p0::rolling_fixture
