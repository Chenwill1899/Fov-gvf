#include <sstream>
#include <filesystem>
#include "new_controller/navigation_io.hpp"
#include "new_controller/rolling_navigation.hpp"
#include "pc_gvf/verified_start.hpp"

#include <cstdio>
#include <unistd.h>
#include <deque>
#include <iostream>
#include <memory>

using namespace ego4p0;
namespace {
constexpr double dt=BaseConfig::physics_dt_s, speed_limit=BaseConfig::max_speed_mps;
constexpr double tube=NavigationConfig::occupancy_tube_radius_m, seed_radius=2.5;
constexpr double acceleration=BaseConfig::reference_command_acceleration_mps2;
constexpr double emergency=BaseConfig::plant_acceleration_mps2, beta=dt/BaseConfig::plant_tau_s;
using Vec=Direction3;

Vec unit(Vec v){return unit_direction(v)?v:Vec{};}
Vec limit(Vec v,double cap){
  const double n=nav_norm(v);
  return n>cap?ref_scale(v,cap/n):v;
}
Vec steer(const Vec &velocity,const Vec &toward,double max_angle) {
  const auto h=unit(velocity);
  if(nav_norm(h)==0.)return toward;
  const double c=std::clamp(direction_dot(h,toward),-1.,1.);
  const double a=std::acos(c);
  if(a<=max_angle)return toward;
  const auto perp=unit(nav_add(toward,h,-c));
  if(nav_norm(perp)==0.)return h;
  return nav_add(ref_scale(h,std::cos(max_angle)),perp,std::sin(max_angle));
}


struct CruiseRoute {
  std::vector<Vec> points;
  std::vector<double> arc;
  void load(const char *logfile) {
    const auto file=std::filesystem::path(logfile).parent_path()/"intent_waypoints.csv";
    std::ifstream in(file);std::string line;std::getline(in,line);
    while(std::getline(in,line)){
      std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);Vec pt{};
      if(!(s>>pt[0]>>pt[1]>>pt[2]))throw std::runtime_error("invalid guide input");
      if(!points.empty()&&pt[0]<=points.back()[0])throw std::runtime_error("requires x-monotonic R05 test route");
      arc.push_back(points.empty()?0.:arc.back()+nav_norm(nav_add(pt,points.back(),-1.)));points.push_back(pt);
    }
    if(points.size()<2)throw std::runtime_error("missing geometric guide");
  }
  struct Motion { Vec p,v,u,w; };
  Motion next(const Motion &state) const {
    const Vec &pos=state.p,&v=state.v;
    auto it=std::lower_bound(points.begin(),points.end(),pos[0],[](const Vec&a,double x){return a[0]<x;});
    size_t i=std::clamp<size_t>(it-points.begin(),1,points.size()-1);
    const double t=std::clamp((pos[0]-points[i-1][0])/(points[i][0]-points[i-1][0]),0.,1.);
    const double s=std::min(arc.back(),arc[i-1]+t*(arc[i]-arc[i-1])+1.5);
    size_t j=std::clamp<size_t>(std::lower_bound(arc.begin(),arc.end(),s)-arc.begin(),1,arc.size()-1);
    Vec aim=nav_add(points[j-1],nav_add(points[j],points[j-1],-1.),(s-arc[j-1])/(arc[j]-arc[j-1]));
    const Vec diff=nav_add(aim,pos,-1.);const double dist=nav_norm(diff);
    if(dist<1e-8)throw std::runtime_error("missing direction");
    const Vec head=unit(v);Vec target=limit(ref_scale(nav_cross(head,unit(diff)),1.6*12./dist),80./12.);
    const Vec omega=nav_add(state.w,limit(nav_add(target,state.w,-1.),40.*dt));
    const double rate=nav_norm(omega);const Vec axis=unit(omega);
    Vec heading=nav_add(ref_scale(head,std::cos(rate*dt)),nav_cross(axis,head),std::sin(rate*dt));
    heading=nav_add(heading,axis,(1.-std::cos(rate*dt))*direction_dot(axis,head));
    const Vec nv=ref_scale(unit(heading),12.);const Vec input=nav_add(v,nav_add(nv,v,-1.),1./beta);
    return {nav_add(pos,nv,dt),nv,input,omega};
  }
};

struct Flight {
  CruiseRoute trial_route;
  struct ClearBall {Vec center;double radius;};
  NavigationPipeline pipeline;
  RollingNavigator rolling{pipeline};
  std::unique_ptr<NavigationObservation> incoming;
  std::deque<std::unique_ptr<NavigationObservation>> previous;
  std::deque<ClearBall> verified_balls;
  bool seed_valid=false, direct_only=false;
  Vec start{}, old_command{}, old_omega{};
  bool brake_valid=false;
  Vec expected_position{}, expected_velocity{};
  std::ofstream log;
  uint64_t group=0, observed=0;
  int decision=0;

  Flight(const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h,uint64_t id,
         Vec origin,bool seed,bool direct,const char *logfile)
      :incoming(std::make_unique<NavigationObservation>()),seed_valid(seed),direct_only(direct),start(origin),log(logfile) {
    trial_route.load(logfile);
    pipeline.configure(cal,w,h,id);
    pipeline.allocate(*incoming);
    log<<"step,mode,rolling_reason,rolling_tracking,field_attempted,angular_ready,spatial_ready,reference_commit,history,planning_ns,certification_ns,control_ns,group,command_x,command_y,command_z,step_compute_ns,prepare7_ns,steer7_ns,curve7_ns,stop7_ns,pre_ns,pool_ns,cam_wall_ns,cam_cpu_ns,cam_project_ns,cam_compress_ns,cam_repair_ns,field_ns,spatial_ns,occupancy_ns\n";
  }

  bool read(uint64_t id,const BodyState &state) {
    for(auto &image:incoming->depth.D){
      if(std::fread(image.data(),sizeof(float),image.size(),stdin)!=image.size())return false;
    }
    navigation_identity(*incoming,++observed,id);
    incoming->observation_pose=state;
    incoming->depth.t_ns=state.t_ns;
    incoming->depth.observations.ready=false;
    incoming->depth.local.ready=false;
    incoming->identity.simulation_stamp_ns=state.t_ns;
    incoming->identity.camera_stamp_ns.fill(state.t_ns);
    return true;
  }

  void retain() {
    if(!incoming->depth.local.ready)return;
    const Vec p=incoming->observation_pose.p_W;
    if(!previous.empty()&&nav_norm(nav_add(p,previous.front()->observation_pose.p_W,-1.))<.25)return;
    std::unique_ptr<NavigationObservation> buffer;
    if(previous.size()>=12){
      buffer=std::move(previous.back());
      previous.pop_back();
    }else{
      buffer=std::make_unique<NavigationObservation>();
      pipeline.allocate(*buffer);
    }
    previous.push_front(std::move(incoming));
    incoming=std::move(buffer);
  }

  Vec step(const BodyState &actual,const Vec &velocity,const Vec &target,bool final_target,double guide_speed) {
    const auto t0=field_now_ns(); int64_t prepare=0, steering=0, curve=0, stop=0;
    ++group;
    decision=0;
    const Vec p=actual.p_W;
    const auto offset=nav_add(target,p,-1.);
    const double remaining=nav_norm(offset);
    const Vec desired_world=unit(offset);
    const Vec desired_body=nav_unrotate(actual.R_WB,desired_world);
    const auto &geometry=pipeline.builder().local_geometry();
    auto raw=[&](const NavigationObservation &frame,const Vec &center,double radius){
      return RawDepthBallOracle(frame.depth,geometry,frame.observation_pose)(center,radius);
    };
    auto observed_space=[&](const Vec &center,double radius){
      if(nav_norm(nav_add(center,p,-1.))+radius>BaseConfig::local_range_m)
        return TubeStatus::OutsideObservationRange;
      auto status=raw(*incoming,center,radius);
      if(status==TubeStatus::ObservedClear)return status;
      if(seed_valid&&nav_norm(nav_add(center,start,-1.))+radius<=seed_radius)return TubeStatus::ObservedClear;
      for(const auto &frame:previous)
        if(raw(*frame,center,radius)==TubeStatus::ObservedClear)return TubeStatus::ObservedClear;
      return status;
    };
    auto cached_clear=[&](const Vec &center,double radius){
      for(const auto &ball:verified_balls){
        const double r=ball.radius-radius;
        if(r<0.)continue;
        const auto delta=nav_add(center,ball.center,-1.);
        if(direction_dot(delta,delta)<=r*r)return true;
      }
      return false;
    };
    auto oracle=[&](const Vec &center,double radius){
      if(nav_norm(nav_add(center,p,-1.))+radius>BaseConfig::local_range_m)
        return TubeStatus::OutsideObservationRange;
      if(cached_clear(center,radius))return TubeStatus::ObservedClear;
      return observed_space(center,radius);
    };
    std::vector<ClearBall> proof_balls;
    auto covered=[&](const Vec &a,const Vec &b,double radius){
      const Vec delta=nav_add(b,a,-1.);
      const auto center=nav_add(a,delta,.5);
      radius+=.5*nav_norm(delta);
      if(oracle(center,radius)!=TubeStatus::ObservedClear)return false;
      if(!cached_clear(center,radius+.025)){
        const double expanded=radius+.045;
        const double verified=observed_space(center,expanded)==TubeStatus::ObservedClear?expanded:radius;
        if(!cached_clear(center,verified))proof_balls.push_back({center,verified});
      }
      return true;
    };
    auto proof=[&](const Vec &next){
      proof_balls.clear();
      const double speed=nav_norm(next);
      const auto endpoint=nav_add(p,next,dt);
      if(!covered(p,endpoint,tube))return false;
      const double stop=speed*speed/(2.*emergency)+speed*dt;
      const auto h=unit(next);
      for(double from=0.;from<stop;){
        const double to=std::min(stop,from+.12);
        if(!covered(nav_add(endpoint,h,from),nav_add(endpoint,h,to),tube))return false;
        from=to;
      }
      return true;
    };
    auto plant=[&](const Vec &input){
      if(nav_norm(input)<1e-8)
        return ref_scale(unit(velocity),std::max(0.,nav_norm(velocity)-emergency*dt));
      return nav_add(velocity,limit(ref_scale(nav_add(input,velocity,-1.),beta),emergency*dt));
    };
    const bool inherited_brake=brake_valid&&nav_norm(nav_add(p,expected_position,-1.))<1e-8&&
      nav_norm(nav_add(velocity,expected_velocity,-1.))<1e-8;

    NavigationState state;
    state.actual_valid=true;
    state.actual=actual;
    state.current_simulation_ns=actual.t_ns;
    const DesiredMotion desired{desired_body,speed_limit,true,remaining>1e-6};
    const bool approaching=final_target&&remaining<4.0;
    const double speed=nav_norm(velocity);
    RollingTick tick;
    // At cruise speed, let the reference, swept step and stopping proof
    // decide whether tracking can continue. Route speed applies on fallback.

    // Retrospective physical gate, fixed before this run. No use of world occupancy to choose commands.
    if(p[0]>=50.766 && !approaching && speed>=11.999){
      rolling.interlock();
      auto &ob=pipeline.process_if(*incoming,state,desired,false,false,[]{return NavigationPlan{false,false,nullptr};});
      prepare=field_now_ns()-t0;
      CruiseRoute::Motion preview{p,velocity,old_command,old_omega},first{};
      for(int n=0;n<10;++n){
        const auto ta=field_now_ns();
        const auto next=trial_route.next(preview);
        if(n==0)first=next;
        const bool dynamics=NavigationLimits::heading_input(next.w,preview.w,dt)
            &&NavigationLimits::compensation_input(next.u,preview.u,dt)
            &&nav_norm(nav_add(next.v,preview.v,-1.))<=80.*dt+1e-8;
        const auto tb=field_now_ns();steering+=tb-ta;
        if(!ob.input_ready||!dynamics||!covered(preview.p,next.p,tube)){
          std::cerr<<"CRUISE_REJECT step="<<group<<" prefix_part="<<n<<" input="<<ob.input_ready<<" dynamics="<<dynamics<<std::endl;
          decision=4;return {};
        }
        curve+=field_now_ns()-tb;
        preview=next;
      }
      const auto ts=field_now_ns();
      if(!proof(first.v)){
        std::cerr<<"CRUISE_REJECT step="<<group<<" straight_stop_or_step"<<std::endl;
        decision=4;return {};
      }
      stop=field_now_ns()-ts;
      old_omega=first.w;old_command=first.u;decision=7;
    }else if(speed>=speed_limit-FixedReferenceTracker::attachment_speed_tolerance_mps&&!approaching){
      tick=rolling.tick_with_oracle(incoming.get(),state,desired,{old_command,old_omega},[&]{return oracle;});
      if(tick.mode==RollingMode::Tracking&&proof(tick.command.next_velocity_W)){
        old_omega=tick.command.omega_W;
        old_command=tick.command.plant_input_W;
        decision=1;
      }
    }else{
      rolling.reset_epoch();
      pipeline.process_if(*incoming,state,desired,false,true,[&]{
        return NavigationPlan{group%10==1,false,nullptr};
      });
    }

    if(decision==0) {
      rolling.interlock();
      const auto &plan=pipeline.output();
      const double desired_speed=approaching?
        std::min(guide_speed,std::sqrt(2.*acceleration*std::max(0.,remaining-.22))):guide_speed;
      const double speed_change=std::clamp(desired_speed-speed,-acceleration*dt,acceleration*dt);
      const double nominal_speed=std::max(0.,speed+speed_change);
      const double max_angle=std::min(6.667,80./std::max(1.,speed))*dt;
      const double max_slew=NavigationConfig::compensation_slew_limit_mps2*dt;
      std::vector<Vec> axes;
      if(plan.spatial_ready){
        const auto sample=pipeline.reference().query(std::min(.30,plan.turn_length_m));
        if(nav_angle(desired_world,sample.tangent_W)<.30)axes.push_back(sample.tangent_W);
      }
      axes.push_back(desired_world);
      if(!direct_only) {
        const double az=std::atan2(desired_world[1],desired_world[0]);
        const double el=std::atan2(desired_world[2],std::hypot(desired_world[0],desired_world[1]));
        for(const double deg:{12.,-12.,25.,-25.,45.,-45.,70.,-70.,100.,-100.,160.,-160.}){
          const double a=az+deg*direction_pi/180.;
          axes.push_back(sphere_point(a,el));
        }
        for(const double deg:{12.,-12.,25.,-25.,40.,-40.})
          axes.push_back(sphere_point(az,std::clamp(el+deg*direction_pi/180.,-1.3,1.3)));
      }
      auto try_axis=[&](const Vec &axis,double next_speed){
        if(nav_norm(axis)==0.)return false;
        const Vec aim=steer(velocity,axis,max_angle);
        const auto next=ref_scale(aim,next_speed);
        const auto requested=nav_add(velocity,nav_add(next,velocity,-1.),1./beta);
        const auto input=limit(nav_add(old_command,limit(nav_add(requested,old_command,-1.),max_slew)),
                               NavigationConfig::compensation_input_limit_mps);
        const auto applied=plant(input);
        if(nav_norm(applied)>speed_limit+1e-8||!proof(applied))return false;
        old_command=input;old_omega={};
        decision=2;
        return true;
      };
      bool accepted=false;
      for(const auto &axis:axes)if(try_axis(axis,nominal_speed)){accepted=true;break;}
      if(!accepted&&speed>0.)
        accepted=try_axis(unit(velocity),std::max(0.,speed-emergency*dt));
      if(!accepted) {
        const auto stopped=plant({});
        if(speed==0.||proof(stopped))decision=3;
        else if(inherited_brake)decision=5;
        else decision=4;
        old_command={};old_omega={};
      }
    }
    // Continue the previously certified straight stop only while the exact
    // applied state matches. Isaac independently checks each swept segment.
    expected_velocity=plant(old_command);
    expected_position=nav_add(p,expected_velocity,dt);
    brake_valid=decision==7||decision==1||decision==2||decision==5||
      (decision==3&&nav_norm(expected_velocity)>0.);
    if(decision==7||decision==1||decision==2||decision==3) {
      for(const auto &ball:proof_balls) {
        verified_balls.push_front(ball);
        if(verified_balls.size()>192)verified_balls.pop_back();
      }
    }
    const auto &plan=pipeline.output();
    const auto elapsed=field_now_ns()-t0;
    const auto &depth=incoming->depth;const auto &oi=depth.observations;
    const int camera=std::max_element(trial_cam_wall.begin(),trial_cam_wall.end())-trial_cam_wall.begin();
    log<<group<<','<<decision<<','<<int(tick.reason)<<','<<(tick.mode==RollingMode::Tracking)<<','
       <<plan.field_attempted<<','<<plan.angular_ready<<','<<plan.spatial_ready<<','<<tick.committed<<','
       <<previous.size()<<','<<tick.planning_ns<<','<<tick.certification_ns<<','<<tick.control_ns<<','
       <<incoming->identity.group<<','<<old_command[0]<<','<<old_command[1]<<','<<old_command[2]<<','<<elapsed<<','<<prepare<<','<<steering<<','<<curve<<','<<stop
       <<','<<plan.preprocessing_ns<<','<<oi.camera_work_ns<<','<<trial_cam_wall[camera]<<','<<trial_cam_cpu[camera]
       <<','<<oi.projection_ns[camera]<<','<<oi.compression_ns[camera]<<','<<depth.repair_ns[camera]
       <<','<<plan.field_ns<<','<<plan.spatial_ns<<','<<plan.occupancy_ns<<'\n';
    retain();
    return old_command;
  }
};
} // namespace

int main(int argc,char **argv) try {
  if(argc!=7)return 2;
  cpu_set_t allowed;CPU_ZERO(&allowed);for(int i=0;i<5;++i)CPU_SET(i,&allowed);
  if(pthread_setaffinity_np(pthread_self(),sizeof(allowed),&allowed))throw std::runtime_error("CPU0-4 unavailable");
  const Vec start{-46.4,-16.8,10.};
  std::string diagnosis;
  const bool seed=pc_gvf::depth_angular::certifyDefaultCloudStart(
    argv[2],argv[3],Eigen::Vector3d(start[0],start[1],start[2]),seed_radius,&diagnosis);
  if(!seed)throw std::runtime_error(diagnosis);
  uint32_t w,h;
  const auto cal=navigation_calibration(argv[1],w,h);
  const auto version=navigation_calibration_version(cal,w,h);
  Flight flight(cal,w,h,version,start,seed,std::string(argv[5])=="direct-only",argv[4]);
  const bool fifo=std::string(argv[6])=="fifo";
  if(fifo)flight.pipeline.enable_fifo5();
  else {cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(0,&mask);if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("CPU0 unavailable");}
  int policy=0;sched_param p{};
  pthread_getschedparam(pthread_self(),&policy,&p);
  flight.pipeline.builder().refresh_worker_schedulers();
  for(unsigned i=0;i<4;++i)
    if(flight.pipeline.builder().worker_policies()[i]!=(fifo?SCHED_FIFO:SCHED_OTHER)||
       flight.pipeline.builder().worker_priorities()[i]!=(fifo?5:0)||
       flight.pipeline.builder().worker_cpus()[i]!=int(i+1))throw std::runtime_error("worker schedule mismatch");
  if(policy!=(fifo?SCHED_FIFO:SCHED_OTHER)||p.sched_priority!=(fifo?5:0))throw std::runtime_error("coordinator schedule mismatch");
  std::cerr<<"seed: "<<diagnosis<<"; scheduler: "<<policy<<'/'<<p.sched_priority<<" (verified five threads)\n";
  { std::ofstream ready(std::filesystem::path(argv[4]).parent_path()/"controller_ready.json");
    ready<<"{\"pid\":"<<getpid()<<",\"coordinator_policy\":"<<policy<<",\"coordinator_priority\":"<<p.sched_priority<<",\"coordinator_cpu\":"<<sched_getcpu()<<",\"worker_policies\":[";
    for(int i=0;i<4;++i){if(i)ready<<',';ready<<flight.pipeline.builder().worker_policies()[i];}
    ready<<"],\"worker_priorities\":[";for(int i=0;i<4;++i){if(i)ready<<',';ready<<flight.pipeline.builder().worker_priorities()[i];}
    ready<<"],\"worker_cpus\":[";for(int i=0;i<4;++i){if(i)ready<<',';ready<<flight.pipeline.builder().worker_cpus()[i];}ready<<"]}\n"; }
  uint64_t frame=0;
  std::array<double,12> msg;
  while(std::fread(msg.data(),sizeof(double),msg.size(),stdin)==msg.size()){
    BodyState actual;
    actual.t_ns=int64_t(frame++)*BaseConfig::period_ns;
    actual.p_W={msg[0],msg[1],msg[2]};
    const Vec velocity{msg[3],msg[4],msg[5]};
    const double c=std::cos(msg[6]),s=std::sin(msg[6]);
    actual.R_WB={c,-s,0.,s,c,0.,0.,0.,1.};
    actual.v_B=nav_unrotate(actual.R_WB,velocity);
    if(!flight.read(version,actual))throw std::runtime_error("truncated depth group");
    if(!std::isfinite(msg[11])||msg[11]<0.||msg[11]>speed_limit)
      throw std::runtime_error("invalid guidance speed");
    const auto input=flight.step(actual,velocity,{msg[7],msg[8],msg[9]},msg[10]!=0.,msg[11]);
    const std::array<double,4> output{input[0],input[1],input[2],double(flight.decision)};
    if(std::fwrite(output.data(),sizeof(double),output.size(),stdout)!=output.size())break;
    std::fflush(stdout);
  }
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
