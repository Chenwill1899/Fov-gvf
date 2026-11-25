#include <sstream>
#include <filesystem>
#include "new_controller/navigation_io.hpp"
#include "new_controller/spherical_navigation.hpp"
#include "directional_guidance.hpp"
#include "pc_gvf/verified_start.hpp"
#include "image_depth_clearance.hpp"

#include <cstdio>
#include <unistd.h>
#include <deque>
#include <iostream>
#include <memory>
#include <string_view>

using namespace ego4p0;
namespace {
using namespace ego4p0::cruise;
constexpr double tube=NavigationConfig::occupancy_tube_radius_m, seed_radius=2.5;
constexpr double acceleration=BaseConfig::reference_command_acceleration_mps2;

struct Flight {
  CruiseRoute trial_route;
  ImageDepthRig image_rig;
  struct ClearBall {Radial center;double radius;};
  SphericalNavigation pipeline;
  std::unique_ptr<CruiseDepthFrame> incoming;
  std::deque<std::unique_ptr<CruiseDepthFrame>> previous;
  std::deque<ClearBall> verified_balls;
  bool seed_valid=false, direct_only=false, directional=false;
  Radial start{}, old_command{}, old_omega{};
  bool brake_valid=false, previous_direct=false;
  enum class DirectionField { Missing, Grid, LocalCap };
  DirectionField direction_field=DirectionField::Missing;
  Radial field_origin{};
  Bearing field_goal{};
  double field_yaw=0.;
  uint32_t field_cursor=0;
  Radial expected_position{}, expected_velocity{};
  std::ofstream log;
  uint64_t group=0, observed=0;
  int decision=0;

  Flight(const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h,uint64_t id,
         Radial origin,bool seed,bool direct,const char *logfile)
      :incoming(std::make_unique<CruiseDepthFrame>()),seed_valid(seed),direct_only(direct),start(origin),log(logfile) {
    trial_route.load(std::filesystem::path(logfile).parent_path()/"intent_waypoints.csv");
    pipeline.configure(cal,w,h,id,true);
    image_rig.configure(pipeline.builder().local_geometry());
    pipeline.allocate(*incoming);
    log<<"step,mode,rolling_reason,rolling_tracking,field_attempted,angular_ready,spatial_ready,reference_commit,history,planning_ns,certification_ns,control_ns,group,command_x,command_y,command_z,step_compute_ns,prepare7_ns,steer7_ns,curve7_ns,stop7_ns,pre_ns,pool_ns,cam_wall_ns,cam_cpu_ns,cam_project_ns,cam_compress_ns,cam_repair_ns,field_ns,spatial_ns,occupancy_ns,cruise_requested,cruise_check,terminal_demand,refine_attempts,refine_clear,field_component_ns,coarse_domain_ns,fine_domain_ns,fine_solve_ns,field_curve_ns,index_merge_ns,local_merge_ns,cam0_start_ns,cam0_end_ns,cam1_start_ns,cam1_end_ns,cam2_start_ns,cam2_end_ns,cam3_start_ns,cam3_end_ns,flowbox_valid,flowbox_normal_m,flowbox_binormal_m,flowbox_source,directional,field_heading_used,curve_points,required_travel_m,certified_travel_m,heading_source,grid_request_reason\n";
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
    incoming->depth.local.panorama_pending=false;
    incoming->identity.simulation_stamp_ns=state.t_ns;
    incoming->identity.camera_stamp_ns.fill(state.t_ns);
    return true;
  }

  void retain() {
    if(!LocalFieldView(incoming->depth,pipeline.builder().local_geometry()).image_evidence_ready())return;
    if(!previous.empty()&&distance(incoming->world_position,previous.front()->world_position)<.25)return;
    std::unique_ptr<CruiseDepthFrame> buffer;
    if(previous.size()>=12){
      buffer=std::move(previous.back());
      previous.pop_back();
    }else{
      buffer=std::make_unique<CruiseDepthFrame>();
      pipeline.allocate(*buffer);
    }
    previous.push_front(std::move(incoming));
    incoming=std::move(buffer);
  }

  bool free_travel(Radial p,Radial velocity,double length,double radius=tube) {
    const auto &g=pipeline.builder().local_geometry();
    if(!pipeline.output().input_ready || !incoming->depth.observations.raw_summaries_ready ||
       !LocalFieldView(incoming->depth,g).image_evidence_ready())return false;
    auto classify=[&](const CruiseDepthFrame &frame,bool positive_only) {
      return image_rig.direction_clearance(frame,g,p,velocity.dir,length,radius,positive_only);
    };
    const auto current=classify(*incoming,false);
    if(current==TubeStatus::ObservedClear)return true;
    if(current!=TubeStatus::Unknown)return false;
    // In a static scene a prior complete observation or the proven starting
    // ball may fill a blind spot. Fresh surface contact always vetoes first.
    const auto relative=spherical::add(p,start,-1.);
    if(seed_valid && std::max(relative.r,spherical::add(relative,{velocity.dir,length}).r)
       +radius+1e-7<=seed_radius)return true;
    for(const auto &old:previous)if(classify(*old,true)==TubeStatus::ObservedClear)return true;
    return false;
  }

  struct DirectionTrace {
    double required_travel=0.,certified_travel=0.;
    int64_t prepare_ns=0,certification_ns=0,first_pre_ns=0;
    std::string_view source="none",request_reason="none";
  };

  DirectionTrace direction_step(int64_t now,Radial p,Radial velocity,double yaw,
                                const CruiseRoute::Tracking &route,bool cruise,double guide_speed,
                                bool approaching,double remaining,bool inherited_brake) {
    DirectionTrace trace;
    const auto begin=field_now_ns();
    const double speed=velocity.r;
    CruiseRoute::Actuation preferred{};
    const double target_speed=approaching?
      std::min(guide_speed,std::sqrt(2.*acceleration*std::max(0.,remaining-.22))):guide_speed;
    const double nominal=cruise?speed_limit:std::max(0.,speed+std::clamp(target_speed-speed,-acceleration*dt,acceleration*dt));
    Bearing goal=route.tangent;
    if(cruise){
      preferred=trial_route.cruise_command({p,velocity,old_command,old_omega},route);
      goal=preferred.v.dir;
    }else if(route.valid && nominal>0.){
      goal=route.box.guidance(route.coordinates,velocity,nominal,NavigationConfig::lateral_gain_per_s,false).desired;
    }
    const Bearing goal_body{spherical::wrap(goal.az-yaw),goal.el};
    const double turn_budget=std::min(6.667,80./std::max(1.,speed))*dt;
    const bool moving=nominal>0. && route.valid;
    if(moving){
      if(direction_field==DirectionField::Missing)trace.request_reason="initial";
      else if(spherical::angle(field_goal,goal)>RollingNavigator::intent_change_rad)trace.request_reason="intent";
      else if(distance(p,field_origin)+required_free_travel(std::max(speed,nominal))>=BaseConfig::local_range_m)
        trace.request_reason="range";
    }
    const bool initial_request=trace.request_reason!="none";
    const auto &angular=pipeline.angular_step(*incoming,now,p,velocity,goal_body,route.valid,initial_request);
    trace.prepare_ns=field_now_ns()-begin;
    auto certify=[&](Radial v,double radius=tube){
      trace.required_travel=required_free_travel(v.r);
      const auto before=field_now_ns();
      const bool clear=free_travel(p,v,trace.required_travel,radius);
      trace.certification_ns+=field_now_ns()-before;
      trace.certified_travel=clear?trace.required_travel:0.;
      return clear;
    };
    auto apply_heading=[&](Bearing world,std::string_view source){
      const bool use_cruise=cruise && spherical::angle(world,goal)<1e-9;
      Radial input{},omega{};
      if(use_cruise){input=preferred.u;omega=preferred.w;}
      else{
        const Radial next{steer(velocity,world,turn_budget),nominal};
        const auto requested=add(velocity,add(next,velocity,-1.),1./beta);
        input=limit(add(old_command,limit(add(requested,old_command,-1.),
                      NavigationConfig::compensation_slew_limit_mps2*dt)),NavigationConfig::compensation_input_limit_mps);
      }
      const auto applied=predicted_velocity(velocity,input);
      if(applied.r>speed_limit+1e-8 || distance(applied,velocity)>80.*dt+1e-8 ||
         !within_input_limits(input,old_command,NavigationConfig::compensation_input_limit_mps,
                             NavigationConfig::compensation_slew_limit_mps2) ||
         (use_cruise && (!within_input_limits(omega,old_omega,NavigationConfig::heading_rate_limit_rad_s,
                             NavigationConfig::heading_acceleration_limit_rad_s2) || distance(applied,preferred.v)>=1e-8)))
        return false;
      double radius=tube;
      if(source=="local_harmonic"){
        const auto start_dir=speed>1e-9?velocity.dir:world;
        const double span=spherical::angle(start_dir,world);
        if(span>=spherical::pi/2.)return false;
        // Both the spherical cap and the applied stop ray must fit. For
        // any q in cap(start_dir,span), angle(q,applied) <= span+offset.
        const double offset=spherical::angle(start_dir,applied.dir);
        radius+=2.*required_free_travel(applied.r)*std::sin(.5*std::min(spherical::pi,span+offset));
      }
      if(!certify(applied,radius))return false;
      decision=use_cruise?7:2;old_command=input;old_omega=omega;
      trace.source=source;
      return true;
    };
    bool local_tried=false;
    auto local_heading=[&](){
      local_tried=true;
      return apply_heading(cruise?goal:steer(velocity,goal,turn_budget),"local_harmonic");
    };
    if(moving && angular.input_ready){
      if(!initial_request){
        if(direction_field==DirectionField::Grid){
          const auto relative=[&](Bearing q){return Bearing{spherical::wrap(q.az-field_yaw),q.el};};
          const auto heading=continue_field_heading(pipeline.field(),relative(speed>1e-9?velocity.dir:goal),
                                                    relative(goal),turn_budget,field_cursor);
          if(!heading)trace.request_reason="curve";
          else if(!apply_heading({spherical::wrap(heading->az+field_yaw),heading->el},"reused_grid_harmonic"))
            trace.request_reason="clearance_or_limits";
        }else if(!local_heading())trace.request_reason="clearance_or_limits";
        if(!decision){
          const auto before=field_now_ns();
          trace.first_pre_ns=angular.preprocessing_ns;
          pipeline.angular_step(*incoming,now,p,velocity,goal_body,route.valid,true);
          trace.prepare_ns+=field_now_ns()-before;
        }
      }
      if(!decision){
        field_origin=p;field_goal=goal;field_yaw=yaw;field_cursor=0;
        const auto heading=angular.angular_ready?field_step_heading(pipeline.field(),goal_body,turn_budget):std::nullopt;
        if(heading && apply_heading({spherical::wrap(heading->az+yaw),heading->el},"grid_harmonic")){
          direction_field=DirectionField::Grid;
        }else if(!local_tried && angular.input_ready && local_heading()){
          direction_field=DirectionField::LocalCap;
        }
      }
    }
    if(!decision){
      direction_field=DirectionField::Missing;
      const auto braking=predicted_velocity(velocity,{});
      if(certify(braking) || speed==0.)decision=3;
      else decision=inherited_brake?5:4;
      old_command={};old_omega={};
    }
    return trace;
  }

  Direction3 step(int64_t now, Radial p, Radial velocity, double yaw, Radial target,
                  bool final_target,double guide_speed,std::optional<double> route_station) {
    const auto t0=field_now_ns(); int64_t prepare=0, steering=0, curve=0, stop=0;
    ++group;
    decision=0;
    const double remaining=final_target?distance(target,p):0.;
    // Current packets share the guide station already measured by RouteIntent.
    // Older radial and Cartesian packets carry only the visibility probe.
    const double progress=route_station?*route_station:trial_route.project(p,trial_route.current_segment);
    const auto route=trial_route.tracking(p,progress,final_target);
    const Bearing desired_body{spherical::wrap(route.tangent.az-yaw),route.tangent.el};
    const bool has_direction=route.valid;
    const auto &geometry=pipeline.builder().local_geometry();
    incoming->prepare(geometry,image_rig,p,yaw);
    auto cached_clear=[&](Radial center,double radius){
      for(const auto &ball:verified_balls){
        const double r=ball.radius-radius;
        // Radial separation is a lower bound on the hypot used by distance().
        if(std::abs(center.r-ball.center.r)>r)continue;
        if(distance(center,ball.center)<=r)return true;
      }
      return false;
    };
    unsigned refine_attempts=0,refine_clear=0;
    auto oracle=[&](Radial center,double radius){
      // Static certificates can fill a blind spot only after the current
      // frame was processed and is available to veto a new surface.
      if(!pipeline.output().input_ready || !LocalFieldView(incoming->depth,geometry).image_evidence_ready())
        return TubeStatus::Unknown;
      if(distance(center,p)+radius>BaseConfig::local_range_m)
        return TubeStatus::OutsideObservationRange;
      const auto current=(*incoming->raw)(center,radius);
      if(current==TubeStatus::ObservedClear || current==TubeStatus::OutsideObservationRange)return current;
      bool certificate=cached_clear(center,radius) ||
          (seed_valid && distance(center,start)+radius<=seed_radius);
      if(!certificate){
        for(const auto &frame:previous){
          if((*frame->raw)(center,radius)==TubeStatus::ObservedClear){certificate=true;break;}
        }
      }
      if(!certificate)return current;
      if(current==TubeStatus::PotentialObstacle){
        ++refine_attempts;
        if(image_rig.surface_intersects(*incoming,geometry,center,radius))return current;
        ++refine_clear;
      }
      return TubeStatus::ObservedClear;
    };
    std::vector<ClearBall> proof_balls;
    auto covered=[&](Radial a,Radial b,double radius){
      const auto delta=add(b,a,-1.);
      const auto center=add(a,delta,.5);
      radius+=.5*delta.r;
      if(oracle(center,radius)!=TubeStatus::ObservedClear)return false;
      if(!cached_clear(center,radius+.025)){
        const double expanded=radius+.045;
        const double verified=oracle(center,expanded)==TubeStatus::ObservedClear?expanded:radius;
        if(!cached_clear(center,verified))proof_balls.push_back({center,verified});
      }
      return true;
    };
    auto proof=[&](Radial next){
      proof_balls.clear();
      const double speed=next.r;
      const auto endpoint=add(p,next,dt);
      if(!covered(p,endpoint,tube))return false;
      const double stop=speed*speed/(2.*emergency)+speed*dt;
      for(double from=0.;from<stop;){
        const double to=std::min(stop,from+.12);
        if(!covered(add(endpoint,{next.dir,from}),add(endpoint,{next.dir,to}),tube))return false;
        from=to;
      }
      return true;
    };
    auto plant=[&](Radial input){return predicted_velocity(velocity,input);};
    const bool inherited_brake=brake_valid && distance(p,expected_position)<1e-8 &&
      distance(velocity,expected_velocity)<1e-8;
    const bool approaching=final_target&&remaining<4.0;
    const double speed=velocity.r;
    SphericalTick tick{};
    PositionFlowBox::Coordinates accepted_coordinates{};
    std::string_view source="none";
    const bool at_cruise=speed>=speed_limit-FixedReferenceTracker::attachment_speed_tolerance_mps&&!approaching;
    const bool request=at_cruise && speed>=speed_limit-.001 && route.valid;
    enum Check { Idle, Accepted, FrameUnavailable, LookaheadUnobserved, Dynamics, Prefix, Stop };
    Check check=Idle;
    DirectionTrace trace;
    if(directional){
      trace=direction_step(now,p,velocity,yaw,route,request,guide_speed,approaching,remaining,inherited_brake);
      prepare=trace.prepare_ns;stop=trace.certification_ns;
      if(decision==7 || decision==2){source="route";accepted_coordinates=route.coordinates;}
      check=decision==7?Accepted:Idle;
    }else if(request){
      const auto &ob=pipeline.process(*incoming,now,p,velocity,desired_body,has_direction,false);
      prepare=field_now_ns()-t0;
      check=Accepted;
      if(!ob.input_ready)check=FrameUnavailable;
      else {
        const auto observed=(*incoming->raw)(target,tube);
        // An ongoing, already certified turn may carry the remote probe out of
        // view. Continue only from the exact previously predicted state. Every
        // actual preview segment and the full stop still require clear evidence.
        const bool continuing_turn=previous_direct && inherited_brake && old_omega.r>.20;
        if(observed==TubeStatus::PotentialObstacle ||
           (observed!=TubeStatus::ObservedClear && !continuing_turn))check=LookaheadUnobserved;
      }
      CruiseRoute::Motion preview{p,velocity,old_command,old_omega},first{};
      auto preview_track=route;
      double station=progress;
      for(int n=0;n<10 && check==Accepted;++n){
        const auto ta=field_now_ns();
        if(n>0)preview_track=trial_route.tracking(preview.p,station);
        if(!preview_track.valid){check=Dynamics;break;}
        const auto next=trial_route.next(preview,preview_track);
        station+=dt*next.v.r*cosine(next.v.dir,preview_track.tangent)/preview_track.station_scale;
        if(n==0)first=next;
        const bool dynamics=within_input_limits(next.w,preview.w,NavigationConfig::heading_rate_limit_rad_s,
                                                  NavigationConfig::heading_acceleration_limit_rad_s2)
            &&within_input_limits(next.u,preview.u,NavigationConfig::compensation_input_limit_mps,
                                  NavigationConfig::compensation_slew_limit_mps2)
            &&distance(next.v,preview.v)<=80.*dt+1e-8
            &&distance(predicted_velocity(preview.v,next.u),next.v)<1e-8;
        const auto tb=field_now_ns();steering+=tb-ta;
        if(!dynamics)check=Dynamics;
        else if(!covered(preview.p,next.p,tube))check=Prefix;
        curve+=field_now_ns()-tb;
        preview=next;
      }
      const auto ts=field_now_ns();
      if(check==Accepted && !proof(first.v))check=Stop;
      stop=field_now_ns()-ts;
      if(check==Accepted){
        pipeline.interlock();
        old_omega=first.w;old_command=first.u;decision=7;
        accepted_coordinates=route.coordinates;source="route";
      }else{
        proof_balls.clear();
      }
    }
    const bool cruise_attached=decision==7;
    previous_direct=cruise_attached;
    // If direct cruise loses its certificate in a speed-limited bend, decelerate
    // on the same route box instead of attaching a new constant-speed reference.
    if(!directional && !cruise_attached && at_cruise && has_direction && guide_speed>=speed_limit-.001){
      tick=pipeline.rolling_tick(*incoming,now,p,velocity,desired_body,old_command,old_omega,oracle);
      if(tick.mode==RollingMode::Tracking&&proof(tick.next_velocity)){
        old_omega=tick.omega;
        old_command=tick.plant_input;
        decision=1;source="rolling";
        accepted_coordinates.normal_m=tick.flowbox_normal_m;
        accepted_coordinates.binormal_m=tick.flowbox_binormal_m;
      }
    }else if(!directional && !cruise_attached){
      pipeline.reset_epoch();
      pipeline.process(*incoming,now,p,velocity,desired_body,has_direction,group%10==1);
    }

    if(decision==0) {
      pipeline.interlock();
      const auto &plan=pipeline.output();
      const double desired_speed=approaching?
        std::min(guide_speed,std::sqrt(2.*acceleration*std::max(0.,remaining-.22))):guide_speed;
      const double speed_change=std::clamp(desired_speed-speed,-acceleration*dt,acceleration*dt);
      const double nominal_speed=std::max(0.,speed+speed_change);
      const double max_angle=std::min(6.667,80./std::max(1.,speed))*dt;
      const double max_slew=NavigationConfig::compensation_slew_limit_mps2*dt;
      auto try_axis=[&](Bearing axis,double next_speed){
        const Radial next{steer(velocity,axis,max_angle),next_speed};
        const auto requested=add(velocity,add(next,velocity,-1.),1./beta);
        const auto input=limit(add(old_command,limit(add(requested,old_command,-1.),max_slew)),
                               NavigationConfig::compensation_input_limit_mps);
        const auto applied=plant(input);
        if(applied.r>speed_limit+1e-8||!proof(applied))return false;
        old_command=input;old_omega={};
        return true;
      };
      bool accepted=false;
      if(route.valid && nominal_speed>0.){
        const auto desired=route.box.guidance(route.coordinates,velocity,nominal_speed,
                                              NavigationConfig::lateral_gain_per_s,false).desired;
        // A field or search candidate changes the nominal tangent for obstacle
        // avoidance. Transport that change onto the already corrected heading.
        auto offer=[&](spherical::Tangent change){
          const auto axis=spherical::exp(desired,spherical::transport(route.tangent,desired,change));
          return try_axis(axis,nominal_speed);
        };
        if(plan.spatial_ready){
          const auto candidate=pipeline.reference_heading(std::min(.30,plan.turn_length_m));
          const auto change=spherical::log(route.tangent,candidate);
          const double deflection=std::hypot(change.east,change.north);
          if(deflection>1e-9 && deflection<.30)accepted=offer(change);
        }
        if(!accepted)accepted=try_axis(desired,nominal_speed);
        if(!accepted && !direct_only){
          for(const double deg:{12.,-12.,25.,-25.,45.,-45.,70.,-70.,100.,-100.,160.,-160.})
            if(offer({deg*direction_pi/180.,0.})){accepted=true;break;}
          if(!accepted)for(const double deg:{12.,-12.,25.,-25.,40.,-40.})
            if(offer({0.,deg*direction_pi/180.})){accepted=true;break;}
        }
        if(accepted){decision=2;accepted_coordinates=route.coordinates;source="route";}
      }
      if(!accepted && speed>0.){
        accepted=try_axis(velocity.dir,std::max(0.,speed-emergency*dt));
        if(accepted)decision=9;
      }
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
    expected_position=add(p,expected_velocity,dt);
    brake_valid=decision==7||decision==1||decision==2||decision==9||decision==5||
      (decision==3&&expected_velocity.r>0.);
    if(!directional && (decision==7||decision==1||decision==2||decision==9||decision==3)) {
      for(const auto &ball:proof_balls) {
        verified_balls.push_front(ball);
        if(verified_balls.size()>192)verified_balls.pop_back();
      }
    }
    const auto &plan=pipeline.output();
    const auto command_wire=spherical::to_xyz(old_command);
    const auto elapsed=field_now_ns()-t0;
    const auto &depth=incoming->depth;const auto &oi=depth.observations;
    const int camera=std::max_element(trial_cam_wall.begin(),trial_cam_wall.end())-trial_cam_wall.begin();
    log<<group<<','<<decision<<','<<int(tick.reason)<<','<<(tick.mode==RollingMode::Tracking)<<','
       <<plan.field_attempted<<','<<plan.angular_ready<<','<<plan.spatial_ready<<','<<tick.committed<<','
       <<previous.size()<<','<<tick.planning_ns<<','<<tick.certification_ns<<','<<tick.control_ns<<','
       <<incoming->identity.group<<','<<command_wire[0]<<','<<command_wire[1]<<','<<command_wire[2]<<','<<elapsed<<','<<prepare<<','<<steering<<','<<curve<<','<<stop
       <<','<<plan.preprocessing_ns+trace.first_pre_ns<<','<<oi.camera_work_ns<<','<<trial_cam_wall[camera]<<','<<trial_cam_cpu[camera]
       <<','<<oi.projection_ns[camera]<<','<<oi.compression_ns[camera]<<','<<depth.repair_ns[camera]
       <<','<<plan.field_ns<<','<<plan.spatial_ns<<','<<plan.occupancy_ns<<','<<request<<','<<int(check)<<','<<approaching<<','<<refine_attempts<<','<<refine_clear;
    const auto &f=pipeline.field();
    for(auto phase:{f.component_ns,f.coarse.domain_ns,f.fine.domain_ns,f.fine.solve_ns,f.curve_ns})
      log<<','<<(plan.field_attempted?phase:0);
    log<<','<<oi.merge_ns<<','<<depth.local.merge_ns;
    for(unsigned c=0;c<4;++c){
      const bool measured=oi.ready && oi.sequence==depth.sequence && oi.stamp_ns==depth.t_ns;
      log<<','<<(measured?pipeline.builder().task_start_offset_ns(c):0)
         <<','<<(measured?pipeline.builder().task_end_offset_ns(c):0);
    }
    log<<','<<(source!="none")<<','<<accepted_coordinates.normal_m<<','<<accepted_coordinates.binormal_m<<','<<source
       <<','<<directional<<','<<(trace.source!="none")<<','<<(plan.angular_ready?f.curve_size:0)
       <<','<<trace.required_travel<<','<<trace.certified_travel<<','<<trace.source<<','<<trace.request_reason<<'\n';
    retain();
    return command_wire;
  }
};
} // namespace

int main(int argc,char **argv) try {
  if(argc!=10)return 2;
  cpu_set_t allowed;CPU_ZERO(&allowed);for(int i=0;i<5;++i)CPU_SET(i,&allowed);
  if(pthread_setaffinity_np(pthread_self(),sizeof(allowed),&allowed))throw std::runtime_error("CPU0-4 unavailable");
  const Direction3 start{std::stod(argv[7]),std::stod(argv[8]),std::stod(argv[9])};
  std::string diagnosis;
  const bool seed=pc_gvf::depth_angular::certifyDefaultCloudStart(
    argv[2],argv[3],Eigen::Vector3d(start[0],start[1],start[2]),seed_radius,&diagnosis);
  if(!seed)throw std::runtime_error(diagnosis);
  uint32_t w,h;
  const auto cal=navigation_calibration(argv[1],w,h);
  const auto version=navigation_calibration_version(cal,w,h);
  const auto start_polar=spherical::from_xyz(start[0],start[1],start[2]);
  const std::string mode(argv[5]);
  const bool angular_only=mode=="directional-route-station";
  const bool station_packet=angular_only || mode=="search-route-station" || mode=="direct-only-route-station";
  const bool radial_intent=station_packet || mode=="search-spherical" || mode=="direct-only-spherical";
  if(!radial_intent && mode!="search" && mode!="direct-only")throw std::runtime_error("invalid intent mode");
  Flight flight(cal,w,h,version,start_polar,seed,mode.rfind("direct-only",0)==0,argv[4]);
  flight.directional=angular_only;
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
    if(!std::all_of(msg.begin(),msg.end(),[](double value){return std::isfinite(value);}) ||
       msg[11]<0. || msg[11]>speed_limit)
      throw std::runtime_error("invalid plant state or guidance");
    // Convert physical telemetry on receipt. Older playbacks also supply a
    // Cartesian goal. The current guide supplies its bearing and range.
    const auto position=spherical::from_xyz(msg[0],msg[1],msg[2]);
    const auto velocity=spherical::from_xyz(msg[3],msg[4],msg[5]);
    const auto target=radial_intent ? read_radial(msg[7],msg[8],msg[9]) :
                                     spherical::from_xyz(msg[7],msg[8],msg[9]);
    BodyState actual;
    actual.t_ns=int64_t(frame++)*BaseConfig::period_ns;
    actual.p_W={msg[0],msg[1],msg[2]};
    const double c=std::cos(msg[6]),s=std::sin(msg[6]);
    actual.R_WB={c,-s,0.,s,c,0.,0.,0.,1.};
    actual.v_B={c*msg[3]+s*msg[4],-s*msg[3]+c*msg[4],msg[5]};
    if(frame==1 && distance(position,start_polar)>1e-8)throw std::runtime_error("startup pose differs from seed");
    std::optional<double> station;
    if(station_packet){
      double s;
      if(std::fread(&s,sizeof(s),1,stdin)!=1 || !std::isfinite(s) || s<0. || s>flight.trial_route.length()+1e-7)
        throw std::runtime_error("invalid route station");
      station=std::min(s,flight.trial_route.length());
    }
    if(!flight.read(version,actual))throw std::runtime_error("truncated depth group");
    // An accumulated yaw can be many turns. Take the principal angle of
    // the same attitude used by the sensor before subtracting a bearing.
    const auto input=flight.step(actual.t_ns,position,velocity,std::atan2(s,c),target,msg[10]!=0.,msg[11],station);
    const std::array<double,4> output{input[0],input[1],input[2],double(flight.decision)};
    if(std::fwrite(output.data(),sizeof(double),output.size(),stdout)!=output.size())break;
    std::fflush(stdout);
  }
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
