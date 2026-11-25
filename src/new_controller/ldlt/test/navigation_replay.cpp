#ifndef EGO4P2_MEASURE_KIND
#define EGO4P2_MEASURE_KIND 1
#endif
#include "new_controller/depth_observations.hpp"
#include "new_controller/harmonic_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
#include "rolling_cases.hpp"
#if EGO4P2_MEASURE_KIND
#include "new_controller/navigation_io.hpp"
#endif
using namespace ego4p0;namespace fs=std::filesystem;
int main(int argc,char**argv){try{
  if(argc!=6&&argc!=7)throw std::runtime_error("navigation_replay ARCHIVE NEW_OUTPUT FIXTURE REPEATS fifo5|normal [--spatial|--fixed-tracking|--rolling]");
  const fs::path archive=argv[1],out=argv[2];const int repeats=std::stoi(argv[4]);const bool fifo=std::string(argv[5])=="fifo5";
  const bool tracking=argc==7&&std::string(argv[6])=="--fixed-tracking";
  const bool rolling=argc==7&&std::string(argv[6])=="--rolling";
  const bool spatial=tracking||rolling||(argc==7&&std::string(argv[6])=="--spatial");
  if((argc==7&&!spatial)||repeats<1||repeats>1000||(!fifo&&std::string(argv[5])!="normal")||!fs::create_directory(out))throw std::runtime_error("bad replay scope");
  uint32_t w,h;
  const auto cal=navigation_calibration(archive/"simulation.sensors.json",w,h);
  const auto fixture=YAML::LoadFile(argv[3]);Direction3 anchor;for(int i=0;i<3;++i)anchor[i]=fixture["q0"][i].as<double>();if(!unit_direction(anchor))throw std::runtime_error("invalid explicit fixture");
  static_assert(EGO4P2_MEASURE_KIND==1,"Replay requires the unified Step2 input");
  NavigationPipeline pipeline;pipeline.configure(cal,w,h,1);NavigationObservation packet;pipeline.allocate(packet);auto&frame=packet.depth;auto&builder=pipeline.builder();
  std::ifstream raw(archive/"last_depth.bin",std::ios::binary);for(auto&d:frame.D)raw.read(reinterpret_cast<char*>(d.data()),d.size()*sizeof(float));if(!raw)throw std::runtime_error("archive input");const auto real=frame.D;
  if(fifo)builder.enable_fifo5_workers();
  cpu_set_t cpu;CPU_ZERO(&cpu);CPU_SET(0,&cpu);if(pthread_setaffinity_np(pthread_self(),sizeof(cpu),&cpu))throw std::runtime_error("coordinator affinity");
  if(fifo){sched_param p{};p.sched_priority=5;if(pthread_setschedparam(pthread_self(),SCHED_FIFO,&p))throw std::runtime_error("FIFO permission required");}
  int policy=0;sched_param p{};pthread_getschedparam(pthread_self(),&policy,&p);builder.refresh_worker_schedulers();
  if(policy!=(fifo?SCHED_FIFO:SCHED_OTHER)||p.sched_priority!=(fifo?5:0))throw std::runtime_error("scheduler readback");
  std::ofstream meta(out/"preprocess_execution.json");meta<<"{\"coordinator_scheduler\":"<<policy<<",\"coordinator_priority\":"<<p.sched_priority<<",\"coordinator_cpu\":0,\"worker_cpus\":[";
  auto arr=[&](const auto&a){for(int i=0;i<4;++i){if(i)meta<<',';meta<<a[i];}};arr(builder.worker_cpus());meta<<"],\"worker_policies\":[";arr(builder.worker_policies());meta<<"],\"worker_priorities\":[";arr(builder.worker_priorities());meta<<"],\"worker_affinity_errors\":[";arr(builder.worker_affinity_errors());meta<<"],\"worker_scheduler_errors\":[";arr(builder.worker_scheduler_errors());meta<<"],\"period_ns\":5000000,\"variant\":"<<(spatial?2:1)<<",\"omega_radius_m\":"<<ControllerLimits::radius_m<<",\"secondary_summary_in_camera_threads\":"<<(EGO4P2_MEASURE_KIND?"true":"false")<<",\"first_block_edge\":4,\"second_block_edge\":3,\"coarse_solver\":\"eigen_ldlt\",\"fine_solver\":\"eigen_ldlt\",\"synthetic_state_on_static_archive\":true,\"motion_output_enabled\":false,\"spatial_reference_enabled\":"<<(spatial?"true":"false")<<",\"continuous_tube_check_enabled\":"<<(spatial?"true":"false")<<",\"spatial_ramp_time_s\":"<<SpatialReferenceConfig::ramp_time_s
    <<",\"spatial_ramp_length_m\":"<<SpatialReferenceConfig::ramp_length_m
    <<",\"spatial_grid_m\":"<<SpatialReferenceConfig::lookup_step_m
    <<",\"fixed_tracking_fixture\":"<<(tracking?"true":"false")
    <<",\"rolling_fixture\":"<<(rolling?"true":"false")<<"}\n";meta.close();
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
#if EGO4P2_MEASURE_KIND
  std::ofstream(out/"navigation_config.json")<<NavigationConfig::json<<'\n';
#endif
  if(rolling){
    rolling_fixture::run(pipeline,packet,out/"rolling.csv",repeats,fifo);
    return 0;
  }
  if(tracking){
    tracking_fixture::run(out/"tracking.csv",repeats,fifo);
    return 0;
  }
  std::ofstream times(out/"timings.csv");times<<std::setprecision(12)<<"case,mode,iteration,scheduled_ns,start_ns,finish_ns,compute_ms,response_ms,lateness_ms,skipped_releases,preprocess_ms,capture_ms,field_ms,coarse_ms,fine_ms,curve_ms,coarse_nodes,fine_nodes,angular_ready,invalid,field_reused,field_version,curve_version,reference_version,q0x,q0y,q0z,goalx,goaly,goalz,coarse_residual,fine_residual,angular_length,exit_clipped,spatial_ms,occupancy_ms,spatial_ready,spatial_reused,occupancy_reused,geometry_version,turn_length_m,checked_horizon_m,observed_sections,unknown_sections,obstacle_sections,outside_sections"<<','<<navigation_detail_header()<<",minor_faults,voluntary_switches,involuntary_switches\n";
  uint64_t group=0;
  for(const std::string name:{"archive_turn","archive_straight","open90","open180","open3d","all_unknown"}){
    frame.D=real;if(name.rfind("open",0)==0)for(auto&d:frame.D)std::fill(d.begin(),d.end(),10.f);if(name=="all_unknown")for(auto&d:frame.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());
    Direction3 q=name.rfind("open",0)==0?Direction3{1,0,0}:anchor;
    Direction3 goal=name=="archive_straight"?q:name=="open90"?sphere_point(direction_pi/2,0):name=="open180"?sphere_point(direction_pi,0):name=="open3d"?sphere_point(direction_pi/4,direction_pi/9):sphere_point(20*direction_pi/180,0);
    for(const std::string mode:{"cold","new_group_reuse","intent_change"}){
      int64_t release=field_now_ns()+BaseConfig::period_ns;
      for(int i=-10;i<repeats;++i){timespec deadline{release/1000000000LL,release%1000000000LL};int e;do{e=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr);}while(e==EINTR);if(e)throw std::runtime_error("periodic release failed");
        rusage resources_before{};
        if(getrusage(RUSAGE_THREAD,&resources_before))throw std::runtime_error("thread resource counters unavailable");
        const auto begin=field_now_ns();
        if(mode!="intent_change"||i==-10){++group;frame.sequence=group;frame.t_ns=int64_t(group)*BaseConfig::period_ns;}
        auto current_goal=goal;if(mode=="intent_change"&&i%2){const double c=std::cos(.25*direction_pi/180),s=std::sin(.25*direction_pi/180);current_goal={c*goal[0]-s*goal[1],s*goal[0]+c*goal[1],goal[2]};}
        DesiredMotion desired{current_goal,12,true,true};double pre=0,capture=0,field_ms=0;bool angular=false;const char*invalid="none";uint64_t reference_version=0;
#if EGO4P2_MEASURE_KIND
        packet.identity={group,1,group,frame.t_ns,{frame.t_ns,frame.t_ns,frame.t_ns,frame.t_ns},15};packet.observation_pose={};packet.observation_pose.t_ns=frame.t_ns;packet.observation_pose_valid=true;
        NavigationState state;state.actual_valid=true;state.actual.t_ns=frame.t_ns;state.current_simulation_ns=frame.t_ns;for(int k=0;k<3;++k)state.actual.v_B[k]=12*q[k];
        const auto&o=pipeline.process(packet,state,desired,mode=="cold",spatial);const auto&r=pipeline.field();pre=o.preprocessing_ns/1e6;capture=o.capture_ns/1e6;field_ms=o.field_ns/1e6;angular=o.angular_ready;invalid=navigation_invalid_name(o.invalid);reference_version=o.reference_version;
#endif
        const auto finish=field_now_ns();
        rusage resources_after{};
        if(getrusage(RUSAGE_THREAD,&resources_after))throw std::runtime_error("thread resource counters unavailable");
        const auto skipped=std::max<int64_t>(0,(finish-release)/BaseConfig::period_ns);
        if(i>=0){times<<name<<','<<mode<<','<<i<<','<<release<<','<<begin<<','<<finish<<','<<(finish-begin)/1e6<<','<<(finish-release)/1e6<<','<<(begin-release)/1e6<<','<<skipped<<','<<pre<<','<<capture<<','<<field_ms<<','<<r.coarse.solve_ns/1e6<<','<<r.fine.solve_ns/1e6<<','<<r.curve_ns/1e6<<','<<r.coarse.nodes<<','<<r.fine.nodes<<','<<angular<<','<<invalid<<','<<r.reused<<','<<r.field_version<<','<<r.curve_version<<','<<reference_version;for(double x:q)times<<','<<x;for(double x:current_goal)times<<','<<x;times<<','<<r.coarse.residual<<','<<r.fine.residual<<','<<r.angular_length<<','<<r.exit_clipped<<','<<o.spatial_ns/1e6<<','<<o.occupancy_ns/1e6
          <<','<<o.spatial_ready<<','<<o.spatial_reused<<','<<o.occupancy_reused<<','<<o.geometry_version
          <<','<<o.turn_length_m<<','<<o.checked_horizon_m<<','<<o.observed_sections
          <<','<<o.unknown_sections<<','<<o.obstacle_sections<<','<<o.outside_sections;
          navigation_detail_csv(times,pipeline);
          times<<','<<resources_after.ru_minflt-resources_before.ru_minflt
               <<','<<resources_after.ru_nvcsw-resources_before.ru_nvcsw
               <<','<<resources_after.ru_nivcsw-resources_before.ru_nivcsw<<'\n';}
        release+=(skipped+1)*BaseConfig::period_ns;
      }
    }
#if EGO4P2_MEASURE_KIND
    if(repeats<=2)navigation_export(out/name,pipeline);
#endif
  }
  if(!times)throw std::runtime_error("timing export failed");return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
