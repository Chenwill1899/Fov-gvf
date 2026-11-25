// Shared driver for the frozen 3-D implementation and its intrinsic-coordinate
// replacement. Inputs, world-space outputs and scheduling are identical.
#include "new_controller/navigation_io.hpp"
#include <iostream>
#include <sys/resource.h>

using namespace ego4p0;
namespace fs=std::filesystem;

int main(int argc,char **argv){try{
  if(argc!=7)throw std::invalid_argument("intrinsic_2d_benchmark ARCHIVE NEW_OUTPUT FIXTURE REPEATS fifo5|correctness BASELINE|CURRENT");
  const fs::path archive=argv[1],out=argv[2];
  const int repeats=std::stoi(argv[4]);
  const std::string mode=argv[5],variant=argv[6];
  const bool fifo=mode=="fifo5";
  if((!fifo&&mode!="correctness")||repeats<1||repeats>1000||(!fifo&&repeats>2)||
     (variant!="BASELINE"&&variant!="CURRENT")||!fs::create_directory(out))
    throw std::invalid_argument("invalid benchmark scope or existing output");
  uint32_t width,height;
  const auto calibration=navigation_calibration(archive/"simulation.sensors.json",width,height);
  const auto fixture=YAML::LoadFile(argv[3]);
  Direction3 anchor{};
  for(int i=0;i<3;++i)anchor[i]=fixture["q0"][i].as<double>();
  if(!unit_direction(anchor))throw std::invalid_argument("fixture requires unit 3-D q0");
  NavigationPipeline pipeline;
  pipeline.configure(calibration,width,height,1);
  NavigationObservation observation;
  pipeline.allocate(observation);
  navigation_read_depth(archive/"last_depth.bin",observation);
  const auto raw=observation.depth.D;
  if(fifo)pipeline.enable_fifo5();
  else {
    cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(0,&mask);
    if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("coordinator affinity failed");
  }
  int policy=0;sched_param priority{};
  if(pthread_getschedparam(pthread_self(),&policy,&priority)||policy!=(fifo?SCHED_FIFO:SCHED_OTHER)||priority.sched_priority!=(fifo?5:0))throw std::runtime_error("coordinator scheduler mismatch");
  pipeline.builder().refresh_worker_schedulers();
  auto array=[](std::ostream &s,const auto &a){s<<'[';for(size_t i=0;i<a.size();++i){if(i)s<<',';s<<a[i];}s<<']';};
  std::ofstream metadata(out/"execution_metadata.json");
  metadata<<"{\"variant\":\""<<variant<<"\",\"performance_run\":"<<(fifo?"true":"false")
          <<",\"world_dimensions\":3,\"coordinator_policy\":"<<policy<<",\"coordinator_priority\":"<<priority.sched_priority
          <<",\"coordinator_cpu\":0,\"worker_policies\":";array(metadata,pipeline.builder().worker_policies());
  metadata<<",\"worker_priorities\":";array(metadata,pipeline.builder().worker_priorities());
  metadata<<",\"worker_cpus\":";array(metadata,pipeline.builder().worker_cpus());
  metadata<<",\"worker_affinity_errors\":";array(metadata,pipeline.builder().worker_affinity_errors());
  metadata<<",\"worker_scheduler_errors\":";array(metadata,pipeline.builder().worker_scheduler_errors());
  metadata<<",\"period_ns\":"<<BaseConfig::period_ns<<",\"motion_output_enabled\":false,\"simulation_started\":false}\n";
  metadata.close();
  std::ofstream(out/"configuration.json")<<NavigationConfig::json<<'\n';
  std::ofstream csv(out/"timings.csv");
  csv<<std::setprecision(17)<<"case,mode,iteration,compute_ms,response_ms,lateness_ms,skipped_releases,preprocess_ms,capture_ms,field_ms,spatial_ms,occupancy_ms,angular_ready,spatial_ready,invalid,turn_length_m,checked_horizon_m,observed_sections,unknown_sections,obstacle_sections,outside_sections,q0x,q0y,q0z,goalx,goaly,goalz,field_reused,field_version,curve_version,reference_version,geometry_version,spatial_reused,occupancy_reused,coarse_nodes,fine_nodes,coarse_residual,fine_residual,angular_length,exit_clipped,curve_size";
  for(const char *point:{"start","middle","end"})for(const char *quantity:{"px","py","pz","tx","ty","tz","kx","ky","kz","kappa"})csv<<','<<point<<'_'<<quantity;
  csv<<'\n';
  uint64_t group=0;
  // Run anchor translation last so its intentionally changed cache counters
  // cannot leak into the otherwise identical version-counter comparison.
  for(const std::string scenario:{"cold","new_group_reuse","same_group","intent_change","anchor_translation"}){
   for(const std::string name:{"archive_turn","archive_straight","open90","open180","open3d","all_unknown"}){
    observation.depth.D=raw;
    if(name.rfind("open",0)==0)for(auto&d:observation.depth.D)std::fill(d.begin(),d.end(),10.f);
    if(name=="all_unknown")for(auto&d:observation.depth.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());
    Direction3 start=name.rfind("open",0)==0?Direction3{1,0,0}:anchor;
    const Direction3 goal=name=="archive_straight"?start:name=="open90"?sphere_point(direction_pi/2,0):name=="open180"?sphere_point(direction_pi,0):name=="open3d"?sphere_point(direction_pi/4,direction_pi/9):sphere_point(20*direction_pi/180,0);
      int64_t release=field_now_ns()+BaseConfig::period_ns;
      for(int index=-10;index<repeats;++index){
        if(fifo){timespec deadline{release/1000000000LL,release%1000000000LL};int error;do{error=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr);}while(error==EINTR);if(error)throw std::runtime_error("release timer failed");}
        const auto begin=field_now_ns();
        if(scenario=="cold"||scenario=="new_group_reuse"||index==-10)navigation_identity(observation,++group);
        auto current=goal;
        if(scenario=="intent_change"&&index%2){const double c=std::cos(.25*direction_pi/180),s=std::sin(.25*direction_pi/180);current={c*goal[0]-s*goal[1],s*goal[0]+c*goal[1],goal[2]};}
        NavigationState state;state.actual_valid=true;state.actual.t_ns=observation.depth.t_ns;state.current_simulation_ns=observation.depth.t_ns;
        if(scenario=="anchor_translation")state.actual.p_W={.002*(index+10),0.,0.};
        for(int k=0;k<3;++k)state.actual.v_B[k]=12*start[k];
        const auto &result=pipeline.process(observation,state,DesiredMotion{current,12,true,true},scenario=="cold",true);
        const auto finish=field_now_ns();
        const auto skipped=fifo?std::max<int64_t>(0,(finish-release)/BaseConfig::period_ns):0;
        if(index>=0){
          csv<<name<<','<<scenario<<','<<index<<','<<(finish-begin)/1e6<<','<<(fifo?finish-release:finish-begin)/1e6<<','<<(fifo?begin-release:0)/1e6<<','<<skipped
             <<','<<result.preprocessing_ns/1e6<<','<<result.capture_ns/1e6<<','<<result.field_ns/1e6<<','<<result.spatial_ns/1e6<<','<<result.occupancy_ns/1e6
             <<','<<result.angular_ready<<','<<result.spatial_ready<<','<<navigation_invalid_name(result.invalid)<<','<<result.turn_length_m<<','<<result.checked_horizon_m
             <<','<<result.observed_sections<<','<<result.unknown_sections<<','<<result.obstacle_sections<<','<<result.outside_sections;
          for(double value:start)csv<<','<<value;for(double value:current)csv<<','<<value;
          const auto &field=pipeline.field();
          csv<<','<<field.reused<<','<<field.field_version<<','<<field.curve_version<<','<<result.reference_version<<','<<result.geometry_version
             <<','<<result.spatial_reused<<','<<result.occupancy_reused<<','<<field.coarse.nodes<<','<<field.fine.nodes
             <<','<<field.coarse.residual<<','<<field.fine.residual<<','<<field.angular_length<<','<<field.exit_clipped<<','<<field.curve_size;
          for(double sample_s:{0.,result.checked_horizon_m*.5,result.checked_horizon_m}){
            SpatialSample sample{};
            if(result.spatial_ready)sample=pipeline.reference().query(sample_s);
            for(const auto &vector:{sample.position_W,sample.tangent_W,sample.curvature_W})for(double value:vector)csv<<','<<value;
            csv<<','<<sample.kappa;
          }
          csv<<'\n';
        }
        release+=(skipped+1)*BaseConfig::period_ns;
      }
      if(!fifo)navigation_export(out/(name+"_"+scenario),pipeline);
    }
  }
  if(!csv)throw std::runtime_error("measurement output failed");
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 2;}}
