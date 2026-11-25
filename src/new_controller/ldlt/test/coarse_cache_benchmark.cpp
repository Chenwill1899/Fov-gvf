// Same driver for validated local-fine planning and its exact coarse graph cache.
// Both variants retain 3-D input/output and identical CPU/wall clock scopes.
#include "new_controller/navigation_io.hpp"
#include <iostream>
#include <sys/resource.h>

using namespace ego4p0;
namespace fs=std::filesystem;

static void profile_header(std::ostream &csv){
  csv<<",scheduled_ns,begin_ns,end_ns,next_release_ns,cycle_thread_cpu_ns,pipeline_thread_cpu_ns,field_thread_cpu_ns,minor_faults,major_faults,voluntary_switches,involuntary_switches,coarse_topology_only,coarse_topology_ns,coarse_topology_failure,coarse_graph_reused,coarse_graph_cache_ns";
  for(const char *name:{"coarse","fine"}){
    for(const char *metric:{"status","iterations","reused","domain_ns","solve_ns","solve_cpu_ns","assembly_ns","iteration_ns","cache_ns","residual_ns","store_ns"})csv<<','<<name<<'_'<<metric;
    for(const char *metric:{"validation_ns","conversion_ns","import_ns","analysis_ns","factor_ns","solve_ns","verify_ns","total_ns","iterations","unknowns","nonzeros","factor_nonzeros","symbolic_reused","numeric_reused","ok","residual"})csv<<','<<name<<"_backend_"<<metric;
  }
}

static void profile_data(std::ostream &csv,const NavigationPipeline &pipeline,
                         int64_t release,int64_t begin,int64_t end,int64_t next,
                         int64_t cpu,const rusage &before,const rusage &after){
  const auto &field=pipeline.field();
  csv<<','<<release<<','<<begin<<','<<end<<','<<next<<','<<cpu
     <<','<<pipeline.output().coordinator_cpu_ns<<','<<field.cpu_ns
     <<','<<after.ru_minflt-before.ru_minflt<<','<<after.ru_majflt-before.ru_majflt
     <<','<<after.ru_nvcsw-before.ru_nvcsw<<','<<after.ru_nivcsw-before.ru_nivcsw;
  csv<<','<<field.coarse_topology_only<<','<<field.coarse_topology_ns<<','<<field.coarse_topology_failure;
#ifdef EGO4P3_COARSE_CACHE_BASELINE
  csv<<",0,0";
#else
  csv<<','<<field.coarse_graph_reused<<','<<field.coarse_graph_cache_ns;
#endif
  for(const auto *layer:{&field.coarse,&field.fine}){
    csv<<','<<field_status_name(layer->status)<<','<<layer->iterations<<','<<layer->reused
       <<','<<layer->domain_ns<<','<<layer->solve_ns<<','<<layer->solve_cpu_ns
       <<','<<layer->assembly_ns<<','<<layer->iteration_ns<<','<<layer->cache_ns<<','<<layer->residual_ns<<','<<layer->store_ns;
    const auto &backend=layer==&field.coarse?pipeline.planner().coarse_trial_stats():pipeline.planner().fine_trial_stats();
    for(int64_t value:{backend.validation_ns,backend.conversion_ns,backend.import_ns,backend.analysis_ns,backend.factor_ns,backend.solve_ns,backend.verify_ns,backend.total_ns})csv<<','<<value;
    csv<<','<<backend.iterations<<','<<backend.unknowns<<','<<backend.nonzeros<<','<<backend.factor_nonzeros
       <<','<<backend.symbolic_reused<<','<<backend.numeric_reused<<','<<backend.ok<<','<<backend.residual;
  }
}

int main(int argc,char **argv){try{
  if(argc!=8)throw std::invalid_argument("coarse_cache_benchmark ARCHIVE NEW_OUTPUT FIXTURE REPEATS fifo5|correctness BASELINE|CURRENT archive|synthetic_known_near");
  const fs::path archive=argv[1],out=argv[2];
  const int repeats=std::stoi(argv[4]);
  const std::string mode=argv[5],variant=argv[6],dataset=argv[7];
  const bool synthetic=dataset=="synthetic_known_near";
  const bool fifo=mode=="fifo5";
  if((!fifo&&mode!="correctness")||repeats<1||
     (variant!="BASELINE"&&variant!="CURRENT")||(!synthetic&&dataset!="archive")||!fs::create_directory(out))
    throw std::invalid_argument("invalid benchmark scope or existing output");
  uint32_t width,height;
  std::array<CameraCalibration,4> calibration;
  if(synthetic){
    width=320;height=240;
    for(unsigned camera=0;camera<4;++camera){
      const double yaw=camera*direction_pi/2,c=std::cos(yaw),s=std::sin(yaw);
      calibration[camera].K={192.,0.,160.,0.,208.,120.,0.,0.,1.};
      calibration[camera].T_BC={s,0.,c,.2*c,-c,0.,s,.2*s,0.,-1.,0.,.02,0.,0.,0.,1.};
      calibration[camera].near_clip_z_m=.65;
      calibration[camera].frame="synthetic_camera_"+std::to_string(camera);
    }
    std::ofstream(out/"synthetic_fixture.json")<<"{\"source\":\"explicit synthetic camera model; not measured camera calibration\",\"positive_evidence_scope\":\"observed-clear tube sections; full-tube acceptance is not presumed\",\"width\":320,\"height\":240,\"fx\":192,\"fy\":208,\"cx\":160,\"cy\":120,\"camera_yaws_rad\":[0,1.5707963267948966,3.141592653589793,4.71238898038469],\"radial_camera_offset_m\":0.2,\"vertical_camera_offset_m\":0.02,\"near_clip_z_m\":0.65,\"raw_optical_depth_fill_m\":10}\n";
  }else calibration=navigation_calibration(archive/"simulation.sensors.json",width,height);
  const auto fixture=YAML::LoadFile(argv[3]);
  Direction3 anchor{};
  for(int i=0;i<3;++i)anchor[i]=fixture["q0"][i].as<double>();
  if(!unit_direction(anchor))throw std::invalid_argument("fixture requires unit 3-D q0");
  NavigationPipeline pipeline;
  pipeline.configure(calibration,width,height,1);
  NavigationObservation observation;
  pipeline.allocate(observation);
  if(synthetic)for(auto &camera:observation.depth.D)std::fill(camera.begin(),camera.end(),10.f);
  else navigation_read_depth(archive/"last_depth.bin",observation);
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
  metadata<<"{\"variant\":\""<<variant<<"\",\"dataset\":\""<<dataset<<"\",\"explicit_synthetic_calibration\":"<<(synthetic?"true":"false")<<",\"performance_run\":"<<(fifo?"true":"false")
          <<",\"world_dimensions\":3,\"coordinator_policy\":"<<policy<<",\"coordinator_priority\":"<<priority.sched_priority
          <<",\"coordinator_cpu\":0,\"worker_policies\":";array(metadata,pipeline.builder().worker_policies());
  metadata<<",\"worker_priorities\":";array(metadata,pipeline.builder().worker_priorities());
  metadata<<",\"worker_cpus\":";array(metadata,pipeline.builder().worker_cpus());
  metadata<<",\"worker_affinity_errors\":";array(metadata,pipeline.builder().worker_affinity_errors());
  metadata<<",\"worker_scheduler_errors\":";array(metadata,pipeline.builder().worker_scheduler_errors());
  metadata<<",\"period_ns\":"<<BaseConfig::period_ns<<",\"motion_output_enabled\":false,\"simulation_started\":false,\"absolute_timestamps\":true,\"clock\":\"CLOCK_MONOTONIC\",\"cpu_clock\":\"CLOCK_THREAD_CPUTIME_ID\",\"warmups_per_case_mode\":10,\"main_cpu_excludes_camera_workers\":true}\n";
  metadata.close();
  std::ofstream(out/"configuration.json")<<NavigationConfig::json<<'\n';
  std::ofstream csv(out/"timings.csv");
  csv<<std::setprecision(17)<<"case,mode,iteration,compute_ms,response_ms,lateness_ms,skipped_releases,preprocess_ms,capture_ms,field_ms,spatial_ms,occupancy_ms,angular_ready,spatial_ready,invalid,turn_length_m,checked_horizon_m,observed_sections,unknown_sections,obstacle_sections,outside_sections,q0x,q0y,q0z,goalx,goaly,goalz,field_reused,field_version,curve_version,reference_version,geometry_version,spatial_reused,occupancy_reused,coarse_nodes,fine_nodes,coarse_residual,fine_residual,angular_length,exit_clipped,curve_size";
  for(const char *point:{"start","middle","end"})for(const char *quantity:{"px","py","pz","tx","ty","tz","kx","ky","kz","kappa"})csv<<','<<point<<'_'<<quantity;
  profile_header(csv);
  csv<<'\n';
  uint64_t group=0;
  // Keep the prior workload order. Both variants already include translation
  // cache reuse; this comparison grants no translation-specific exceptions.
  const std::vector<std::string> cases=synthetic?std::vector<std::string>{"synthetic_known_near_straight","synthetic_known_near_3d"}:std::vector<std::string>{"archive_turn","archive_straight","open90","open180","open3d","all_unknown"};
  for(const std::string scenario:{"cold","new_group_reuse","same_group","intent_change","anchor_translation"}){
   for(const std::string name:cases){
    observation.depth.D=raw;
    if(name.rfind("open",0)==0)for(auto&d:observation.depth.D)std::fill(d.begin(),d.end(),10.f);
    if(name=="all_unknown")for(auto&d:observation.depth.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());
    Direction3 start=synthetic||name.rfind("open",0)==0?Direction3{1,0,0}:anchor;
    if(name=="synthetic_known_near_3d")start=sphere_point(2*direction_pi/180,0);
    const Direction3 goal=name=="archive_straight"||name=="synthetic_known_near_straight"?start:name=="synthetic_known_near_3d"?sphere_point(12*direction_pi/180,5*direction_pi/180):name=="open90"?sphere_point(direction_pi/2,0):name=="open180"?sphere_point(direction_pi,0):name=="open3d"?sphere_point(direction_pi/4,direction_pi/9):sphere_point(20*direction_pi/180,0);
      int64_t release=field_now_ns()+BaseConfig::period_ns;
      for(int index=-10;index<repeats;++index){
        if(fifo){timespec deadline{release/1000000000LL,release%1000000000LL};int error;do{error=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr);}while(error==EINTR);if(error)throw std::runtime_error("release timer failed");}
        rusage resources_before{};
        if(getrusage(RUSAGE_THREAD,&resources_before))throw std::runtime_error("thread resource counters unavailable");
        const auto begin=field_now_ns();
        const auto cpu_begin=field_thread_cpu_ns();
        if(scenario=="cold"||scenario=="new_group_reuse"||index==-10)navigation_identity(observation,++group);
        auto current=goal;
        if(scenario=="intent_change"&&index%2){const double c=std::cos(.25*direction_pi/180),s=std::sin(.25*direction_pi/180);current={c*goal[0]-s*goal[1],s*goal[0]+c*goal[1],goal[2]};}
        NavigationState state;state.actual_valid=true;state.actual.t_ns=observation.depth.t_ns;state.current_simulation_ns=observation.depth.t_ns;
        if(scenario=="anchor_translation")state.actual.p_W={.002*(index+10),0.,0.};
        for(int k=0;k<3;++k)state.actual.v_B[k]=12*start[k];
        const auto &result=pipeline.process(observation,state,DesiredMotion{current,12,true,true},scenario=="cold",true);
        const auto cpu_end=field_thread_cpu_ns();
        const auto finish=field_now_ns();
        rusage resources_after{};
        if(getrusage(RUSAGE_THREAD,&resources_after))throw std::runtime_error("thread resource counters unavailable");
        const auto skipped=fifo?std::max<int64_t>(0,(finish-release)/BaseConfig::period_ns):0;
        const auto next_release=release+(skipped+1)*BaseConfig::period_ns;
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
          profile_data(csv,pipeline,fifo?release:0,begin,finish,fifo?next_release:0,
                       cpu_end-cpu_begin,resources_before,resources_after);
          csv<<'\n';
        }
        release=next_release;
      }
      if(!fifo)navigation_export(out/(name+"_"+scenario),pipeline);
    }
  }
  if(!csv)throw std::runtime_error("measurement output failed");
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 2;}}
