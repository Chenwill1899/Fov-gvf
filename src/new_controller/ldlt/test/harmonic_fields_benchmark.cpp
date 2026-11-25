#include "new_controller/harmonic_io.hpp"
#include "new_controller/depth_observations.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>
#include <time.h>
using namespace ego4p0;namespace fs=std::filesystem;
void save_block_levels(const fs::path &dir,const DepthFrame &f,const LocalGeometryTables &geometry,const HarmonicPlanner &planner) {
  const auto &idx=f.observations;const auto &table=geometry.virtual_tables();const auto &h=planner.image_hierarchy();
  auto binary=[&](const std::string &name,const auto &v){std::ofstream file(dir/name,std::ios::binary);file.write(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(typename std::decay_t<decltype(v)>::value_type));if(!file)throw std::runtime_error("block evidence write failed");};
  binary("sample_panorama_rho.bin",f.local.min_range);binary("sample_panorama_camera_mask.bin",f.local.observation_cameras);binary("sample_panorama_complete_mask.bin",f.local.complete_cameras);binary("sample_local_excluded_camera_mask.bin",f.local.excluded_camera_mask);binary("sample_panorama_state.bin",f.local.node_state);
  std::ofstream level1(dir/"sample_block_observations.csv"),mins(dir/"sample_virtual_blocks.csv");level1<<std::setprecision(17)<<"camera,block_id,u0,v0,u1,v1,center_u,center_v,valid_pixels,total_pixels,mean_z,stamp_ns,sequence\n";mins<<std::setprecision(17)<<"camera,block_id,min_rho,raw_min_rho,valid_pixels,raw_valid_pixels,total_pixels\n";
  std::ofstream block_cells(dir/"sample_virtual_block_cells.bin",std::ios::binary);
  for(uint32_t c=0;c<4;++c)for(uint32_t id=0;id<idx.blocks_per_camera;++id){const auto &b=idx.blocks[c][id];const auto u=id%idx.block_columns*idx.block_edge,v=id/idx.block_columns*idx.block_edge,u1=std::min(u+idx.block_edge,idx.width),v1=std::min(v+idx.block_edge,idx.height);level1<<c<<','<<id<<','<<u<<','<<v<<','<<u1<<','<<v1<<','<<(u+u1-1)*.5<<','<<(v+v1-1)*.5<<','<<b.valid_pixels<<','<<(u1-u)*(v1-v)<<','<<b.mean_z<<','<<f.t_ns<<','<<f.sequence<<'\n';mins<<c<<','<<id<<','<<b.min_rho<<','<<b.raw_min_rho<<','<<b.valid_pixels<<','<<b.raw_valid_pixels<<','<<(u1-u)*(v1-v)<<'\n';const auto cell=table.block_cell(c,id);block_cells.write(reinterpret_cast<const char*>(&cell),4);}
  std::ofstream first_sources(dir/"sample_virtual_node_blocks.bin",std::ios::binary),second_sources(dir/"coarse_node_blocks.bin",std::ios::binary);
  for(uint32_t cell=0;cell<idx.layout.cells();++cell){const auto &a=table.node(cell);first_sources.write(reinterpret_cast<const char*>(a.blocks.data()),16);}
  for(uint32_t cell=0;cell<planner.coarse_stencil().layout.cells();++cell){const auto &a=planner.coarse_image_sources().node(cell);second_sources.write(reinterpret_cast<const char*>(a.blocks.data()),16);}
  std::ofstream meta(dir/"hierarchy.json");meta<<std::setprecision(17)<<"{\"width\":"<<idx.width<<",\"height\":"<<idx.height<<",\"block_edge\":"<<idx.block_edge<<",\"first_block_edge\":"<<h.first_edge<<",\"second_block_edge\":"<<h.second_edge<<",\"first_columns\":"<<h.first_columns<<",\"first_rows\":"<<h.first_rows<<",\"columns\":"<<h.columns<<",\"rows\":"<<h.rows<<",\"first_total\":"<<4*idx.blocks_per_camera<<",\"second_total\":"<<4*h.columns*h.rows<<",\"maximum_block_half_angle_rad\":"<<table.maximum_block_half_angle()<<",\"radius_m\":"<<ControllerLimits::radius_m<<",\"local_range_m\":"<<ControllerLimits::local_range_m<<",\"distance_bin_m\":0.05}\n";
  if(!level1||!mins||!block_cells||!first_sources||!second_sources||!meta)throw std::runtime_error("block evidence stream failed");
}

int main(int argc,char **argv){try {
  if(argc<6||argc>9)throw std::runtime_error("usage: harmonic_fields_benchmark ARCHIVED_RUN NEW_OUTPUT REPEATS SOLVER FIRST_BLOCK [no_exports] [serial|parallel] [fifo5|normal]");
  const std::string solver_name=argv[4];const uint32_t first_edge=static_cast<uint32_t>(std::stoul(argv[5]));if(first_edge!=3&&first_edge!=4)throw std::runtime_error("first block must be 3 or 4");
  bool do_exports=true,parallel=true,fifo5=true;
  for(int arg=6;arg<argc;++arg) {
    const std::string option=argv[arg];
    if(option=="no_exports")do_exports=false;
    else if(option=="serial")parallel=false;
    else if(option=="parallel")parallel=true;
    else if(option=="fifo5")fifo5=true;
    else if(option=="normal")fifo5=false; // Explicit diagnostic comparison only.
    else throw std::runtime_error("unknown benchmark option");
  }
  const fs::path source=argv[1],out=argv[2];const int repeats=std::stoi(argv[3]);if(repeats<1||repeats>1000||!fs::create_directory(out))throw std::runtime_error("invalid output/repeats");
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  std::ofstream(out/"base_config_sha256.txt")<<BaseConfig::sha256<<'\n';
  const auto meta=YAML::LoadFile((source/"simulation.sensors.json").string());const uint32_t w=meta["render_width"].as<uint32_t>(),h=meta["render_height"].as<uint32_t>();std::array<CameraCalibration,4>cal;
  for(uint32_t c=0;c<4;++c){const auto v=meta["cameras"][c];auto &a=cal[c];a.K={v["fx"].as<double>(),0,v["cx"].as<double>(),0,v["fy"].as<double>(),v["cy"].as<double>(),0,0,1};for(uint32_t r=0;r<3;++r){for(uint32_t j=0;j<3;++j)a.T_BC[r*4+j]=v["robot_from_optical_rotation"][r][j].as<double>();a.T_BC[r*4+3]=v["camera_position_robot_m"][r].as<double>();}a.T_BC[15]=1;}
  // Avoid the reader core/sibling; select the final policy before any measured work.
  cpu_set_t allowed;CPU_ZERO(&allowed);if(sched_getaffinity(0,sizeof(allowed),&allowed))throw std::runtime_error("affinity inventory failed");for(auto cpu:cpu_siblings(5))if(cpu>=0&&cpu<CPU_SETSIZE)CPU_CLR(cpu,&allowed);
  if(pthread_setaffinity_np(pthread_self(),sizeof(allowed),&allowed))throw std::runtime_error("benchmark affinity failed");
  PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder builder;builder.configure(cal,w,h,1,1,16,first_edge,5,parallel,true,true,true);DepthFrame f;f.allocate(w*h);builder.allocate(f);
  HarmonicConfig config;
  std::ofstream profile(out/"solver_profile.json");
  profile<<"{\"corridor_deg\":"<<config.corridor_deg<<",\"max_step_deg\":"<<config.max_step_deg<<",\"residual_tolerance\":"<<config.residual_tolerance<<",\"fine_mode\":\""
#ifdef EGO4P2_LOCAL_FINE_PROFILE
    <<(config.fine_neumann_roi?"local_corridor_neumann":"coarse_dirichlet_coupling")
#else
    <<"coarse_dirichlet_coupling"
#endif
    <<"\",\"revision\":"<<config.revision<<"}\n";
  HarmonicPlanner planner;planner.configure(cal,w,h,config,first_edge);
  if(solver_name=="native_ic0")planner.set_native_preconditioner(CoarsePreconditioner::IC0);
  else if(solver_name=="native_sgs")planner.set_native_preconditioner(CoarsePreconditioner::SymmetricGaussSeidel);
  else {
    const bool both=solver_name.size()>5&&solver_name.substr(solver_name.size()-5)=="_both";
    const auto name=solver_name.substr(0,solver_name.size()-(both?5:7));
    SparseTrialMode mode;
    if(name=="eigen_ldlt")mode=SparseTrialMode::EigenLDLT;else if(name=="eigen_llt")mode=SparseTrialMode::EigenLLT;
    else if(name=="cholmod")mode=SparseTrialMode::Cholmod;else if(name=="amgcl")mode=SparseTrialMode::Amg;
    else throw std::runtime_error("unknown trial solver");
    planner.set_trial_solvers(mode,both);
  }
  HarmonicDepthInput data;planner.allocate(data,w,h);HarmonicResult result;planner.allocate(result);
  // Start persistent camera workers with the full allowed mask before binding
  // the coordinator. Solves and curve integration remain on this one thread.
  if(!CPU_ISSET(0,&allowed))throw std::runtime_error("coordinator CPU0 is unavailable");
  cpu_set_t coordinator;CPU_ZERO(&coordinator);CPU_SET(0,&coordinator);
  if(pthread_setaffinity_np(pthread_self(),sizeof(coordinator),&coordinator))throw std::runtime_error("coordinator affinity failed");
  if(fifo5) {
    builder.enable_fifo5_workers();
    sched_param requested{};requested.sched_priority=5;
    if(pthread_setschedparam(pthread_self(),SCHED_FIFO,&requested))
      throw std::runtime_error("coordinator FIFO5 setup failed; use the FIFO launcher with scheduling permission");
  }
  int actual_policy=0;sched_param actual_priority{};
  if(pthread_getschedparam(pthread_self(),&actual_policy,&actual_priority) ||
      actual_policy!=(fifo5?SCHED_FIFO:SCHED_OTHER) || actual_priority.sched_priority!=(fifo5?5:0))
    throw std::runtime_error("coordinator scheduler readback mismatch");
  builder.refresh_worker_schedulers();
  std::ofstream execution(out/"preprocess_execution.json");
  execution<<"{\"parallel\":"<<(parallel?"true":"false")<<",\"persistent_camera_threads\":"<<(parallel?4:0)<<",\"coordinator_cpu\":"<<sched_getcpu()<<",\"coordinator_scheduler\":"<<actual_policy<<",\"coordinator_priority\":"<<actual_priority.sched_priority<<",\"requested_scheduler\":\""<<(fifo5?"SCHED_FIFO":"SCHED_OTHER")<<"\",\"requested_priority\":"<<(fifo5?5:0)<<",\"worker_cpus\":[";
  for(uint32_t c=0;c<4;++c){if(c)execution<<',';execution<<builder.worker_cpus()[c];}
  execution<<"],\"worker_affinity_errors\":[";
  for(uint32_t c=0;c<4;++c){if(c)execution<<',';execution<<builder.worker_affinity_errors()[c];}
  execution<<"],\"worker_scheduler_errors\":[";
  for(uint32_t c=0;c<4;++c){if(c)execution<<',';execution<<builder.worker_scheduler_errors()[c];}
  execution<<"],\"worker_policies\":[";
  for(uint32_t c=0;c<4;++c){if(c)execution<<',';execution<<builder.worker_policies()[c];}
  execution<<"],\"worker_priorities\":[";
  for(uint32_t c=0;c<4;++c){if(c)execution<<',';execution<<builder.worker_priorities()[c];}
  execution<<"],\"release_protocol\":\"periodic5ms_skip_missed_no_catchup\",\"period_ns\":"<<BaseConfig::period_ns<<",\"motion_output_enabled\":false,\"solver_parallelism\":1,\"curve_parallelism\":1}\n";
  if(!execution)throw std::runtime_error("preprocess execution metadata export failed");
  std::ofstream times(out/"timings.csv");times<<std::setprecision(12)<<"case,mode,iteration,total_ms,capture_ms,coarse_domain_ms,coarse_solve_ms,fine_domain_ms,fine_solve_ms,curve_ms,coarse_iterations,fine_iterations,coarse_residual,fine_residual,coarse_nodes,fine_nodes,steps,status,termination,reused,expanded,component_ms,field_ms,coarse_cache_ms,coarse_assembly_ms,coarse_iteration_ms,coarse_residual_ms,coarse_store_ms,fine_cache_ms,fine_assembly_ms,fine_iteration_ms,fine_residual_ms,fine_store_ms,query_ms,segment_ms,refined_nodes,angular_length_rad,field_version,curve_version,raw_edges,merged_edges,edge_merge_ms,pc_setup_ms,pc_apply_ms,pc_applications,pc_setup_reused,pc_ready,pc_failure_node,pc_failure_pivot,solver,curve_guard,corner_small_step,fine_attempts,curve_failure_kind,curve_failure_cell,curve_rejected_trials,curve_min_accepted_step_rad,coarse_import_ms,coarse_analysis_ms,coarse_factor_ms,coarse_linear_ms,coarse_verify_ms,coarse_symbolic_reused,coarse_numeric_reused,coarse_unknowns,coarse_matrix_nnz,coarse_factor_nnz,fine_import_ms,fine_analysis_ms,fine_factor_ms,fine_linear_ms,fine_verify_ms,fine_symbolic_reused,fine_numeric_reused,fine_unknowns,fine_matrix_nnz,fine_factor_nnz,first_block_edge,first_observations,second_observations,preprocess_ms,input_field_total_ms,available_cells,unknown_cells,excluded_cells,input_parallel,preprocess_camera_work_ms,preprocess_direction_merge_ms,preprocess_omega_ms,repair_max_ms\n";
  std::ofstream releases(out/"releases.csv");
  releases<<"case,mode,iteration,scheduled_ns,start_ns,finish_ns,lateness_ms,compute_ms,response_ms,skipped_releases\n";
  FrameObservationBuilder other_builder;other_builder.configure(cal,w,h,1,1,16,first_edge==3?4:3,5,false,true,true,true);DepthFrame other;other.allocate(w*h);other_builder.allocate(other);
  const auto &grid=planner.fine_stencil().layout;std::vector<uint8_t>base_state(grid.cells()),base_cameras(grid.cells());
  for(const std::string name:{"real_first","real_last","supported_first","supported_last","angular_open","angular_gap","angular_narrow"}) {
    if(name.rfind("angular_",0)!=0) {
      const std::string prefix=name.find("last")!=std::string::npos ? "last" : "first";std::ifstream input(source/(prefix+"_depth.bin"),std::ios::binary);for(auto &d:f.D)input.read(reinterpret_cast<char*>(d.data()),d.size()*sizeof(float));if(!input)throw std::runtime_error("archived depth read failed");
    } else for(auto &d:f.D)std::fill(d.begin(),d.end(),10.f);
    ++f.sequence;f.t_ns=f.sequence*BaseConfig::period_ns;builder.process(f,rays);data.capture(f);if(!data.ready)throw std::runtime_error("snapshot capture failed");
    if(name.rfind("angular_",0)==0) {
      std::fill(data.fine_state.begin(),data.fine_state.end(),0);std::fill(data.cameras.begin(),data.cameras.end(),0);
      for(uint32_t i=0;i<grid.cells();++i){const double a=-180+(i%grid.azimuth_bins+.5),b=-90+(i/grid.azimuth_bins+.5);const bool narrow=name=="angular_narrow";
        if(a>=-45&&a<45&&b>=(narrow?0:-20)&&b<(narrow?1:20)) {data.fine_state[i]=uint8_t(CandidateStatus::Available);data.cameras[i]=1;}
        if(name=="angular_gap"&&std::abs(a)<3&&b<12&&b>=-20)data.fine_state[i]=uint8_t(CandidateStatus::Unavailable);
      }
    }
    std::copy(data.fine_state.begin(),data.fine_state.end(),base_state.begin());std::copy(data.cameras.begin(),data.cameras.end(),base_cameras.begin());
    HarmonicIntent intent;intent.q0=sphere_point(-20*direction_pi/180,(name=="angular_narrow"?.5:0)*direction_pi/180);intent.desired={sphere_point(20*direction_pi/180,(name=="angular_narrow"?.5:0)*direction_pi/180),BaseConfig::max_speed_mps,true,true};intent.R_WB={1,0,0,0,1,0,0,0,1};
    if(name.rfind("supported_",0)==0) {
      other.D=f.D;++other.sequence;other.t_ns=other.sequence*BaseConfig::period_ns;other_builder.process(other,rays);
      double best=-2.;uint32_t selected=UINT32_MAX;
      for(uint32_t i=0;i<data.fine_state.size();++i)if(data.fine_state[i]==uint8_t(CandidateStatus::Available)&&other.local.node_state[i]==uint8_t(CandidateStatus::Available)){const double score=direction_dot(planner.fine_stencil().nodes[i].direction,intent.q0);if(score>best){best=score;selected=i;}}
      if(selected==UINT32_MAX)throw std::runtime_error("no common available anchor for offline supported test");
      intent.q0=planner.fine_stencil().nodes[selected].direction;
    }
    for(const std::string mode:{"cold","boundary_change","new_group_reuse"}) {
      planner.invalidate_cache();planner.build(data,intent,result);
      int64_t next_release=field_now_ns()+BaseConfig::period_ns;
      for(int i=-3;i<repeats;++i) {
        timespec release{next_release/1000000000LL,next_release%1000000000LL};
        int err;do {err=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&release,nullptr);}while(err==EINTR);
        if(err)throw std::runtime_error("periodic benchmark sleep failed");
        const auto compute_begin=field_now_ns();
        double preprocess_ms=0.;
        if(mode=="cold"){++f.sequence;f.t_ns=f.sequence*BaseConfig::period_ns;const auto preprocess_begin=field_now_ns();builder.process(f,rays);preprocess_ms=(field_now_ns()-preprocess_begin)/1e6;planner.invalidate_cache();}
        intent.desired.d_desired=sphere_point((mode=="boundary_change"&&i%2 ? 16. : 20.)*direction_pi/180,(name=="angular_narrow"?.5:0)*direction_pi/180);
        data.capture(f); // receiving-side summary cost, excluded from field worker total
        if(name.rfind("angular_",0)==0){std::copy(base_state.begin(),base_state.end(),data.fine_state.begin());std::copy(base_cameras.begin(),base_cameras.end(),data.cameras.begin());}
        data.sequence=f.sequence*100000+uint64_t(i+4);data.stamp_ns=f.t_ns+int64_t(i+4)*BaseConfig::period_ns; // synthetic replay identities
        planner.build(data,intent,result);
        const auto compute_end=field_now_ns();
        const auto skipped=std::max<int64_t>(0,(compute_end-next_release)/BaseConfig::period_ns);
        if(i>=0)releases<<name<<','<<mode<<','<<i<<','<<next_release<<','<<compute_begin<<','<<compute_end<<','<<(compute_begin-next_release)/1e6<<','<<(compute_end-compute_begin)/1e6<<','<<(compute_end-next_release)/1e6<<','<<skipped<<'\n';
        next_release+=(skipped+1)*BaseConfig::period_ns;
        const bool supported=data.fine_state[planner.fine_stencil().cell(intent.q0)]==uint8_t(CandidateStatus::Available);
        // Experimental guard may correctly stop at real Unknown/Obstacle.
        // Record status and termination; never count an early Blocked as Exit.
        if(supported&&result.status==FieldStatus::NoInput)throw std::runtime_error("supported fixture lost its input");
        if(!supported&&(result.status!=FieldStatus::NeedsRefinement||result.curve_size))throw std::runtime_error("unknown fixture fabricated a curve");
        if(i>=0)times<<name<<','<<mode<<','<<i<<','<<result.total_ns/1e6<<','<<data.capture_ns/1e6<<','<<result.coarse.domain_ns/1e6<<','<<result.coarse.solve_ns/1e6<<','<<result.fine.domain_ns/1e6<<','<<result.fine.solve_ns/1e6<<','<<result.curve_ns/1e6<<','<<result.coarse.iterations<<','<<result.fine.iterations<<','<<result.coarse.residual<<','<<result.fine.residual<<','<<result.coarse.nodes<<','<<result.fine.nodes<<','<<result.integration_steps<<','<<field_status_name(result.status)<<','<<curve_end_name(result.termination)<<','<<result.reused<<','<<result.expanded<<','<<result.component_ns/1e6<<','<<result.field_ns/1e6<<','<<result.coarse.cache_ns/1e6<<','<<result.coarse.assembly_ns/1e6<<','<<result.coarse.iteration_ns/1e6<<','<<result.coarse.residual_ns/1e6<<','<<result.coarse.store_ns/1e6<<','<<result.fine.cache_ns/1e6<<','<<result.fine.assembly_ns/1e6<<','<<result.fine.iteration_ns/1e6<<','<<result.fine.residual_ns/1e6<<','<<result.fine.store_ns/1e6<<','<<result.query_ns/1e6<<','<<result.segment_ns/1e6<<','<<result.adaptive.refined_nodes<<','<<result.angular_length<<','<<result.field_version<<','<<result.curve_version<<','<<result.adaptive.raw_edges<<','<<result.adaptive.edges<<','<<result.adaptive.edge_merge_ns/1e6<<','<<planner.preconditioner_stats().setup_ns/1e6<<','<<planner.preconditioner_stats().apply_ns/1e6<<','<<planner.preconditioner_stats().applications<<','<<planner.preconditioner_stats().setup_reused<<','<<planner.preconditioner_stats().ready<<','<<planner.preconditioner_stats().failure_node<<','<<planner.preconditioner_stats().failure_pivot<<','<<solver_name<<','<<EGO4P0_CURVE_GUARD<<','<<EGO4P0_CORNER_SMALL_STEP<<','<<result.fine_attempts<<','<<unsigned(result.curve_failure_kind)<<','<<result.curve_failure_cell<<','<<result.curve_rejected_trials<<','<<result.curve_min_accepted_step_rad<<','<<planner.coarse_trial_stats().import_ns/1e6<<','<<planner.coarse_trial_stats().analysis_ns/1e6<<','<<planner.coarse_trial_stats().factor_ns/1e6<<','<<planner.coarse_trial_stats().solve_ns/1e6<<','<<planner.coarse_trial_stats().verify_ns/1e6<<','<<planner.coarse_trial_stats().symbolic_reused<<','<<planner.coarse_trial_stats().numeric_reused<<','<<planner.coarse_trial_stats().unknowns<<','<<planner.coarse_trial_stats().nonzeros<<','<<planner.coarse_trial_stats().factor_nonzeros<<','<<planner.fine_trial_stats().import_ns/1e6<<','<<planner.fine_trial_stats().analysis_ns/1e6<<','<<planner.fine_trial_stats().factor_ns/1e6<<','<<planner.fine_trial_stats().solve_ns/1e6<<','<<planner.fine_trial_stats().verify_ns/1e6<<','<<planner.fine_trial_stats().symbolic_reused<<','<<planner.fine_trial_stats().numeric_reused<<','<<planner.fine_trial_stats().unknowns<<','<<planner.fine_trial_stats().nonzeros<<','<<planner.fine_trial_stats().factor_nonzeros<<','<<first_edge<<','<<4*f.observations.blocks_per_camera<<','<<4*data.columns*data.rows<<','<<preprocess_ms<<','<<preprocess_ms+data.capture_ns/1e6+result.total_ns/1e6<<','<<f.local.available_cells<<','<<f.local.unknown_cells<<','<<f.local.excluded_cells<<','<<parallel<<','<<(mode=="cold"?f.observations.camera_work_ns/1e6:0.)<<','<<(mode=="cold"?f.observations.merge_ns/1e6:0.)<<','<<(mode=="cold"?f.local.merge_ns/1e6:0.)<<','<<(mode=="cold"?f.preprocessing_ns/1e6:0.)<<'\n';
      }
      if(do_exports&&(mode=="cold"||mode=="boundary_change")) {
        const auto dir=out/(mode=="cold" ? name : name+"_boundary_change");fs::create_directory(dir);save_harmonic(dir,result);save_block_levels(dir,f,builder.local_geometry(),planner);
        std::ofstream raw(dir/"raw.bin",std::ios::binary),filled(dir/"filled.bin",std::ios::binary),coeff(dir/"range_coefficients.bin",std::ios::binary);
        for(uint32_t c=0;c<4;++c){raw.write(reinterpret_cast<const char*>(f.D[c].data()),f.D[c].size()*sizeof(float));filled.write(reinterpret_cast<const char*>(f.D_filled[c].data()),f.D_filled[c].size()*sizeof(float));const auto &v=builder.local_geometry().virtual_tables().coefficients(c);coeff.write(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(double));}
        fs::copy_file(source/"simulation.sensors.json",dir/"simulation.sensors.json");
        std::ofstream input(dir/"input.json");input<<"{\"q0\":[";for(uint32_t k=0;k<3;++k){if(k)input<<',';input<<std::setprecision(17)<<intent.q0[k];}input<<"],\"desired\":[";for(uint32_t k=0;k<3;++k){if(k)input<<',';input<<intent.desired.d_desired[k];}input<<"],\"synthetic_angular_domain\":"<<(name.rfind("angular_",0)==0 ? "true" : "false")<<",\"offline_q0_selected_from_available\":"<<(name.rfind("supported_",0)==0?"true":"false")<<",\"q0_selected_from_common_3_4_available\":true,\"replay_group_identity_synthetic\":true,\"new_simulation\":false,\"desired_speed_mps\":"<<intent.desired.v_desired<<"}";
      }
    }
    std::cout<<name<<": "<<field_status_name(result.status)<<'/'<<curve_end_name(result.termination)<<", coarse="<<result.coarse.nodes<<", fine="<<result.fine.nodes<<'\n';
  }
  if(!times)throw std::runtime_error("benchmark timing export failed");
  return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}}
