#pragma once
#include "new_controller/navigation_pipeline.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
namespace ego4p0 {
inline std::array<CameraCalibration,4> navigation_calibration(const std::filesystem::path&path,uint32_t&w,uint32_t&h){
  const auto m=YAML::LoadFile(path.string());w=m["render_width"].as<uint32_t>();h=m["render_height"].as<uint32_t>();depth_pixel_count(w,h);
  if(m["cameras"].size()!=4||m["depth_type"].as<std::string>()!="optical_axis_z_m")throw std::invalid_argument("four optical Z cameras required");
  std::array<CameraCalibration,4>cal;
  for(uint32_t c=0;c<4;++c){const auto a=m["cameras"][c];auto&v=cal[c];v.K={a["fx"].as<double>(),0,a["cx"].as<double>(),0,a["fy"].as<double>(),a["cy"].as<double>(),0,0,1};v.frame=a["frame_id"].as<std::string>();if(a["near_clip_z_m"])v.near_clip_z_m=a["near_clip_z_m"].as<double>();for(int i=0;i<3;++i){for(int j=0;j<3;++j)v.T_BC[4*i+j]=a["robot_from_optical_rotation"][i][j].as<double>();v.T_BC[4*i+3]=a["camera_position_robot_m"][i].as<double>();}v.T_BC[15]=1;}
  return cal;
}
inline uint64_t navigation_calibration_version(const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h){
  uint64_t value=1469598103934665603ULL;auto mix=[&](uint64_t x){for(int i=0;i<8;++i){value^=(x>>(8*i))&255;value*=1099511628211ULL;}};mix(w);mix(h);
  for(const auto&c:cal){for(double d:c.K){uint64_t bits;std::memcpy(&bits,&d,8);mix(bits);}for(double d:c.T_BC){uint64_t bits;std::memcpy(&bits,&d,8);mix(bits);}uint64_t bits=0x7ff8000000000000ULL;if(std::isfinite(c.near_clip_z_m))std::memcpy(&bits,&c.near_clip_z_m,8);mix(bits);for(unsigned char a:c.frame){value^=a;value*=1099511628211ULL;}}
  return value?value:1;
}
inline void navigation_read_depth(const std::filesystem::path&file,NavigationObservation&o){std::ifstream f(file,std::ios::binary);for(auto&d:o.depth.D)f.read(reinterpret_cast<char*>(d.data()),d.size()*sizeof(float));if(!f||f.peek()!=std::char_traits<char>::eof())throw std::runtime_error("depth payload size mismatch");}
inline void navigation_identity(NavigationObservation&o,uint64_t group,uint64_t calibration=1){o.depth.sequence=group;o.depth.t_ns=int64_t(group)*BaseConfig::period_ns;o.identity={group,calibration,group,o.depth.t_ns,{o.depth.t_ns,o.depth.t_ns,o.depth.t_ns,o.depth.t_ns},15};o.observation_pose={};o.observation_pose.t_ns=o.depth.t_ns;o.observation_pose_valid=true;}
inline const char *navigation_detail_header(){
  return "coordinator_cpu_ms,field_cpu_ms,field_reset_ms,component_ms,coarse_domain_ms,coarse_edge_merge_ms,field_export_ms,fine_domain_ms,field_finalize_ms,fine_attempts,integration_steps,gradient_queries,segment_checks,coarse_assembly_ms,coarse_cache_ms,coarse_residual_check_ms,coarse_store_ms,fine_assembly_ms,fine_cache_ms,fine_residual_check_ms,fine_store_ms,coarse_validation_ms,coarse_conversion_ms,coarse_import_ms,coarse_analysis_ms,coarse_factor_ms,coarse_backsolve_ms,coarse_backend_verify_ms,fine_validation_ms,fine_conversion_ms,fine_import_ms,fine_analysis_ms,fine_factor_ms,fine_backsolve_ms,fine_backend_verify_ms,coarse_status,coarse_topology_only,coarse_topology_ms,coarse_solve_cpu_ms,fine_solve_cpu_ms,coarse_graph_reused,coarse_graph_cache_ms";
}
inline void navigation_detail_csv(std::ostream &stream,const NavigationPipeline &pipeline){
  const auto &r=pipeline.field();
  const bool ready=pipeline.output().field_attempted;
  const auto &planner=pipeline.planner();
  const auto &coarse=planner.coarse_trial_stats();
  const auto &fine=planner.fine_trial_stats();
  const std::array<int64_t,9> phases{
    pipeline.output().coordinator_cpu_ns,r.cpu_ns,r.reset_ns,r.component_ns,
    r.coarse.domain_ns,r.adaptive.edge_merge_ns,r.export_ns,r.fine.domain_ns,r.finalize_ns};
  for(size_t i=0;i<phases.size();++i)stream<<','<<(ready||i==0?phases[i]/1e6:0.);
  stream<<','<<(ready?r.fine_attempts:0)<<','<<(ready?r.integration_steps:0)
        <<','<<(ready?r.gradient_queries:0)<<','<<(ready?r.segment_checks:0);
  for(const auto *layer:{&r.coarse,&r.fine}){
    for(auto ns:{layer->assembly_ns,layer->cache_ns,layer->residual_ns,layer->store_ns})stream<<','<<(ready?ns/1e6:0.);
  }
  for(const auto *solver:{&coarse,&fine}){
    for(auto ns:{solver->validation_ns,solver->conversion_ns,solver->import_ns,solver->analysis_ns,
                 solver->factor_ns,solver->solve_ns,solver->verify_ns})stream<<','<<(ready?ns/1e6:0.);
  }
  stream<<','<<(ready?field_status_name(r.coarse.status):"no_input")
        <<','<<(ready&&r.coarse_topology_only)<<','<<(ready?r.coarse_topology_ns/1e6:0.)
        <<','<<(ready?r.coarse.solve_cpu_ns/1e6:0.)<<','<<(ready?r.fine.solve_cpu_ns/1e6:0.)
        <<','<<(ready&&r.coarse_graph_reused)<<','<<(ready?r.coarse_graph_cache_ns/1e6:0.);
}
inline void spatial_export(const std::filesystem::path&dir,const SpatialReference&r,double horizon){
  std::filesystem::create_directories(dir);auto vec=[](std::ostream&s,const Direction3&v){s<<'['<<v[0]<<','<<v[1]<<','<<v[2]<<']';};
    std::ofstream samples(dir/"spatial_curve.csv");samples<<std::setprecision(17)<<"s,x,y,z,tx,ty,tz,kx,ky,kz,kappa,zeta,error_bound_m\n";
    std::vector<double>knots;for(double s:r.nodes())knots.push_back(s);for(double s=0;s<horizon;s+=SpatialReferenceConfig::lookup_step_m)knots.push_back(s);knots.push_back(horizon);std::sort(knots.begin(),knots.end());knots.erase(std::unique(knots.begin(),knots.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),knots.end());
    for(double s:knots){auto a=r.query(s);samples<<a.s;for(auto v:a.position_W)samples<<','<<v;for(auto v:a.tangent_W)samples<<','<<v;for(auto v:a.curvature_W)samples<<','<<v;samples<<','<<a.kappa<<','<<a.zeta<<','<<a.position_error_bound_m<<'\n';}

    std::ofstream pieces(dir/"angular_interpolant.json");pieces<<std::setprecision(17)<<"{\"pieces\":[";bool comma=false;for(const auto&b:r.angular_path().pieces()){if(comma)pieces<<',';comma=true;pieces<<"{\"begin\":"<<b.begin<<",\"length\":"<<b.length<<",\"controls\":[";for(int i=0;i<4;++i){if(i)pieces<<',';vec(pieces,b.b[i]);}pieces<<"]}";}pieces<<"],\"anchor_W\":";vec(pieces,r.anchor());pieces<<",\"R_WB\":[";for(int j=0;j<9;++j){if(j)pieces<<',';pieces<<r.rotation()[j];}pieces<<"],\"q0\":";vec(pieces,r.angular_path().query(0).q);pieces<<",\"length\":"<<r.angular_path().length()<<",\"ramp\":"<<r.profile().ramp<<",\"plateau\":"<<r.profile().plateau<<",\"peak\":"<<r.profile().peak<<",\"turn_length\":"<<r.profile().length<<"}\n";
}
inline void navigation_export(const std::filesystem::path&dir,const NavigationPipeline&p){
  std::filesystem::create_directories(dir);const auto&o=p.output();std::ofstream f(dir/"navigation_state.json");
  f<<std::setprecision(17)<<"{\"group\":"<<o.observation.group<<",\"calibration_version\":"<<o.observation.calibration_version<<",\"observation_sim_ns\":"<<o.observation.simulation_stamp_ns<<",\"current_sim_ns\":"<<o.current_simulation_ns<<",\"configuration_version\":"<<o.configuration_version<<",\"configuration_sha256\":\""<<NavigationConfig::sha256<<"\",\"reference_version\":"<<o.reference_version<<",\"reference_kind\":\""<<(o.spatial_enabled?"spatial_reference":"angular_Q")<<"\",\"field_version\":"<<o.field_version<<",\"curve_version\":"<<o.curve_version<<",\"input_ready\":"<<o.input_ready<<",\"angular_ready\":"<<o.angular_ready<<",\"invalidation_reason\":\""<<navigation_invalid_name(o.invalid)<<"\",\"q0_source\":\""<<o.q0_source<<"\",\"spatial_reference_enabled\":"<<o.spatial_enabled
    <<",\"continuous_tube_check_enabled\":"<<o.spatial_enabled
    <<",\"spatial_ready\":"<<o.spatial_ready<<",\"spatial_reused\":"<<o.spatial_reused
    <<",\"occupancy_reused\":"<<o.occupancy_reused<<",\"geometry_version\":"<<o.geometry_version
    <<",\"turn_length_m\":"<<o.turn_length_m<<",\"turn_time_s\":"<<o.turn_length_m/BaseConfig::max_speed_mps
    <<",\"checked_horizon_m\":"<<o.checked_horizon_m
    <<",\"observed_sections\":"<<o.observed_sections<<",\"unknown_sections\":"<<o.unknown_sections
    <<",\"obstacle_sections\":"<<o.obstacle_sections<<",\"outside_sections\":"<<o.outside_sections
    <<",\"spatial_ns\":"<<o.spatial_ns<<",\"occupancy_ns\":"<<o.occupancy_ns<<",\"motion_output_enabled\":false,\"motion_publishers\":0,\"omega_radius_m\":"<<ControllerLimits::radius_m<<"}\n";
  const auto&r=p.field();std::ofstream q(dir/"angular_Q.csv");q<<std::setprecision(17)<<"index,qx,qy,qz\n";
  if(o.angular_ready)for(uint32_t i=0;i<r.curve_size;++i)q<<i<<','<<r.curve[i].direction[0]<<','<<r.curve[i].direction[1]<<','<<r.curve[i].direction[2]<<'\n';
  if(o.spatial_ready){
    spatial_export(dir,p.reference(),o.checked_horizon_m);
    std::ofstream sections(dir/"tube_sections.csv");sections<<std::setprecision(17)<<"s0,s1,cover_radius_m,status,cx,cy,cz\n";for(auto&s:p.tube().sections())sections<<s.s0<<','<<s.s1<<','<<s.cover_radius_m<<','<<tube_status_name(s.status)<<','<<s.cover_center_W[0]<<','<<s.cover_center_W[1]<<','<<s.cover_center_W[2]<<'\n';
  }
}
} // namespace ego4p0
