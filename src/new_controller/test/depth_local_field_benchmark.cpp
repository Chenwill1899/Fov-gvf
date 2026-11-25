#include "new_controller/depth_observation_io.hpp"
#include "new_controller/depth_local_io.hpp"
#include "new_controller/desired_direction.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>
#include <sched.h>

using namespace ego4p0;namespace fs=std::filesystem;
static int64_t now() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char **argv) {
 try {
  if(argc!=5) throw std::runtime_error("usage: depth_local_field_benchmark ARCHIVED_RUN first|last NEW_OUTPUT on|off");
  const fs::path source=argv[1],out=argv[3];const std::string prefix=argv[2];const bool local=std::string(argv[4])=="on";
  if((prefix!="first" && prefix!="last") || (std::string(argv[4])!="on" && std::string(argv[4])!="off") || !fs::create_directory(out))
    throw std::runtime_error("invalid benchmark prefix/mode or output directory exists");
  const auto cal=YAML::LoadFile((source/"simulation.sensors.json").string()),meta=YAML::LoadFile((source/"reader_meta.json").string());
  const uint32_t w=cal["render_width"].as<uint32_t>(),h=cal["render_height"].as<uint32_t>(),n=depth_pixel_count(w,h);
  std::array<CameraCalibration,4> cameras;
  for(uint32_t c=0;c<4;++c) {
    const auto v=cal["cameras"][c];auto &a=cameras[c];a.K={v["fx"].as<double>(),0,v["cx"].as<double>(),0,v["fy"].as<double>(),v["cy"].as<double>(),0,0,1};
    for(uint32_t r=0;r<3;++r) {for(uint32_t k=0;k<3;++k) a.T_BC[r*4+k]=v["robot_from_optical_rotation"][r][k].as<double>();a.T_BC[r*4+3]=v["camera_position_robot_m"][r].as<double>();}
    a.T_BC[15]=1;if(v["near_clip_z_m"]) a.near_clip_z_m=v["near_clip_z_m"].as<double>();
  }
  DepthFrame f;f.allocate(n);f.t_ns=meta[prefix+"_image_stamp_ns"].as<int64_t>();
  std::ifstream events(source/"frames.csv");std::string line;std::getline(events,line);
  while(std::getline(events,line)) {std::istringstream row(line);std::string seq,stamp;std::getline(row,seq,',');std::getline(row,stamp,',');if(std::stoll(stamp)==f.t_ns) f.sequence=std::stoull(seq);}
  if(!f.sequence || fs::file_size(source/(prefix+"_depth.bin"))!=4*n*sizeof(float)) throw std::runtime_error("archived input identity/size mismatch");
  std::ifstream input(source/(prefix+"_depth.bin"),std::ios::binary);for(auto &d:f.D) input.read(reinterpret_cast<char*>(d.data()),n*sizeof(float));if(!input) throw std::runtime_error("input read failed");
  const auto init=now();PixelRayTables rays;rays.configure(cameras,w,h);FrameObservationBuilder builder;
  builder.configure(cameras,w,h,1,1,16,3,5,true,true,true,local);builder.allocate(f);const auto initialization_ns=now()-init;
  cpu_set_t allowed;CPU_ZERO(&allowed);sched_getaffinity(0,sizeof(allowed),&allowed);
  if(CPU_ISSET(0,&allowed)) {cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(0,&mask);if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask)) throw std::runtime_error("coordinator affinity failed");}
  struct Timing {int64_t all,work,merge,shared_projection_local_max,local_merge;};std::vector<Timing> times(100);
  for(int iteration=-10;iteration<100;++iteration) {
    f.observations.ready=false;const auto begin=now();builder.process(f,rays);const auto end=now();
    if(iteration>=0) times[iteration]={end-begin,f.observations.camera_work_ns,f.observations.merge_ns,
      local ? *std::max_element(f.local.shared_projection_ns.begin(),f.local.shared_projection_ns.end()) : 0,local ? f.local.merge_ns : 0};
  }
  std::ofstream csv(out/"timings.csv");csv<<std::setprecision(12)<<"iteration,all_input_ms,camera_work_ms,index_merge_ms,shared_projection_local_max_ms,local_merge_ms\n";
  for(size_t i=0;i<times.size();++i) {const auto &t=times[i];csv<<i<<','<<t.all/1e6<<','<<t.work/1e6<<','<<t.merge/1e6<<','<<t.shared_projection_local_max/1e6<<','<<t.local_merge/1e6<<'\n';}
  save_observation_index(out,"sample",f);
  if(local) save_local_field(out,"sample",f,builder.local_geometry());
  fs::copy_file(source/"simulation.sensors.json",out/"simulation.sensors.json");
  std::ofstream raw(out/"raw_depth.bin",std::ios::binary),filled(out/"filled_depth.bin",std::ios::binary),b(out/"pixel_rays_b.bin",std::ios::binary),d(out/"pixel_rays_d.bin",std::ios::binary);
  for(uint32_t c=0;c<4;++c) {
    raw.write(reinterpret_cast<const char*>(f.D[c].data()),n*sizeof(float));filled.write(reinterpret_cast<const char*>(f.D_filled[c].data()),n*sizeof(float));
    for(const auto &r:rays.camera(c)) {b.write(reinterpret_cast<const char*>(r.b.data()),3*sizeof(double));d.write(reinterpret_cast<const char*>(r.d.data()),3*sizeof(double));}
  }
  std::ofstream query(out/"size_query_timings.csv");query<<"iteration,query_ns,status,desired_ns,desired_speed_mps\n";
  uint64_t checksum=0;BodyState body;body.p_W={-64,-11.2,2.8};
  for(uint32_t i=0;i<2000 && local;++i) {
    const double a=(i%360)*direction_pi/180.;const double range=.1+(i%29)*.1;
    const std::array<double,3> center{range*std::cos(a),range*std::sin(a),.1};
    const auto begin=now();const auto q=LocalFieldView(f,builder.local_geometry()).sphere(center);const auto queried=now();
    const auto desired=desired_motion({DesiredInputMode::BodyVelocity,{std::cos(a),std::sin(a),0},0,0},body);const auto end=now();
    checksum+=static_cast<int>(q.status);query<<i<<','<<queried-begin<<','<<static_cast<int>(q.status)<<','<<end-queried<<','<<desired.v_desired<<'\n';
  }
  std::ofstream result(out/"benchmark.json");result<<std::setprecision(17)<<"{\"scope\":\"offline archived Isaac snapshot; 10 warmups/100 forced rebuilds, no new simulation/FIFO claim\",\"local_enabled\":"<<(local?"true":"false")
    <<",\"initialization_ms\":"<<initialization_ns/1e6<<",\"width\":"<<w<<",\"height\":"<<h<<",\"stamp_ns\":"<<f.t_ns<<",\"sequence\":"<<f.sequence
    <<",\"coordinator_cpu\":"<<sched_getcpu()<<",\"scheduler\":"<<sched_getscheduler(0)<<",\"worker_cpus\":[";
  for(uint32_t c=0;c<4;++c) {if(c) result<<',';result<<builder.worker_cpus()[c];}
  result<<"],\"local_excluded_blocks\":"<<f.local.excluded_blocks<<",\"local_excluded_cells\":"<<f.local.excluded_cells
    <<",\"local_available_cells\":"<<f.local.available_cells<<",\"local_unknown_cells\":"<<f.local.unknown_cells<<",\"planning_model\":\"body_origin_virtual_omnidirectional\",\"translation_ignored\":true,\"geometry_builds\":"<<builder.local_geometry().builds()<<",\"query_checksum\":"<<checksum<<"}\n";
  if(!csv || !raw || !filled || !b || !d || !query || !result) throw std::runtime_error("benchmark export failed");
  std::cout<<out<<'\n';return 0;
 } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
