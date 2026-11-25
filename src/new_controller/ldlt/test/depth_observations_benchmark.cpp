#include "new_controller/depth_observation_io.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <sched.h>
#include <sstream>
#include <time.h>

using namespace ego4p0;
namespace fs=std::filesystem;
static int64_t now_ns() {timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000000LL+t.tv_nsec;}
static void summary(std::ostream &s,std::vector<int64_t> values) {
  std::sort(values.begin(),values.end());double sum=0;for(auto n:values) sum+=n;
  auto percentile=[&](double q) {double x=q*(values.size()-1);size_t i=static_cast<size_t>(x);return values[i]+(x-i)*(values[std::min(i+1,values.size()-1)]-values[i]);};
  s<<"{\"count\":"<<values.size()<<",\"mean_ms\":"<<sum/values.size()/1e6<<",\"p99_ms\":"<<percentile(.99)/1e6<<",\"max_ms\":"<<values.back()/1e6<<'}';
}
int main(int argc,char **argv) {
  try {
    if(argc!=4 && argc!=6 && argc!=9 && argc!=10) throw std::runtime_error("usage: depth_observations_benchmark ARCHIVED_ISAAC_RUN first|last NEW_OUTPUT_DIR [AZIMUTH_DEG ELEVATION_DEG [BLOCK_EDGE PARALLEL BOUNDARY_LOOKUP [FUSED_SCAN]]]");
    const fs::path source=argv[1],out=argv[3];const std::string prefix=argv[2];
    if(prefix!="first" && prefix!="last") throw std::runtime_error("prefix must be first or last");
    if(!fs::create_directory(out)) throw std::runtime_error("output directory must be new");
    const double alpha=argc>=6 ? std::stod(argv[4]) : 1.,beta=argc>=6 ? std::stod(argv[5]) : 1.;
    const uint32_t block=argc>=9 ? std::stoul(argv[6]) : 3;const bool parallel=argc>=9 ? std::stoi(argv[7])!=0 : true,boundary=argc>=9 ? std::stoi(argv[8])!=0 : true,fused=argc==10 ? std::stoi(argv[9])!=0 : true;
    const auto cal=YAML::LoadFile((source/"simulation.sensors.json").string()),meta=YAML::LoadFile((source/"reader_meta.json").string());
    if(cal["depth_type"].as<std::string>()!="optical_axis_z_m") throw std::runtime_error("optical Z meters required");
    const size_t width=cal["render_width"].as<size_t>(),height=cal["render_height"].as<size_t>(),pixels=depth_pixel_count(width,height);
    std::array<CameraCalibration,4> cameras;
    for(size_t i=0;i<4;++i) {
      const auto c=cal["cameras"][i];auto &camera=cameras[i];
      camera.K={c["fx"].as<double>(),0,c["cx"].as<double>(),0,c["fy"].as<double>(),c["cy"].as<double>(),0,0,1};
      for(size_t r=0;r<3;++r) {for(size_t k=0;k<3;++k) camera.T_BC[r*4+k]=c["robot_from_optical_rotation"][r][k].as<double>();camera.T_BC[r*4+3]=c["camera_position_robot_m"][r].as<double>();}
      camera.T_BC[15]=1;
    }
    DepthFrame frame;frame.allocate(pixels);frame.t_ns=meta[prefix+"_image_stamp_ns"].as<int64_t>();
    std::ifstream frames(source/"frames.csv");std::string line;std::getline(frames,line);
    while(std::getline(frames,line)) {std::istringstream row(line);std::string seq,stamp;std::getline(row,seq,',');std::getline(row,stamp,',');if(std::stoll(stamp)==frame.t_ns) frame.sequence=std::stoull(seq);}
    if(!frame.sequence) throw std::runtime_error("snapshot group sequence not found in original frames.csv");
    const auto raw=source/(prefix+"_depth.bin");if(fs::file_size(raw)!=4*pixels*sizeof(float)) throw std::runtime_error("raw image shape mismatch");
    std::ifstream input(raw,std::ios::binary);for(auto &image:frame.D) input.read(reinterpret_cast<char*>(image.data()),pixels*sizeof(float));
    if(!input) throw std::runtime_error("raw read failed");
    PixelRayTables rays;FrameObservationBuilder builder;const auto init=now_ns();rays.configure(cameras,width,height);builder.configure_real(cameras,width,height,alpha,beta,16,block,5,parallel,boundary,fused);builder.allocate(frame);const auto initialization_ns=now_ns()-init;
    // Worker masks have already been selected from the launcher's background
    // set. Pin only this offline coordinator afterwards for comparable trials.
    cpu_set_t allowed;CPU_ZERO(&allowed);sched_getaffinity(0,sizeof(allowed),&allowed);
    if(CPU_ISSET(0,&allowed)) {cpu_set_t coordinator;CPU_ZERO(&coordinator);CPU_SET(0,&coordinator);if(pthread_setaffinity_np(pthread_self(),sizeof(coordinator),&coordinator)) throw std::runtime_error("benchmark coordinator affinity failed");}
    std::vector<int64_t> repairs(100),directions(100),regions(100),all(100);
    std::vector<std::array<int64_t,17>> camera_times(100);
    for(int iteration=-10;iteration<100;++iteration) {
      frame.observations.ready=false; // Offline construction benchmark; deliberately bypass same-group reuse.
      const auto start=now_ns();builder.process(frame,rays);const auto end=now_ns();
      if(iteration>=0) {
        repairs[iteration]=frame.preprocessing_ns;directions[iteration]=frame.observations.camera_work_ns;regions[iteration]=frame.observations.merge_ns;all[iteration]=end-start;
        for(size_t c=0;c<4;++c) {camera_times[iteration][c]=frame.repair_ns[c];camera_times[iteration][4+c]=frame.observations.compression_ns[c];camera_times[iteration][8+c]=frame.observations.projection_ns[c];camera_times[iteration][12+c]=frame.observations.region_camera_ns[c];}
        camera_times[iteration][16]=frame.observations.merge_ns;
      }
    }
    DepthObservationView queries(frame);std::vector<int64_t> pixel_times(2000),region_times(2000);double checksum=0.;
    std::ofstream query_timing(out/"query_timings.csv");query_timing<<"iteration,pixel_ns,region_ns,valid_pixels,total_pixels,tiles_used,pixels_scanned\n";
    for(size_t i=0;i<2000;++i) {
      const auto start=now_ns();const auto p=queries.pixel(i%4,i%width,(i*7)%height);const auto end=now_ns();pixel_times[i]=end-start;checksum+=p.z.value_or(0.f);
      PixelRegion q{0,0,static_cast<int>(width),static_cast<int>(height)};
      if(i%2) q={1,1,static_cast<int>(width)-1,static_cast<int>(height)-1};
      const auto rstart=now_ns();const auto r=queries.region(i%4,q);region_times[i]=now_ns()-rstart;checksum+=r.z.value_or(0.f)+r.valid_pixels;
      query_timing<<i<<','<<pixel_times[i]<<','<<region_times[i]<<','<<r.valid_pixels<<','<<r.total_pixels<<','<<r.tiles_used<<','<<r.pixels_scanned<<'\n';
    }
    save_observation_index(out,"sample",frame);
    fs::copy_file(raw,out/"raw_depth.bin");fs::copy_file(source/"simulation.sensors.json",out/"simulation.sensors.json");
    std::ofstream filled(out/"filled_depth.bin",std::ios::binary),b(out/"pixel_rays_b.bin",std::ios::binary),d(out/"pixel_rays_d.bin",std::ios::binary);
    for(size_t camera=0;camera<4;++camera) {
      filled.write(reinterpret_cast<const char*>(frame.D_filled[camera].data()),pixels*sizeof(float));
      for(const auto &ray:rays.camera(camera)) {b.write(reinterpret_cast<const char*>(ray.b.data()),3*sizeof(double));d.write(reinterpret_cast<const char*>(ray.d.data()),3*sizeof(double));}
    }
    std::ofstream timing(out/"build_timings.csv");timing<<"iteration,repair_max_ms,camera_work_ms,all_input_ms";
    for(const char *stage:{"repair","compression","projection","region"}) for(size_t c=0;c<4;++c) timing<<','<<stage<<"_camera"<<c<<"_ms";
    timing<<",merge_ms\n"<<std::setprecision(12);
    for(size_t i=0;i<100;++i) {timing<<i<<','<<repairs[i]/1e6<<','<<directions[i]/1e6<<','<<all[i]/1e6;for(auto ns:camera_times[i]) timing<<','<<ns/1e6;timing<<'\n';}
    sched_param policy{};sched_getparam(0,&policy);
    std::ofstream stats(out/"benchmark.json");stats<<std::setprecision(12)<<"{\"scope\":\"offline real archived Isaac snapshot, 10 warmups and 100 cache builds; not an online/FIFO deadline result\",\"cpu\":"<<sched_getcpu()
      <<",\"policy\":"<<sched_getscheduler(0)<<",\"priority\":"<<policy.sched_priority<<",\"initialization_ms\":"<<initialization_ns/1e6<<",\"repair_max_elapsed\":";summary(stats,repairs);
    stats<<",\"camera_work_wall\":";summary(stats,directions);stats<<",\"merge\":";summary(stats,regions);stats<<",\"all_input\":";summary(stats,all);
    stats<<",\"pixel_query\":";summary(stats,pixel_times);stats<<",\"region_query\":";summary(stats,region_times);
    stats<<",\"block_edge\":"<<block<<",\"parallel\":"<<(parallel?"true":"false")<<",\"boundary_lookup\":"<<(boundary?"true":"false")
      <<",\"worker_repair\":true,\"fused_scan\":"<<(fused?"true":"false")<<",\"repair_timing_scope\":\"max camera repair, inside and overlapping camera_work_wall\""
      <<",\"forced_rebuild_for_benchmark\":true,\"table_builds\":"<<builder.table_builds()<<",\"index_builds\":"<<builder.builds()<<",\"worker_cpus\":[";
    for(size_t c=0;c<4;++c) {if(c) stats<<',';stats<<builder.worker_cpus()[c];}
    stats<<"],\"query_checksum\":"<<checksum<<"}\n";
    if(!filled || !b || !d || !timing || !query_timing || !stats) throw std::runtime_error("artifact export failed");
    std::cout<<"Saved direction/ROI benchmark: "<<out<<'\n';return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
