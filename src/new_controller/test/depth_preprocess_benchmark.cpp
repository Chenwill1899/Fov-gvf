#include "new_controller/depth_preprocess.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sched.h>
#include <time.h>

using namespace ego4p0;
namespace fs=std::filesystem;
static int64_t mono_ns() { timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000000LL+t.tv_nsec; }
static void stats(std::ostream &s,std::vector<int64_t> values) {
  std::sort(values.begin(),values.end());double sum=0;
  for(auto v:values) sum+=v;
  auto percentile=[&](double p) {double x=p*(values.size()-1);size_t i=static_cast<size_t>(x);return values[i]+(x-i)*(values[std::min(i+1,values.size()-1)]-values[i]);};
  s<<"{\"count\":"<<values.size()<<",\"mean_ms\":"<<sum/values.size()/1e6
   <<",\"p50_ms\":"<<percentile(.5)/1e6<<",\"p99_ms\":"<<percentile(.99)/1e6<<",\"max_ms\":"<<values.back()/1e6<<'}';
}
int main(int argc,char **argv) {
  try {
    if(argc!=4) throw std::runtime_error("usage: depth_preprocess_benchmark raw_depth.bin simulation.sensors.json NEW_OUTPUT_DIR");
    const fs::path raw=argv[1],calibration=argv[2],out=argv[3];
    if(!fs::create_directory(out)) throw std::runtime_error("output directory must be new");
    const auto c=YAML::LoadFile(calibration.string());
    if(c["depth_type"].as<std::string>()!="optical_axis_z_m") throw std::runtime_error("optical-axis meter depth required");
    const size_t width=c["render_width"].as<size_t>(),height=c["render_height"].as<size_t>(),pixels=depth_pixel_count(width,height);
    if(c["cameras"].size()!=4 || fs::file_size(raw)!=4*pixels*sizeof(float)) throw std::runtime_error("wrong four-camera raw shape");
    std::array<CameraCalibration,4> cal;
    for(size_t i=0;i<4;++i) {
      auto camera=c["cameras"][i];
      cal[i].K={camera["fx"].as<double>(),0,camera["cx"].as<double>(),0,camera["fy"].as<double>(),camera["cy"].as<double>(),0,0,1};
    }
    DepthFrame frame;frame.allocate(pixels);std::ifstream file(raw,std::ios::binary);
    for(auto &image:frame.D) file.read(reinterpret_cast<char*>(image.data()),pixels*sizeof(float));
    if(!file) throw std::runtime_error("raw read failed");
    PixelRayTables tables;const auto begin=mono_ns();tables.configure(cal,width,height);const auto lut_ns=mono_ns()-begin;
    std::vector<int64_t> whole(1000);std::array<std::vector<int64_t>,4> per_camera;
    for(auto &t:per_camera) t.resize(1000);
    for(int round=-50;round<1000;++round) {
      const auto start=mono_ns();
      for(size_t i=0;i<4;++i) {
        const auto t=mono_ns();frame.repair[i]=fill_invalid_depth(frame.D[i].data(),frame.D_filled[i].data(),width,height);
        if(round>=0) per_camera[i][round]=mono_ns()-t;
      }
      if(round>=0) whole[round]=mono_ns()-start;
    }
    fs::copy_file(raw,out/"raw_depth.bin");fs::copy_file(calibration,out/"simulation.sensors.json");
    std::ofstream repaired(out/"filled_depth.bin",std::ios::binary),b(out/"pixel_rays_b.bin",std::ios::binary),d(out/"pixel_rays_d.bin",std::ios::binary);
    for(size_t i=0;i<4;++i) {
      repaired.write(reinterpret_cast<const char*>(frame.D_filled[i].data()),pixels*sizeof(float));
      for(const auto &ray:tables.camera(i)) {
        b.write(reinterpret_cast<const char*>(ray.b.data()),3*sizeof(double));d.write(reinterpret_cast<const char*>(ray.d.data()),3*sizeof(double));
      }
    }
    if(!repaired || !b || !d) throw std::runtime_error("artifact export failed");
    std::ofstream summary(out/"benchmark.json");summary<<std::setprecision(12);
    sched_param param{};sched_getparam(0,&param);
    summary<<"{\"scope\":\"offline archived Isaac pixels; 50 warmups plus 1000 cached repetitions; not a simulation/FIFO deadline test\",\"cpu\":"<<sched_getcpu()
      <<",\"policy\":"<<sched_getscheduler(0)<<",\"priority\":"<<param.sched_priority<<",\"width\":"<<width<<",\"height\":"<<height
      <<",\"ray_build_ms\":"<<lut_ns/1e6<<",\"ray_builds\":"<<tables.builds()<<",\"ray_bytes\":"<<tables.bytes()<<",\"four_camera_repair\":";
    stats(summary,whole);summary<<",\"cameras\":[";
    for(size_t i=0;i<4;++i) {
      if(i) summary<<',';
      summary<<"{\"invalid\":"<<frame.repair[i].invalid<<",\"filled\":"<<frame.repair[i].filled<<",\"unresolved\":"<<frame.repair[i].unresolved<<",\"repair\":";stats(summary,per_camera[i]);summary<<'}';
    }
    summary<<"]}\n";
    std::ofstream timing(out/"timings.csv");timing<<"iteration,four_camera_ms,camera1_ms,camera2_ms,camera3_ms,camera4_ms\n"<<std::setprecision(12);
    for(size_t i=0;i<whole.size();++i) {
      timing<<i<<','<<whole[i]/1e6;
      for(size_t camera=0;camera<4;++camera) timing<<','<<per_camera[camera][i]/1e6;
      timing<<'\n';
    }
    std::cout<<"Saved offline four-camera preprocessing benchmark: "<<out<<'\n';
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 2; }
}
