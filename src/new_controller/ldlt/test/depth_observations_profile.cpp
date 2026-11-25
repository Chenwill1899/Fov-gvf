// Offline diagnostic only. Production FrameObservationBuilder is unchanged.
// The fused mirror times loop boundaries; the blocked version separates math
// kernels with bounded temporary vectors (never a persisted point cloud).
// Every measured result is checked against the original production builder.
#include "new_controller/depth_observation_io.hpp"
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <iostream>
#include <sched.h>
#include <sstream>
#include <time.h>

using namespace ego4p0;
namespace fs=std::filesystem;
static int64_t clock_ns(clockid_t clock=CLOCK_MONOTONIC) {
  timespec t{};if(clock_gettime(clock,&t)) throw std::runtime_error("clock_gettime failed");
  return t.tv_sec*1000000000LL+t.tv_nsec;
}
enum Stage { RepairFront,RepairLeft,RepairBack,RepairRight,Reset,Transform,Hypot,Alpha,Beta,Bin,
             FusedPixels,Prefix,Scatter,Tiles,Residual,Total,Cpu,Direction,Region,StageCount };
static const char *names[StageCount]={"repair_front_ns","repair_left_ns","repair_back_ns","repair_right_ns",
  "reset_ns","transform_ns","hypot_ns","atan2_alpha_ns","atan2_beta_ns","bin_stage_ns",
  "fused_pixel_ns","prefix_coverage_ns","scatter_ns","tiles_ns","residual_ns","total_ns","thread_cpu_ns",
  "direction_ns","region_ns"};
using Times=std::array<int64_t,StageCount>;

class DiagnosticBuilder {
 public:
  static constexpr uint32_t block_size=512;
  DiagnosticBuilder(const std::array<CameraCalibration,4> &cal,uint32_t w,uint32_t h):cal_(cal),w_(w),height_(h),pixels_(w*h) {
    layout_.configure(1.,1.);cells_.resize(4*pixels_);sources_.resize(4*pixels_);cursor_.resize(layout_.cells());
  }
  template<bool Split> void build(DepthFrame &frame,const PixelRayTables &rays,Times &times) {
    auto &index=frame.observations;auto begin=clock_ns();
    index.ready=false;index.sequence=frame.sequence;index.stamp_ns=frame.t_ns;
    index.source_count=0;index.observed_cells=0;
    index.valid_depth_pixels.fill(0);index.geometry_rejected.fill(0);index.camera_observations.fill(0);index.camera_covered_cells.fill(0);
    std::fill(index.offsets.begin(),index.offsets.end(),0);
    std::fill(index.camera_coverage.begin(),index.camera_coverage.end(),0);
    auto previous=clock_ns();times[Reset]=previous-begin;uint32_t used=0;
    for(uint32_t camera=0;camera<4;++camera) {
      const auto &t=cal_[camera].T_BC;const auto &table=rays.camera(camera);
      if constexpr(!Split) {
        // Same operations, order, branches and floating point expressions as
        // FrameObservationBuilder::build; no per-pixel timestamps.
        for(uint32_t pixel=0;pixel<pixels_;++pixel) {
          const float z=frame.D_filled[camera][pixel];if(!valid_depth(z)) continue;
          ++index.valid_depth_pixels[camera];const auto &b=table[pixel].b;
          const double cx=z*b[0],cy=z*b[1],cz=z*b[2];
          const double x=t[0]*cx+t[1]*cy+t[2]*cz+t[3];
          const double y=t[4]*cx+t[5]*cy+t[6]*cz+t[7];
          const double zz=t[8]*cx+t[9]*cy+t[10]*cz+t[11];
          if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(zz) || (x==0. && y==0. && zz==0.)) {
            ++index.geometry_rejected[camera];continue;
          }
          const double alpha=(x==0. && y==0.) ? 0. : std::atan2(y,x);
          const double beta=std::atan2(zz,std::hypot(x,y));
          const auto cell=layout_.cell(alpha,beta);
          if(!cell) {++index.geometry_rejected[camera];continue;}
          ++index.offsets[*cell+1];index.camera_coverage[*cell]|=static_cast<uint8_t>(1U<<camera);
          cells_[used]=*cell;sources_[used]=camera*pixels_+pixel;++used;++index.camera_observations[camera];
        }
      } else {
        for(uint32_t base=0;base<pixels_;base+=block_size) {
          uint32_t n=0;const uint32_t end=std::min(base+block_size,pixels_);auto start=clock_ns();
          for(uint32_t pixel=base;pixel<end;++pixel) {
            const float z=frame.D_filled[camera][pixel];if(!valid_depth(z)) continue;
            ++index.valid_depth_pixels[camera];const auto &b=table[pixel].b;
            const double cx=z*b[0],cy=z*b[1],cz=z*b[2];
            const double x=t[0]*cx+t[1]*cy+t[2]*cz+t[3];
            const double y=t[4]*cx+t[5]*cy+t[6]*cz+t[7];
            const double zz=t[8]*cx+t[9]*cy+t[10]*cz+t[11];
            if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(zz) || (x==0. && y==0. && zz==0.)) {
              ++index.geometry_rejected[camera];continue;
            }
            x_[n]=x;y_[n]=y;z_[n]=zz;pixel_[n]=pixel;++n;
          }
          auto next=clock_ns();times[Transform]+=next-start;start=next;
          for(uint32_t k=0;k<n;++k) h_[k]=std::hypot(x_[k],y_[k]);
          next=clock_ns();times[Hypot]+=next-start;start=next;
          for(uint32_t k=0;k<n;++k) a_[k]=(x_[k]==0. && y_[k]==0.) ? 0. : std::atan2(y_[k],x_[k]);
          next=clock_ns();times[Alpha]+=next-start;start=next;
          for(uint32_t k=0;k<n;++k) b_[k]=std::atan2(z_[k],h_[k]);
          next=clock_ns();times[Beta]+=next-start;start=next;
          for(uint32_t k=0;k<n;++k) {
            const auto cell=layout_.cell(a_[k],b_[k]);
            if(!cell) {++index.geometry_rejected[camera];continue;}
            ++index.offsets[*cell+1];index.camera_coverage[*cell]|=static_cast<uint8_t>(1U<<camera);
            cells_[used]=*cell;sources_[used]=camera*pixels_+pixel_[k];++used;++index.camera_observations[camera];
          }
          next=clock_ns();times[Bin]+=next-start;
        }
      }
    }
    auto next=clock_ns();if constexpr(!Split) times[FusedPixels]=next-previous;previous=next;
    for(uint32_t cell=0;cell<layout_.cells();++cell) {
      index.offsets[cell+1]+=index.offsets[cell];cursor_[cell]=index.offsets[cell];
      const uint8_t coverage=index.camera_coverage[cell];if(coverage) ++index.observed_cells;
      for(uint32_t camera=0;camera<4;++camera) if(coverage&(1U<<camera)) ++index.camera_covered_cells[camera];
    }
    next=clock_ns();times[Prefix]=next-previous;previous=next;
    for(uint32_t i=0;i<used;++i) index.sources[cursor_[cells_[i]]++]=sources_[i];
    index.source_count=used;
    next=clock_ns();times[Scatter]=next-previous;previous=next;
    for(uint32_t camera=0;camera<4;++camera) {
      for(auto &tile:index.tiles[camera]) tile=DepthTile{};
      for(uint32_t ty=0;ty<index.tile_rows;++ty) for(uint32_t tx=0;tx<index.tile_columns;++tx) {
        auto &tile=index.tiles[camera][ty*index.tile_columns+tx];const uint32_t edge=index.tile_edge;
        const uint32_t y1=std::min((ty+1)*edge,height_),x1=std::min((tx+1)*edge,w_);
        for(uint32_t v=ty*edge;v<y1;++v) for(uint32_t u=tx*edge;u<x1;++u) {
          const float z=frame.D_filled[camera][v*w_+u];if(!valid_depth(z)) continue;
          if(!tile.valid_pixels || z<tile.min_z) tile.min_z=z;
          ++tile.valid_pixels;
        }
      }
    }
    times[Tiles]=clock_ns()-previous;index.ready=true;
  }
 private:
  std::array<CameraCalibration,4> cal_;uint32_t w_,height_,pixels_;DirectionGridLayout layout_;
  std::vector<uint32_t> cells_,sources_,cursor_;
  // At most 512 temporary vectors, reused for each block. Not exported/stored in frames.
  std::array<double,block_size> x_{},y_{},z_{},h_{},a_{},b_{};
  std::array<uint32_t,block_size> pixel_{};
};

static void require_equal(const FrameObservationIndex &a,const FrameObservationIndex &b) {
  if(!a.ready || !b.ready || a.sequence!=b.sequence || a.stamp_ns!=b.stamp_ns || a.offsets!=b.offsets
     || a.source_count!=b.source_count || a.observed_cells!=b.observed_cells || a.camera_coverage!=b.camera_coverage
     || a.valid_depth_pixels!=b.valid_depth_pixels || a.geometry_rejected!=b.geometry_rejected
     || a.camera_observations!=b.camera_observations || a.camera_covered_cells!=b.camera_covered_cells
     || !std::equal(a.sources.begin(),a.sources.begin()+a.source_count,b.sources.begin()))
    throw std::runtime_error("diagnostic direction index differs from production builder");
  for(size_t c=0;c<4;++c) for(size_t k=0;k<a.tiles[c].size();++k) {
    const auto &x=a.tiles[c][k],&y=b.tiles[c][k];
    if(x.valid_pixels!=y.valid_pixels || std::memcmp(&x.min_z,&y.min_z,sizeof(float)))
      throw std::runtime_error("diagnostic tile differs from production builder");
  }
}

int main(int argc,char **argv) {
  try {
    // This copied-loop diagnostic describes V1 pixel sources. Preserve its
    // frozen executed binary/header bundle; do not compare it to V2 block means.
    if(FrameObservationIndex{}.block_edge!=1) throw std::runtime_error(
      "V1 per-pixel profiler requires its frozen V1 executable/headers in depth_index_profile_20261103_131114; use depth_observations_benchmark for current block observations");
    if(argc!=4 && argc!=5) throw std::runtime_error("usage: depth_observations_profile ARCHIVED_RUN first|last NEW_OUTPUT_DIR [ITERATIONS]");
    const fs::path source=argv[1],out=argv[3];const std::string prefix=argv[2];
    const int iterations=argc==5 ? std::stoi(argv[4]) : 200;
    if((prefix!="first" && prefix!="last") || iterations<10 || iterations>10000) throw std::runtime_error("invalid snapshot or iteration count");
    if(!fs::create_directory(out)) throw std::runtime_error("output directory must be new");
    const auto cal=YAML::LoadFile((source/"simulation.sensors.json").string()),meta=YAML::LoadFile((source/"reader_meta.json").string());
    if(cal["depth_type"].as<std::string>()!="optical_axis_z_m") throw std::runtime_error("optical Z required");
    const uint32_t w=cal["render_width"].as<uint32_t>(),h=cal["render_height"].as<uint32_t>();const size_t pixels=depth_pixel_count(w,h);
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
    if(!frame.sequence) throw std::runtime_error("snapshot sequence missing");
    const auto raw=source/(prefix+"_depth.bin");if(fs::file_size(raw)!=4*pixels*sizeof(float)) throw std::runtime_error("raw image shape mismatch");
    std::ifstream input(raw,std::ios::binary);for(auto &image:frame.D) input.read(reinterpret_cast<char*>(image.data()),pixels*sizeof(float));
    if(!input) throw std::runtime_error("raw read failed");
    PixelRayTables rays;rays.configure(cameras,w,h);FrameObservationBuilder original;original.configure_debug(cameras,w,h,1,1,16,1);original.allocate(frame);
    DiagnosticBuilder diagnostic(cameras,w,h);
    for(size_t c=0;c<4;++c) fill_invalid_depth(frame.D[c].data(),frame.D_filled[c].data(),w,h);
    original.build(frame,rays);const FrameObservationIndex expected=frame.observations;
    struct Row {int iteration,mode;Times times;};std::vector<Row> rows;rows.reserve(iterations*3);
    for(int iteration=-10;iteration<iterations;++iteration) for(int order=0;order<3;++order) {
      const int mode=((iteration+12)%3+order)%3;Times times{};
      const auto cpu=clock_ns(CLOCK_THREAD_CPUTIME_ID),start=clock_ns();auto previous=start;
      for(size_t c=0;c<4;++c) {
        frame.repair[c]=fill_invalid_depth(frame.D[c].data(),frame.D_filled[c].data(),w,h);
        const auto next=clock_ns();times[RepairFront+c]=next-previous;previous=next;
      }
      if(mode==0) original.build(frame,rays);
      else if(mode==1) diagnostic.build<false>(frame,rays,times);
      else diagnostic.build<true>(frame,rays,times);
      times[Total]=clock_ns()-start;times[Cpu]=clock_ns(CLOCK_THREAD_CPUTIME_ID)-cpu;
      if(mode==0) {
        times[Direction]=frame.observations.direction_build_ns;times[Region]=frame.observations.region_build_ns;
        times[Residual]=times[Total]-times[Direction]-times[Region];
        for(size_t c=0;c<4;++c) times[Residual]-=times[RepairFront+c];
      } else {
        times[Residual]=times[Total];for(int s=0;s<Residual;++s) times[Residual]-=times[s];
      }
      // Validation and recording are OUTSIDE the measured intervals, for every run.
      require_equal(expected,frame.observations);
      if(iteration>=0) rows.push_back({iteration,mode,times});
    }
    // Restore production metadata before exporting standard independent-check artifacts.
    original.build(frame,rays);save_observation_index(out,"sample",frame);
    fs::copy_file(raw,out/"raw_depth.bin");fs::copy_file(source/"simulation.sensors.json",out/"simulation.sensors.json");
    std::ofstream filled(out/"filled_depth.bin",std::ios::binary),b(out/"pixel_rays_b.bin",std::ios::binary),d(out/"pixel_rays_d.bin",std::ios::binary);
    for(size_t c=0;c<4;++c) {
      filled.write(reinterpret_cast<const char*>(frame.D_filled[c].data()),pixels*sizeof(float));
      for(const auto &ray:rays.camera(c)) {b.write(reinterpret_cast<const char*>(ray.b.data()),3*sizeof(double));d.write(reinterpret_cast<const char*>(ray.d.data()),3*sizeof(double));}
    }
    const char *modes[]={"original","fused_diagnostic","blocked_diagnostic"};
    std::ofstream csv(out/"stage_timings.csv");csv<<"iteration,mode";for(auto name:names) csv<<','<<name;csv<<'\n';
    for(const auto &row:rows) {csv<<row.iteration<<','<<modes[row.mode];for(auto ns:row.times) csv<<','<<ns;csv<<'\n';}
    sched_param policy{};sched_getparam(0,&policy);
    std::ofstream provenance(out/"provenance.json");provenance<<"{\"archived_run\":\""<<source.string()<<"\",\"snapshot\":\""<<prefix<<"\",\"sequence\":"<<frame.sequence
      <<",\"stamp_ns\":"<<frame.t_ns<<",\"iterations_per_mode\":"<<iterations<<",\"warmups_per_mode\":10,\"modes_rotate_each_iteration\":true,\"cpu\":"<<sched_getcpu()
      <<",\"scheduler\":"<<sched_getscheduler(0)<<",\"priority\":"<<policy.sched_priority<<",\"block_size\":"<<DiagnosticBuilder::block_size
      <<",\"valid_sources\":"<<frame.observations.source_count<<",\"observed_cells\":"<<frame.observations.observed_cells
      <<",\"all_results_equal_to_production\":true,\"comparisons\":"<<3*(iterations+10)<<",\"scope\":\"offline cached archived pixels, no new Isaac or FIFO timing; blocked stage attribution changes loop layout\"}\n";
    if(!csv || !provenance || !filled || !b || !d) throw std::runtime_error("artifact export failed");
    std::cout<<"Verified "<<3*(iterations+10)<<" builds against production; saved "<<out<<'\n';return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
