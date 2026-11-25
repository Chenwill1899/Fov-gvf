#include "new_controller/depth_observations.hpp"
#include <cstring>
#include <iostream>
#include <numeric>
#include <random>
using namespace ego4p0;
static void require(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
static std::array<CameraCalibration,4> cameras() {
  std::array<CameraCalibration,4> cal;
  for(auto &c:cal) {c.K={2,.3,1,0,3,.5,0,0,1};c.T_BC={1,0,0,.2,0,1,0,-.1,0,0,1,.05,0,0,0,1};}
  return cal;
}
static void equal(const FrameObservationIndex &a,const FrameObservationIndex &b) {
  require(a.offsets==b.offsets && a.camera_coverage==b.camera_coverage && a.source_count==b.source_count
          && a.observed_cells==b.observed_cells && a.valid_depth_pixels==b.valid_depth_pixels
          && a.usable_blocks==b.usable_blocks && a.geometry_rejected==b.geometry_rejected
          && a.camera_observations==b.camera_observations && a.camera_covered_cells==b.camera_covered_cells
          && std::equal(a.sources.begin(),a.sources.begin()+a.source_count,b.sources.begin()),"serial/parallel index differs");
  for(size_t c=0;c<4;++c) {
    for(size_t k=0;k<a.blocks[c].size();++k) require(a.blocks[c][k].valid_pixels==b.blocks[c][k].valid_pixels
      && std::memcmp(&a.blocks[c][k].mean_z,&b.blocks[c][k].mean_z,sizeof(float))==0,"block mean changed with threading/angle path");
    for(size_t k=0;k<a.tiles[c].size();++k) require(a.tiles[c][k].valid_pixels==b.tiles[c][k].valid_pixels
      && std::memcmp(&a.tiles[c][k].min_z,&b.tiles[c][k].min_z,sizeof(float))==0,"full-resolution ROI summary changed");
  }
}
int main() {
  const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  std::mt19937_64 rng(84135);std::uniform_real_distribution<double> uniform(-1.,1.);
  uint64_t angle_comparisons=0,exact_fallbacks=0;
  for(const auto &resolution:std::array<std::array<double,2>,5>{{{1,1},{7,7},{.5,2},{360,180},{137,70}}}) {
    DirectionGridLayout layout;layout.configure(resolution[0],resolution[1]);DirectionBoundaries boundaries;boundaries.configure(layout);
    auto check=[&](double x,double y,double z,uint32_t hint) {
      bool exact=false;const auto fast=boundaries.cell(x,y,z,hint,exact),reference=boundaries.exact(x,y,z);
      require(fast==reference,"boundary lookup differs from exact atan2 grid");++angle_comparisons;exact_fallbacks+=exact;
    };
    for(int k=0;k<200000;++k) {
      const double scale=std::pow(10.,static_cast<int>(rng()%601)-300);
      check(uniform(rng)*scale,uniform(rng)*scale,uniform(rng)*scale,rng()%layout.cells());
    }
    for(uint32_t a=0;a<=layout.azimuth_bins;++a) for(double delta:{-1e-8,-1e-12,0.,1e-12,1e-8}) {
      const double alpha=std::min(-direction_pi+a*layout.azimuth_step,direction_pi)+delta;
      check(std::cos(alpha),std::sin(alpha),.37,0);
    }
    for(uint32_t b=0;b<=layout.elevation_bins;++b) for(double delta:{-1e-8,-1e-12,0.,1e-12,1e-8}) {
      const double beta=std::min(-direction_pi/2+b*layout.elevation_step,direction_pi/2)+delta;
      check(std::cos(beta)*.8,std::cos(beta)*.6,std::sin(beta),layout.cells()-1);
    }
    for(double zero:{0.,-0.}) {check(-1.,zero,0.,0);check(zero,zero,1.,0);check(zero,zero,-1.,0);}
    check(1e308,1e308,1e308,0);check(1e-320,-1e-320,1e-320,0);check(0,0,0,0);check(nan,1,1,0);check(inf,1,1,0);
  }
  require(exact_fallbacks>0,"near-boundary fallback not exercised");
  {
    auto cal=cameras();const double co=std::cos(.37),si=std::sin(.37);
    for(auto &c:cal) {c.T_BC[0]=co;c.T_BC[1]=-si;c.T_BC[4]=si;c.T_BC[5]=co;}
    PixelRayTables rays;rays.configure(cal,5,4);FrameObservationBuilder builder;builder.configure_debug(cal,5,4);
    DepthFrame f;f.allocate(20);builder.allocate(f);f.sequence=1;f.t_ns=2;
    for(size_t c=0;c<4;++c) for(size_t k=0;k<20;++k) f.D_filled[c][k]=static_cast<float>(k+1+c);
    f.D_filled[0][0]=nan;f.D_filled[0][1]=inf;f.D_filled[0][2]=0;f.D_filled[0][5]=-1;
    for(auto &z:f.D_filled[1]) z=nan;
    const auto before=f.D_filled;require(builder.build(f,rays),"first group not built");
    require(f.observations.block_columns==2 && f.observations.block_rows==2 && f.observations.blocks_per_camera==4,"partial block layout");
    for(uint32_t cell=0;cell<builder.layout().cells();++cell) for(uint32_t k=0;k<f.observations.offsets[cell+1]-f.observations.offsets[cell];++k) {
      const auto o=*DepthObservationView(f).observation(cell,k);double sum=0.;uint32_t count=0;
      for(uint32_t v=o.v0;v<o.v1;++v) for(uint32_t u=o.u0;u<o.u1;++u) {const float z=before[o.camera][v*5+u];if(std::isfinite(z) && z>0) {sum+=z;++count;}}
      require(count && o.optical_z==static_cast<float>(sum/count) && o.valid_pixels==count && o.total_pixels==(o.u1-o.u0)*(o.v1-o.v0),"block mean/provenance wrong");
      require(o.u==(o.u0+o.u1-1)*.5 && o.v==(o.v0+o.v1-1)*.5,"partial block center not half-pixel");
      const double by=(o.v-.5)/3,bx=(o.u-1-.3*by)/2,z=o.optical_z;
      const double x=z*(co*bx-si*by)+.2,y=z*(si*bx+co*by)-.1,zz=z+.05;
      require(builder.layout().cell(std::atan2(y,x),std::atan2(zz,std::hypot(x,y)))==cell,"precomputed R*b or camera translation wrong");
    }
    for(size_t c=0;c<4;++c) require(std::memcmp(before[c].data(),f.D_filled[c].data(),20*sizeof(float))==0,"compression modified full-resolution image");
    require(f.observations.camera_observations[1]==0 && f.observations.usable_blocks[1]==0,"all-invalid blocks fabricated");
    require(!builder.configure_debug(cal,5,4) && builder.table_builds()==1,"identical calibration regenerated rotated rays/workers");
    for(int k=0;k<1000;++k) require(!builder.build(f,rays),"same group rebuilt");
    require(builder.builds()==1 && builder.reused_groups()==1000,"reuse counters wrong");
    ++f.sequence;require(builder.build(f,rays),"new sequence not built");++f.t_ns;require(builder.build(f,rays),"new stamp not built");
    for(int cpu:builder.worker_cpus()) for(int forbidden:cpu_siblings(5)) require(cpu!=forbidden,"camera worker on reader CPU/SMT sibling");
    require(std::all_of(builder.worker_affinity_errors().begin(),builder.worker_affinity_errors().end(),[](int x){return !x;})
      && std::all_of(builder.worker_scheduler_errors().begin(),builder.worker_scheduler_errors().end(),[](int x){return !x;}),"worker scheduling setup failed");
    cal[0].T_BC[3]+=.1;require(builder.configure_debug(cal,5,4) && builder.table_builds()==2 && builder.build(f,rays),"changed extrinsics reused stale rays/index");
  }
  {
    auto cal=cameras();PixelRayTables rays;rays.configure(cal,3,3);FrameObservationBuilder builder;builder.configure_debug(cal,3,3);
    DepthFrame f;f.allocate(9);builder.allocate(f);f.sequence=1;for(size_t c=0;c<4;++c) {std::fill(f.D_filled[c].begin(),f.D_filled[c].end(),10.f);f.D_filled[c][0]=1;f.D[c]=f.D_filled[c];}
    builder.build(f,rays);require(f.observations.blocks[0][0].mean_z==9.f && DepthObservationView(f).region(0,{0,0,3,3}).z==1.f && f.D[0][0]==1.f,"full-resolution minimum replaced by block mean");
  }
  {
    auto cal=cameras();PixelRayTables rays;rays.configure(cal,320,240);
    FrameObservationBuilder serial,parallel;serial.configure_debug(cal,320,240,1,1,16,3,5,false,false);parallel.configure_debug(cal,320,240);
    DepthFrame a,b;a.allocate(76800);b.allocate(76800);serial.allocate(a);parallel.allocate(b);
    require(a.observations.blocks_per_camera==8560 && a.observations.sources.size()==34240,"320x240 block count");
    for(size_t c=0;c<4;++c) for(size_t k=0;k<76800;++k) a.D_filled[c][k]=b.D_filled[c][k]=k%19 ? static_cast<float>(1+(rng()%10000)*.01) : nan;
    for(uint64_t seq=1;seq<=20;++seq) {a.sequence=b.sequence=seq;a.t_ns=b.t_ns=seq*2;serial.build(a,rays);parallel.build(b,rays);equal(a.observations,b.observations);}
    require(b.observations.source_count==34240 && std::accumulate(b.observations.fast_angle_observations.begin(),b.observations.fast_angle_observations.end(),0U)>0,"parallel boundary path unused");
  }
  std::cout<<"PASS: "<<angle_comparisons<<" exact-vs-boundary directions (fallbacks "<<exact_fallbacks
    <<"), partial/all-invalid/mixed blocks, block-center R*b with skew/rotation/translation, full-resolution ROI retained, same-group reuse/calibration cache, CPU/SMT exclusion, serial-exact vs parallel-boundary equality\n";
}
