#include "new_controller/depth_size_query.hpp"
#include <iostream>
#include <random>
using namespace ego4p0;
static void require(bool b,const char *m) {if(!b) throw std::runtime_error(m);}
int main() {try {
  std::mt19937 rng(79231);auto uniform=[&](){return double(rng())/rng.max();};uint64_t samples=0;
  constexpr uint32_t w=13,h=11;std::array<CameraCalibration,4> cal;
  for(uint32_t trial=0;trial<24;++trial) {
    for(auto &c:cal) {
      const double a=uniform()*2*direction_pi,b=(uniform()-.5)*direction_pi,co=std::cos(a),si=std::sin(a),cb=std::cos(b),sb=std::sin(b);
      c.K={12,.3,w*.5,0,13,h*.5,0,0,1};
      c.T_BC={co*cb,-si,co*sb,(uniform()-.5)*.5,si*cb,co,si*sb,(uniform()-.5)*.5,-sb,0,cb,(uniform()-.5)*.5,0,0,0,1};
    }
    const double resolution=trial%3==0 ? 180 : trial%3==1 ? 17 : 1;
    PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder builder;
    builder.configure_real(cal,w,h,resolution,std::min(resolution,90.),16,3,5,false,true,true,true);
    DepthFrame f;f.allocate(w*h);builder.allocate(f);f.sequence=trial+1;f.t_ns=f.sequence*2000000;
    for(auto &image:f.D) for(auto &z:image) z=uniform()<.25 ? std::numeric_limits<float>::quiet_NaN() : float(.01+uniform()*10);
    builder.process(f,rays);const auto &layout=builder.layout();const auto &geo=builder.local_geometry();std::vector<uint8_t> expected(layout.cells());
    for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<f.observations.blocks_per_camera;++id) {
      const auto &depth=f.observations.blocks[c][id];const auto &g=geo.block(c,id);const auto box=block_exclusion(depth,g,cal[c].T_BC,layout);
      if(!box.present) continue;
      for(uint32_t cell=0;cell<layout.cells();++cell) if(box.contains(cell,layout)) expected[cell]|=uint8_t(1U<<c);
      const uint32_t u0=id%f.observations.block_columns*3,v0=id/f.observations.block_columns*3,u1=std::min(u0+3,w),v1=std::min(v0+3,h);
      const auto inv=PixelRayTables::inverse_intrinsics(cal[c].K);
      for(uint32_t k=0;k<120;++k) {
        const double u=double(u0)-.5+(u1-u0)*uniform(),v=double(v0)-.5+(v1-v0)*uniform();
        const double z=depth.min_z+(std::min(double(depth.max_z),g.max_limit)-depth.min_z)*uniform();
        std::array<double,3> optical{},point{};
        for(uint32_t a=0;a<3;++a) optical[a]=inv[a*3]*u+inv[a*3+1]*v+inv[a*3+2];
        const double polar=2*uniform()-1,az=uniform()*2*direction_pi,xy=std::sqrt(1-polar*polar);
        const std::array<double,3> offset{.1*xy*std::cos(az),.1*xy*std::sin(az),.1*polar};
        for(uint32_t a=0;a<3;++a) {point[a]=cal[c].T_BC[a*4+3]+offset[a];for(uint32_t q=0;q<3;++q) point[a]+=z*cal[c].T_BC[a*4+q]*optical[q];}
        const auto cell=layout.cell(std::atan2(point[1],point[0]),std::atan2(point[2],std::hypot(point[0],point[1])));
        require(cell && box.contains(*cell,layout),"block inflated frustum escaped spherical exclusion/angular wrap");++samples;
      }
    }
    require(expected==f.local.excluded_camera_mask,"2D difference/prefix mask differs from direct rectangle union");
    for(uint32_t cell=0;cell<layout.cells();++cell) require(LocalFieldView(f,geo).cell(cell)->status!=CandidateStatus::Available,"remaining grid silently became free");
    for(auto &image:f.D) std::fill(image.begin(),image.end(),10.f);
    ++f.sequence;builder.process(f,rays);
    require(f.local.excluded_cells==0,"prior group exclusion retained");
    const auto total=f.observations.blocks_per_camera*4;uint32_t complete=0;for(auto n:f.local.complete_far_blocks) complete+=n;
    require(complete==total,"complete original far block not classified as observation evidence");
    for(auto &image:f.D) std::fill(image.begin(),image.end(),std::numeric_limits<float>::quiet_NaN());
    ++f.sequence;builder.process(f,rays);uint32_t unknown=0;for(auto n:f.local.unknown_blocks) unknown+=n;
    require(f.local.excluded_cells==0 && unknown==total,"unresolved image blocks not kept unknown");
  }
  // Explicit proof of availability uses actual projection and ORIGINAL pixels.
  for(uint32_t c=0;c<4;++c) {const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);cal[c].K={30,0,32,0,30,24,0,0,1};cal[c].near_clip_z_m=.05;
    cal[c].T_BC={si,0,co,.22*co,-co,0,si,.22*si,0,-1,0,0,0,0,0,1};}
  PixelRayTables rays;rays.configure(cal,65,49);FrameObservationBuilder builder;builder.configure_real(cal,65,49,1,1,16,3,5,false,true,true,true);
  DepthFrame f;f.allocate(65*49);builder.allocate(f);for(auto &image:f.D) std::fill(image.begin(),image.end(),10.f);f.sequence=1;f.t_ns=1;builder.process(f,rays);
  LocalFieldView view(f,builder.local_geometry());require(view.candidate({1,0,0})==CandidateStatus::Available,"complete raw sphere not available");
  require(view.candidate({.05,0,0})==CandidateStatus::Unknown && view.candidate({0,0,0})==CandidateStatus::Unknown,"blind/current pose granted availability");
  const auto projection=builder.local_geometry().project(0,{1,0,0});const uint32_t p=uint32_t((projection.region.v0+projection.region.v1)/2)*65+uint32_t((projection.region.u0+projection.region.u1)/2);
  f.D[0][p]=std::numeric_limits<float>::quiet_NaN();++f.sequence;builder.process(f,rays);
  require(view.candidate({1,0,0})==CandidateStatus::Unknown,"filled raw hole certified candidate available");
  f.D[0][p]=.3f;++f.sequence;builder.process(f,rays);require(view.candidate({1,0,0})==CandidateStatus::Unavailable,"thin near block not excluded");
  std::cout<<"PASS: "<<samples<<" inflated block-frustum samples, seam/poles/coarse cells, exact rectangle-union parity, stale reset, raw far/holes, actual geometry availability/unknown\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
