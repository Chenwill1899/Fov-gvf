#include "new_controller/depth_size_query.hpp"
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <random>
using namespace ego4p0;
static std::atomic<bool> counting{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {if(counting) ++allocations;if(void *p=std::malloc(n?n:1)) return p;throw std::bad_alloc();}
void operator delete(void *p) noexcept {std::free(p);}void operator delete(void *p,std::size_t) noexcept {std::free(p);}
void *operator new[](std::size_t n){return ::operator new(n);}void operator delete[](void *p) noexcept {::operator delete(p);}void operator delete[](void *p,std::size_t) noexcept {::operator delete(p);}
static void require(bool b,const char *m){if(!b) throw std::runtime_error(m);}
static std::array<CameraCalibration,4> cameras(uint32_t w,uint32_t h) {
  std::array<CameraCalibration,4> cal;
  for(uint32_t c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);cal[c].K={.6*w,.2,w*.5,0,.65*w,h*.5,0,0,1};
    cal[c].T_BC={si,0,co,.22*co,-co,0,si,.22*si,0,-1,0,.02,0,0,0,1};}
  return cal;
}
int main(){try{
  std::mt19937 rng(733);uint64_t points=0,queries=0;
  for(const auto shape:std::array<std::array<uint32_t,2>,3>{{{5,4},{35,29},{320,240}}}){
    const uint32_t w=shape[0],h=shape[1],n=w*h;auto cal=cameras(w,h);PixelRayTables rays;rays.configure(cal,w,h);
    FrameObservationBuilder parallel,serial;parallel.configure(cal,w,h,1,1,16,3,5,true,true,true,true);serial.configure(cal,w,h,1,1,16,3,5,false,true,true,true);
    DepthFrame f,ref;for(auto *p:{&f,&ref})p->allocate(n);parallel.allocate(f);serial.allocate(ref);
    require(parallel.virtual_center() && parallel.local_geometry().virtual_center(),"production default is not virtual");
    const auto &table=parallel.local_geometry().virtual_tables();
    for(uint32_t c=0;c<4;++c)for(uint32_t p=0;p<n;++p){const auto &b=rays.at(c,p%w,p/w).b;require(std::abs(table.coefficient(c,p)-std::hypot(b[0],b[1],b[2]))<1e-12,"rho scale differs from norm(K^-1 pixel)");
      f.D[c][p]=ref.D[c][p]=rng()%11==0 ? std::numeric_limits<float>::quiet_NaN() : float(.05+(rng()%10000)*.001);}
    f.sequence=ref.sequence=1;f.t_ns=ref.t_ns=2000000;parallel.process(f,rays);serial.process(ref,rays);
    require(f.local.node_state==ref.local.node_state && f.local.excluded_camera_mask==ref.local.excluded_camera_mask && f.local.complete_cameras==ref.local.complete_cameras,"serial/parallel panorama differs");
    require(std::memcmp(f.local.min_range.data(),ref.local.min_range.data(),f.local.min_range.size()*sizeof(double))==0,"serial/parallel rho differs");
    const auto &layout=parallel.layout();LocalFieldView view(f,parallel.local_geometry());
    for(uint32_t c=0;c<4;++c)for(uint32_t id=0;id<f.observations.blocks_per_camera;++id){const auto &b=f.observations.blocks[c][id];
      const uint32_t u0=id%f.observations.block_columns*3,v0=id/f.observations.block_columns*3;
      double minimum=std::numeric_limits<double>::infinity();
      for(uint32_t v=v0;v<std::min(v0+3,h);++v)for(uint32_t u=u0;u<std::min(u0+3,w);++u){const uint32_t p=v*w+u;const float z=f.D_filled[c][p];if(!valid_depth(z))continue;
        const double rho=z*table.coefficient(c,p);minimum=std::min(minimum,rho);if(rho>ControllerLimits::local_range_m)continue;
        const auto &q=rays.at(c,u,v).b;const auto &t=cal[c].T_BC;std::array<double,3> direction{};
        for(uint32_t k=0;k<3;++k)direction[k]=t[k*4]*q[0]+t[k*4+1]*q[1]+t[k*4+2]*q[2];
        const double norm=std::hypot(direction[0],direction[1],direction[2]);
        for(uint32_t j=0;j<7;++j){std::array<double,3> center{};for(uint32_t k=0;k<3;++k)center[k]=rho*direction[k]/norm;
          if(j)center[(j-1)/2]+=(j%2?-.1:.1);
          if(std::hypot(center[0],center[1],center[2])<1e-12)continue;
          const auto cell=*layout.cell(std::atan2(center[1],center[0]),std::atan2(center[2],std::hypot(center[0],center[1])));
          require(f.local.excluded_camera_mask[cell]&(1U<<c),"near virtual surface/body-radius escaped distance-bin template");++points;}
      }
      require((!std::isfinite(minimum)&&!valid_range(b.min_rho)) || b.min_rho==minimum,"block min rho incorrect");
      require(!b.valid_pixels || b.direction_cell==table.block_cell(c,id),"direction depends on current depth");
    }
    for(uint32_t trial=0;trial<300;++trial){const uint32_t a0=rng()%layout.azimuth_bins,a1=std::min<uint32_t>(layout.azimuth_bins,a0+1+rng()%9),b0=rng()%layout.elevation_bins,b1=std::min<uint32_t>(layout.elevation_bins,b0+1+rng()%9);const auto q=view.panorama_region({a0,a1,b0,b1});uint32_t missing=0;std::optional<double> minimum;
      for(uint32_t b=b0;b<b1;++b)for(uint32_t a=a0;a<a1;++a){const uint32_t cell=b*layout.azimuth_bins+a;missing+=!f.local.complete_cameras[cell];const double rho=f.local.min_range[cell];if(valid_range(rho)&&(!minimum||rho<*minimum))minimum=rho;}
      require(q.minimum==minimum && q.missing==missing && q.total==(a1-a0)*(b1-b0),"panorama tile/prefix query not exact");++queries;}
    // Translations/physical near planes can change arbitrarily without changing virtual outputs.
    auto moved=cal;for(auto &c:moved){c.T_BC[3]+=10.;c.T_BC[7]-=4.;c.T_BC[11]+=2.;c.near_clip_z_m=.8;}
    FrameObservationBuilder other;other.configure(moved,w,h,1,1,16,3,5,false,true,true,true);DepthFrame shifted;shifted.allocate(n);other.allocate(shifted);shifted.D=f.D;shifted.sequence=f.sequence;shifted.t_ns=f.t_ns;other.process(shifted,rays);
    require(shifted.local.node_state==f.local.node_state && shifted.local.excluded_camera_mask==f.local.excluded_camera_mask && std::memcmp(shifted.local.min_range.data(),f.local.min_range.data(),f.local.min_range.size()*sizeof(double))==0,"physical translation/near clip leaked into virtual planning");
    require(!view.block_ray(0,0),"virtual planner returned a physical optical ray API");
    counting=true;allocations=0;
    for(uint32_t k=0;k<20;++k){for(auto &d:f.D)std::fill(d.begin(),d.end(),10.f);++f.sequence;parallel.process(f,rays);require(!parallel.process(f,rays),"same group rebuilt");(void)LocalFieldView(f,parallel.local_geometry()).candidate({1,0,0});}
    counting=false;require(allocations==0 && f.local.excluded_cells==0,"steady virtual path allocated or retained stale exclusion");
  }
  // Tilted views, seam crossings, poles, non-divisor and very coarse layouts.
  for(uint32_t trial=0;trial<12;++trial) {
    auto cal=cameras(9,7);const double yaw=trial*direction_pi/6,tilt=(double(trial%5)-2)*direction_pi/4;
    for(auto &c:cal) {const double cy=std::cos(yaw),sy=std::sin(yaw),ct=std::cos(tilt),st=std::sin(tilt);
      c.T_BC={cy*ct,-sy,cy*st,.4,sy*ct,cy,sy*st,-.3,-st,0,ct,.2,0,0,0,1};}
    const double resolution=trial%3==0 ? 180 : trial%3==1 ? 7 : 1;
    FrameObservationBuilder b;b.configure(cal,9,7,resolution,std::min(resolution,90.),16,3,5,false,true,true,true);
    PixelRayTables rays;rays.configure(cal,9,7);DepthFrame f;f.allocate(63);b.allocate(f);f.sequence=1;f.t_ns=1;
    for(auto &d:f.D)std::fill(d.begin(),d.end(),.8f);
    b.process(f,rays);
    for(uint32_t c=0;c<4;++c)for(uint32_t p=0;p<63;++p) {
      const auto &ray=b.local_geometry().pixel(c,p);const double rho=.8f*b.local_geometry().virtual_tables().coefficient(c,p),norm=std::hypot(ray.q[0],ray.q[1],ray.q[2]);
      for(uint32_t j=0;j<40;++j){const double z=-1.+2.*double(rng())/rng.max(),az=2*direction_pi*double(rng())/rng.max(),xy=std::sqrt(1-z*z);
        const std::array<double,3> point{rho*ray.q[0]/norm+.1*xy*std::cos(az),rho*ray.q[1]/norm+.1*xy*std::sin(az),rho*ray.q[2]/norm+.1*z};
        const auto cell=*b.layout().cell(std::atan2(point[1],point[0]),std::atan2(point[2],std::hypot(point[0],point[1])));
        require(f.local.excluded_camera_mask[cell]&(1U<<c),"tilted/seam/pole/coarse virtual template misses radius sample");++points;
      }
    }
  }
  constexpr uint32_t w=65,h=49;auto cal=cameras(w,h);PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder builder;builder.configure(cal,w,h,1,1,16,3,5,false,true,true,true);DepthFrame f;f.allocate(w*h);builder.allocate(f);for(auto &d:f.D)std::fill(d.begin(),d.end(),10.f);f.sequence=1;f.t_ns=1;builder.process(f,rays);
  LocalFieldView view(f,builder.local_geometry());require(view.candidate({1,0,0})==CandidateStatus::Available,"missing physical clipping plane still blocks virtual center");require(f.local.available_cells>0,"Omega empty with complete far observations");
  require(view.candidate({0,0,1})==CandidateStatus::Unknown,"unobserved elevation fabricated free");
  for(auto &d:f.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());
  ++f.sequence;builder.process(f,rays);require(f.local.available_cells==0 && f.local.unknown_cells==builder.layout().cells(),"missing depth fabricated virtual domain");
  // Overlap cameras face different directions: rho, not optical Z, selects nearer.
  const double a=direction_pi/6,co=std::cos(a),si=std::sin(a);cal[1].T_BC={si,0,co,9,-co,0,si,8,0,-1,0,7,0,0,0,1};builder.configure(cal,w,h,1,1,16,3,5,false,true,true,true);builder.allocate(f);
  std::fill(f.D[0].begin(),f.D[0].end(),2.f);std::fill(f.D[1].begin(),f.D[1].end(),2.1f);++f.sequence;builder.process(f,rays);
  const auto id=*builder.layout().cell(direction_pi/6,0);const double rho=f.local.min_range[id];require(rho>2.09 && rho<2.15 && (f.local.complete_cameras[id]&3)==3,"cross-camera merge used optical Z instead of common rho");
  for(auto &d:f.D)std::fill(d.begin(),d.end(),10.f);
  ++f.sequence;builder.process(f,rays);
  const auto old_key=*view.input_key();
  for(auto &c:cal)c.T_BC={1,0,0,4,0,1,0,5,0,0,1,6,0,0,0,1};
  builder.configure(cal,w,h,1,1,16,3,5,false,true,true,true);
  require(!view.ready(),"changed orientation reused old virtual frame");
  // Same-sized preallocated slots remain usable, but old inactive cells must clear.
  builder.process(f,rays);
  const auto front=*builder.layout().cell(0,0);
  require(f.local.node_state[front]==0 && !valid_range(f.local.min_range[front]) && *view.input_key()!=old_key,"orientation cache retained free nodes outside new coverage");
  std::cout<<"PASS: "<<points<<" virtual near/radius samples, "<<queries<<" panorama queries, direct rho/block checks, fixed lookup, translation/clip invariance, raw unknown/Omega, overlap rho, same-group/zero-allocation reset\n";return 0;
}catch(const std::exception &e){counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
