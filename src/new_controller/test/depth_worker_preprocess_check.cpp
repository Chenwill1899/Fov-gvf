#include "new_controller/depth_observations.hpp"
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <random>

static std::atomic<bool> count_allocations{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {if(count_allocations.load()) ++allocations;if(void *p=std::malloc(n?n:1)) return p;throw std::bad_alloc();}
void operator delete(void *p) noexcept {std::free(p);} void operator delete(void *p,std::size_t) noexcept {std::free(p);}
void *operator new[](std::size_t n) {return ::operator new(n);} void operator delete[](void *p) noexcept {::operator delete(p);} void operator delete[](void *p,std::size_t) noexcept {::operator delete(p);}
using namespace ego4p0;
static void require(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
static void compare(const DepthFrame &a,const DepthFrame &b) {
  const auto &x=a.observations,&y=b.observations;
  require(x.offsets==y.offsets && x.source_count==y.source_count && x.camera_coverage==y.camera_coverage
    && std::equal(x.sources.begin(),x.sources.begin()+x.source_count,y.sources.begin()) && x.valid_depth_pixels==y.valid_depth_pixels
    && x.usable_blocks==y.usable_blocks && x.geometry_rejected==y.geometry_rejected && x.camera_observations==y.camera_observations,"parallel repair/fused index differs from separate reference");
  for(size_t c=0;c<4;++c) {
    require(std::memcmp(a.D[c].data(),b.D[c].data(),a.D[c].size()*sizeof(float))==0,"raw input overwritten");
    require(std::memcmp(a.D_filled[c].data(),b.D_filled[c].data(),a.D_filled[c].size()*sizeof(float))==0,"raw-only repair order/payload differs");
    require(a.repair[c].invalid==b.repair[c].invalid && a.repair[c].filled==b.repair[c].filled && a.repair[c].unresolved==b.repair[c].unresolved,"repair counters differ");
    for(size_t k=0;k<x.blocks[c].size();++k) require(x.blocks[c][k].valid_pixels==y.blocks[c][k].valid_pixels
      && std::memcmp(&x.blocks[c][k].mean_z,&y.blocks[c][k].mean_z,sizeof(float))==0,"fused mean changed original block accumulation order");
    for(size_t k=0;k<x.tiles[c].size();++k) require(x.tiles[c][k].valid_pixels==y.tiles[c][k].valid_pixels
      && std::memcmp(&x.tiles[c][k].min_z,&y.tiles[c][k].min_z,sizeof(float))==0,"fused summary differs at tile boundary");
  }
}
int main() {
  std::mt19937 rng(147);float payload_nan;const uint32_t payload=0x7fc01234;std::memcpy(&payload_nan,&payload,4);
  const float inf=std::numeric_limits<float>::infinity();uint32_t cases=0;
  for(const auto shape:std::array<std::array<uint32_t,2>,12>{{{1,1},{2,4},{3,3},{4,5},{15,17},{16,16},{17,31},{31,32},{32,33},{35,29},{49,47},{320,240}}}) {
    const uint32_t w=shape[0],h=shape[1],n=w*h;std::array<CameraCalibration,4> cal;
    for(auto &c:cal) {c.K={100,.1,w*.5,0,95,h*.5,0,0,1};c.T_BC={0,0,1,.2,1,0,0,-.1,0,1,0,.05,0,0,0,1};}
    PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder reference,separate,fused;
    reference.configure_debug(cal,w,h,1,1,16,3,5,false,true,false);
    separate.configure_debug(cal,w,h,1,1,16,3,5,true,true,false);
    fused.configure_debug(cal,w,h,1,1,16,3,5,true,true,true);
    DepthFrame a,b,c;for(auto *f:{&a,&b,&c}) f->allocate(n);reference.allocate(a);separate.allocate(b);fused.allocate(c);
    for(uint32_t camera=0;camera<4;++camera) for(uint32_t p=0;p<n;++p) {
      const uint32_t type=rng()%13;float z;
      if(type<2) z=payload_nan;else if(type==2) z=inf;else if(type==3) z=-inf;else if(type==4) z=0;else if(type==5) z=-1;
      else if(type==6) z=1e30f;else if(type==7) z=1e-30f;else z=(rng()%10000+1)*.003f;
      a.D[camera][p]=z;
    }
    // A 7x7 invalid island exercises unresolved original NaN bit preservation.
    for(uint32_t v=0;v<std::min(h,7U);++v) for(uint32_t u=0;u<std::min(w,7U);++u) a.D[0][v*w+u]=payload_nan;
    for(uint32_t camera=0;camera<4;++camera) {
      std::memcpy(b.D[camera].data(),a.D[camera].data(),n*sizeof(float));std::memcpy(c.D[camera].data(),a.D[camera].data(),n*sizeof(float));
      a.repair[camera]=fill_invalid_depth(a.D[camera].data(),a.D_filled[camera].data(),w,h);
    }
    a.sequence=b.sequence=c.sequence=1;a.t_ns=b.t_ns=c.t_ns=2000000;
    reference.build(a,rays);separate.process(b,rays);fused.process(c,rays);compare(a,b);compare(a,c);
    require(b.preprocessing_ns==*std::max_element(b.repair_ns.begin(),b.repair_ns.end()) && c.preprocessing_ns==*std::max_element(c.repair_ns.begin(),c.repair_ns.end()),"repair max timing not aligned with worker kernels");
    require(c.observations.fused_scan && c.observations.repaired_from_raw && !b.observations.fused_scan,"pipeline metadata wrong");
    for(int k=0;k<400;++k) {
      const uint32_t x0=rng()%w,x1=x0+1+rng()%(w-x0),y0=rng()%h,y1=y0+1+rng()%(h-y0),camera=rng()%4;
      uint32_t valid=0;std::optional<float> minimum;
      for(uint32_t v=y0;v<y1;++v) for(uint32_t u=x0;u<x1;++u) {const float z=a.D_filled[camera][v*w+u];if(valid_depth(z)) {++valid;if(!minimum || z<*minimum) minimum=z;}}
      const auto q=DepthObservationView(c).region(camera,{static_cast<int>(x0),static_cast<int>(y0),static_cast<int>(x1),static_cast<int>(y1)});
      require(q.z==minimum && q.valid_pixels==valid && q.total_pixels==(x1-x0)*(y1-y0),"fused partial ROI differs from brute force");
    }
    allocations=0;count_allocations=true;
    for(uint64_t seq=2;seq<=11;++seq) {
      b.sequence=c.sequence=seq;separate.process(b,rays);fused.process(c,rays);
      require(!separate.process(b,rays) && !fused.process(c,rays),"raw group processed twice");
    }
    count_allocations=false;require(allocations==0,"worker repair/fused scan allocated in steady processing");
    compare(a,b);compare(a,c);++cases;
  }
  std::cout<<"PASS: "<<cases<<" image shapes, original-neighbor repair/payload/raw preservation, fused 3x3/16x16 crossing/edge summaries, 4800 brute-force ROIs, same-group reuse and zero steady allocations\n";
}
