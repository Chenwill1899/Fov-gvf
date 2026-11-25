#include "new_controller/depth_observations.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
#include <random>
#include <thread>

static std::atomic<bool> count_allocations{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {if(count_allocations.load()) ++allocations;if(void *p=std::malloc(n?n:1)) return p;throw std::bad_alloc();}
void operator delete(void *p) noexcept {std::free(p);} void operator delete(void *p,std::size_t) noexcept {std::free(p);}
void *operator new[](std::size_t n) {return ::operator new(n);} void operator delete[](void *p) noexcept {::operator delete(p);} void operator delete[](void *p,std::size_t) noexcept {::operator delete(p);}
using namespace ego4p0;
static void require(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
static std::array<CameraCalibration,4> cameras() {
  std::array<CameraCalibration,4> c;
  for(auto &a:c) {a.K={1,0,0,0,1,0,0,0,1};a.T_BC={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};}
  return c;
}
int main() {
  const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  {
    DirectionGridLayout l;l.configure(1,1);
    require(l.cells()==64800,"1 degree grid shape");
    require(l.cell(-direction_pi,0)==l.cell(direction_pi,0),"azimuth seam duplicated");
    require(*l.cell(0,direction_pi/2)/l.azimuth_bins==179 && *l.cell(0,-direction_pi/2)/l.azimuth_bins==0,"pole cell bounds");
    require(!l.cell(nan,0) && !l.cell(0,direction_pi),"bad angles accepted");
    l.configure(7,7);require(l.azimuth_bins==52 && l.elevation_bins==26,"truncated final cells");
  }
  {
    auto cal=cameras();cal[0].T_BC[3]=.5; // nonzero camera offset distinguishes b from normalized d
    PixelRayTables rays;rays.configure(cal,2,1);
    FrameObservationBuilder builder;builder.configure_debug(cal,2,1,1,1,16,1);
    DepthFrame f;f.allocate(2);builder.allocate(f);for(auto &d:f.D_filled) std::fill(d.begin(),d.end(),nan);
    f.D_filled[0][1]=2;f.t_ns=123;f.sequence=7;builder.build(f,rays);
    const auto expected=builder.layout().cell(0,std::atan2(2.,2.5));
    const DepthObservationView view(f);auto source=view.observation(*expected,0);
    require(source && source->camera==0 && source->u==1 && source->v==0 && source->optical_z==2 && source->stamp_ns==123 && source->sequence==7,"optical Z, camera translation or provenance wrong");
    require(f.observations.source_count==1,"invalid filled pixels entered directions");
    cal[0].T_BC[3]=1.;builder.configure_debug(cal,2,1,1,1,16,1);builder.build(f,rays);
    const auto shifted=builder.layout().cell(0,std::atan2(2.,3.));require(shifted!=expected && DepthObservationView(f).observation(*shifted,0).has_value(),"extrinsic translation ignored");
  }
  {
    auto cal=cameras();
    // camera optical Z maps to body +X, +Y, -X, -Y respectively.
    cal[0].T_BC={0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1};
    cal[1].T_BC={-1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1};
    cal[2].T_BC={0,0,-1,0,-1,0,0,0,0,1,0,0,0,0,0,1};
    cal[3].T_BC={1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1};
    PixelRayTables rays;rays.configure(cal,1,1);FrameObservationBuilder b;b.configure_debug(cal,1,1);DepthFrame f;f.allocate(1);b.allocate(f);
    for(auto &d:f.D_filled) d[0]=3;
    f.sequence=1;b.build(f,rays);
    for(uint32_t i=0;i<4;++i) {
      const double a=i==3 ? -direction_pi/2 : i*direction_pi/2;
      auto o=DepthObservationView(f).observation(*b.layout().cell(a,0),0);require(o && o->camera==i,"four-camera rotation/seam");
    }
  }
  {
    auto cal=cameras();PixelRayTables rays;rays.configure(cal,1,1);FrameObservationBuilder b;b.configure_debug(cal,1,1);DepthFrame f;f.allocate(1);b.allocate(f);
    for(size_t i=0;i<4;++i) f.D_filled[i][0]=static_cast<float>(i+1);
    f.sequence=9;f.t_ns=900;b.build(f,rays);
    auto cell=*b.layout().cell(0,direction_pi/2);require(f.observations.offsets[cell+1]-f.observations.offsets[cell]==4 && f.observations.camera_coverage[cell]==15,"overlapping sources were merged");
    for(uint32_t i=0;i<4;++i) {auto o=DepthObservationView(f).observation(cell,i);require(o && o->camera==i && o->optical_z==i+1,"cross-camera optical depth was compared/discarded");}
    require(f.observations.observed_cells==1,"empty angular cells fabricated");
    f.sequence=10;require(!DepthObservationView(f).ready() && !DepthObservationView(f).region(0,{0,0,1,1}).z,"stale generation queried");
    // Body-origin surface has no direction, but its numeric depth remains queryable.
    cal[0].T_BC[11]=-1;b.configure_debug(cal,1,1);f.D_filled[0][0]=1;b.build(f,rays);
    require(f.observations.geometry_rejected[0]==1 && f.observations.camera_observations[0]==0 && DepthObservationView(f).pixel(0,0,0).z==1.f,"undefined direction handling");
    for(auto &d:f.D_filled) d[0]=nan;
    ++f.sequence;b.build(f,rays);
    const auto missing=DepthObservationView(f).region(0,{0,0,1,1});
    require(f.observations.ready && f.observations.source_count==0 && f.observations.observed_cells==0
            && std::all_of(f.observations.camera_coverage.begin(),f.observations.camera_coverage.end(),[](uint8_t x){return x==0;})
            && !missing.z && missing.total_pixels==1 && missing.valid_pixels==0,"empty frame retained old sources or fabricated query data");
  }
  {
    auto cal=cameras();PixelRayTables rays;rays.configure(cal,35,29);FrameObservationBuilder builder;builder.configure_debug(cal,35,29);DepthFrame f;f.allocate(35*29);builder.allocate(f);f.sequence=12;f.t_ns=144;
    for(uint32_t c=0;c<4;++c) for(size_t i=0;i<35*29;++i) f.D_filled[c][i]=(i%5==0) ? nan : static_cast<float>(c*1000+(i%97)+1);
    f.D_filled[0][1]=inf;f.D_filled[0][2]=0;f.D_filled[0][3]=-2;
    builder.build(f,rays);DepthObservationView view(f);
    require(!view.pixel(-1,0,0).z && view.pixel(0,-1,0).status==DepthQueryStatus::OutsideImage && !view.pixel(0,35,0).z,"pixel bounds/camera");
    require(!view.pixel(0,0,0).z && !view.pixel(0,1,0).z && !view.pixel(0,2,0).z && !view.pixel(0,3,0).z,"unusable values fabricated");
    require(!view.region(0,{-1,0,2,2}).z && view.region(0,{0,0,0,2}).status==DepthQueryStatus::EmptyRegion && view.region(0,{2,0,1,2}).status==DepthQueryStatus::InvalidRegion,"ROI bounds/empty/reversed");
    auto all=view.region(0,{0,0,35,29});require(all.pixels_scanned==0 && all.tiles_used==6 && all.total_pixels==1015,"whole ROI rescanned pixels or lost edge tiles");
    auto missing=view.region(0,{0,0,1,1});require(!missing.z && missing.valid_pixels==0 && missing.total_pixels==1,"all-invalid ROI has fictitious maximum range");
    std::mt19937 rng(42);
    allocations=0;count_allocations=true;
    for(int i=0;i<4000;++i) {
      int x0=rng()%35,x1=x0+1+rng()%(35-x0),y0=rng()%29,y1=y0+1+rng()%(29-y0);const int c=rng()%4;
      auto q=view.region(c,{x0,y0,x1,y1});uint32_t valid=0;std::optional<float> expected;
      for(int v=y0;v<y1;++v) for(int u=x0;u<x1;++u) {float z=f.D_filled[c][v*35+u];if(std::isfinite(z) && z>0) {++valid;if(!expected || z<*expected) expected=z;}}
      require(q.z==expected && q.valid_pixels==valid && q.total_pixels==static_cast<uint32_t>((x1-x0)*(y1-y0)) && q.sequence==12 && q.stamp_ns==144,"tile ROI differs from independent brute force");
    }
    ++f.sequence;builder.build(f,rays);count_allocations=false;require(allocations==0,"steady build/query allocated");
  }
  {
    auto cal=cameras();PixelRayTables rays;rays.configure(cal,2,2);FrameObservationBuilder builder;builder.configure_debug(cal,2,2);
    TripleBuffer<DepthFrame> buffers;for(auto &f:buffers.slots) {f.allocate(4);builder.allocate(f);}
    std::atomic<bool> done{false};
    std::thread writer([&] {for(uint64_t n=1;n<=1000;++n) {auto &f=buffers.writer();f.sequence=n;f.t_ns=n*2;for(uint32_t c=0;c<4;++c) std::fill(f.D_filled[c].begin(),f.D_filled[c].end(),static_cast<float>(n+c));builder.build(f,rays);buffers.publish();}done=true;});
    uint64_t reads=0;
    do {if(const auto *f=buffers.latest()) {DepthObservationView v(*f);const auto n=f->sequence;require(v.ready() && v.region(2,{0,0,2,2}).z==static_cast<float>(n+2),"image/index generation mismatch");std::this_thread::yield();require(f->sequence==n && v.ready(),"borrowed image/index overwritten");++reads;}}while(!done);
    writer.join();require(reads>0 && buffers.latest()->sequence==1000,"concurrent publication incomplete");
  }
  std::cout<<"PASS: optical-Z b/extrinsic translation+rotation, multi-source preservation, seam/poles/empty cells, unresolved rejection, ROI bounds/partial/all-invalid, 4000 brute-force ROI comparisons, zero steady allocations, 1000 concurrent protected image/index groups\n";
}
