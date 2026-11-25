#include "new_controller/depth_size_query.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
static std::atomic<bool> counting{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {if(counting) ++allocations;if(void *p=std::malloc(n?n:1)) return p;throw std::bad_alloc();}
void operator delete(void *p) noexcept {std::free(p);}void operator delete(void *p,std::size_t) noexcept {std::free(p);}
void *operator new[](std::size_t n) {return ::operator new(n);}void operator delete[](void *p) noexcept {::operator delete(p);}void operator delete[](void *p,std::size_t) noexcept {::operator delete(p);}
using namespace ego4p0;
static void require(bool b,const char *m) {if(!b) throw std::runtime_error(m);}
int main() {
 try {
  constexpr uint32_t w=35,h=29,n=w*h;std::array<CameraCalibration,4> cal;
  for(auto &c:cal) {c.K={25,0,w*.5,0,25,h*.5,0,0,1};c.T_BC={1,0,0,.2,0,1,0,0,0,0,1,.02,0,0,0,1};c.near_clip_z_m=.05;}
  PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder builder;builder.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true);
  TripleBuffer<DepthFrame> slots;for(auto &f:slots.slots) {f.allocate(n);builder.allocate(f);}
  const DepthFrame *current=nullptr;FieldInputKey previous{};uint32_t full_grid_checks=0;
  allocations=0;counting=true;
  for(uint32_t group=1;group<=120;++group) {
    auto &f=slots.writer();for(auto &image:f.D) std::fill(image.begin(),image.end(),10.f);
    const uint32_t c=group%4,p=(group*17)%n;const bool close=group%5!=0;
    if(close) f.D[c][p]=.3f;
    f.sequence=group;f.t_ns=group*2000000;builder.process(f,rays);
    require(f.observations.offsets.empty() && f.observations.sources.empty() && !f.observations.full_provenance,"compact path retained full representative CSR");
    require(f.observations.offsets.capacity()==0 && f.observations.sources.capacity()==0,"compact source capacity not released during initialization");
    require(!builder.process(f,rays),"same group rebuilt");
    slots.publish();current=slots.latest();LocalFieldView view(*current,builder.local_geometry());
    require(view.ready() && current->local.excluded_blocks==(close?1U:0U) && (close || current->local.excluded_cells==0),"stale counts across groups/frame slots");
    const auto source=view.pixel_obstacle(c,p%w,p/w);
    require(bool(source)==close,"on-demand close pixel missing");
    for(uint32_t cell=0;cell<builder.layout().cells();++cell) {
      const auto summary=*view.cell(cell);
      require(summary.status!=(CandidateStatus::Available),"unexcluded direction became free without geometry");
      require(summary.excluded_cameras==current->local.excluded_camera_mask[cell],"cell exclusion camera bits mismatch");
      if(!close) require(summary.excluded_cameras==0,"prior slot exclusion survived reset");
      ++full_grid_checks;
    }
    if(close) require(view.cell(source->direction_cell)->excluded_cameras&(1U<<c),"near source not excluded");
    require(!DepthObservationView(*current).observation(0,0),"compact source query silently materialized a list");
    const auto ray=*view.block_ray(0,0);require(ray.evidence.raw_valid_pixels==9 && ray.evidence.begin_known && ray.raw_min_z,"on-demand ray lost raw summary");
    require(!view.block_ray(4,0) && !view.block_ray(0,99999),"out-of-range ray fabricated");
    const auto key=*view.input_key(1,2,3,4);require(key!=previous,"new group did not invalidate field key");previous=key;
    for(uint32_t k=0;k<4;++k) {auto revisions=std::array<uint64_t,4>{1,2,3,4};++revisions[k];require(*view.input_key(revisions[0],revisions[1],revisions[2],revisions[3])!=key,"coordinate/parameter/domain/boundary key unchanged");}
  }
  counting=false;require(allocations==0,"compact group/query path allocated");
  require(builder.local_geometry().builds()==1 && builder.builds()==120 && builder.reused_groups()==120,"same-group/static-parameter cache wrong");
  // Explicit full representative provenance remains available for diagnostics.
  FrameObservationBuilder debug;debug.configure_debug(cal,w,h,1,1,16,3,5,false,true,true,true);
  DepthFrame reference;reference.allocate(n);debug.allocate(reference);reference.D=current->D;reference.sequence=current->sequence;reference.t_ns=current->t_ns;debug.process(reference,rays);
  require(reference.observations.cell_counts==current->observations.cell_counts && reference.observations.camera_coverage==current->observations.camera_coverage,"debug full mode changes compact evidence");
  require(reference.observations.full_provenance && !reference.observations.sources.empty(),"explicit debug provenance absent");
  require(current->observations.source_count==4*current->observations.blocks_per_camera && current->local.excluded_blocks==0,"far-range coarse observations discarded or made local obstacles");
  const auto old_key=*LocalFieldView(*current,builder.local_geometry()).input_key();
  cal[0].T_BC[3]+=.01;builder.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true);
  require(!LocalFieldView(*current,builder.local_geometry()).ready(),"changed coordinate geometry reused old frame evidence");
  auto &fresh=slots.writer();builder.allocate(fresh);fresh.D=current->D;fresh.sequence=current->sequence;fresh.t_ns=current->t_ns;builder.process(fresh,rays);
  require(*LocalFieldView(fresh,builder.local_geometry()).input_key()!=old_key,"calibration/parameter change reused field key");
  std::cout<<"PASS: compact default/no CSR, 120 groups over protected slots, "<<full_grid_checks<<" cell resets/no automatic free, raw ray on-demand, far coarse retained, coordinate/parameter/domain/boundary cache keys, debug parity, zero steady allocations\n";return 0;
 }catch(const std::exception &e) {counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
