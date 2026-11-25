#pragma once
#include "new_controller/depth_block_domain.hpp"
#include <chrono>

namespace ego4p0 {
class LocalFieldBuilder {
 public:
  template<class BlockRayLookup> void configure(const std::array<CameraCalibration,4> &cal,uint32_t w,uint32_t h,
       const DirectionGridLayout &layout,uint32_t edge,BlockRayLookup block_q,bool virtual_center=false,const DirectionGridLayout *unused_layout=nullptr) {
    (void)unused_layout;virtual_center_=virtual_center;
    geometry_.configure(cal,w,h,edge,block_q,virtual_center,&layout);layout_=layout;w_=w;h_=h;edge_=edge;columns_=(w+edge-1)/edge;stride_=layout.azimuth_bins+1;
    far_bad_prefix_.assign(size_t(layout.azimuth_bins+1)*(layout.elevation_bins+1),0);
    for(auto &work:work_) {work.difference.resize(size_t(stride_)*(layout.elevation_bins+1));work.excluded.assign(layout.cells(),0);work.has_exclusion=false;}
  }
  const LocalGeometryTables &geometry() const noexcept {return geometry_;}
  void allocate(DepthFrame &f) const {f.local.allocate(layout_,virtual_center_);}
  void begin_camera(uint32_t c) noexcept {
    auto &work=work_[c];work.near=work.complete=work.unknown=0;
  }
  void block(uint32_t c,uint32_t id,DepthFrame &f) noexcept {
    auto &work=work_[c];const auto &b=f.observations.blocks[c][id];const auto &g=geometry_.block(c,id);
    if(virtual_center_) {
      if(b.valid_pixels && valid_range(b.min_rho) && b.min_rho<=ControllerLimits::local_range_m) {
        if(!work.near) std::fill(work.difference.begin(),work.difference.end(),0);
        ++work.near;const auto &table=geometry_.virtual_tables();
        const auto box=table.window(table.block_cell(c,id),table.distance_bin(b.min_rho),true);
        if(box.a0<=box.a1) rectangle(work,box.a0,box.a1,box.b0,box.b1);
        else {rectangle(work,box.a0,layout_.azimuth_bins,box.b0,box.b1);rectangle(work,0,box.a1,box.b0,box.b1);}
      } else {
        if(b.raw_complete && b.raw_min_rho>ControllerLimits::local_range_m) ++work.complete;
        else ++work.unknown;
      }
      return;
    }
    const uint32_t u=id%columns_*edge_,v=id/columns_*edge_;
    const uint32_t total=(std::min(u+edge_,w_)-u)*(std::min(v+edge_,h_)-v);
    const auto evidence=block_domain_evidence(b,g,total);
    if(evidence==BlockDomainEvidence::CompleteFarObservation) {++work.complete;return;}
    if(evidence==BlockDomainEvidence::Unknown) {++work.unknown;return;}
    if(!work.near) std::fill(work.difference.begin(),work.difference.end(),0);
    ++work.near;const auto box=block_exclusion(b,g,geometry_.camera(c).T_BC,layout_);
    if(box.a0<=box.a1) rectangle(work,box.a0,box.a1,box.b0,box.b1);
    else {rectangle(work,box.a0,layout_.azimuth_bins,box.b0,box.b1);rectangle(work,0,box.a1,box.b0,box.b1);}
  }
  void finish_camera(uint32_t c,DepthFrame &f) noexcept {
    auto &work=work_[c];
    if(!work.near) {
      if(work.has_exclusion) std::fill(work.excluded.begin(),work.excluded.end(),0);
      work.has_exclusion=false;
    } else {
    for(uint32_t b=0;b<layout_.elevation_bins;++b) {int32_t row=0;
      for(uint32_t a=0;a<layout_.azimuth_bins;++a) {
        const size_t i=size_t(b)*stride_+a;row+=work.difference[i];
        const int32_t count=row+(b ? work.difference[i-stride_] : 0);work.difference[i]=count;
        work.excluded[b*layout_.azimuth_bins+a]=count>0;
      }
    }
    work.has_exclusion=true;
    }
    f.local.near_blocks[c]=work.near;f.local.complete_far_blocks[c]=work.complete;f.local.unknown_blocks[c]=work.unknown;
  }
  void merge(DepthFrame &f,bool defer_panorama=false) noexcept {
    const auto start=now();auto &d=f.local;d.ready=false;const bool had_exclusion=d.excluded_cells!=0;d.excluded_blocks=0;
    for(uint32_t c=0;c<4;++c) d.excluded_blocks+=d.near_blocks[c];
    if(!d.excluded_blocks) {
      if(had_exclusion) std::fill(d.excluded_camera_mask.begin(),d.excluded_camera_mask.end(),0);
      d.excluded_cells=0;
    } else {
    d.excluded_cells=0;
    for(uint32_t cell=0;cell<layout_.cells();++cell) {
      uint8_t mask=0;for(uint32_t c=0;c<4;++c) if(work_[c].excluded[cell]) mask|=uint8_t(1U<<c);
      d.excluded_camera_mask[cell]=mask;d.excluded_cells+=mask!=0;
    }
    }
    if(virtual_center_ && d.geometry_id!=geometry_.configuration_id()) {
      std::fill(d.min_range.begin(),d.min_range.end(),std::numeric_limits<double>::quiet_NaN());
      std::fill(d.observation_cameras.begin(),d.observation_cameras.end(),0);
      std::fill(d.complete_cameras.begin(),d.complete_cameras.end(),0);std::fill(d.node_state.begin(),d.node_state.end(),0);
    }
    d.panorama_pending=virtual_center_ && defer_panorama;
    if(virtual_center_ && !d.panorama_pending)merge_panorama(f);
    d.stamp_ns=f.t_ns;d.sequence=f.sequence;d.configuration_id=f.observations.configuration_id;
    d.geometry_id=geometry_.configuration_id();d.merge_ns=now()-start;d.ready=!d.panorama_pending;
  }
  // All inputs to this pass belong to f, including its merged exclusion mask.
  // The prefix scratch is overwritten on every pass, even after another frame.
  void finish_panorama(DepthFrame &f) noexcept {
    const auto start=now();
    merge_panorama(f);
    f.local.merge_ns+=now()-start;
    f.local.panorama_pending=false;f.local.ready=true;
  }
 private:
  void merge_panorama(DepthFrame &f) noexcept {
    auto &d=f.local;const auto &table=geometry_.virtual_tables();
    for(auto &tile:d.range_tiles) tile=RangeTile{};
    const uint32_t tile_columns=(layout_.azimuth_bins+7)/8;
    for(const uint32_t cell:table.active_cells()) {
      const auto &sources=table.node(cell);double minimum=std::numeric_limits<double>::infinity();uint8_t observed=0,complete=0;
      for(uint32_t c=0;c<4;++c) if(sources.cameras&(1U<<c)) {
        const auto &block=f.observations.blocks[c][sources.blocks[c]];
        if(valid_range(block.min_rho)) {minimum=std::min(minimum,block.min_rho);observed|=uint8_t(1U<<c);}
        if(block.raw_complete) complete|=uint8_t(1U<<c);
      }
      d.min_range[cell]=std::isfinite(minimum) ? minimum : std::numeric_limits<double>::quiet_NaN();
      d.observation_cameras[cell]=observed;d.complete_cameras[cell]=complete;
    }
    for(uint32_t b=0;b<layout_.elevation_bins;++b) {
      uint32_t missing_row=0,far_bad_row=0;
      for(uint32_t a=0;a<layout_.azimuth_bins;++a) {
        const uint32_t cell=b*layout_.azimuth_bins+a;const double minimum=d.min_range[cell];const bool complete=d.complete_cameras[cell];
        missing_row+=!complete;far_bad_row+=!complete || !(minimum>ControllerLimits::local_range_m+ControllerLimits::radius_m);
        const size_t i=size_t(b+1)*stride_+a+1;
        d.missing_prefix[i]=d.missing_prefix[i-stride_]+missing_row;
        far_bad_prefix_[i]=far_bad_prefix_[i-stride_]+far_bad_row;
        if(std::isfinite(minimum)) {auto &tile=d.range_tiles[b/8*tile_columns+a/8];if(!tile.valid || minimum<tile.minimum) tile.minimum=minimum;++tile.valid;}
      }
    }
    d.available_cells=0;d.unknown_cells=layout_.cells();
    for(const uint32_t cell:table.active_cells()) {
      CandidateStatus status=CandidateStatus::Unknown;
      if(d.observation_cameras[cell]) {
        if(d.excluded_camera_mask[cell]) status=CandidateStatus::Unavailable;
        else if(d.complete_cameras[cell] && prefix_count(far_bad_prefix_,table.domain_window(cell))==0)
          status=CandidateStatus::Available;
      }
      d.node_state[cell]=uint8_t(status);d.available_cells+=status==CandidateStatus::Available;d.unknown_cells-=status!=CandidateStatus::Unknown;
    }
  }
  uint32_t prefix_count(const std::vector<uint32_t> &prefix,const PanoramaWindow &box) const noexcept {
    auto rect=[&](uint32_t a0,uint32_t a1) {return prefix[size_t(box.b1)*stride_+a1]-prefix[size_t(box.b0)*stride_+a1]
      -prefix[size_t(box.b1)*stride_+a0]+prefix[size_t(box.b0)*stride_+a0];};
    return box.a0<=box.a1 ? rect(box.a0,box.a1) : rect(box.a0,layout_.azimuth_bins)+rect(0,box.a1);
  }
  static int64_t now() noexcept {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  struct alignas(64) Work {std::vector<int32_t> difference;std::vector<uint8_t> excluded;uint32_t near=0,complete=0,unknown=0;bool has_exclusion=false;};
  void rectangle(Work &w,uint32_t a0,uint32_t a1,uint32_t b0,uint32_t b1) noexcept {
    ++w.difference[size_t(b0)*stride_+a0];--w.difference[size_t(b0)*stride_+a1];
    --w.difference[size_t(b1)*stride_+a0];++w.difference[size_t(b1)*stride_+a1];
  }
  std::array<Work,4> work_;LocalGeometryTables geometry_;DirectionGridLayout layout_;
  bool virtual_center_=false;std::vector<uint32_t> far_bad_prefix_;
  uint32_t w_=0,h_=0,edge_=3,columns_=0,stride_=0;
};
} // namespace ego4p0
