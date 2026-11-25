#pragma once
#include "new_controller/depth_preprocess.hpp"
#include "new_controller/depth_direction_boundaries.hpp"
#include "new_controller/depth_camera_workers.hpp"
#include "new_controller/depth_local_field.hpp"
#include <chrono>

namespace ego4p0 {
inline std::array<int64_t,4> trial_cam_wall{},trial_cam_cpu{};
inline int64_t trial_cpu_now(){timespec t;clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1000000000LL+t.tv_nsec;}
class FrameObservationBuilder {
 public:
  bool configure(const std::array<CameraCalibration,4> &cal,size_t width,size_t height,
                 double azimuth_deg=1.,double elevation_deg=1.,uint32_t tile_edge=16,
                 uint32_t block_edge=3,int reader_cpu=5,bool parallel=true,bool boundary_lookup=true,bool fused_scans=true,bool local_field=false,bool full_provenance=false,bool virtual_center=true,bool second_level_summary=false) {
    depth_pixel_count(width,height);
    if(!tile_edge || tile_edge>256 || (block_edge!=1 && block_edge!=3 && block_edge!=4)) throw std::invalid_argument("invalid tile/block size");
    bool same=configuration_id_ && second_level_summary==second_level_summary_ && virtual_center==virtual_center_ && width==width_ && height==height_ && azimuth_deg==layout_.azimuth_deg
      && elevation_deg==layout_.elevation_deg && tile_edge==tile_edge_ && block_edge==block_edge_
      && reader_cpu==reader_cpu_ && parallel==parallel_ && boundary_lookup==boundary_lookup_ && fused_scans==fused_scans_ && local_field==local_field_ && full_provenance==full_provenance_;
    for(size_t c=0;c<4;++c) same=same && cal[c].K==cal_[c].K && cal[c].T_BC==cal_[c].T_BC;
    if(local_field) for(size_t c=0;c<4;++c) same=same && (cal[c].near_clip_z_m==cal_[c].near_clip_z_m
      || (std::isnan(cal[c].near_clip_z_m) && std::isnan(cal_[c].near_clip_z_m)));
    if(same) return false;
    pool_.stop();configuration_id_=0;
    layout_.configure(azimuth_deg,elevation_deg);boundaries_.configure(layout_);
    width_=width;height_=height;tile_edge_=tile_edge;block_edge_=block_edge;reader_cpu_=reader_cpu;
    parallel_=parallel;boundary_lookup_=boundary_lookup;fused_scans_=fused_scans;local_field_=local_field;full_provenance_=full_provenance;virtual_center_=virtual_center;cal_=cal;
    second_level_summary_=second_level_summary;
    columns_=(width_+block_edge_-1)/block_edge_;rows_=(height_+block_edge_-1)/block_edge_;blocks_=columns_*rows_;
    cursor_.resize(full_provenance_ ? layout_.cells() : 0);
    row_tile_.resize(height_);column_tile_.resize(width_);
    const uint32_t tile_columns=(width_+tile_edge_-1)/tile_edge_;
    for(uint32_t v=0;v<height_;++v) row_tile_[v]=v/tile_edge_*tile_columns;
    for(uint32_t u=0;u<width_;++u) column_tile_[u]=u/tile_edge_;
    for(uint32_t camera=0;camera<4;++camera) {
      const auto &t=cal[camera].T_BC;for(double x:t) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera extrinsics");
      const auto inverse=PixelRayTables::inverse_intrinsics(cal[camera].K);auto &work=work_[camera];
      work.rays.resize(blocks_);work.cells.resize(blocks_);work.sources.resize(full_provenance_ ? blocks_ : 0);
      if(!full_provenance_) work.sources.shrink_to_fit(); // initialization only
      for(uint32_t v=0;v<rows_;++v) for(uint32_t u=0;u<columns_;++u) {
        auto &ray=work.rays[v*columns_+u];ray.u0=u*block_edge_;ray.v0=v*block_edge_;
        ray.u1=std::min(ray.u0+block_edge_,width_);ray.v1=std::min(ray.v0+block_edge_,height_);
        const uint32_t first_tile=row_tile_[ray.v0]+column_tile_[ray.u0],last_tile=row_tile_[ray.v1-1]+column_tile_[ray.u1-1];
        ray.tile=first_tile==last_tile ? first_tile : std::numeric_limits<uint32_t>::max();
        const double x=(ray.u0+ray.u1-1)*.5,y=(ray.v0+ray.v1-1)*.5;
        std::array<double,3> b{};for(size_t k=0;k<3;++k) b[k]=inverse[k*3]*x+inverse[k*3+1]*y+inverse[k*3+2];
        for(size_t r=0;r<3;++r) {
          ray.q[r]=t[r*4]*b[0]+t[r*4+1]*b[1]+t[r*4+2]*b[2];
          if(!std::isfinite(ray.q[r])) throw std::invalid_argument("nonfinite rotated block ray");
        }
        ray.hint=boundaries_.exact(ray.q[0],ray.q[1],ray.q[2]).value_or(0);
      }
    }
    if(local_field_) local_builder_.configure(cal_,width_,height_,layout_,block_edge_,[this](uint32_t c,uint32_t id){return work_[c].rays[id].q;},virtual_center_,&layout_);
    if(parallel_) pool_.start(reader_cpu_,this,[](void *owner,uint32_t c,uint32_t repair,DepthFrame &f) {
      auto &self=*static_cast<FrameObservationBuilder*>(owner);
      self.camera_work(c,f,repair!=0);
    });
    configuration_id_=next_configuration_.fetch_add(1,std::memory_order_relaxed)+1;++table_builds_;
    return true;
  }
  void allocate(DepthFrame &frame) const {
    frame.observations.allocate(width_,height_,layout_,tile_edge_,block_edge_,full_provenance_);
    auto&i=frame.observations;i.second_ready=false;i.second_columns=second_level_summary_?(columns_+2)/3:0;i.second_rows=second_level_summary_?(rows_+2)/3:0;
    for(auto&v:i.second_level)v.resize(size_t(i.second_columns)*i.second_rows);
    if(local_field_) local_builder_.allocate(frame);else frame.local.ready=false;
  }
  const DirectionGridLayout &layout() const noexcept { return layout_; }
  uint64_t table_builds() const noexcept {return table_builds_;}
  uint64_t builds() const noexcept {return builds_;}
  uint64_t reused_groups() const noexcept {return reused_groups_;}
  bool parallel() const noexcept {return parallel_;}
  int64_t task_start_offset_ns(unsigned camera)const noexcept{return work_[camera].start_ns-dispatch_ns_;}
  int64_t task_end_offset_ns(unsigned camera)const noexcept{return work_[camera].end_ns-dispatch_ns_;}
  bool boundary_lookup() const noexcept {return boundary_lookup_;}
  bool fused_scans() const noexcept {return fused_scans_;}
  bool local_field_enabled() const noexcept {return local_field_;}
  bool configure_debug(const std::array<CameraCalibration,4> &cal,size_t w,size_t h,double a=1.,double b=1.,uint32_t tile=16,
       uint32_t block=3,int cpu=5,bool parallel=true,bool boundary=true,bool fused=true,bool local=false) {
    return configure(cal,w,h,a,b,tile,block,cpu,parallel,boundary,fused,local,true,false);
  }
  bool configure_real(const std::array<CameraCalibration,4> &cal,size_t w,size_t h,double a=1.,double b=1.,uint32_t tile=16,
       uint32_t block=3,int cpu=5,bool parallel=true,bool boundary=true,bool fused=true,bool local=false) {
    return configure(cal,w,h,a,b,tile,block,cpu,parallel,boundary,fused,local,false,false);
  }
  bool virtual_center() const noexcept {return virtual_center_;}
  const LocalGeometryTables &local_geometry() const noexcept {return local_builder_.geometry();}
  const std::array<int,4> &worker_cpus() const noexcept {return pool_.cpus();}
  const std::array<int,4> &worker_affinity_errors() const noexcept {return pool_.affinity_errors();}
  const std::array<int,4> &worker_scheduler_errors() const noexcept {return pool_.scheduler_errors();}
  void enable_fifo5_workers() {if(parallel_) pool_.enable_fifo5();}
  void refresh_worker_schedulers() {if(parallel_) pool_.refresh_schedulers();}
  const std::array<int,4> &worker_policies() const noexcept {return pool_.policies();}
  const std::array<int,4> &worker_priorities() const noexcept {return pool_.priorities();}
  bool build(DepthFrame &frame,const PixelRayTables &rays) {
    return construct(frame,rays,false); // Already-filled input, for independent checks.
  }
  bool process(DepthFrame &frame,const PixelRayTables &rays,bool defer_panorama=false) {
    return construct(frame,rays,true,defer_panorama);
  }
 private:
  bool construct(DepthFrame &frame,const PixelRayTables &rays,bool repair,bool defer_panorama=false) {
    auto &index=frame.observations;const uint32_t pixels=width_*height_;
    if(!configuration_id_ || index.width!=width_ || index.height!=height_ || index.layout.cells()!=layout_.cells()
       || index.layout.azimuth_deg!=layout_.azimuth_deg || index.layout.elevation_deg!=layout_.elevation_deg
       || index.tile_edge!=tile_edge_ || index.block_edge!=block_edge_ || index.full_provenance!=full_provenance_ || rays.width()!=width_ || rays.height()!=height_)
      throw std::invalid_argument("unconfigured or mismatched observation index");
    for(const auto &d:frame.D_filled) if(d.size()!=pixels) throw std::invalid_argument("filled image size differs from index");
    if(local_field_ && (frame.local.excluded_camera_mask.size()!=layout_.cells() || frame.local.virtual_center!=virtual_center_))
      throw std::invalid_argument("local evidence buffers not allocated");
    index.raw_summaries_ready=std::all_of(frame.D.begin(),frame.D.end(),[pixels](const auto &d){return d.size()==pixels;});
    if(local_field_ && !index.raw_summaries_ready) throw std::invalid_argument("local evidence requires original images");
    if(repair) for(const auto &d:frame.D) if(d.size()!=pixels) throw std::invalid_argument("raw image size differs from index");
    if(index.ready && index.sequence==frame.sequence && index.stamp_ns==frame.t_ns && index.configuration_id==configuration_id_ && index.repaired_from_raw==repair) {
      if(!defer_panorama && frame.local.panorama_pending && local_field_ &&
         frame.local.configuration_id==configuration_id_ && frame.local.geometry_id==local_builder_.geometry().configuration_id() &&
         frame.local.sequence==frame.sequence && frame.local.stamp_ns==frame.t_ns)
        local_builder_.finish_panorama(frame);
      ++reused_groups_;return false;
    }
    index.ready=false;frame.local.ready=false;frame.local.panorama_pending=false;index.sequence=frame.sequence;index.stamp_ns=frame.t_ns;
    if(second_level_summary_)for(const auto&v:index.second_level)if(v.size()!=size_t((columns_+2)/3)*((rows_+2)/3))throw std::invalid_argument("second-level buffers not allocated");
    index.second_ready=false;index.repaired_from_raw=repair;index.fused_scan=fused_scans_;index.virtual_center=virtual_center_;
    index.configuration_id=configuration_id_;index.source_count=0;index.observed_cells=0;index.camera_covered_cells.fill(0);
    const auto start=now();dispatch_ns_=start;
    if(parallel_) pool_.run(frame,repair ? 1 : 0);else for(uint32_t camera=0;camera<4;++camera) camera_work(camera,frame,repair);
    if(repair) frame.preprocessing_ns=*std::max_element(frame.repair_ns.begin(),frame.repair_ns.end());
    index.second_ready=second_level_summary_;
    const auto merge_start=now();
    std::fill(index.cell_counts.begin(),index.cell_counts.end(),0);std::fill(index.camera_coverage.begin(),index.camera_coverage.end(),0);
    for(uint32_t camera=0;camera<4;++camera) {
      const auto &work=work_[camera];
      for(uint32_t k=0;k<work.used;++k) {
        const uint32_t cell=work.cells[k];if(!index.cell_counts[cell]++) ++index.observed_cells;
        if(!(index.camera_coverage[cell]&(1U<<camera))) ++index.camera_covered_cells[camera];
        index.camera_coverage[cell]|=static_cast<uint8_t>(1U<<camera);
      }
      index.source_count+=work.used;
    }
    if(full_provenance_) {
      index.offsets[0]=0;
      for(uint32_t cell=0;cell<layout_.cells();++cell) {index.offsets[cell+1]=index.offsets[cell]+index.cell_counts[cell];cursor_[cell]=index.offsets[cell];}
      for(uint32_t camera=0;camera<4;++camera) {const auto &work=work_[camera];
        for(uint32_t k=0;k<work.used;++k) index.sources[cursor_[work.cells[k]]++]=camera*blocks_+work.sources[k];
      }
    }
    const auto end=now();index.camera_work_ns=merge_start-start;index.merge_ns=end-merge_start;index.build_total_ns=end-start;
    if(local_field_) local_builder_.merge(frame,defer_panorama);
    index.ready=true;++builds_;return true;
  }
 private:
  static int64_t now() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  struct BlockRay {std::array<double,3> q{};uint32_t u0=0,v0=0,u1=0,v1=0,hint=0,tile=0;};
  struct alignas(64) CameraWork {std::vector<BlockRay> rays;std::vector<uint32_t> cells,sources;uint32_t used=0;int64_t start_ns=0,end_ns=0;};
  int64_t dispatch_ns_=0;
  void camera_work(uint32_t camera,DepthFrame &frame,bool repair) {
    const auto tw=now(),tc=trial_cpu_now();
    if(repair) {
      const auto start=now();frame.repair[camera]=fill_invalid_depth(frame.D[camera].data(),frame.D_filled[camera].data(),width_,height_);
      frame.repair_ns[camera]=now()-start;
    }
    project_camera(camera,frame);if(!fused_scans_) region_camera(camera,frame);
    trial_cam_cpu[camera]=trial_cpu_now()-tc;
    work_[camera].end_ns=now();work_[camera].start_ns=tw;
    trial_cam_wall[camera]=work_[camera].end_ns-tw;
  }
  template<bool Fused> void compress_camera(uint32_t camera,DepthFrame &frame) noexcept {
    auto &index=frame.observations;auto &work=work_[camera];const auto &image=frame.D_filled[camera];
    auto &averages=index.blocks[camera];uint32_t valid_pixels=0,usable=0;
    if constexpr(Fused) {
      for(auto &tile:index.tiles[camera]) tile=DepthTile{};
      for(auto &tile:index.raw_tiles[camera]) tile=DepthTile{};
      index.region_camera_ns[camera]=0;
    }
    for(uint32_t id=0;id<blocks_;++id) {
      const auto &ray=work.rays[id];double sum=0.;uint32_t valid=0,raw_valid=0;
      double min_rho=std::numeric_limits<double>::infinity(),raw_min_rho=min_rho;
      float maximum=0.f;
      float minimum=std::numeric_limits<float>::infinity(),raw_minimum=std::numeric_limits<float>::infinity();
      for(uint32_t v=ray.v0;v<ray.v1;++v) for(uint32_t u=ray.u0;u<ray.u1;++u) {
        const float raw=index.raw_summaries_ready ? frame.D[camera][v*width_+u] : 0.f;
        if(valid_depth(raw)) {
          ++raw_valid;if(raw<raw_minimum) raw_minimum=raw;
          if constexpr(Fused) if(ray.tile==std::numeric_limits<uint32_t>::max()) {
            auto &tile=index.raw_tiles[camera][row_tile_[v]+column_tile_[u]];
            if(!tile.valid_pixels || raw<tile.min_z) tile.min_z=raw;
            ++tile.valid_pixels;
          }
        }
        const float z=image[v*width_+u];if(!valid_depth(z)) continue;
        if(local_field_ && virtual_center_) {
          const double coefficient=local_builder_.geometry().virtual_tables().coefficient(camera,v*width_+u);
          const double rho=double(z)*coefficient;min_rho=std::min(min_rho,rho);
          if(valid_depth(raw)) raw_min_rho=std::min(raw_min_rho,raw==z ? rho : double(raw)*coefficient);
        }
        sum+=static_cast<double>(z);++valid;if(z<minimum) minimum=z;if(z>maximum) maximum=z;
        if constexpr(Fused) {
          if(ray.tile!=std::numeric_limits<uint32_t>::max()) {if(z<minimum) minimum=z;}
          else {
            auto &tile=index.tiles[camera][row_tile_[v]+column_tile_[u]];
            if(!tile.valid_pixels || z<tile.min_z) tile.min_z=z;
            ++tile.valid_pixels;
          }
        }
      }
      if constexpr(Fused) if(valid && ray.tile!=std::numeric_limits<uint32_t>::max()) {
        auto &tile=index.tiles[camera][ray.tile];if(!tile.valid_pixels || minimum<tile.min_z) tile.min_z=minimum;
        tile.valid_pixels+=valid;
      }
      if constexpr(Fused) if(raw_valid && ray.tile!=std::numeric_limits<uint32_t>::max()) {
        auto &tile=index.raw_tiles[camera][ray.tile];if(!tile.valid_pixels || raw_minimum<tile.min_z) tile.min_z=raw_minimum;
        tile.valid_pixels+=raw_valid;
      }
      averages[id].min_rho=std::isfinite(min_rho) ? min_rho : std::numeric_limits<double>::quiet_NaN();
      averages[id].raw_complete=raw_valid==(ray.u1-ray.u0)*(ray.v1-ray.v0) && std::isfinite(raw_min_rho);
      averages[id].raw_min_rho=std::isfinite(raw_min_rho) ? raw_min_rho : std::numeric_limits<double>::quiet_NaN();
      averages[id].valid_pixels=valid;averages[id].mean_z=valid ? static_cast<float>(sum/valid) : std::numeric_limits<float>::quiet_NaN();
      averages[id].max_z=valid ? maximum : std::numeric_limits<float>::quiet_NaN();
      averages[id].min_z=valid ? minimum : std::numeric_limits<float>::quiet_NaN();
      averages[id].raw_valid_pixels=raw_valid;averages[id].raw_min_z=raw_valid ? raw_minimum : std::numeric_limits<float>::quiet_NaN();
      valid_pixels+=valid;if(valid) ++usable;
    }
    index.valid_depth_pixels[camera]=valid_pixels;index.usable_blocks[camera]=usable;
  }
  void project_camera(uint32_t camera,DepthFrame &frame) noexcept {
    auto &index=frame.observations;auto &work=work_[camera];const auto &t=cal_[camera].T_BC;auto &averages=index.blocks[camera];
    uint32_t rejected=0,exact=0,fast=0;const auto start=now();
    if(fused_scans_) compress_camera<true>(camera,frame);else compress_camera<false>(camera,frame);
    if(second_level_summary_)aggregate_second(camera,frame);
    const auto compressed=now();work.used=0;
    if(local_field_) local_builder_.begin_camera(camera);
    for(uint32_t id=0;id<blocks_;++id) {
      auto &average=averages[id];average.direction_cell=std::numeric_limits<uint32_t>::max();average.mean_in_local_range=false;
      if(!average.valid_pixels) {if(local_field_) local_builder_.block(camera,id,frame);continue;}
      const auto &ray=work.rays[id];const double z=average.mean_z;
      const double x=virtual_center_ ? 0. : z*ray.q[0]+t[3],y=virtual_center_ ? 0. : z*ray.q[1]+t[7],zz=virtual_center_ ? 0. : z*ray.q[2]+t[11];
      bool used_exact=true;
      const auto cell=virtual_center_ ? std::optional<uint32_t>(ray.hint) : boundary_lookup_ ? boundaries_.cell(x,y,zz,ray.hint,used_exact) : boundaries_.exact(x,y,zz);
      if(!cell) ++rejected;
      else {
        average.direction_cell=*cell;
        if(virtual_center_ || !used_exact) ++fast;else ++exact;
        work.cells[work.used]=*cell;if(full_provenance_) work.sources[work.used]=id;++work.used;
      }
      if(local_field_) {
        average.mean_in_local_range=virtual_center_ ? valid_range(average.min_rho) && average.min_rho<=ControllerLimits::local_range_m : x*x+y*y+zz*zz<=ControllerLimits::local_range_m*ControllerLimits::local_range_m;
        local_builder_.block(camera,id,frame);
      }
    }
    index.geometry_rejected[camera]=rejected;
    index.camera_observations[camera]=work.used;index.exact_angle_observations[camera]=exact;index.fast_angle_observations[camera]=fast;
    if(local_field_) local_builder_.finish_camera(camera,frame);
    index.compression_ns[camera]=compressed-start;index.projection_ns[camera]=now()-compressed;
    if(local_field_) frame.local.shared_projection_ns[camera]=index.projection_ns[camera];
  }
  void aggregate_second(uint32_t camera,DepthFrame&frame)noexcept{
    auto&i=frame.observations;auto&second=i.second_level[camera];std::fill(second.begin(),second.end(),ImageLevelSummary{});
    for(uint32_t id=0;id<blocks_;++id){const auto&v=i.blocks[camera][id];auto&s=second[(id/columns_/3)*i.second_columns+(id%columns_/3)];const auto&r=work_[camera].rays[id];
      if(!s.valid)s.mean_z=0.;
      if(v.valid_pixels){s.mean_z+=double(v.mean_z)*v.valid_pixels;s.valid+=v.valid_pixels;}
      s.raw_valid+=v.raw_valid_pixels;s.total+=(r.u1-r.u0)*(r.v1-r.v0);
      if(std::isfinite(v.min_rho)&&v.min_rho>0&&(!std::isfinite(s.min_rho)||v.min_rho<s.min_rho))s.min_rho=v.min_rho;
      if(std::isfinite(v.raw_min_rho)&&v.raw_min_rho>0&&(!std::isfinite(s.raw_min_rho)||v.raw_min_rho<s.raw_min_rho))s.raw_min_rho=v.raw_min_rho;
    }
    for(auto&s:second)s.mean_z=s.valid?s.mean_z/s.valid:std::numeric_limits<double>::quiet_NaN();
  }
  void region_camera(uint32_t camera,DepthFrame &frame) noexcept {
    auto &index=frame.observations;const auto start=now();
    for(auto &tile:index.tiles[camera]) tile=DepthTile{};
    for(auto &tile:index.raw_tiles[camera]) tile=DepthTile{};
    for(uint32_t ty=0;ty<index.tile_rows;++ty) for(uint32_t tx=0;tx<index.tile_columns;++tx) {
      auto &tile=index.tiles[camera][ty*index.tile_columns+tx];
      auto &raw_tile=index.raw_tiles[camera][ty*index.tile_columns+tx];
      const uint32_t y1=std::min((ty+1)*tile_edge_,height_),x1=std::min((tx+1)*tile_edge_,width_);
      for(uint32_t v=ty*tile_edge_;v<y1;++v) for(uint32_t u=tx*tile_edge_;u<x1;++u) {
        const float raw=index.raw_summaries_ready ? frame.D[camera][v*width_+u] : 0.f;
        if(valid_depth(raw)) {if(!raw_tile.valid_pixels || raw<raw_tile.min_z) raw_tile.min_z=raw;++raw_tile.valid_pixels;}
        const float z=frame.D_filled[camera][v*width_+u];if(!valid_depth(z)) continue;
        if(!tile.valid_pixels || z<tile.min_z) tile.min_z=z;
        ++tile.valid_pixels;
      }
    }
    index.region_camera_ns[camera]=now()-start;
  }
  uint32_t width_=0,height_=0,tile_edge_=16,block_edge_=3,columns_=0,rows_=0,blocks_=0;
  int reader_cpu_=5;bool virtual_center_=true;bool parallel_=true,boundary_lookup_=true,fused_scans_=true,local_field_=false,full_provenance_=false;
  bool second_level_summary_=false;
  DirectionGridLayout layout_;DirectionBoundaries boundaries_;
  std::array<CameraCalibration,4> cal_{};std::array<CameraWork,4> work_;
  std::vector<uint32_t> cursor_,row_tile_,column_tile_;CameraWorkerPool pool_;
  LocalFieldBuilder local_builder_;
  uint64_t configuration_id_=0,table_builds_=0,builds_=0,reused_groups_=0;
  inline static std::atomic<uint64_t> next_configuration_{0};
};

// Borrowed view: valid only while the caller owns the corresponding frame slot.
// No allocation, interpolation, implicit out-of-image clipping or free-space decision.
class DepthObservationView {
 public:
  explicit DepthObservationView(const DepthFrame &frame):frame_(frame),index_(frame.observations) {}
  bool ready() const noexcept {
    return index_.ready && index_.sequence==frame_.sequence && index_.stamp_ns==frame_.t_ns;
  }
  const FrameObservationIndex &index() const noexcept { return index_; }
  std::optional<DirectionObservation> block(uint32_t camera,uint32_t id) const noexcept {
    if(!ready() || camera>=4 || id>=index_.blocks_per_camera) return std::nullopt;
    const auto &b=index_.blocks[camera][id];if(!b.valid_pixels) return std::nullopt;
    const uint32_t u0=id%index_.block_columns*index_.block_edge,v0=id/index_.block_columns*index_.block_edge;
    const uint32_t u1=std::min(u0+index_.block_edge,index_.width),v1=std::min(v0+index_.block_edge,index_.height);
    return DirectionObservation{camera,(u0+u1-1)*.5,(v0+v1-1)*.5,b.mean_z,frame_.t_ns,frame_.sequence,u0,v0,u1,v1,b.valid_pixels,(u1-u0)*(v1-v0)};
  }
  std::optional<DirectionObservation> observation(uint32_t cell,uint32_t relative_index) const noexcept {
    if(!ready() || !index_.full_provenance || cell>=index_.layout.cells()) return std::nullopt;
    const uint32_t start=index_.offsets[cell],end=index_.offsets[cell+1];
    if(relative_index>=end-start) return std::nullopt;
    const uint32_t code=index_.sources[start+relative_index],blocks=index_.blocks_per_camera;
    const uint32_t camera=code/blocks,id=code%blocks,u0=id%index_.block_columns*index_.block_edge,v0=id/index_.block_columns*index_.block_edge;
    const uint32_t u1=std::min(u0+index_.block_edge,index_.width),v1=std::min(v0+index_.block_edge,index_.height);
    const auto &block=index_.blocks[camera][id];
    return DirectionObservation{camera,(u0+u1-1)*.5,(v0+v1-1)*.5,block.mean_z,frame_.t_ns,frame_.sequence,
                                u0,v0,u1,v1,block.valid_pixels,(u1-u0)*(v1-v0)};
  }
  DepthQueryResult pixel(int camera,int u,int v,DepthImageKind kind=DepthImageKind::Filled) const noexcept {
    auto result=initial(camera,{u,v,u,v});
    if(!ready()) return result;
    if(kind==DepthImageKind::Raw && !index_.raw_summaries_ready) return result;
    if(camera<0 || camera>=4) {result.status=DepthQueryStatus::InvalidCamera;return result;}
    if(u<0 || v<0 || u>=static_cast<int>(index_.width) || v>=static_cast<int>(index_.height)) {
      result.status=DepthQueryStatus::OutsideImage;return result;
    }
    result.region={u,v,u+1,v+1};result.total_pixels=1;result.pixels_scanned=1;
    const float z=(kind==DepthImageKind::Raw ? frame_.D : frame_.D_filled)[camera][static_cast<uint32_t>(v)*index_.width+static_cast<uint32_t>(u)];
    if(valid_depth(z)) {result.z=z;result.valid_pixels=1;result.status=DepthQueryStatus::Ok;}
    else result.status=DepthQueryStatus::NoUsableDepth;
    return result;
  }
  DepthQueryResult region(int camera,PixelRegion q,DepthImageKind kind=DepthImageKind::Filled) const noexcept {
    return region_impl<false>(camera,q,kind,nullptr);
  }
  // Return original and repaired ROI statistics from the same tile walk.
  std::array<DepthQueryResult,2> region_pair(int camera,PixelRegion q) const noexcept {
    auto raw=initial(camera,q);
    if(!index_.raw_summaries_ready)
      return {raw,region(camera,q)};
    auto filled=region_impl<true>(camera,q,DepthImageKind::Filled,&raw);
    if(filled.total_pixels==0)raw.status=filled.status;
    return {raw,filled};
  }
 private:
  template<bool Pair>
  DepthQueryResult region_impl(int camera,PixelRegion q,DepthImageKind kind,
                               DepthQueryResult *raw) const noexcept {
    auto result=initial(camera,q);
    if(!ready()) return result;
    if(kind==DepthImageKind::Raw && !index_.raw_summaries_ready) return result;
    if(camera<0 || camera>=4) {result.status=DepthQueryStatus::InvalidCamera;return result;}
    if(q.u0>q.u1 || q.v0>q.v1) {result.status=DepthQueryStatus::InvalidRegion;return result;}
    if(q.u0<0 || q.v0<0 || q.u1>static_cast<int>(index_.width) || q.v1>static_cast<int>(index_.height)) {
      result.status=DepthQueryStatus::OutsideImage;return result;
    }
    if(q.u0==q.u1 || q.v0==q.v1) {result.status=DepthQueryStatus::EmptyRegion;return result;}
    result.total_pixels=static_cast<uint32_t>((q.u1-q.u0)*(q.v1-q.v0));
    auto add=[](DepthQueryResult &result,float z,uint32_t count) {
      if(!count) return;
      if(!result.z || z<*result.z) result.z=z;
      result.valid_pixels+=count;
    };
    const uint32_t edge=index_.tile_edge;
    const uint32_t tx0=static_cast<uint32_t>(q.u0)/edge,tx1=static_cast<uint32_t>(q.u1-1)/edge;
    const uint32_t ty0=static_cast<uint32_t>(q.v0)/edge,ty1=static_cast<uint32_t>(q.v1-1)/edge;
    for(uint32_t ty=ty0;ty<=ty1;++ty) for(uint32_t tx=tx0;tx<=tx1;++tx) {
      const uint32_t left=tx*edge,right=std::min(left+edge,index_.width);
      const uint32_t top=ty*edge,bottom=std::min(top+edge,index_.height);
      if(q.u0<=static_cast<int>(left) && q.u1>=static_cast<int>(right) && q.v0<=static_cast<int>(top) && q.v1>=static_cast<int>(bottom)) {
        const auto offset=ty*index_.tile_columns+tx;
        const auto &tile=(kind==DepthImageKind::Raw ? index_.raw_tiles : index_.tiles)[camera][offset];
        add(result,tile.min_z,tile.valid_pixels);
        ++result.tiles_used;
        if constexpr(Pair){
          const auto &raw_tile=index_.raw_tiles[camera][offset];
          add(*raw,raw_tile.min_z,raw_tile.valid_pixels);
          ++raw->tiles_used;
        }
      } else {
        const uint32_t x0=std::max(left,static_cast<uint32_t>(q.u0)),x1=std::min(right,static_cast<uint32_t>(q.u1));
        const uint32_t y0=std::max(top,static_cast<uint32_t>(q.v0)),y1=std::min(bottom,static_cast<uint32_t>(q.v1));
        for(uint32_t v=y0;v<y1;++v) for(uint32_t u=x0;u<x1;++u) {
          const auto pixel=v*index_.width+u;
          const float z=(kind==DepthImageKind::Raw ? frame_.D : frame_.D_filled)[camera][pixel];
          ++result.pixels_scanned;
          if(valid_depth(z)) add(result,z,1);
          if constexpr(Pair){
            const float original=frame_.D[camera][pixel];
            ++raw->pixels_scanned;
            if(valid_depth(original))add(*raw,original,1);
          }
        }
      }
    }
    result.status=result.z ? DepthQueryStatus::Ok : DepthQueryStatus::NoUsableDepth;
    if constexpr(Pair){
      raw->total_pixels=result.total_pixels;
      raw->status=raw->z ? DepthQueryStatus::Ok : DepthQueryStatus::NoUsableDepth;
    }
    return result;
  }
  DepthQueryResult initial(int camera,PixelRegion q) const noexcept {
    DepthQueryResult r;r.camera=camera;r.region=q;r.stamp_ns=frame_.t_ns;r.sequence=frame_.sequence;return r;
  }
  const DepthFrame &frame_;
  const FrameObservationIndex &index_;
};
} // namespace ego4p0
