#pragma once
#include "new_controller/depth_observations.hpp"

namespace ego4p0 {
enum class EnvelopeStatus { Unknown,ObservedClear,SimulationInitialClear,PotentialObstacle,
                            CurrentPositionNotEvaluated,OutsideLocalRange,InputNotReady,InvalidInput };
struct CameraEnvelopeQuery {
  SphereFootprint footprint;
  DepthQueryResult filled,raw;
  bool potential_obstacle=false,observed_clear=false;
};
struct EnvelopeResult {
  EnvelopeStatus status=EnvelopeStatus::InputNotReady;
  std::array<CameraEnvelopeQuery,4> cameras;
  int64_t stamp_ns=0;uint64_t sequence=0;
};
// Supplied ONLY by a separate, explicit simulation clearance verification.
// No odometry/stationarity/zero command is accepted as a substitute. A basis is
// valid for this exact input group and body frame; it is never carried forward.
struct SimulationInitialClearance {
  bool verified=false;
  int64_t stamp_ns=0;uint64_t sequence=0;
  double free_ball_radius_m=0.;
  std::string proof_id;
};
// Unexcluded directions remain Unknown until an actual footprint is checked.

struct AngularEvidenceSummary {
  uint32_t representative_observations=0;
  uint8_t representative_cameras=0,excluded_cameras=0;
  CandidateStatus status=CandidateStatus::Unknown;
};
struct PanoramaObservation {
  std::optional<double> rho;uint8_t cameras=0,complete_cameras=0;CandidateStatus status=CandidateStatus::Unknown;
  int64_t stamp_ns=0;uint64_t sequence=0;
};
struct LocalObstacleRef {
  uint32_t camera=0,u=0,v=0;
  float optical_z=0.;bool raw_observed=false;
  int64_t stamp_ns=0;uint64_t sequence=0;
  uint32_t direction_cell=0;double radial_rho=0.;
};
struct LocalRayBundleRef {
  uint32_t camera=0,block_id=0;PixelRegion region;FreeRayEvidence evidence;
  int64_t stamp_ns=0;uint64_t sequence=0;
  std::optional<float> raw_min_z;
};
struct FieldInputKey {
  uint64_t sequence=0;int64_t stamp_ns=0;
  uint64_t observation_configuration=0,geometry_configuration=0;
  uint64_t coordinate_revision=0,parameter_revision=0,domain_revision=0,boundary_revision=0;
  bool operator==(const FieldInputKey &o) const noexcept {
    return sequence==o.sequence && stamp_ns==o.stamp_ns && observation_configuration==o.observation_configuration
      && geometry_configuration==o.geometry_configuration && coordinate_revision==o.coordinate_revision
      && parameter_revision==o.parameter_revision && domain_revision==o.domain_revision && boundary_revision==o.boundary_revision;
  }
  bool operator!=(const FieldInputKey &o) const noexcept {return !(*this==o);}
};
class LocalFieldView {
 public:
  LocalFieldView(const DepthFrame &frame,const LocalGeometryTables &geometry):frame_(frame),geometry_(geometry) {}
  bool ready() const noexcept {
    return frame_.local.ready && frame_.local.virtual_center==geometry_.virtual_center() && DepthObservationView(frame_).ready() && frame_.local.sequence==frame_.sequence
      && frame_.local.stamp_ns==frame_.t_ns && frame_.local.configuration_id==frame_.observations.configuration_id
      && frame_.local.geometry_id==geometry_.configuration_id();
  }
  std::optional<PanoramaObservation> panorama_cell(uint32_t id) const noexcept {
    if(!ready() || !geometry_.virtual_center() || id>=frame_.local.min_range.size()) return std::nullopt;
    const auto &d=frame_.local;PanoramaObservation out;
    if(valid_range(d.min_range[id])) out.rho=d.min_range[id];
    out.cameras=d.observation_cameras[id];out.complete_cameras=d.complete_cameras[id];out.status=CandidateStatus(d.node_state[id]);
    out.stamp_ns=frame_.t_ns;out.sequence=frame_.sequence;return out;
  }
  std::optional<AngularEvidenceSummary> cell(uint32_t id) const noexcept {
    if(!ready() || id>=frame_.observations.layout.cells()) return std::nullopt;
    const uint8_t mask=frame_.local.excluded_camera_mask[id];
    return AngularEvidenceSummary{frame_.observations.cell_counts[id],frame_.observations.camera_coverage[id],mask,
      frame_.local.virtual_center ? CandidateStatus(frame_.local.node_state[id]) : mask ? CandidateStatus::Unavailable : CandidateStatus::Unknown};
  }
  std::optional<BlockDomainEvidence> block_evidence(uint32_t camera,uint32_t id) const noexcept {
    if(!ready() || camera>=4 || id>=frame_.observations.blocks_per_camera) return std::nullopt;
    const auto &i=frame_.observations;const uint32_t u=id%i.block_columns*i.block_edge,v=id/i.block_columns*i.block_edge;
    if(geometry_.virtual_center()) {
      const auto &b=i.blocks[camera][id];
      if(valid_range(b.min_rho) && b.min_rho<=ControllerLimits::local_range_m) return BlockDomainEvidence::ObstacleExclusion;
      const uint32_t total=(std::min(u+i.block_edge,i.width)-u)*(std::min(v+i.block_edge,i.height)-v);
      if(b.raw_valid_pixels==total && valid_range(b.raw_min_rho) && b.raw_min_rho>ControllerLimits::local_range_m) return BlockDomainEvidence::CompleteFarObservation;
      return BlockDomainEvidence::Unknown;
    }
    return block_domain_evidence(i.blocks[camera][id],geometry_.block(camera,id),
      (std::min(u+i.block_edge,i.width)-u)*(std::min(v+i.block_edge,i.height)-v));
  }
  CandidateStatus candidate(const std::array<double,3> &center_B,const SimulationInitialClearance *basis=nullptr) const noexcept {
    if(!ready()) return CandidateStatus::Unknown;
    if(geometry_.virtual_center()) return virtual_candidate(center_B);
    for(double x:center_B) if(!std::isfinite(x)) return CandidateStatus::Unknown;
    const double horizontal=std::hypot(center_B[0],center_B[1]),range=std::hypot(horizontal,center_B[2]);
    if(range<=1e-9 || range>ControllerLimits::local_range_m) return CandidateStatus::Unknown;
    const auto id=frame_.observations.layout.cell(horizontal==0 ? 0. : std::atan2(center_B[1],center_B[0]),std::atan2(center_B[2],horizontal));
    if(id && frame_.local.excluded_camera_mask[*id]) return CandidateStatus::Unavailable;
    const auto q=sphere(center_B,basis);
    if(q.status==EnvelopeStatus::PotentialObstacle) return CandidateStatus::Unavailable;
    if(q.status==EnvelopeStatus::ObservedClear || q.status==EnvelopeStatus::SimulationInitialClear) return CandidateStatus::Available;
    return CandidateStatus::Unknown;
  }
  std::optional<LocalObstacleRef> pixel_obstacle(uint32_t camera,uint32_t u,uint32_t v) const noexcept {
    if(!ready() || camera>=4 || u>=geometry_.width() || v>=geometry_.height()) return std::nullopt;
    const uint32_t p=v*geometry_.width()+u;const float z=frame_.D_filled[camera][p];const auto &ray=geometry_.pixel(camera,p);
    if(!valid_depth(z) || z>ray.z_limit) return std::nullopt;
    if(geometry_.virtual_center()) return LocalObstacleRef{camera,u,v,z,valid_depth(frame_.D[camera][p]),frame_.t_ns,frame_.sequence,
      geometry_.virtual_tables().pixel_cell(camera,p),double(z)*geometry_.virtual_tables().coefficient(camera,p)};
    const auto &t=geometry_.camera(camera).T_BC;
    const double x=z*ray.q[0]+t[3],y=z*ray.q[1]+t[7],zz=z*ray.q[2]+t[11];
    if(x==0 && y==0 && zz==0) return std::nullopt;
    const auto cell=frame_.observations.layout.cell((x==0 && y==0) ? 0. : std::atan2(y,x),std::atan2(zz,std::hypot(x,y)));
    if(!cell) return std::nullopt;
    return LocalObstacleRef{camera,u,v,z,valid_depth(frame_.D[camera][p]),frame_.t_ns,frame_.sequence,*cell};
  }
  std::optional<LocalRayBundleRef> block_ray(uint32_t camera,uint32_t id) const noexcept {
    if(!ready() || camera>=4 || id>=frame_.observations.blocks_per_camera) return std::nullopt;
    if(geometry_.virtual_center()) return std::nullopt; // optical-Z ray audit API is not a virtual planning interface
    const auto &index=frame_.observations;const auto &block=index.blocks[camera][id];
    const uint32_t u=id%index.block_columns*index.block_edge,v=id/index.block_columns*index.block_edge;
    const uint32_t u1=std::min(u+index.block_edge,index.width),v1=std::min(v+index.block_edge,index.height);
    const double near=geometry_.camera(camera).near_clip_z_m;FreeRayEvidence evidence;
    evidence.raw_valid_pixels=block.raw_valid_pixels;evidence.total_pixels=(u1-u)*(v1-v);
    evidence.complete_raw=evidence.raw_valid_pixels==evidence.total_pixels;
    evidence.begin_known=std::isfinite(near) && near>0.;evidence.z_begin=evidence.begin_known ? near : 0.;
    std::optional<float> minimum;
    if(block.raw_valid_pixels) {minimum=block.raw_min_z;evidence.z_end=std::min(double(block.raw_min_z),geometry_.block(camera,id).min_limit);}
    return LocalRayBundleRef{camera,id,{int(u),int(v),int(u1),int(v1)},evidence,frame_.t_ns,frame_.sequence,minimum};
  }
  std::optional<FieldInputKey> input_key(uint64_t coordinate=0,uint64_t parameter=0,uint64_t domain=0,uint64_t boundary=0) const noexcept {
    if(!ready()) return std::nullopt;
    return FieldInputKey{frame_.sequence,frame_.t_ns,frame_.observations.configuration_id,geometry_.configuration_id(),coordinate,parameter,domain,boundary};
  }
  EnvelopeResult sphere(const std::array<double,3> &center_B,const SimulationInitialClearance *basis=nullptr) const noexcept {
    EnvelopeResult result;result.stamp_ns=frame_.t_ns;result.sequence=frame_.sequence;if(!ready()) return result;
    for(double x:center_B) if(!std::isfinite(x)) {result.status=EnvelopeStatus::InvalidInput;return result;}
    const double distance=std::hypot(center_B[0],center_B[1],center_B[2]);
    if(distance<=1e-9) {result.status=EnvelopeStatus::CurrentPositionNotEvaluated;return result;}
    if(distance>ControllerLimits::local_range_m) {result.status=EnvelopeStatus::OutsideLocalRange;return result;}
    if(geometry_.virtual_center()) {
      const auto status=virtual_candidate(center_B);
      result.status=status==CandidateStatus::Available ? EnvelopeStatus::ObservedClear : status==CandidateStatus::Unavailable ? EnvelopeStatus::PotentialObstacle : EnvelopeStatus::Unknown;
      return result;
    }
    bool potential=false,clear=false;const DepthObservationView depth(frame_);
    for(uint32_t c=0;c<4;++c) {
      auto &q=result.cameras[c];q.footprint=geometry_.project(c,center_B);
      if(q.footprint.status!=FootprintStatus::Projected) continue;
      const auto &roi=q.footprint.region;
      if(roi.u0==roi.u1 || roi.v0==roi.v1) continue;
      q.filled=depth.region(c,roi);q.raw=depth.region(c,roi,DepthImageKind::Raw);
      q.potential_obstacle=q.filled.z && *q.filled.z<=q.footprint.z_far;
      const double near=geometry_.camera(c).near_clip_z_m;
      q.observed_clear=q.footprint.fully_inside && std::isfinite(near) && near>0. && q.footprint.z_near>=near
        && q.raw.z && q.raw.valid_pixels==q.raw.total_pixels && *q.raw.z>q.footprint.z_far;
      potential=potential || q.potential_obstacle;clear=clear || q.observed_clear;
    }
    // Any conservative obstacle constraint wins; depths from different optical
    // axes are never directly compared with each other.
    if(potential) result.status=EnvelopeStatus::PotentialObstacle;
    else if(clear) result.status=EnvelopeStatus::ObservedClear;
    else if(basis && basis->verified && basis->stamp_ns==frame_.t_ns && basis->sequence==frame_.sequence
            && !basis->proof_id.empty() && std::isfinite(basis->free_ball_radius_m)
            && basis->free_ball_radius_m>=distance+ControllerLimits::radius_m)
      result.status=EnvelopeStatus::SimulationInitialClear;
    else result.status=EnvelopeStatus::Unknown;
    return result;
  }
 private:
  CandidateStatus virtual_candidate(const std::array<double,3> &center) const noexcept {
    for(double x:center) if(!std::isfinite(x)) return CandidateStatus::Unknown;
    const double range=std::hypot(center[0],center[1],center[2]);
    if(range<=1e-9 || range>ControllerLimits::local_range_m) return CandidateStatus::Unknown;
    const auto &layout=frame_.observations.layout;
    const auto id=layout.cell(std::atan2(center[1],center[0]),std::atan2(center[2],std::hypot(center[0],center[1])));
    if(!id || !frame_.local.observation_cameras[*id]) return CandidateStatus::Unknown;
    if(frame_.local.excluded_camera_mask[*id]) return CandidateStatus::Unavailable;
    const auto &table=geometry_.virtual_tables();const auto box=table.window(*id,table.distance_bin(range));
    const auto query=panorama_region(box);
    if(query.minimum && *query.minimum<=range+ControllerLimits::radius_m) return CandidateStatus::Unavailable;
    return query.missing ? CandidateStatus::Unknown : CandidateStatus::Available;
  }
 public:
  struct PanoramaQuery {std::optional<double> minimum;uint32_t missing=0,total=0;};
  PanoramaQuery panorama_region(const PanoramaWindow &box) const noexcept {
    PanoramaQuery out;if(!ready() || !geometry_.virtual_center()) return out;
    const auto &grid=frame_.observations.layout;const auto &data=frame_.local;const uint32_t stride=grid.azimuth_bins+1,tile_columns=(grid.azimuth_bins+7)/8;
    if(box.b0>=box.b1 || box.b1>grid.elevation_bins || box.a0>grid.azimuth_bins || box.a1>grid.azimuth_bins) return out;
    auto rectangle=[&](uint32_t a0,uint32_t a1) {
      if(a0==a1) return;
      out.total+=(a1-a0)*(box.b1-box.b0);
      out.missing+=data.missing_prefix[size_t(box.b1)*stride+a1]-data.missing_prefix[size_t(box.b0)*stride+a1]
        -data.missing_prefix[size_t(box.b1)*stride+a0]+data.missing_prefix[size_t(box.b0)*stride+a0];
      auto add=[&](double rho) {if(valid_range(rho) && (!out.minimum || rho<*out.minimum)) out.minimum=rho;};
      for(uint32_t by=box.b0/8;by<=(box.b1-1)/8;++by) for(uint32_t ax=a0/8;ax<=(a1-1)/8;++ax) {
        const uint32_t x0=ax*8,x1=std::min(x0+8,grid.azimuth_bins),y0=by*8,y1=std::min(y0+8,grid.elevation_bins);
        if(a0<=x0 && a1>=x1 && box.b0<=y0 && box.b1>=y1) {const auto &tile=data.range_tiles[by*tile_columns+ax];if(tile.valid) add(tile.minimum);}
        else for(uint32_t b=std::max(y0,box.b0);b<std::min(y1,box.b1);++b) for(uint32_t a=std::max(x0,a0);a<std::min(x1,a1);++a) add(data.min_range[b*grid.azimuth_bins+a]);
      }
    };
    if(box.a0<=box.a1) rectangle(box.a0,box.a1);else {rectangle(box.a0,grid.azimuth_bins);rectangle(0,box.a1);}
    return out;
  }
 private:
  const DepthFrame &frame_;const LocalGeometryTables &geometry_;
};
} // namespace ego4p0
