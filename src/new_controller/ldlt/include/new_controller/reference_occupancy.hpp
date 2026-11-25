#pragma once
#include "new_controller/spatial_reference.hpp"
#include "new_controller/depth_size_query.hpp"
namespace ego4p0 {
inline PanoramaWindow reference_cone_window(const DirectionGridLayout&g,const Direction3&d,double delta){
  if(delta>=direction_pi/2)return {0,g.azimuth_bins,0,g.elevation_bins};
  const double beta=std::atan2(d[2],std::hypot(d[0],d[1])),alpha=std::atan2(d[1],d[0]);constexpr double eps=1e-11;
  const double lo=std::max(-direction_pi/2,beta-delta-eps),hi=std::min(direction_pi/2,beta+delta+eps);
  uint32_t b0=std::min(g.elevation_bins-1,uint32_t(std::max(0.,std::floor((lo+direction_pi/2)/g.elevation_step))));
  uint32_t b1=std::min(g.elevation_bins,uint32_t(std::max(0.,std::floor((hi+direction_pi/2)/g.elevation_step)+1)));
  if(std::abs(beta)+delta+eps>=direction_pi/2)return {0,g.azimuth_bins,b0,b1};
  const double width=std::asin(std::clamp(std::sin(delta)/std::cos(beta),0.,1.))+eps;
  const int64_t n=g.azimuth_bins,first=int64_t(std::floor((alpha-width+direction_pi)/g.azimuth_step)),last=int64_t(std::floor((alpha+width+direction_pi)/g.azimuth_step))+1;
  if(last-first>=n)return {0,g.azimuth_bins,b0,b1};
  auto wrap=[n](int64_t a){return uint32_t((a%n+n)%n);};return {wrap(first),wrap(last),b0,b1};
}
// Validate the entire normalized Bezier image against raw Omega, using a convex
// cone enclosing all four control points and de Casteljau subdivision. Checking
// just its samples or original SLERP chords would miss interpolation overshoot.
class AngularDomainCertificate {
 public:
  bool check(const ContinuousAngularPath&path,const DirectionGridLayout&g,const std::vector<uint8_t>&state){
    if(!g.cells()||state.size()!=g.cells()||!(g.azimuth_step>0)||!(g.elevation_step>0))return false;
    // A different curve over an identical Omega needs a new curve proof, but
    // not another full-grid integral image. Compare all bytes, never a hash
    // alone; changed Unknown/obstacle cells or grid geometry invalidate it.
    if(grid_.azimuth_bins!=g.azimuth_bins||grid_.elevation_bins!=g.elevation_bins||
       grid_.azimuth_step!=g.azimuth_step||grid_.elevation_step!=g.elevation_step||
       state_!=state){
      grid_=g;state_=state;stride_=g.azimuth_bins+1;
      bad_.assign(size_t(stride_)*(g.elevation_bins+1),0);
      for(uint32_t b=0;b<g.elevation_bins;++b){uint32_t row=0;for(uint32_t a=0;a<g.azimuth_bins;++a){row+=state[b*g.azimuth_bins+a]!=uint8_t(CandidateStatus::Available);const size_t i=size_t(b+1)*stride_+a+1;bad_[i]=bad_[i-stride_]+row;}}
      ++mask_builds_;
    }
    if(path.pieces().empty()){const auto q=path.query(0).q;return clear(reference_cone_window(g,q,0));}
    for(const auto&p:path.pieces())if(!piece(p.b,0))return false;
    return true;
  }
  uint64_t mask_builds()const{return mask_builds_;}
 private:
  bool clear(const PanoramaWindow&w)const{auto rect=[&](uint32_t a,uint32_t z){return bad_[size_t(w.b1)*stride_+z]-bad_[size_t(w.b0)*stride_+z]-bad_[size_t(w.b1)*stride_+a]+bad_[size_t(w.b0)*stride_+a];};return w.a0<=w.a1?rect(w.a0,w.a1)==0:rect(w.a0,grid_.azimuth_bins)+rect(0,w.a1)==0;}
  bool piece(const std::array<Direction3,4>&b,int depth)const{
    auto q=ref_add(ref_add(b[0],b[1],3),ref_add(ref_scale(b[2],3),b[3]));if(!unit_direction(q))return false;
    double along=1e9,perp=0;for(const auto&v:b){const double a=direction_dot(v,q);along=std::min(along,a);perp=std::max(perp,ref_norm(ref_add(v,q,-a)));}
    if(along>0&&clear(reference_cone_window(grid_,q,std::atan2(perp,along))))return true;
    if(depth>=14)return false;
    auto mid=[](const Direction3&a,const Direction3&c){return ref_scale(ref_add(a,c),.5);};
    const auto a=mid(b[0],b[1]),c=mid(b[1],b[2]),d=mid(b[2],b[3]),e=mid(a,c),f=mid(c,d),h=mid(e,f);
    return piece({b[0],a,e,h},depth+1)&&piece({h,f,d,b[3]},depth+1);
  }
  DirectionGridLayout grid_;uint32_t stride_=0;std::vector<uint32_t>bad_;
  std::vector<uint8_t>state_;uint64_t mask_builds_=0;
};
enum class TubeStatus:uint8_t {Unknown,ObservedClear,PotentialObstacle,OutsideObservationRange};
inline const char*tube_status_name(TubeStatus s){switch(s){case TubeStatus::ObservedClear:return "observed_clear";case TubeStatus::PotentialObstacle:return "potential_obstacle";case TubeStatus::OutsideObservationRange:return "outside_observation_range";default:return "unknown";}}
struct TubeSection {double s0=0,s1=0,cover_radius_m=0;TubeStatus status=TubeStatus::Unknown;Direction3 cover_center_W{};};
class RawDepthBallOracle {
 public:
  RawDepthBallOracle(const DepthFrame&f,const LocalGeometryTables&g,const BodyState&observation)
    :view_(f,g),depth_(f),geometry_(g),observation_(observation),
     observation_norm_(ref_rotation_norm_bound(observation.R_WB)){
    for(uint32_t camera=0;camera<4;++camera){
      std::array<double,9> rotation{};
      const auto &pose=g.physical_camera(camera).T_BC;
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)rotation[3*i+j]=pose[4*i+j];
      camera_norm_[camera]=ref_rotation_norm_bound(rotation);
    }
  }
  TubeStatus operator()(const Direction3&world,double radius)const{
    if(!view_.ready()||!std::isfinite(radius)||radius<=0)return TubeStatus::Unknown;
    radius*=observation_norm_;
    const auto c=ref_unrotate(observation_.R_WB,ref_add(world,observation_.p_W,-1));const double distance=ref_norm(c);
    if(distance+radius>BaseConfig::local_range_m)return TubeStatus::OutsideObservationRange;
    bool clear=false;
    for(uint32_t camera=0;camera<4;++camera){
      const auto projected=geometry_.project_physical(camera,c,radius*camera_norm_[camera]);
      if(projected.status!=FootprintStatus::Projected)continue;
      const auto [raw,filled]=depth_.region_pair(camera,projected.region);
      // A potential obstacle has precedence over every free-space proof.
      // Later cameras cannot reverse this result.
      if(filled.z&&*filled.z<=projected.z_far)return TubeStatus::PotentialObstacle;
      const double near=geometry_.physical_camera(camera).near_clip_z_m;
      clear|=projected.fully_inside&&std::isfinite(near)&&near>0&&projected.z_near>=near&&
        raw.total_pixels&&raw.valid_pixels==raw.total_pixels&&raw.z&&*raw.z>projected.z_far;
    }
    // The complete continuously projected sphere must fit within at least one
    // original camera ROI. Partial cameras are never combined into invented
    // coverage. Free space requires the known near plane and complete raw pixels.
    // No query uses the already inflated Omega, and no body radius is re-added.
    return clear?TubeStatus::ObservedClear:TubeStatus::Unknown;
  }
 private:
  LocalFieldView view_;
  DepthObservationView depth_;
  const LocalGeometryTables&geometry_;
  BodyState observation_;
  double observation_norm_;
  std::array<double,4> camera_norm_{};
};
// The chord midpoint and its radius enclose the full continuous tube section.
inline TubeSection reference_cover(const SpatialPosition &left,const SpatialPosition &right,
                                  double curvature,double curved_through){
  const auto delta=ref_add(right.position,left.position,-1);
  const auto center=ref_add(left.position,delta,.5);
  const double numeric=std::max(left.error_bound_m,right.error_bound_m)+
    64*std::numeric_limits<double>::epsilon()*(1+ref_norm(center));
  const double h=right.s-left.s;
  const double bend=left.s>=curved_through?0:curvature*h*h/8;
  const double radius=NavigationConfig::occupancy_tube_radius_m+ref_norm(delta)*.5+bend+numeric;
  return {left.s,right.s,radius,TubeStatus::Unknown,center};
}

class ContinuousTubeChecker {
 public:
  ContinuousTubeChecker(){sections_.reserve(32768);}
  template<class BallOracle>void check(const SpatialReference&reference,double horizon,BallOracle ball){
    sections_.clear();if(!reference.valid()||!std::isfinite(horizon)||horizon<0)return;
    const double curvature=NavigationConfig::reference_max_curvature_per_m*reference.tangent_norm_bound();
    const double step=SpatialReferenceConfig::lookup_step_m;
    const double curved_through=CurvatureSchedule::make(reference.profile().zeta_end+
      reference.angular_path().length_error()).length;
    auto left=reference.position(0);
    for(uint32_t i=0;;++i){
      const double b=std::min(horizon,(i+1)*step);
      const double h=b-left.s;
      if(h<1e-12)break;
      auto right=reference.position(b);
      auto section=reference_cover(left,right,curvature,curved_through);
      section.status=ball(section.cover_center_W,section.cover_radius_m);
      sections_.push_back(section);
      left=right;
      if(sections_.size()>=32768)throw std::runtime_error("tube interval capacity exceeded");
    }
  }
  const std::vector<TubeSection>&sections()const{return sections_;}
  void invalidate(){sections_.clear();}
  TubeStatus interval_status(double a,double b)const{
    if(!std::isfinite(a)||!std::isfinite(b)||a<0||b<a||sections_.empty()||b>sections_.back().s1+1e-12)return TubeStatus::Unknown;
    TubeStatus result=TubeStatus::ObservedClear;
    for(const auto&s:sections_)if(s.s1>=a&&s.s0<=b){
      if(s.status==TubeStatus::PotentialObstacle)return s.status;
      if(s.status==TubeStatus::Unknown)result=s.status;
      else if(s.status==TubeStatus::OutsideObservationRange&&result==TubeStatus::ObservedClear)result=s.status;
    }
    return result;
  }
 private:std::vector<TubeSection>sections_;
};
inline double observation_check_horizon(const SpatialReference&r,const BodyState&obs){
  const double L=r.profile().length;const auto end=r.query(L);const auto c=ref_add(end.position_W,obs.p_W,-1);const double dot=direction_dot(c,end.tangent_W),discriminant=dot*dot+BaseConfig::local_range_m*BaseConfig::local_range_m-direction_dot(c,c);
  const double tail=discriminant>=0?std::max(0.,-dot+std::sqrt(discriminant)):0.;
  // Geometric ray/observation-ball intersection, not a fixed reference length.
  return L+tail+SpatialReferenceConfig::lookup_step_m;
}
} // namespace ego4p0
