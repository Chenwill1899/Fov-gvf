#pragma once
#include "new_controller/depth_local_geometry.hpp"

namespace ego4p0 {
enum class BlockDomainEvidence { Unknown,CompleteFarObservation,ObstacleExclusion };
struct AngularExclusion {
  uint32_t a0=0,a1=0,b0=0,b1=0; // half-open; a0>a1 means azimuth seam wrap
  bool present=false;
  bool contains(uint32_t cell,const DirectionGridLayout &layout) const noexcept {
    const uint32_t a=cell%layout.azimuth_bins,b=cell/layout.azimuth_bins;
    return present && b>=b0 && b<b1 && (a0<=a1 ? a>=a0 && a<a1 : a>=a0 || a<a1);
  }
};
inline BlockDomainEvidence block_domain_evidence(const BlockDepth &b,const LocalBlockRay &g,uint32_t total) noexcept {
  if(b.valid_pixels && b.min_z<=g.max_limit) return BlockDomainEvidence::ObstacleExclusion;
  if(b.raw_valid_pixels==total && valid_depth(b.raw_min_z) && b.raw_min_z>g.max_limit)
    return BlockDomainEvidence::CompleteFarObservation;
  return BlockDomainEvidence::Unknown;
}
// One ball encloses ALL candidate near surfaces in this image block, then is
// inflated by the body radius. Bounds are conservative exclusions, never free.
inline AngularExclusion block_exclusion(const BlockDepth &b,const LocalBlockRay &g,
    const std::array<double,16> &t,const DirectionGridLayout &layout) noexcept {
  AngularExclusion out;if(!b.valid_pixels || b.min_z>g.max_limit) return out;
  const double lo=b.min_z,hi=std::min(double(b.max_z),g.max_limit),mid=(lo+hi)*.5;
  const double x=mid*g.q[0]+t[3],y=mid*g.q[1]+t[7],z=mid*g.q[2]+t[11];
  const double horizontal=std::hypot(x,y),distance=std::hypot(horizontal,z);
  const double radius=(hi-lo)*.5*g.q_norm+hi*g.q_deviation+ControllerLimits::radius_m
    +1e-12*(1.+distance+hi*g.q_norm);
  out.present=true;
  if(distance<=radius) return {0,layout.azimuth_bins,0,layout.elevation_bins,true};
  const double delta=std::asin(std::min(1.,radius/distance)),beta=std::atan2(z,horizontal);
  constexpr double pad=1e-10;
  const double lower=std::max(-direction_pi/2,beta-delta-pad),upper=std::min(direction_pi/2,beta+delta+pad);
  out.b0=std::min(uint32_t(std::max(0.,std::floor((lower+direction_pi/2)/layout.elevation_step))),layout.elevation_bins-1);
  out.b1=std::min(uint32_t(std::max(0.,std::floor((upper+direction_pi/2)/layout.elevation_step)+1)),layout.elevation_bins);
  if(horizontal<=radius || std::abs(beta)+delta+pad>=direction_pi/2) {out.a1=layout.azimuth_bins;return out;}
  const double alpha=std::atan2(y,x),half=std::asin(std::min(1.,radius/horizontal))+pad;
  auto wrap=[](double a) {if(a< -direction_pi) a+=2*direction_pi;if(a>=direction_pi) a-=2*direction_pi;return a;};
  const bool wraps=wrap(alpha-half)>wrap(alpha+half);
  out.a0=std::min(uint32_t(std::max(0.,std::floor((wrap(alpha-half)+direction_pi)/layout.azimuth_step))),layout.azimuth_bins-1);
  out.a1=std::min(uint32_t(std::max(0.,std::floor((wrap(alpha+half)+direction_pi)/layout.azimuth_step)+1)),layout.azimuth_bins);
  if(wraps && out.a0<=out.a1) {out.a0=0;out.a1=layout.azimuth_bins;}
  return out;
}
} // namespace ego4p0
