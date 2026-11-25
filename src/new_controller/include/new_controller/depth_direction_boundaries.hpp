#pragma once
#include "new_controller/depth_observation_storage.hpp"

namespace ego4p0 {
// Immutable sine/cosine boundaries. A precomputed q-direction supplies the usual
// cell hint; translations that move out of it use an ordered boundary search.
// Within 1e-10 of an angular boundary, use the exact legacy atan2/cell convention.
class DirectionBoundaries {
 public:
  void configure(const DirectionGridLayout &layout) {
    layout_=layout;alpha_.resize(layout.azimuth_bins+1);beta_.resize(layout.elevation_bins+1);
    for(uint32_t k=0;k<=layout.azimuth_bins;++k) {
      const double a=k==layout.azimuth_bins ? direction_pi : -direction_pi+k*layout.azimuth_step;
      alpha_[k]={std::cos(a),std::sin(a)};
    }
    alpha_.front()={-1.,0.};alpha_.back()={-1.,0.};
    for(uint32_t k=0;k<=layout.elevation_bins;++k) {
      const double b=k==layout.elevation_bins ? direction_pi/2 : -direction_pi/2+k*layout.elevation_step;
      beta_[k]={std::cos(b),std::sin(b)};
    }
    beta_.front()={0.,-1.};beta_.back()={0.,1.};
  }
  std::optional<uint32_t> exact(double x,double y,double z) const noexcept {
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || (x==0. && y==0. && z==0.)) return std::nullopt;
    return layout_.cell((x==0. && y==0.) ? 0. : std::atan2(y,x),std::atan2(z,std::hypot(x,y)));
  }
  std::optional<uint32_t> cell(double x,double y,double z,uint32_t hint,bool &used_exact) const noexcept {
    used_exact=false;
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || (x==0. && y==0. && z==0.)) return std::nullopt;
    const double scale=std::max(std::abs(x),std::abs(y)),horizontal=std::hypot(x,y);
    const double s=std::max(std::abs(z),horizontal);
    if(scale<1e-250 || !std::isfinite(horizontal) || s>std::numeric_limits<double>::max()/4) {used_exact=true;return exact(x,y,z);}
    // Relative cross-product guards avoid four divisions per representative.
    // Extreme scales fall back rather than risk under/overflow in comparisons.
    const double alpha_guard=epsilon*scale,beta_guard=epsilon*s;
    bool near=(x<0. && std::abs(y)<=alpha_guard) || horizontal<=beta_guard;
    uint32_t a=hint%layout_.azimuth_bins,b=hint/layout_.azimuth_bins;
    auto alpha_before=[&](uint32_t k) {
      if(!k) return false;
      if(k==layout_.azimuth_bins) return true;
      const auto &edge=alpha_[k];
      const bool half_v=y>0. || (y==0. && x>=0.);
      const bool half_e=edge[1]>0. || (edge[1]==0. && edge[0]>=0.);
      const double cross=x*edge[1]-y*edge[0];
      if(std::abs(cross)<=alpha_guard) near=true;
      return half_v!=half_e ? half_v<half_e : cross>0.;
    };
    const bool alpha_low=alpha_before(a),alpha_high=alpha_before(a+1);
    if(alpha_low && a>0 && !alpha_before(a-1)) --a;
    else if(!alpha_high && a+2<=layout_.azimuth_bins && alpha_before(a+2)) ++a;
    else if(alpha_low && a>1 && !alpha_before(a-2)) a-=2;
    else if(!alpha_high && a+3<=layout_.azimuth_bins && alpha_before(a+3)) a+=2;
    else if(alpha_low || !alpha_high) {
      uint32_t lo=1,hi=layout_.azimuth_bins;
      while(lo<hi) {const uint32_t mid=lo+(hi-lo)/2;if(alpha_before(mid)) hi=mid;else lo=mid+1;}
      a=lo-1;
    }
    auto beta_before=[&](uint32_t k) {
      if(!k) return false;
      if(k==layout_.elevation_bins) return true;
      const auto &edge=beta_[k];const double cross=z*edge[0]-horizontal*edge[1];
      if(std::abs(cross)<=beta_guard) near=true;
      return cross<0.;
    };
    const bool beta_low=beta_before(b),beta_high=beta_before(b+1);
    if(beta_low && b>0 && !beta_before(b-1)) --b;
    else if(!beta_high && b+2<=layout_.elevation_bins && beta_before(b+2)) ++b;
    else if(beta_low && b>1 && !beta_before(b-2)) b-=2;
    else if(!beta_high && b+3<=layout_.elevation_bins && beta_before(b+3)) b+=2;
    else if(beta_low || !beta_high) {
      uint32_t lo=1,hi=layout_.elevation_bins;
      while(lo<hi) {const uint32_t mid=lo+(hi-lo)/2;if(beta_before(mid)) hi=mid;else lo=mid+1;}
      b=lo-1;
    }
    if(near) {used_exact=true;return exact(x,y,z);}
    return b*layout_.azimuth_bins+a;
  }
 private:
  static constexpr double epsilon=1e-10;
  DirectionGridLayout layout_;
  std::vector<std::array<double,2>> alpha_,beta_;
};
} // namespace ego4p0
