#pragma once
#include "new_controller/harmonic_solver.hpp"

#ifndef EGO4P0_CURVE_GUARD
#define EGO4P0_CURVE_GUARD 0
#endif
#ifndef EGO4P0_CORNER_SMALL_STEP
#define EGO4P0_CORNER_SMALL_STEP 0
#endif
static_assert(EGO4P0_CURVE_GUARD==0||EGO4P0_CURVE_GUARD==1,"invalid curve guard mode");
static_assert(EGO4P0_CORNER_SMALL_STEP==0||EGO4P0_CORNER_SMALL_STEP==1,"invalid corner step mode");
namespace ego4p0 {
enum CurveFailureBits : uint8_t {CurveArtificial=1,CurveUnknown=2,CurveObstacle=4,CurveNumerical=8,CurveNoProgress=16};
struct CurveEvidence {
  uint8_t kind=0;uint32_t cell=UINT32_MAX;
  void add(uint8_t reason,uint32_t at=UINT32_MAX) {
    const bool real=reason&(CurveUnknown|CurveObstacle),had_real=kind&(CurveUnknown|CurveObstacle);
    if(!kind||(real&&!had_real))cell=at;
    kind|=reason;
  }
  bool pure_artificial()const{return kind==CurveArtificial;}
};
class FineCurveEvidence {
 public:
  static void missing(const FieldLayer &f,uint32_t i,CurveEvidence &e) {
    const auto state=CandidateStatus(f.state[i]);
    e.add(state==CandidateStatus::Available ? CurveArtificial : state==CandidateStatus::Unavailable ? CurveObstacle : CurveUnknown,i);
  }
  static bool covered(const FieldLayer &f,uint32_t i,CurveEvidence &e) {
    if(f.state[i]!=uint8_t(CandidateStatus::Available)){missing(f,i,e);return false;}
    if(f.boundary[i])return true;
    missing(f,i,e);return false;
  }
  static CurveEvidence failed_query(const SphericalStencil &s,const FieldLayer &f,const Direction3 &q) {
    CurveEvidence e;const auto i=s.cell(q);
    if(!covered(f,i,e))return e;
    if(std::abs(q[2])>std::sin(84*direction_pi/180)) {
      const bool north=q[2]>=0;
      for(uint32_t j=0;j<s.nodes.size();++j)if((north ? s.nodes[j].direction[2] : -s.nodes[j].direction[2])>=std::sin(84*direction_pi/180)) {
        if(!f.boundary[j]||f.state[j]!=uint8_t(CandidateStatus::Available))missing(f,j,e);
      }
    }
    if(!e.kind)e.add(CurveNumerical,i);
    return e;
  }
  static CurveEvidence coupling(const SphericalStencil &s,const FieldLayer &f,uint32_t i) {
    CurveEvidence e;
    if(!covered(f,i,e))return e;
    for(auto j:s.nodes[i].neighbor)if(j!=UINT32_MAX&&f.state[j]==uint8_t(CandidateStatus::Available)&&!f.boundary[j])e.add(CurveArtificial,j);
    if(!e.kind)e.add(CurveNumerical,i);
    return e;
  }
  // Identical conservative supercover to the legacy checker. The additional
  // state check rejects inconsistent BCs that claim a raw Unknown is usable.
  static bool segment(const SphericalStencil &s,const FieldLayer &f,const Direction3 &a,const Direction3 &b,CurveEvidence &e,uint32_t depth=0) {
    if(f.boundary.size()!=s.nodes.size()||f.state.size()!=s.nodes.size()||!std::isfinite(direction_dot(a,a))||!std::isfinite(direction_dot(b,b))||std::abs(direction_dot(a,a)-1)>1e-8||std::abs(direction_dot(b,b)-1)>1e-8) {e.add(CurveNumerical);return false;}
    const auto ia=s.cell(a),ib=s.cell(b);if(!covered(f,ia,e)||!covered(f,ib,e))return false;
    Direction3 mid{};for(uint32_t k=0;k<3;++k)mid[k]=a[k]+b[k];if(!unit_direction(mid)){e.add(CurveNumerical);return false;}
    const auto im=s.cell(mid);if(!covered(f,im,e))return false;
    const double aa=std::atan2(a[1],a[0]),ab=std::atan2(b[1],b[0]),ba=std::atan2(a[2],std::hypot(a[0],a[1])),bb=std::atan2(b[2],std::hypot(b[0],b[1]));
    const auto na=s.layout.azimuth_bins;
    if(std::hypot(mid[0],mid[1])<1e-8) {
      const uint32_t row=mid[2]>0 ? s.layout.elevation_bins-1 : 0;
      for(uint32_t j=row*na;j<(row+1)*na;++j)if(!covered(f,j,e))return false;
      return true;
    }
    if(std::abs(wrapped_angle(ab-aa))>.25*s.layout.azimuth_step||std::abs(bb-ba)>.25*s.layout.elevation_step||(ia!=im&&ib!=im)) {
      if(depth>=20){e.add(CurveNumerical);return false;}
      return segment(s,f,a,mid,e,depth+1)&&segment(s,f,mid,b,e,depth+1);
    }
    const uint32_t ra=ia/na,rb=ib/na,ca=ia%na,cb=ib%na;
    if(ra!=rb&&ca!=cb&&(!covered(f,ra*na+cb,e)||!covered(f,rb*na+ca,e)))return false;
    return true;
  }
  static double arc_length(const Direction3 &a,const Direction3 &b) {
    const Direction3 c{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
    return std::atan2(std::hypot(c[0],c[1],c[2]),std::clamp(direction_dot(a,b),-1.,1.));
  }
};
} // namespace ego4p0
