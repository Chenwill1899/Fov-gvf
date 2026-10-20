#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <algorithm>
namespace pc_gvf { namespace depth_angular {
// Conservative broad phase for a whole piecewise capsule path. It may only
// cull distant obstacles; possible intersections still need exact segment tests.
struct CapsulePathBounds {
 Eigen::Vector3d a=Eigen::Vector3d::Zero(),direction=Eigen::Vector3d::Zero();
 double inverse_length_squared=0.,radius=0.;bool valid=false;
 double distanceSquared(const Eigen::Vector3d& p)const {
  const double t=inverse_length_squared>0.?std::max(0.,std::min(1.,(p-a).dot(direction)*inverse_length_squared)):0.;
  return (p-a-t*direction).squaredNorm();
 }
 bool mayTouch(const Eigen::Vector3d& p,double extra=0.)const {
  if(!valid||!p.allFinite()||!std::isfinite(extra)||extra<0.)return true;
  const double r=radius+extra;return distanceSquared(p)<=r*r;
 }
};
template<class Segments>CapsulePathBounds capsulePathBounds(const Segments& path) {
 CapsulePathBounds result;if(path.empty())return result;
 result.a=path.front().a;result.direction=path.back().b-result.a;
 const double length_squared=result.direction.squaredNorm();
 if(!result.a.allFinite()||!result.direction.allFinite()||!std::isfinite(length_squared))return result;
 result.inverse_length_squared=length_squared>1e-16?1./length_squared:0.;
 double radius=0.,bulge_squared=0.;
 for(const auto& s:path) {
  if(!s.a.allFinite()||!s.b.allFinite()||!std::isfinite(s.radius)||s.radius<0.)return result;
  radius=std::max(radius,s.radius);
  bulge_squared=std::max({bulge_squared,result.distanceSquared(s.a),result.distanceSquared(s.b)});
 }
 result.radius=radius+std::sqrt(bulge_squared)+1e-9;
 result.valid=std::isfinite(result.radius);return result;
}
// One immutable risk-accounting scope. New positive derived proofs may make
// a previously conservative negative query provable; revision tokens prevent
// that negative result from surviving proof-cache growth.
class CapsuleProofMemo {
public:
 using Revision=std::array<std::uint64_t,4>;
 explicit CapsuleProofMemo(std::size_t capacity=4096):capacity_(capacity){}
 template<class Versions,class Query>bool evaluate(const Eigen::Vector3d& a,const Eigen::Vector3d& b,double r,Versions&& versions,Query&& query){
  if(!a.allFinite()||!b.allFinite()||!std::isfinite(r))return query();
  const std::array<double,7> key{{a.x(),a.y(),a.z(),b.x(),b.y(),b.z(),r}};
  const Revision before=versions();auto old=entries_.find(key);
  if(old!=entries_.end()&&(old->second.known||old->second.revision==before))return old->second.known;
  const bool known=query();const Revision after=versions();
  if(known||before==after){
   if(old!=entries_.end())old->second={known,after};
   else if(entries_.size()<capacity_)entries_.emplace(key,Entry{known,after});
  }else if(old!=entries_.end())entries_.erase(old);
  return known;
 }
 std::size_t size()const{return entries_.size();}
 void clear(){entries_.clear();}
private:
 struct Entry{bool known;Revision revision;};std::map<std::array<double,7>,Entry> entries_;std::size_t capacity_;
};
} }
