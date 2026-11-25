#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ego4p0::spherical {

constexpr double pi = 3.14159265358979323846;
struct Bearing { double az=0., el=0.; };           // longitude, latitude [rad]
struct Radial  { Bearing dir{}; double r=0.; };    // nonnegative length
struct Tangent { double east=0., north=0.; };      // local orthonormal frame

inline double wrap(double angle) { return std::remainder(angle,2.*pi); }
inline Bearing opposite(Bearing p) { return {wrap(p.az+pi),-p.el}; }
inline double cos_lat(double el) { return std::abs(el)==pi/2. ? 0. : std::cos(el); }

// Only the sensor, file and actuator adapters need these two functions.
inline Radial from_xyz(double x,double y,double z) {
  const double horizontal=std::hypot(x,y);
  return {{std::atan2(y,x),std::atan2(z,horizontal)},std::hypot(horizontal,z)};
}
inline std::array<double,3> to_xyz(Radial p) {
  const double horizontal=p.r*cos_lat(p.dir.el);
  return {horizontal*std::cos(p.dir.az),horizontal*std::sin(p.dir.az),p.r*std::sin(p.dir.el)};
}

// The inverse great-circle problem in the moving frame at a. The north
// component uses an identity about the nearer of the two meridians to
// resolve nearly coincident bearings without subtracting large terms.
struct Separation { Tangent toward; double cos_arc; };
inline Separation separation(Bearing a,Bearing b) {
  const double d=wrap(b.az-a.az), ca=cos_lat(a.el), cb=cos_lat(b.el);
  const double sa=std::sin(a.el), sb=std::sin(b.el), cd=std::cos(d);
  const double east=std::abs(d)==pi ? 0. : cb*std::sin(d);
  if(cb==0.)return {{0.,ca*sb},sa*sb};
  if(ca==0.)return {{east,-sa*cb*cd},sa*sb};
  const double north=cd>=0. ? std::sin(b.el-a.el)+sa*cb*(1.-cd)
                            : std::sin(b.el+a.el)-sa*cb*(1.+cd);
  return {{east,north},std::clamp(sa*sb+ca*cb*cd,-1.,1.)};
}
inline double cosine(Bearing a,Bearing b) {
  return std::clamp(std::sin(a.el)*std::sin(b.el)+cos_lat(a.el)*cos_lat(b.el)*std::cos(b.az-a.az),-1.,1.);
}
inline double angle(Bearing a,Bearing b) {
  auto s=separation(a,b);
  return std::atan2(std::hypot(s.toward.east,s.toward.north),s.cos_arc);
}
inline Tangent log(Bearing a,Bearing b) {
  const auto s=separation(a,b);
  const double n=std::hypot(s.toward.east,s.toward.north);
  if(n<=4.*std::numeric_limits<double>::epsilon() && s.cos_arc<0.)return {pi,0.};
  if(n==0.)return {};
  const double k=std::atan2(n,s.cos_arc)/n;
  return {s.toward.east*k,s.toward.north*k};
}

// Direct great-circle problem. atan2 retains angular resolution at the poles.
inline Bearing advance(Bearing start,Tangent unit,double arc) {
  const double sa=std::sin(start.el), ca=cos_lat(start.el);
  const double c=std::cos(arc), s=std::sin(arc);
  const double meridian=ca*c-sa*s*unit.north, east=s*unit.east;
  return {wrap(start.az+std::atan2(east,meridian)),
          std::atan2(sa*c+ca*s*unit.north,std::hypot(meridian,east))};
}
inline Bearing exp(Bearing start,Tangent step) {
  const double n=std::hypot(step.east,step.north);
  if(n==0.)return start;
  return advance(start,{step.east/n,step.north/n},n);
}
inline Tangent transport(Bearing a,Bearing b,Tangent v) {
  const double dl=wrap(b.az-a.az)/2.;
  const double y=std::sin((a.el+b.el)/2.)*std::sin(dl);
  const double x=std::cos((a.el-b.el)/2.)*std::cos(dl);
  // At an exact antipode log() chooses the eastward great circle. With the
  // opposite pole its destination frame also depends on the chosen longitude.
  double rotation=-2.*std::atan2(y,x);
  if(std::hypot(x,y)<=4.*std::numeric_limits<double>::epsilon()){
    rotation=std::abs(a.el)==pi/2. ? std::copysign(1.,a.el)*wrap(2.*dl-pi) : 0.;
  }
  const double c=std::cos(rotation),s=std::sin(rotation);
  return {c*v.east-s*v.north,s*v.east+c*v.north};
}

inline Radial scale(Radial p,double k) {
  return k<0. ? Radial{opposite(p.dir),p.r*(-k)} : Radial{p.dir,p.r*k};
}

// Signed triangle composition in the (a,b) great circle; no world point is
// assembled. The half-chord identity preserves the small angle in a distance.
inline Radial add(Radial a,Radial b,double k=1.) {
  const double br=k*b.r;
  if(br==0.)return a;
  if(a.r==0.)return scale(b,k);
  const auto s=separation(a.dir,b.dir);
  const double n=std::hypot(s.toward.east,s.toward.north);
  const double forward=s.cos_arc>=0. ? std::fma(-br,n*n/(1.+s.cos_arc),a.r+br)
                                     : std::fma(br,n*n/(1.-s.cos_arc),a.r-br);
  const double side=br*n;
  if(n==0.)return scale({a.dir,1.},forward);
  const double r=std::hypot(forward,side);
  return {advance(a.dir,{s.toward.east/n,s.toward.north/n},std::atan2(side,forward)),r};
}
inline double distance(Radial a,Radial b) {
  const double u=std::sin((b.dir.el-a.dir.el)/2.), v=std::sin((b.dir.az-a.dir.az)/2.);
  const double half_chord=std::sqrt(u*u+cos_lat(a.dir.el)*cos_lat(b.dir.el)*v*v);
  return std::hypot(a.r-b.r,2.*(std::sqrt(a.r)*std::sqrt(b.r))*half_chord);
}

// Rotate the initial bearing of the geodesic from axis to value. The
// colatitude stays fixed along the resulting small circle.
inline Bearing rotate_about(Bearing axis,Bearing value,double rotation) {
  if(rotation==0.)return value;
  const auto step=log(axis,value);
  const double c=std::cos(rotation), s=std::sin(rotation);
  return exp(axis,{c*step.east-s*step.north,s*step.east+c*step.north});
}
inline Radial cross(Bearing a,Bearing b) {
  const auto s=separation(a,b);
  const double n=std::hypot(s.toward.east,s.toward.north);
  if(n==0.)return {};
  return {advance(a,{-s.toward.north/n,s.toward.east/n},pi/2.),n};
}

} // namespace ego4p0::spherical
