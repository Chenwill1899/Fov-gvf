#pragma once
#include "new_controller/harmonic_types.hpp"
namespace ego4p0 {
// Vector/coordinate helpers for observed and actual state, without spatial paths.
inline Direction3 nav_add(Direction3 a,const Direction3&b,double k=1.){for(int i=0;i<3;++i)a[i]+=k*b[i];return a;}
inline Direction3 nav_scale(Direction3 a,double k){for(double&v:a)v*=k;return a;}
inline double nav_norm(const Direction3&a){return std::hypot(a[0],a[1],a[2]);}
inline Direction3 nav_cross(const Direction3&a,const Direction3&b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Direction3 nav_rotate(const std::array<double,9>&R,const Direction3&q){Direction3 a{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)a[i]+=R[3*i+j]*q[j];return a;}
inline Direction3 nav_unrotate(const std::array<double,9>&R,const Direction3&q){Direction3 a{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)a[i]+=R[3*j+i]*q[j];return a;}
inline double nav_angle(const Direction3&a,const Direction3&b){return std::atan2(nav_norm(nav_cross(a,b)),direction_dot(a,b));}
} // namespace ego4p0
