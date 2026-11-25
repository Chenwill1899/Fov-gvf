#pragma once
#include "new_controller/depth_input.hpp"
#include <algorithm>
#include <cstring>
#include <limits>

namespace ego4p0 {
inline size_t depth_pixel_count(size_t width,size_t height) {
  if(!width || !height || width>4096 || height>4096)
    throw std::invalid_argument("invalid depth image dimensions");
  return width*height;
}

// Finite positive optical-axis Z in meters. No range clamp or smoothing.
inline bool valid_depth(float z) noexcept { return std::isfinite(z) && z>0.f; }

// Source and destination must not overlap. Both are preallocated by the owner.
// All neighborhoods read src only: a newly repaired pixel never propagates.
inline DepthRepairStats fill_invalid_depth(const float *src,float *dst,size_t width,size_t height) {
  const size_t pixels=depth_pixel_count(width,height),bytes=pixels*sizeof(float);
  const auto a=reinterpret_cast<uintptr_t>(src),b=reinterpret_cast<uintptr_t>(dst);
  if(!src || !dst || (a<=b ? b-a<bytes : a-b<bytes))
    throw std::invalid_argument("depth repair needs separate original/output buffers");
  std::memcpy(dst,src,bytes);  // preserve every valid/unresolved pixel bit-for-bit
  DepthRepairStats result;
  for(size_t v=0;v<height;++v) for(size_t u=0;u<width;++u) {
    const size_t index=v*width+u;
    if(valid_depth(src[index])) continue;
    ++result.invalid;
    const size_t x0=u ? u-1 : 0,x1=std::min(u+1,width-1);
    const size_t y0=v ? v-1 : 0,y1=std::min(v+1,height-1);
    double sum=0.;unsigned count=0;
    for(size_t y=y0;y<=y1;++y) for(size_t x=x0;x<=x1;++x) {
      const float z=src[y*width+x];
      if(valid_depth(z)) { sum+=static_cast<double>(z);++count; }
    }
    if(count) { dst[index]=static_cast<float>(sum/count);++result.filled; }
    else ++result.unresolved;
  }
  return result;
}

struct PixelRay {
  std::array<double,3> b{},d{}; // K^-1[u,v,1], then unit camera-optical direction
};

class PixelRayTables {
 public:
  static std::array<double,9> inverse_intrinsics(const std::array<double,9> &k) { return invert(k); }
  // Called before starting receiver/control threads. Identical K/shape reuses
  // storage and values; extrinsics do not enter a camera-optical ray table.
  bool configure(const std::array<CameraCalibration,4> &cal,size_t width,size_t height) {
    const size_t pixels=depth_pixel_count(width,height);
    bool unchanged=builds_ && width==width_ && height==height_;
    for(size_t i=0;i<4;++i) unchanged=unchanged && K_[i]==cal[i].K;
    if(unchanged) return false;
    std::array<std::vector<PixelRay>,4> tables;
    std::array<std::array<double,9>,4> keys;
    for(size_t camera=0;camera<4;++camera) {
      keys[camera]=cal[camera].K;
      const auto inverse=invert(keys[camera]);
      tables[camera].resize(pixels);
      for(size_t v=0;v<height;++v) for(size_t u=0;u<width;++u) {
        auto &ray=tables[camera][v*width+u];
        for(size_t k=0;k<3;++k)
          ray.b[k]=inverse[k*3]*static_cast<double>(u)+inverse[k*3+1]*static_cast<double>(v)+inverse[k*3+2];
        const double norm=std::hypot(ray.b[0],ray.b[1],ray.b[2]);
        if(!std::isfinite(norm) || norm<=0.) throw std::invalid_argument("invalid pixel ray");
        for(size_t k=0;k<3;++k) ray.d[k]=ray.b[k]/norm;
      }
    }
    tables_=std::move(tables);K_=keys;width_=width;height_=height;++builds_;
    return true;
  }
  const PixelRay &at(size_t camera,size_t u,size_t v) const {
    if(camera>=4 || u>=width_ || v>=height_) throw std::out_of_range("pixel ray coordinate");
    return tables_[camera][v*width_+u];
  }
  const std::vector<PixelRay> &camera(size_t index) const { return tables_.at(index); }
  size_t width() const noexcept { return width_; }
  size_t height() const noexcept { return height_; }
  uint64_t builds() const noexcept { return builds_; }
  size_t bytes() const noexcept { return 4*width_*height_*sizeof(PixelRay); }
 private:
  static std::array<double,9> invert(const std::array<double,9> &matrix) {
    double scale=0.;
    for(double x:matrix) {
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera K");
      scale=std::max(scale,std::abs(x));
    }
    if(scale==0.) throw std::invalid_argument("singular camera K");
    std::array<double,9> a{},inverse{};
    for(size_t i=0;i<9;++i) a[i]=matrix[i]/scale;
    const double det=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
    if(!std::isfinite(det) || det==0.) throw std::invalid_argument("singular camera K");
    inverse={a[4]*a[8]-a[5]*a[7],a[2]*a[7]-a[1]*a[8],a[1]*a[5]-a[2]*a[4],
             a[5]*a[6]-a[3]*a[8],a[0]*a[8]-a[2]*a[6],a[2]*a[3]-a[0]*a[5],
             a[3]*a[7]-a[4]*a[6],a[1]*a[6]-a[0]*a[7],a[0]*a[4]-a[1]*a[3]};
    for(double &x:inverse) {
      x=(x/det)/scale;
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera K inverse");
    }
    return inverse;
  }
  size_t width_=0,height_=0;uint64_t builds_=0;
  std::array<std::array<double,9>,4> K_{};
  std::array<std::vector<PixelRay>,4> tables_;
};
} // namespace ego4p0
