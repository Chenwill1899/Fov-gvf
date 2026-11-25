#pragma once
#include "new_controller/depth_preprocess.hpp"
#include "new_controller/spherical_geometry.hpp"
#include <algorithm>

namespace ego4p0 {
inline bool valid_range(double rho) noexcept {return std::isfinite(rho) && rho>0.;}
struct PanoramaWindow {uint32_t a0=0,a1=0,b0=0,b1=0;};
struct PanoramaNodeSources {std::array<uint32_t,4> blocks{};uint8_t cameras=0;};
// Intentional approximation: co-located optical centers at the body origin.
// K and rotations are retained; physical translations and clipping planes are
// not used by this table or by the corresponding planning queries.
class VirtualPanoramaTables {
 public:
  static constexpr double distance_bin_m=.05;
  static constexpr uint32_t distance_bins=uint32_t(ControllerLimits::local_range_m/distance_bin_m+0.5)+1,range_tile_edge=8;
  void configure(const std::array<CameraCalibration,4> &cal,uint32_t w,uint32_t h,
                 const DirectionGridLayout &layout,uint32_t edge) {
    layout_=layout;w_=w;h_=h;edge_=edge;columns_=(w+edge-1)/edge;rows_=(h+edge-1)/edge;
    const uint32_t blocks=columns_*rows_;maximum_block_half_angle_=0.;
    std::array<std::array<spherical::Radial,3>,4> camera_axes;
    for(uint32_t c=0;c<4;++c) {
      const auto inverse=PixelRayTables::inverse_intrinsics(cal[c].K);const auto &t=cal[c].T_BC;
      for(unsigned axis=0;axis<3;++axis)
        camera_axes[c][axis]=spherical::from_xyz(t[axis],t[4+axis],t[8+axis]);
      coefficients_[c].resize(w*h);pixel_cells_[c].resize(w*h);block_cells_[c].resize(blocks);
      auto q=[&](double u,double v) {
        std::array<double,3> b{},rotated{};
        for(uint32_t a=0;a<3;++a) b[a]=inverse[a*3]*u+inverse[a*3+1]*v+inverse[a*3+2];
        for(uint32_t a=0;a<3;++a) rotated[a]=t[a*4]*b[0]+t[a*4+1]*b[1]+t[a*4+2]*b[2];
        return rotated;
      };
      for(uint32_t v=0;v<h;++v) for(uint32_t u=0;u<w;++u) {
        std::array<double,3> b{};for(uint32_t a=0;a<3;++a) b[a]=inverse[a*3]*u+inverse[a*3+1]*v+inverse[a*3+2];
        coefficients_[c][v*w+u]=std::hypot(b[0],b[1],b[2]);
        const auto rotated=q(u,v);pixel_cells_[c][v*w+u]=cell(rotated);
      }
      for(uint32_t id=0;id<blocks;++id) {
        const uint32_t u0=id%columns_*edge,v0=id/columns_*edge,u1=std::min(u0+edge,w),v1=std::min(v0+edge,h);
        const auto center=q((u0+u1-1)*.5,(v0+v1-1)*.5);const double norm=std::hypot(center[0],center[1],center[2]);
        block_cells_[c][id]=cell(center);double deviation=0.;
        for(double u:{double(u0)-.5,double(u1)-.5}) for(double v:{double(v0)-.5,double(v1)-.5}) {
          const auto corner=q(u,v);deviation=std::max(deviation,std::hypot(corner[0]-center[0],corner[1]-center[1],corner[2]-center[2]));
        }
        const double half=deviation>=norm ? direction_pi : std::asin(std::min(1.,deviation/norm));
        maximum_block_half_angle_=std::max(maximum_block_half_angle_,half);
      }
    }
    active_cells_.clear();
    nodes_.assign(layout.cells(),PanoramaNodeSources{});
    for(uint32_t b=0;b<layout.elevation_bins;++b) for(uint32_t a=0;a<layout.azimuth_bins;++a) {
      auto &node=nodes_[b*layout.azimuth_bins+a];node.blocks.fill(UINT32_MAX);
      const spherical::Bearing bearing{alpha_center(a),beta_center(b)};
      for(uint32_t c=0;c<4;++c) {
        // Gnomonic coordinates are ratios of angular cosines with the
        // calibrated chart axes. Retain their gains for rounded extrinsics.
        const auto &axes=camera_axes[c];const auto &k=cal[c].K;
        const double axial=axes[2].r*spherical::cosine(bearing,axes[2].dir);
        if(axial<=0.)continue;
        const double horizontal=axes[0].r*spherical::cosine(bearing,axes[0].dir);
        const double vertical=axes[1].r*spherical::cosine(bearing,axes[1].dir);
        const double u=(k[0]*horizontal+k[1]*vertical)/axial+k[2];
        const double v=(k[3]*horizontal+k[4]*vertical)/axial+k[5];
        if(u<-.5 || u>=double(w)-.5 || v<-.5 || v>=double(h)-.5) continue;
        node.blocks[c]=uint32_t((v+.5)/edge)*columns_+uint32_t((u+.5)/edge);node.cameras|=uint8_t(1U<<c);
      }
    }
    for(uint32_t cell=0;cell<layout.cells();++cell) if(nodes_[cell].cameras) active_cells_.push_back(cell);
    query_.resize(size_t(layout.elevation_bins)*distance_bins);obstacle_.resize(query_.size());
    // Grid-center quantization and whole image-block angular extent are bounded
    // separately. All templates are conservative within the virtual model.
    const double quantization=(layout.azimuth_step+layout.elevation_step)*.5;
    for(uint32_t b=0;b<layout.elevation_bins;++b) for(uint32_t k=0;k<distance_bins;++k) {
      const double lower=std::max(0.,double(k)*distance_bin_m-1e-9);
      const double inflation=lower<=ControllerLimits::radius_m ? direction_pi : std::asin(ControllerLimits::radius_m/lower);
      query_[size_t(b)*distance_bins+k]=row_window(b,inflation+quantization);
      obstacle_[size_t(b)*distance_bins+k]=row_window(b,inflation+quantization+maximum_block_half_angle_);
    }
    domain_windows_.resize(layout.cells());
    for(uint32_t cell=0;cell<layout.cells();++cell) domain_windows_[cell]=window(cell,distance_bin(ControllerLimits::local_range_m));
  }
  const std::vector<uint32_t> &active_cells() const noexcept {return active_cells_;}
  const PanoramaWindow &domain_window(uint32_t cell) const noexcept {return domain_windows_[cell];}
  double coefficient(uint32_t c,uint32_t p) const noexcept {return coefficients_[c][p];}
  const std::vector<double> &coefficients(uint32_t c) const noexcept {return coefficients_[c];}
  uint32_t pixel_cell(uint32_t c,uint32_t p) const noexcept {return pixel_cells_[c][p];}
  uint32_t block_cell(uint32_t c,uint32_t id) const noexcept {return block_cells_[c][id];}
  const PanoramaNodeSources &node(uint32_t cell) const noexcept {return nodes_[cell];}
  double maximum_block_half_angle() const noexcept {return maximum_block_half_angle_;}
  const DirectionGridLayout &layout() const noexcept {return layout_;}
  static uint32_t distance_bin(double rho) noexcept {
    return uint32_t(std::clamp(std::floor(rho/distance_bin_m),0.,double(distance_bins-1)));
  }
  PanoramaWindow window(uint32_t cell,uint32_t bin,bool obstacle=false) const noexcept {
    const uint32_t a=cell%layout_.azimuth_bins,b=cell/layout_.azimuth_bins;
    const auto &t=(obstacle ? obstacle_ : query_)[size_t(b)*distance_bins+bin];
    if(t.padding*2+1>=layout_.azimuth_bins) return {0,layout_.azimuth_bins,t.b0,t.b1};
    const int64_t na=layout_.azimuth_bins;
    const uint32_t a0=uint32_t((int64_t(a)-t.padding+na)%na),a1=uint32_t((int64_t(a)+t.padding+1)%na);
    return {a0,a1,t.b0,t.b1}; // wrap if a0>a1
  }
 private:
  struct RowWindow {uint32_t padding=0,b0=0,b1=0;};
  uint32_t cell(const std::array<double,3> &q) const {
    const auto result=layout_.cell((q[0]==0. && q[1]==0.) ? 0. : std::atan2(q[1],q[0]),std::atan2(q[2],std::hypot(q[0],q[1])));
    if(!result) throw std::invalid_argument("invalid virtual ray");
    return *result;
  }
  double alpha_center(uint32_t a) const noexcept {const double lo=-direction_pi+a*layout_.azimuth_step;return (lo+std::min(lo+layout_.azimuth_step,direction_pi))*.5;}
  double beta_center(uint32_t b) const noexcept {const double lo=-direction_pi/2+b*layout_.elevation_step;return (lo+std::min(lo+layout_.elevation_step,direction_pi/2))*.5;}
  RowWindow row_window(uint32_t b,double delta) const noexcept {
    constexpr double pad=1e-10;const double beta=beta_center(b);RowWindow out;
    if(delta>=direction_pi/2) return {layout_.azimuth_bins,0,layout_.elevation_bins};
    const double lo=std::max(-direction_pi/2,beta-delta-pad),hi=std::min(direction_pi/2,beta+delta+pad);
    out.b0=std::min(uint32_t(std::max(0.,std::floor((lo+direction_pi/2)/layout_.elevation_step))),layout_.elevation_bins-1);
    out.b1=std::min(uint32_t(std::floor((hi+direction_pi/2)/layout_.elevation_step)+1),layout_.elevation_bins);
    if(std::abs(beta)+delta+pad>=direction_pi/2) out.padding=layout_.azimuth_bins;
    else out.padding=uint32_t(std::ceil((std::asin(std::min(1.,std::sin(delta)/std::cos(beta)))+pad)/layout_.azimuth_step))+1;
    return out;
  }
  DirectionGridLayout layout_;uint32_t w_=0,h_=0,edge_=3,columns_=0,rows_=0;
  double maximum_block_half_angle_=0.;
  std::array<std::vector<double>,4> coefficients_;
  std::array<std::vector<uint32_t>,4> pixel_cells_,block_cells_;
  std::vector<uint32_t> active_cells_;std::vector<PanoramaWindow> domain_windows_;
  std::vector<PanoramaNodeSources> nodes_;std::vector<RowWindow> query_,obstacle_;
};
} // namespace ego4p0
