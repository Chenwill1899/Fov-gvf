#pragma once
#include "new_controller/depth_preprocess.hpp"
#include "new_controller/depth_virtual_panorama.hpp"

namespace ego4p0 {
struct LocalPixelRay {std::array<double,3> q{};double z_limit=0.;};
struct LocalBlockRay {std::array<double,3> q{};double z_limit=0.,min_limit=0.,max_limit=0.,q_norm=0.,q_deviation=0.;};
enum class FootprintStatus { Projected,NearPlaneOrBehind,Invalid };
struct SphereFootprint {
  FootprintStatus status=FootprintStatus::Invalid;
  PixelRegion region;
  double z_near=0.,z_far=0.,u_min=0.,u_max=0.,v_min=0.,v_max=0.;
  bool fully_inside=false;
};
class LocalGeometryTables {
 public:
  template<class BlockRayLookup> void configure(const std::array<CameraCalibration,4> &cal,uint32_t width,uint32_t height,
                                               uint32_t block_edge,BlockRayLookup block_q,bool virtual_center=false,const DirectionGridLayout *layout=nullptr) {
    configuration_id_=0; // failed reconfiguration must invalidate old borrowed views
    const uint32_t pixels=depth_pixel_count(width,height);cal_=cal;physical_cal_=cal;virtual_center_=virtual_center;
    if(virtual_center_) {
      for(auto &c:cal_) c.T_BC[3]=c.T_BC[7]=c.T_BC[11]=0.;
      if(!layout) throw std::invalid_argument("virtual model needs direction layout");
    }width_=width;height_=height;block_edge_=block_edge;
    columns_=(width+block_edge-1)/block_edge;rows_=(height+block_edge-1)/block_edge;
    for(uint32_t c=0;c<4;++c) {
      const auto &t=cal_[c].T_BC;const auto &k=cal_[c].K;
      for(double x:t) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite local extrinsic");
      for(uint32_t i=0;i<3;++i) for(uint32_t j=0;j<3;++j) {
        double dot=0.;for(uint32_t a=0;a<3;++a) dot+=t[a*4+i]*t[a*4+j];
        if(std::abs(dot-(i==j?1.:0.))>1e-6) throw std::invalid_argument("local geometry needs rigid extrinsics");
      }
      if(std::abs(k[6])+std::abs(k[7])+std::abs(k[8]-1.)>1e-12 || k[0]<=0 || k[4]<=0)
        throw std::invalid_argument("sphere projection needs perspective optical-Z intrinsics");
      const double det=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);
      if(std::abs(det-1.)>1e-6 || std::abs(t[12])+std::abs(t[13])+std::abs(t[14])+std::abs(t[15]-1)>1e-9)
        throw std::invalid_argument("invalid local rigid transform");
      if(t[3]*t[3]+t[7]*t[7]+t[11]*t[11]>=ControllerLimits::local_range_m*ControllerLimits::local_range_m)
        throw std::invalid_argument("camera origin must be inside local ball for 0<Z<=Z_limit");
      const auto inverse=PixelRayTables::inverse_intrinsics(k);pixels_[c].resize(pixels);blocks_[c].resize(columns_*rows_);
      auto &projection=projection_[c];
      for(uint32_t i=0;i<3;++i) {
        projection.offset[i]=0.;
        for(uint32_t j=0;j<3;++j) {projection.R_CB[i*3+j]=t[j*4+i];projection.offset[i]-=t[j*4+i]*t[j*4+3];}
      }
      for(uint32_t row=0;row<2;++row) {
        projection.xy[row*2]=k[row*3];projection.xy[row*2+1]=k[row*3+1];projection.center[row]=k[row*3+2];
        projection.norm2[row]=k[row*3]*k[row*3]+k[row*3+1]*k[row*3+1];
      }
      for(uint32_t v=0;v<height;++v) for(uint32_t u=0;u<width;++u) {
        std::array<double,3> b{};for(uint32_t a=0;a<3;++a) b[a]=inverse[a*3]*u+inverse[a*3+1]*v+inverse[a*3+2];
        auto &p=pixels_[c][v*width+u];
        for(uint32_t a=0;a<3;++a) p.q[a]=t[a*4]*b[0]+t[a*4+1]*b[1]+t[a*4+2]*b[2];
        p.z_limit=virtual_center_ ? ControllerLimits::local_range_m/std::hypot(b[0],b[1],b[2]) : exit_z(p.q,t);
      }
      for(uint32_t id=0;id<columns_*rows_;++id) {
        auto &b=blocks_[c][id];b.q=block_q(c,id);b.z_limit=exit_z(b.q,t);
        b.q_norm=std::hypot(b.q[0],b.q[1],b.q[2]);b.q_deviation=0.;
        b.min_limit=std::numeric_limits<double>::infinity();b.max_limit=0.;
        const uint32_t u0=id%columns_*block_edge,v0=id/columns_*block_edge;
        // Enclose the full image block footprint including pixel half-edges.
        for(double v:{double(v0)-.5,double(std::min(v0+block_edge,height))-.5})
          for(double u:{double(u0)-.5,double(std::min(u0+block_edge,width))-.5}) {
            std::array<double,3> ray{},rotated{};
            for(uint32_t a=0;a<3;++a) ray[a]=inverse[a*3]*u+inverse[a*3+1]*v+inverse[a*3+2];
            for(uint32_t a=0;a<3;++a) rotated[a]=t[a*4]*ray[0]+t[a*4+1]*ray[1]+t[a*4+2]*ray[2]-b.q[a];
            b.q_deviation=std::max(b.q_deviation,std::hypot(rotated[0],rotated[1],rotated[2]));
          }
        b.q_deviation=std::nextafter(b.q_deviation,std::numeric_limits<double>::infinity());
        for(uint32_t v=v0;v<std::min(v0+block_edge,height);++v) for(uint32_t u=u0;u<std::min(u0+block_edge,width);++u) {
          const double z=pixels_[c][v*width+u].z_limit;b.min_limit=std::min(b.min_limit,z);b.max_limit=std::max(b.max_limit,z);
        }
      }
    }
    if(virtual_center_) virtual_tables_.configure(cal,width,height,*layout,block_edge);
    ++builds_;configuration_id_=++next_configuration_;
  }
  bool virtual_center() const noexcept {return virtual_center_;}
  const VirtualPanoramaTables &virtual_tables() const noexcept {return virtual_tables_;}
  const CameraCalibration &physical_camera(uint32_t c) const noexcept {return physical_cal_[c];}
  const LocalPixelRay &pixel(uint32_t c,uint32_t p) const noexcept {return pixels_[c][p];}
  const LocalBlockRay &block(uint32_t c,uint32_t id) const noexcept {return blocks_[c][id];}
  const CameraCalibration &camera(uint32_t c) const noexcept {return cal_[c];}
  uint64_t builds() const noexcept {return builds_;}
  uint64_t configuration_id() const noexcept {return configuration_id_;}
  bool configured() const noexcept {return configuration_id_!=0;}
  uint32_t width() const noexcept {return width_;} uint32_t height() const noexcept {return height_;}
  SphereFootprint project(uint32_t camera,const std::array<double,3> &center_B) const noexcept {
    SphereFootprint out;if(!configured() || camera>=4) return out;
    for(double x:center_B) if(!std::isfinite(x)) return out;
    const auto &projection=projection_[camera];std::array<double,3> c=projection.offset;
    for(uint32_t a=0;a<3;++a) for(uint32_t j=0;j<3;++j) c[a]+=projection.R_CB[a*3+j]*center_B[j];
    constexpr double r=ControllerLimits::radius_m;out.z_near=c[2]-r;out.z_far=c[2]+r;
    if(c[2]<=r) {out.status=FootprintStatus::NearPlaneOrBehind;return out;}
    // Tangent extrema of (k0*x+k1*y)/z for a sphere, also supporting skew.
    const double denominator=c[2]*c[2]-r*r;
    auto bounds=[&](uint32_t row,double &lo,double &hi) {
      const double s=projection.xy[row*2]*c[0]+projection.xy[row*2+1]*c[1];
      const double delta=r*std::sqrt(std::max(0.,s*s+projection.norm2[row]*denominator));
      lo=(s*c[2]-delta)/denominator+projection.center[row];hi=(s*c[2]+delta)/denominator+projection.center[row];
    };
    bounds(0,out.u_min,out.u_max);bounds(1,out.v_min,out.v_max);
    if(!std::isfinite(out.u_min) || !std::isfinite(out.u_max) || !std::isfinite(out.v_min) || !std::isfinite(out.v_max)) return out;
    out.status=FootprintStatus::Projected;
    // Extra edge pixels conservatively enclose the continuous projection.
    const double u0=std::floor(out.u_min),u1=std::ceil(out.u_max)+1.,v0=std::floor(out.v_min),v1=std::ceil(out.v_max)+1.;
    out.fully_inside=u0>=0 && v0>=0 && u1<=width_ && v1<=height_;
    // Clipping is explicit and never sufficient to certify a full sphere.
    out.region={static_cast<int>(std::clamp(u0,0.,double(width_))),static_cast<int>(std::clamp(v0,0.,double(height_))),
                static_cast<int>(std::clamp(u1,0.,double(width_))),static_cast<int>(std::clamp(v1,0.,double(height_)))};
    return out;
  }
 private:
  static double exit_z(const std::array<double,3> &q,const std::array<double,16> &t) {
    const double a=q[0]*q[0]+q[1]*q[1]+q[2]*q[2],b=q[0]*t[3]+q[1]*t[7]+q[2]*t[11];
    const double c=ControllerLimits::local_range_m*ControllerLimits::local_range_m-t[3]*t[3]-t[7]*t[7]-t[11]*t[11];
    const double discriminant=std::sqrt(b*b+a*c);
    const double z=b>=0 ? c/(discriminant+b) : (discriminant-b)/a;
    if(!std::isfinite(z) || z<=0) throw std::invalid_argument("invalid local optical-depth limit");
    return z;
  }
  std::array<CameraCalibration,4> cal_,physical_cal_;
  bool virtual_center_=false;VirtualPanoramaTables virtual_tables_;
  struct ProjectionParameters {std::array<double,9> R_CB{};std::array<double,3> offset{};std::array<double,4> xy{};std::array<double,2> center{},norm2{};};
  std::array<ProjectionParameters,4> projection_;
  std::array<std::vector<LocalPixelRay>,4> pixels_;
  std::array<std::vector<LocalBlockRay>,4> blocks_;
  uint32_t width_=0,height_=0,block_edge_=3,columns_=0,rows_=0;
  uint64_t builds_=0,configuration_id_=0;
  inline static std::atomic<uint64_t> next_configuration_{0};
};
} // namespace ego4p0
