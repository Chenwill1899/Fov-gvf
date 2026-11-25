#pragma once

#include "new_controller/navigation_pipeline.hpp"
#include "new_controller/spherical_geometry.hpp"
#include <optional>

namespace ego4p0 {

class ImageDepthRig;
struct CruiseDepthFrame;

struct CruiseBallOracle {
  const CruiseDepthFrame &frame;
  const LocalGeometryTables &geometry;
  const ImageDepthRig &rig;
  TubeStatus operator()(spherical::Radial world,double radius) const;
};

struct CruiseDepthFrame : NavigationObservation {
  spherical::Radial world_position{};
  double yaw=0.;
  std::optional<CruiseBallOracle> raw;

  void prepare(const LocalGeometryTables &geometry,const ImageDepthRig &rig,
               spherical::Radial measured_position,double measured_yaw) {
    world_position=measured_position;
    yaw=measured_yaw;
    raw.emplace(CruiseBallOracle{*this,geometry,rig});
  }

  spherical::Radial in_body(spherical::Radial world) const {
    auto relative=spherical::add(world,world_position,-1.);
    relative.dir.az-=yaw;
    return relative;
  }
};

class ImageDepthRig {
  using Radial=spherical::Radial;
  struct Camera {
    Radial origin{},depth_axis{};
    std::array<Radial,2> chart_axis{};
    std::array<double,2> principal{};
    double near_clip=0.,half_pixel=0.,calibration_slack=0.;
    double min_coefficient=std::numeric_limits<double>::infinity();
    std::vector<Radial> rays;
  };

 public:
  void configure(const LocalGeometryTables &geometry) {
    configuration_=0;
    width_=int(geometry.width());
    height_=int(geometry.height());
    if(!geometry.configured() || !width_ || !height_)
      throw std::invalid_argument("unconfigured cruise camera geometry");
    for(unsigned i=0;i<4;++i) {
      auto &camera=cameras_[i];
      const auto &cal=geometry.physical_camera(i);
      const auto &t=cal.T_BC;
      camera.origin=spherical::from_xyz(t[3],t[7],t[11]);
      camera.near_clip=cal.near_clip_z_m;

      // Calibration is the sensor boundary. Rows of K R^{-1} measure
      // u*Z, v*Z and Z. Store each image axis by its bearing and gain.
      const std::array<double,9> rotation{t[0],t[1],t[2],t[4],t[5],t[6],t[8],t[9],t[10]};
      const auto inverse=PixelRayTables::inverse_intrinsics(rotation);
      std::array<std::array<double,3>,3> row{};
      for(int a=0;a<3;++a)for(int b=0;b<3;++b)for(int k=0;k<3;++k)
        row[a][b]+=cal.K[3*a+k]*inverse[3*k+b];
      // Old calibrated envelopes used R^T and inflated by ||R||. Include
      // that envelope as well when readback is a rounded rigid rotation.
      camera.calibration_slack=0.;
      for(int a=0;a<3;++a) {
        double row_error=0.,column_error=0.;
        for(int b=0;b<3;++b) {
          double rows=0.,columns=0.;
          for(int k=0;k<3;++k) {
            rows+=rotation[3*a+k]*rotation[3*b+k];
            columns+=rotation[3*k+a]*rotation[3*k+b];
          }
          row_error+=std::abs(rows-(a==b));
          column_error+=std::abs(columns-(a==b));
        }
        camera.calibration_slack=std::max({camera.calibration_slack,row_error,column_error});
      }
      const auto &z=row[2];
      camera.depth_axis=spherical::from_xyz(z[0],z[1],z[2]);
      const double gain=camera.depth_axis.r;
      for(int a=0;a<2;++a) {
        const double offset=direction_dot(row[a],z)/(gain*gain);
        camera.principal[a]=offset;
        camera.chart_axis[a]=spherical::from_xyz((row[a][0]-offset*z[0])/gain,
          (row[a][1]-offset*z[1])/gain,(row[a][2]-offset*z[2])/gain);
      }
      camera.rays.resize(size_t(width_)*height_);
      camera.min_coefficient=std::numeric_limits<double>::infinity();
      for(size_t p=0;p<camera.rays.size();++p) {
        const auto &q=geometry.pixel(i,p).q;
        camera.rays[p]=spherical::from_xyz(q[0],q[1],q[2]);
        camera.min_coefficient=std::min(camera.min_coefficient,camera.rays[p].r);
      }
      // K^{-1} is affine in (u,v) at fixed sensor Z, so half of the
      // two basis lengths encloses every point within a pixel.
      const auto ik=PixelRayTables::inverse_intrinsics(cal.K);
      auto increment=[&](int column){
        double squared=0.;
        for(int a=0;a<3;++a) {
          const double x=t[4*a]*ik[column]+t[4*a+1]*ik[3+column]+t[4*a+2]*ik[6+column];
          squared+=x*x;
        }
        return std::sqrt(squared);
      };
      camera.half_pixel=.5*(increment(0)+increment(1))+1e-9;
    }
    configuration_=geometry.configuration_id();
  }

  TubeStatus clearance(const CruiseDepthFrame &frame,const LocalGeometryTables &geometry,
                       Radial world,double radius) const {
    if(!ready(frame,geometry) || !valid(world,radius))return TubeStatus::Unknown;
    const Radial center=frame.in_body(world);
    if(!valid(center,radius))return TubeStatus::Unknown;
    radius+=coordinate_roundoff(frame,world);
    if(center.r+radius>BaseConfig::local_range_m)return TubeStatus::OutsideObservationRange;
    const DepthObservationView view(frame.depth);
    bool clear=false;
    for(unsigned i=0;i<4;++i) {
      const auto &cam=cameras_[i];
      const auto relative=spherical::add(center,cam.origin,-1.);
      const auto footprint=window(cam,relative,radius);
      if(footprint.status==FootprintStatus::Invalid)return TubeStatus::Unknown;
      if(footprint.z_far<=0.)continue;
      const auto [raw,filled]=view.region_pair(i,footprint.region);
      if((raw.z && *raw.z<=footprint.z_far) || (filled.z && *filled.z<=footprint.z_far))
        return TubeStatus::PotentialObstacle;
      clear|=footprint.fully_inside && std::isfinite(cam.near_clip) && cam.near_clip>0. &&
             footprint.z_near>=cam.near_clip && raw.total_pixels &&
             raw.valid_pixels==raw.total_pixels && raw.z && *raw.z>footprint.z_far;
    }
    return clear?TubeStatus::ObservedClear:TubeStatus::Unknown;
  }

  // Negative results can only refine an existing static-space certificate.
  bool surface_intersects(const CruiseDepthFrame &frame,const LocalGeometryTables &geometry,
                          Radial world,double radius) const {
    if(!ready(frame,geometry) || !valid(world,radius))return true;
    const auto center=frame.in_body(world);
    if(!valid(center,radius))return true;
    // The original physical surface veto has a 0.1 micrometre contact band.
    radius+=1e-7+coordinate_roundoff(frame,world);
    const DepthObservationView view(frame.depth);
    for(unsigned i=0;i<4;++i) {
      const auto &cam=cameras_[i];
      const auto relative=spherical::add(center,cam.origin,-1.);
      // A pixel at depth Z has radial uncertainty at most Z*half_pixel.
      // At a hit, (min_coefficient-half_pixel)*Z <= center_range+radius.
      // Use this a priori bound to include grazing edge pixels in the ROI.
      const double lower=cam.min_coefficient-cam.half_pixel;
      if(!(lower>0.))return true;
      const double padded=radius+cam.half_pixel*(relative.r+radius)/lower;
      const auto footprint=window(cam,relative,padded);
      if(footprint.status==FootprintStatus::Invalid)return true;
      if(footprint.z_far<=0.)continue;
      const auto [raw,filled]=view.region_pair(i,footprint.region);
      if((!raw.z || *raw.z>footprint.z_far) && (!filled.z || *filled.z>footprint.z_far))continue;
      if(scan(frame.depth,cam,i,relative,radius,footprint))return true;
    }
    return false;
  }

  // Certify the continuous radius-R tube for a requested travel length. A
  // positive result is a lower bound on free travel, not a point depth.
  TubeStatus direction_clearance(const CruiseDepthFrame &frame,const LocalGeometryTables &geometry,
                                 Radial origin,spherical::Bearing axis,double length,double radius,
                                 bool positive_evidence_only=false) const {
    if(!ready(frame,geometry) || !valid(origin,radius) || !valid({axis,length},radius))
      return TubeStatus::Unknown;
    const auto start=frame.in_body(origin);
    axis.az=spherical::wrap(axis.az-frame.yaw);
    const auto end=spherical::add(start,{axis,length});
    radius+=coordinate_roundoff(frame,origin)+1e-7;
    if(std::max(start.r,end.r)+radius>BaseConfig::local_range_m)return TubeStatus::OutsideObservationRange;
    const DepthObservationView view(frame.depth);
    bool covered=false;
    std::array<Radial,4> origins;
    for(unsigned i=0;i<4;++i){
      const auto &cam=cameras_[i];
      const auto a=spherical::add(start,cam.origin,-1.);
      origins[i]=a;
      if(covered)continue;
      const auto b=spherical::add(a,{axis,length});
      const auto wa=window(cam,a,radius),wb=window(cam,b,radius);
      if(wa.fully_inside && wb.fully_inside && cam.near_clip>0. &&
         std::min(wa.z_near,wb.z_near)>=cam.near_clip){
        // Projection of the convex hull is in the rectangle spanning both caps
        // when both caps are strictly in front of the near plane.
        PixelRegion rect{std::min(wa.region.u0,wb.region.u0),std::min(wa.region.v0,wb.region.v0),
                         std::max(wa.region.u1,wb.region.u1),std::max(wa.region.v1,wb.region.v1)};
        const auto raw=view.region_pair(i,rect)[0];
        covered|=raw.total_pixels && raw.valid_pixels==raw.total_pixels && raw.z &&
                 *raw.z>std::max(wa.z_far,wb.z_far);
      }
    }
    // A historical frame is queried solely for a new positive certificate.
    // If it cannot cover the tube, no scan could make it positive.
    if(positive_evidence_only && !covered)return TubeStatus::Unknown;
    for(unsigned i=0;i<4;++i){
      const auto &cam=cameras_[i];
      const auto a=origins[i];
      // Broad phase only. Exact contact uses along-track and cross-track
      // components of each depth ray, including its finite pixel footprint.
      const auto mid=spherical::add(a,{axis,.5*length});
      const double bound=radius+.5*length, lower=cam.min_coefficient-cam.half_pixel;
      if(!(lower>0.))return TubeStatus::Unknown;
      const auto foot=window(cam,mid,bound+cam.half_pixel*(mid.r+bound)/lower);
      if(foot.status==FootprintStatus::Invalid)return TubeStatus::Unknown;
      if(foot.z_far<=0. || foot.region.u0>=foot.region.u1 || foot.region.v0>=foot.region.v1)continue;
      const auto [raw,filled]=view.region_pair(i,foot.region);
      if((!raw.z || *raw.z>foot.z_far) && (!filled.z || *filled.z>foot.z_far))continue;
      const double origin_along=a.r*spherical::cosine(a.dir,axis);
      auto contact=[&](size_t pixel,float z){
        const auto &ray=cam.rays[pixel];
        const double rho=z*ray.r;
        const double along=rho*spherical::cosine(ray.dir,axis)-origin_along;
        const double t=std::clamp(along,0.,length);
        // Law of cosines with lengths only. Subtract the projection onto the
        // finite stop ray; the roundoff margin makes near tangencies veto.
        const double range_term=(rho-a.r)*(rho-a.r);
        const double angle_term=2.*rho*a.r*(1.-spherical::cosine(ray.dir,a.dir));
        const double squared=std::fma(t,t-2.*along,range_term+angle_term);
        const double limit=radius+z*cam.half_pixel;
        const double magnitude=1.+rho+a.r+length+limit;
        const double roundoff=128.*std::numeric_limits<double>::epsilon()*magnitude*magnitude;
        return squared<=limit*limit+roundoff;
      };
      if(scan_pixels(frame.depth,i,foot,contact))return TubeStatus::PotentialObstacle;
    }
    return covered?TubeStatus::ObservedClear:TubeStatus::Unknown;
  }

 private:
  static double coordinate_roundoff(const CruiseDepthFrame &frame,Radial world) {
    return 128.*std::numeric_limits<double>::epsilon()*(1.+world.r+frame.world_position.r);
  }
  bool ready(const CruiseDepthFrame &frame,const LocalGeometryTables &g) const {
    return configuration_ && configuration_==g.configuration_id() &&
           valid(frame.world_position,1.) && std::isfinite(frame.yaw) &&
           LocalFieldView(frame.depth,g).image_evidence_ready() && frame.depth.observations.raw_summaries_ready;
  }
  static bool valid(Radial p,double radius) {
    return std::isfinite(p.r) && p.r>=0. && std::isfinite(p.dir.az) && std::isfinite(p.dir.el)
      && std::abs(p.dir.el)<=spherical::pi/2.
      && std::isfinite(radius) && radius>0.;
  }

  // The footprint is an angular cap. A candidate image slope t is tangent
  // exactly when (s-t*d)^2 = r^2*(gain^2+t^2), with s the center's
  // projection along one chart axis and d its axial projection in metres.
  SphereFootprint window(const Camera &cam,Radial center,double radius) const {
    SphereFootprint out;
    const double d=center.r*spherical::cosine(center.dir,cam.depth_axis.dir);
    radius+=cam.calibration_slack*(center.r+radius)+1e-10*(1.+center.r+radius);
    out.z_near=(d-radius)*cam.depth_axis.r;
    out.z_far=(d+radius)*cam.depth_axis.r;
    if(d<=radius) {
      out.status=FootprintStatus::NearPlaneOrBehind;
      // Lateral balls can cross the optical plane far outside the image.
      // The four image edges are great-circle half-spaces; reject only
      // when their separating plane excludes the entire padded ball.
      if(out.z_far<=0.)return out;
      for(int a=0;a<2;++a) {
        const auto &axis=cam.chart_axis[a];
        const double s=center.r*axis.r*spherical::cosine(center.dir,axis.dir);
        const double lo=-.5-cam.principal[a],hi=(a==0?width_:height_)-.5-cam.principal[a];
        if(s-lo*d < -radius*std::hypot(axis.r,lo) || s-hi*d > radius*std::hypot(axis.r,hi))
          return out;
      }
      out.region={0,0,width_,height_};
      return out;
    }
    const double denom=(d-radius)*(d+radius);
    auto bounds=[&](int a,double &lo,double &hi){
      const auto &axis=cam.chart_axis[a];
      const double s=center.r*axis.r*spherical::cosine(center.dir,axis.dir);
      const double spread=radius*std::hypot(s,axis.r*std::sqrt(denom));
      lo=cam.principal[a]+(s*d-spread)/denom;
      hi=cam.principal[a]+(s*d+spread)/denom;
    };
    bounds(0,out.u_min,out.u_max);
    bounds(1,out.v_min,out.v_max);
    if(!std::isfinite(out.u_min) || !std::isfinite(out.u_max) ||
       !std::isfinite(out.v_min) || !std::isfinite(out.v_max))return out;
    out.status=FootprintStatus::Projected;
    const double u0=std::floor(out.u_min),u1=std::ceil(out.u_max)+1.;
    const double v0=std::floor(out.v_min),v1=std::ceil(out.v_max)+1.;
    out.fully_inside=u0>=0. && u1<=width_ && v0>=0. && v1<=height_;
    out.region={int(std::clamp(u0,0.,double(width_))),int(std::clamp(v0,0.,double(height_))),
                int(std::clamp(u1,0.,double(width_))),int(std::clamp(v1,0.,double(height_)))};
    return out;
  }

  bool scan(const DepthFrame &frame,const Camera &cam,unsigned i,Radial center,double radius,
            const SphereFootprint &foot) const {
    auto contact=[&](size_t p,float z){
      const auto &ray=cam.rays[p];
      const Radial sample{ray.dir,double(z)*ray.r};
      const double limit=radius+double(z)*cam.half_pixel+1e-10*(1.+center.r+sample.r);
      return spherical::distance(center,sample)<=limit;
    };
    return scan_pixels(frame,i,foot,contact);
  }

  template<class Contact>
  bool scan_pixels(const DepthFrame &frame,unsigned i,const SphereFootprint &foot,Contact contact) const {
    const auto &idx=frame.observations;
    auto pixels=[&](const PixelRegion &area){
      for(int v=area.v0;v<area.v1;++v)for(int u=area.u0;u<area.u1;++u){
        const size_t p=size_t(v)*width_+u;
        auto hits=[&](float z){return valid_depth(z) && z>=foot.z_near && z<=foot.z_far && contact(p,z);};
        const float raw=frame.D[i][p],filled=frame.D_filled[i][p];
        if(hits(raw) || (raw!=filled && hits(filled)))return true;
      }
      return false;
    };
    const auto &r=foot.region;
    const int edge=int(idx.tile_edge);
    if((r.u1-r.u0)*(r.v1-r.v0)<=edge*edge)return pixels(r);
    for(int v=r.v0;v<r.v1;){
      const int v1=std::min(r.v1,(v/edge+1)*edge);
      for(int u=r.u0;u<r.u1;){
        const int u1=std::min(r.u1,(u/edge+1)*edge);
        const size_t t=size_t(v/edge)*idx.tile_columns+u/edge;
        const auto &raw=idx.raw_tiles[i][t],&filled=idx.tiles[i][t];
        if(((raw.valid_pixels && raw.min_z<=foot.z_far) ||
            (filled.valid_pixels && filled.min_z<=foot.z_far)) && pixels({u,v,u1,v1}))return true;
        u=u1;
      }
      v=v1;
    }
    return false;
  }

  int width_=0,height_=0;
  uint64_t configuration_=0;
  std::array<Camera,4> cameras_{};
};

inline TubeStatus CruiseBallOracle::operator()(spherical::Radial world,double radius) const {
  return rig.clearance(frame,geometry,world,radius);
}
} // namespace ego4p0
