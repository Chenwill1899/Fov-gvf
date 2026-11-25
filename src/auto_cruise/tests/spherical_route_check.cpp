#include "../spherical_route.hpp"
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <random>

using namespace ego4p0::cruise;
using namespace ego4p0::spherical;
using Vec=Eigen::Vector3d;

namespace {
Radial encode(const Vec &v){return from_xyz(v.x(),v.y(),v.z());}
Vec decode(Radial p){
  const auto v=to_xyz(p);
  return {v[0],v[1],v[2]};
}
void require(bool condition,const char *message){
  if(!condition)throw std::runtime_error(message);
}
Vec capped(Vec v,double cap){return v.norm()>cap ? Vec(v.normalized()*cap) : v;}

// Cartesian equations independent of the runtime route and sphere operations.
struct Guide {
  const std::array<Vec,6> points{{{-0.5,.001,4.},{-.5,-.001,4.},{0.,0.,4.},{0.,0.,2.},{0.,0.,0.},{0.,0.,-2.}}};
  std::array<double,6> lengths{};
  Guide(){for(size_t i=1;i<points.size();++i)lengths[i]=lengths[i-1]+(points[i]-points[i-1]).norm();}
  double project(Vec p)const {
    double best=1e20,progress=0.;
    for(size_t i=0;i+1<points.size();++i){
      const Vec d=points[i+1]-points[i],r=p-points[i];
      const double t=std::clamp(d.dot(r)/d.squaredNorm(),0.,1.);
      const double e=(r-t*d).norm();
      if(e<best){best=e;progress=lengths[i]+t*d.norm();}
    }
    return progress;
  }
  Vec at(double s)const {
    if(s<=0.)return points.front();
    for(size_t i=0;i+1<points.size();++i)if(s<=lengths[i+1])
      return points[i]+(points[i+1]-points[i]).normalized()*(s-lengths[i]);
    return points.back();
  }
};
}

int main() try {
  const Guide guide;
  std::mt19937_64 generator(42);
  std::uniform_real_distribution<double> random(-1.,1.);
  auto sample=[&](){return Vec(random(generator),random(generator),random(generator));};
  for(bool polarized:{false,true}){
    CruiseRoute route;
    std::stringstream stream;
    stream<<std::setprecision(17)<<(polarized?"az,el,range":"x,y,z")<<"\n";
    for(const auto &point:guide.points){
      const auto p=encode(point);
      if(polarized)stream<<p.dir.az<<','<<p.dir.el<<','<<p.r<<'\n';
      else stream<<point.x()<<','<<point.y()<<','<<point.z()<<'\n';
    }
    route.load(stream);
    for(size_t i=0;i<1000;++i){
      size_t cursor=0;
      const Vec p=guide.at(guide.lengths.back()*i/1000.)+sample()*.015;
      const double s=guide.project(p);
      require(std::abs(route.project(encode(p),cursor)-s)<2e-12,"guide progress mismatch");
      // Independent closed-form integral of each smoothed tangent jump.
      const Vec first=(guide.points[1]-guide.points[0]).normalized();
      Vec center=guide.points[0]+s*first, derivative=first, second=Vec::Zero();
      bool inside_piece=true;
      for(size_t j=1;j+1<guide.points.size();++j){
        const Vec jump=(guide.points[j+1]-guide.points[j]).normalized()-
                       (guide.points[j]-guide.points[j-1]).normalized();
        const double h=std::min({1.8,guide.lengths[j],guide.lengths.back()-guide.lengths[j]});
        const double t=s-guide.lengths[j];
        inside_piece &= std::abs(std::abs(t)-h)>1e-9;
        if(t>=h){center+=jump*t;derivative+=jump;}
        else if(t>=-h){
          center+=jump*((t+h)*(t+h)/(4*h));
          derivative+=jump*((t+h)/(2*h));
          second+=jump/(2*h);
        }
      }
      const auto ref=route.reference(s);
      require((decode(ref.position)-center).norm()<1e-10,"route centerline mismatch");
      require((decode(ref.derivative)-derivative).norm()<1e-10,"route tangent mismatch");
      const Vec axis=derivative.normalized();
      const Vec curvature=(second-axis*axis.dot(second))/derivative.squaredNorm();
      // At exact polynomial joins the curvature has two one-sided values.
      if(inside_piece)
        require((decode(ref.curvature)-curvature).norm()<1e-7,"route curvature mismatch");
      const Vec v=sample().normalized()*12.;
      const auto heading=v.normalized();
      const Vec input=sample()*16.;
      const Vec applied=v+capped((input-v)*beta,emergency*dt);
      require((decode(predicted_velocity(encode(v),encode(input)))-applied).norm()<1e-12,"plant mismatch");
      const auto stopped=predicted_velocity(encode(v),{});
      require((decode(stopped)-heading*(v.norm()-emergency*dt)).norm()<1e-12,"braking mismatch");
    }
  }
  const Radial forward{{0.,0.},12.};
  require(std::abs(angle(steer(forward,{pi,0.},.03),forward.dir)-.03)<1e-14,"antipodal slew bound");
  CruiseRoute straight;
  std::stringstream line("x,y,z\n0,0,0\n100,0,0\n");
  straight.load(line);
  CruiseRoute::Motion motion{from_xyz(2.,.10,-.10),forward,forward,{}};
  for(int i=0;i<200;++i){
    const auto track=straight.tracking(motion.p,2.+i*12.*dt);
    require(track.valid,"straight route chart invalid");
    if(i==0){
      require(std::abs(track.coordinates.normal_m-.10)<1e-12 &&
              std::abs(track.coordinates.binormal_m+.10)<1e-12,"parallel offset missed");
      const auto low=track.box.guidance(track.coordinates,{},.05,2.,false);
      const Vec correction=decode({low.desired,.05});
      require(correction.allFinite() && correction.y()<0. && correction.z()>0. &&
              std::abs(std::hypot(correction.y(),correction.z())-.95*.05)<1e-12,
              "low-speed lateral saturation");
    }
    const auto next=straight.next(motion,track);
    require(distance(predicted_velocity(motion.v,next.u),next.v)<1e-11,"preview and plant disagree");
    motion=next;
  }
  const Vec end=decode(motion.p);
  require(std::hypot(end.y(),end.z())<.025,"parallel offset failed to converge in cruise");
  const auto passed=straight.tracking(from_xyz(100.1,.02,0.),100.,true);
  require(passed.valid && cosine(passed.tangent,forward.dir)<-.999,"terminal overshoot direction");
  std::cout<<"Route geometry, actuator, low-speed and cruise flow-box checks passed\n";
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
