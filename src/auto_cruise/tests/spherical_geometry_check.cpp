#include "new_controller/spherical_geometry.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using namespace ego4p0::spherical;
using XYZ=std::array<double,3>;
XYZ plus(XYZ a, XYZ b,double k=1.) {
  for(int i=0;i<3;++i)a[i]+=k*b[i];
  return a;
}
double norm(XYZ a) { return std::hypot(a[0],a[1],a[2]); }
double dot(XYZ a,XYZ b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
XYZ normal(XYZ a,XYZ b) {
  return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
void check(bool ok,const char* label) { if(!ok)throw std::runtime_error(label); }

int main() try {
  const std::vector<Bearing> special={{0.,0.},{pi,0.},{0.,pi/2.},{1.3,pi/2.},{-2.,-pi/2.},
    {pi-1e-11,.7},{-pi+1e-11,.7},{1.,pi/2.-1e-12},{-2.,-pi/2.+1e-12}};
  std::mt19937_64 gen(20261120);
  std::uniform_real_distribution<double> rand(-1.,1.);
  const int edge_cases=int(special.size()*special.size());
  for(int i=0;i<edge_cases+10000;++i) {
    const Bearing a=i<edge_cases?special[i/special.size()]:Bearing{rand(gen)*pi,rand(gen)*pi/2.};
    const Bearing b=i<edge_cases?special[i%special.size()]:Bearing{rand(gen)*pi,rand(gen)*pi/2.};
    const XYZ qa=to_xyz({a,1.}), qb=to_xyz({b,1.});
    const double theta=std::atan2(norm(normal(qa,qb)),dot(qa,qb));
    check(std::abs(theta-angle(a,b))<3e-15,"inverse");
    check(angle(exp(a,log(a,b)),b)<4e-14,"exp(log)");
    const double ra=100.*std::abs(rand(gen)),rb=100.*std::abs(rand(gen));
    const Radial x{a,ra},y{b,rb};
    for(double k:{-1.,0.,.3,1.}) {
      const auto expected=plus(to_xyz(x),to_xyz(y),k);
      check(norm(plus(expected,to_xyz(add(x,y,k)),-1.))<2e-12,"radial addition");
    }
    check(distance(x,x)==0. && add(x,x,-1.).r==0.,"zero displacement");
    check(std::abs(distance(x,y)-norm(plus(to_xyz(x),to_xyz(y),-1.)))<3e-12,"physical distance");
    const double t=rand(gen)*2.;
    auto turned=plus({},qa,(1.-std::cos(t))*dot(qa,qb));
    turned=plus(plus(turned,qb,std::cos(t)),normal(qa,qb),std::sin(t));
    check(norm(plus(turned,to_xyz({rotate_about(a,b,t),1.}),-1.))<5e-14,"small circle");
    check(norm(plus(normal(qa,qb),to_xyz(cross(a,b)),-1.))<2e-14,"normal");
    // Transport maps the initial geodesic tangent to the terminal tangent.
    const Tangent v{.6,-.8};
    const auto tv=transport(a,b,v);
    check(std::abs(std::hypot(tv.east,tv.north)-1.)<2e-15,"transport norm");
    if(theta>1e-10 && theta<pi-1e-10){
      const auto f=transport(a,b,log(a,b)), back=log(b,a);
      check(std::hypot(f.east+back.east,f.north+back.north)<8e-14,"transport direction");
    }
  }
  std::cout<<"10000 seeded trials plus "<<edge_cases<<" seam, pole and antipode pairs passed\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
