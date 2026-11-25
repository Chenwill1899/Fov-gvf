#include "new_controller/reference_occupancy.hpp"
#include <iostream>

using namespace ego4p0;
namespace {
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}

// Independent pre-change Cartesian Bernstein expression. No chart coefficients
// or new evaluator are used to compute the expected geometry and arc lengths.
AngularQuery cartesian(const ContinuousAngularPath::Piece &p,double u){
  const double v=1-u;Direction3 C{},Cp{};
  for(int k=0;k<3;++k){
    C[k]=v*v*v*p.b[0][k]+3*v*v*u*p.b[1][k]+3*v*u*u*p.b[2][k]+u*u*u*p.b[3][k];
    Cp[k]=3*(v*v*(p.b[1][k]-p.b[0][k])+2*v*u*(p.b[2][k]-p.b[1][k])+u*u*(p.b[3][k]-p.b[2][k]));
  }
  const double n=ref_norm(C);const auto q=ref_scale(C,1/n);
  return {q,ref_scale(ref_add(Cp,q,-direction_dot(q,Cp)),1/n)};
}
double old_integral(const ContinuousAngularPath::Piece &p,double a,double b){
  constexpr double x[4]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
  constexpr double w[4]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
  const double m=(a+b)/2,h=(b-a)/2;double sum=0.;
  for(int j=0;j<4;++j)sum+=w[j]*(ref_norm(cartesian(p,m-h*x[j]).dq_dzeta)+ref_norm(cartesian(p,m+h*x[j]).dq_dzeta));
  return h*sum;
}
AngularQuery old_query(const ContinuousAngularPath &path,double z){
  const auto &pieces=path.pieces();
  if(pieces.empty())return path.query(0.);
  const auto *p=&pieces.back();
  for(const auto &candidate:pieces)if(z<=candidate.begin+candidate.length){p=&candidate;break;}
  const double target=std::clamp(z-p->begin,0.,p->length);
  double lo=0.,hi=1.;
  for(int i=0;i<42;++i){const double mid=(lo+hi)/2;if(old_integral(*p,0.,mid)<target)lo=mid;else hi=mid;}
  auto q=cartesian(*p,(lo+hi)/2);unit_direction(q.dq_dzeta);return q;
}
std::vector<Direction3> curve(int kind){
  std::vector<Direction3> q;
  for(int i=0;i<=120;++i){
    const double t=double(i)/120;
    if(kind==0)q.push_back(sphere_point(direction_pi*t/2.,0.));
    if(kind==1)q.push_back(sphere_point(direction_pi*t,.28*std::sin(2*direction_pi*t)));
    if(kind==2)q.push_back(sphere_point(2.9+.7*t,.4+.2*std::sin(direction_pi*t)));
    // A tilted great-circle passes through a pole; it must retain vertical
    // motion, with a fresh local atlas chart independent of azimuth singularity.
    if(kind==3)q.push_back({std::cos(direction_pi*t),0.,std::sin(direction_pi*t)});
  }
  return q;
}
}

int main(){try{
  const std::array<double,9> rotation{0,-1,0,.8,0,-.6,.6,0,.8};
  double max_q=0.,max_dq=0.,max_arc=0.;size_t samples=0;
  for(int kind=0;kind<4;++kind){
    const auto input=curve(kind);SpatialReference reference;
    require(reference.build(input,{2,-1,3},rotation),"intrinsic reference build");
    const auto &path=reference.angular_path();
    for(const auto &p:path.pieces()){
      for(const auto &control:p.b)require(direction_dot(control,p.normal)>.5,"positive chart weights");
      double old_length=0.;for(int cell=0;cell<8;++cell)old_length+=old_integral(p,double(cell)/8.,double(cell+1)/8.);
      max_arc=std::max(max_arc,std::abs(old_length-p.length));
      require(std::abs(old_length-p.length)<2e-13,"intrinsic metric preserves angular length");
      for(int j=0;j<=64;++j){
        const double u=double(j)/64;const auto a=cartesian(p,u),b=ContinuousAngularPath::evaluate(p,u);
        max_q=std::max(max_q,ref_norm(ref_add(a.q,b.q,-1)));
        max_dq=std::max(max_dq,ref_norm(ref_add(a.dq_dzeta,b.dq_dzeta,-1)));
        require(ref_norm(ref_add(a.q,b.q,-1))<2e-13,"same 3D curve after 2D lift");
        require(ref_norm(ref_add(a.dq_dzeta,b.dq_dzeta,-1))<2e-13,"same 3D derivative after 2D lift");
        ++samples;
      }
    }
    for(int i=1;i<80;++i){
      const double z=path.length()*i/80;const auto expected=old_query(path,z),actual=path.query(z);
      require(ref_norm(ref_add(expected.q,actual.q,-1))<2e-11,"same arc reparameterization");
      require(ref_norm(ref_add(expected.dq_dzeta,actual.dq_dzeta,-1))<2e-10,"same unit angular tangent");
      const auto fast=path.query(z,true);
      require(ref_norm(ref_add(expected.q,fast.q,-1))<=fast.arc_error+2e-11,"fast inverse certified error");
    }
    const double L=reference.profile().length;Direction3 integral{};
    constexpr int n=1600;
    for(int i=0;i<=n;++i){
      const double s=L*i/n;const auto angle=reference.profile().query(s)[0];
      integral=ref_add(integral,old_query(path,angle).q,(i==0||i==n?1:i%2?4:2)*(L/n)/3.);
    }
    const auto expected=ref_add(Direction3{2,-1,3},ref_rotate(rotation,integral));
    const auto endpoint=reference.position(L);
    require(ref_norm(ref_add(expected,endpoint.position,-1))<endpoint.error_bound_m+2e-7,"spatial integration error bound preserved");
    if(kind==3)require(std::abs(reference.query(L*.25).tangent_W[2])>.1,"three-dimensional flight retained");
  }
  ContinuousAngularPath path;
  require(path.build({sphere_point(0.,0.),sphere_point(.03,0.),sphere_point(.06,0.)}),"certificate path");
  DirectionGridLayout grid;grid.configure(1,1);
  std::vector<uint8_t> state(grid.cells(),uint8_t(CandidateStatus::Available));
  AngularDomainCertificate certificate;
  require(certificate.check(path,grid,state),"chart retains full-arc certificate");
  state[*grid.cell(.03,0.)]=uint8_t(CandidateStatus::Unknown);
  require(!certificate.check(path,grid,state),"chart cannot bridge unknown arc");
  // Tiny arcs expose cancellation in chart coefficients. Exercise arbitrary
  // rotations around the original 1e-11 duplicate-point threshold without
  // widening that threshold. Proper SLERP weights retain unit amplitude and
  // a nonzero short chord must not be treated as a missing velocity direction.
  uint64_t random=0x8a769d5fULL;size_t tiny_accepted=0,tiny_collapsed=0;
  auto unit_random=[&]{random^=random<<13;random^=random>>7;random^=random<<17;return double(random>>11)*(1./9007199254740992.);};
  double tiny_direction_error=0.,tiny_derivative_error=0.,tiny_arc_error=0.;
  const std::array<double,9> identity{1,0,0,0,1,0,0,0,1};
  for(int trial=0;trial<48;++trial){
    const double alpha=(2*unit_random()-1)*direction_pi,beta=(2*unit_random()-1)*1.45,spin=2*direction_pi*unit_random();
    const auto start=sphere_point(alpha,beta);
    const Direction3 east{-std::sin(alpha),std::cos(alpha),0},north{-std::sin(beta)*std::cos(alpha),-std::sin(beta)*std::sin(alpha),std::cos(beta)};
    const auto tangent=ref_add(ref_scale(east,std::cos(spin)),north,std::sin(spin));
    for(double angle:{.5e-11,.99e-11,1.01e-11,2e-11,1e-10,1e-9,1e-8,1e-7}){
      const auto end=ref_add(ref_scale(start,std::cos(angle)),tangent,std::sin(angle));
      SpatialReference tiny;const bool accepted=tiny.build({start,end},{.2,-.1,.3},identity);
      require(accepted,"nonzero tiny arc must remain regular after normalized SLERP");
      ++tiny_accepted;const auto &angular=tiny.angular_path();
      if(angle<1e-11){require(angular.pieces().empty(),"preserve duplicate-point threshold");++tiny_collapsed;}
      for(const auto &piece:angular.pieces()){
        double arc=0.;for(int j=0;j<8;++j)arc+=old_integral(piece,j/8.,(j+1)/8.);
        const double arc_difference=std::abs(arc-piece.length);
        tiny_arc_error=std::max(tiny_arc_error,arc_difference);
        require(arc_difference<=angular.length_error()+2e-15,"tiny arc retains integration bound");
        for(int j=0;j<=32;++j){
          const auto expected=cartesian(piece,j/32.),actual=ContinuousAngularPath::evaluate(piece,j/32.);
          const double q_error=ref_norm(ref_add(expected.q,actual.q,-1)),dq_error=ref_norm(ref_add(expected.dq_dzeta,actual.dq_dzeta,-1));
          tiny_direction_error=std::max(tiny_direction_error,q_error);tiny_derivative_error=std::max(tiny_derivative_error,dq_error);
          require(q_error<2e-15&&dq_error<2e-15,"tiny chart matches Cartesian expression at roundoff");
        }
      }
      const double length=tiny.profile().length;Direction3 independent{};
      constexpr int panels=256;
      for(int j=0;j<=panels;++j){const double s=length*j/panels;
        independent=ref_add(independent,old_query(angular,tiny.profile().query(s)[0]).q,
          (j==0||j==panels?1:j%2?4:2)*(length/panels)/3.);
      }
      const auto result=tiny.position(length);
      require(ref_norm(ref_add(ref_add(Direction3{.2,-.1,.3},independent),result.position,-1))<=result.error_bound_m+2e-15,
        "tiny spatial endpoint enclosed by unchanged numerical bound");
    }
  }
  require(tiny_accepted==384&&tiny_collapsed==96,"tiny boundary coverage");
  ContinuousAngularPath invalid_path;
  require(!invalid_path.build({{1,0,0},{-1,0,0}})&&invalid_path.error()=="ambiguous_antipodal_segment","ambiguous antipodal connection still rejected");
  require(!invalid_path.build({{1,0,0},sphere_point(.01,0.),{1,0,0}})&&invalid_path.error()=="angular_cusp","genuine cusp still rejected");
  std::cout<<"PASS intrinsic 2D charts preserve 3D geometry: "<<samples
    <<" samples, max direction="<<max_q<<", derivative="<<max_dq<<", arc="<<max_arc<<'\n';
  std::cout<<"PASS random rotated tiny arcs: accepted="<<tiny_accepted
    <<", collapsed="<<tiny_collapsed<<", max direction="<<tiny_direction_error
    <<", derivative="<<tiny_derivative_error<<", arc="<<tiny_arc_error<<'\n';
  return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
