// The pre-change Cartesian evaluator is retained only as an independent test
// oracle; production uses the intrinsic two-component evaluator below.
#include "new_controller/harmonic_direction_field.hpp"
#include <iostream>
#include <iomanip>
#include "new_controller/harmonic_solver.hpp"
namespace ego4p0 {
// Stage 9: on-demand C1 scalar reconstruction, then its metric gradient.
// Shared vertex values/derivatives make adjacent bicubic patches C1. Missing
// faces impose zero normal derivative; no outside/unknown potential is read.
// This reconstruction is distinct from the nodal FV residual certification.
class LegacyCartesianDirectionField {
  struct Vertex {double value=0,dx=0,dy=0;};
  static Vertex vertex(const SphericalStencil &s,const FieldLayer &f,int x,int y,int reference_x,int reference_y) {
    const int na=int(s.layout.azimuth_bins),nb=int(s.layout.elevation_bins);double v[2][2]{};bool ok[2][2]{};
    Vertex o;double count=0;
    for(int j=0;j<2;++j)for(int i=0;i<2;++i){const int b=y+j-1,a=(x+i-1+na)%na;if(b<0||b>=nb)continue;const auto id=uint32_t(b*na+a);if(!f.boundary[id])continue;ok[j][i]=true;v[j][i]=f.phi[id];o.value+=v[j][i];++count;}
    if(!count)return o;
    // Diagonally touching observed sectors have no local open connection.
    // Keep their corner traces separate rather than interpolating a blind gap.
    if(count==2&&ok[0][0]==ok[1][1]&&ok[0][1]==ok[1][0]){o.value=v[reference_y][reference_x];return o;}
    o.value/=count;
    double left=0,right=0,bottom=0,top=0,nl=0,nr=0,ns=0,nn=0;
    bool wall_x=false,wall_y=false;
    for(int j=0;j<2;++j){wall_x|=ok[j][0]!=ok[j][1];if(ok[j][0]){left+=v[j][0];++nl;}if(ok[j][1]){right+=v[j][1];++nr;}}
    for(int i=0;i<2;++i){wall_y|=ok[0][i]!=ok[1][i];if(ok[0][i]){bottom+=v[0][i];++ns;}if(ok[1][i]){top+=v[1][i];++nn;}}
    if(!wall_x&&nl&&nr)o.dx=(right/nr-left/nl)/s.layout.azimuth_step;
    if(!wall_y&&ns&&nn)o.dy=(top/nn-bottom/ns)/s.layout.elevation_step;
    return o;
  }
  static void basis(double t,double *h,double *d,double *dh,double *dd) {
    h[0]=2*t*t*t-3*t*t+1;h[1]=1-h[0];d[0]=t*t*t-2*t*t+t;d[1]=t*t*t-t*t;
    dh[0]=6*t*t-6*t;dh[1]=-dh[0];dd[0]=3*t*t-4*t+1;dd[1]=3*t*t-2*t;
  }
  static bool polar(const SphericalStencil &s,const FieldLayer &f,const Direction3 &q,double &value,Direction3 &gradient) {
    // One common Cartesian affine fit over a complete polar cap; unlike a
    // query-cell tangent fit its coefficients do not jump across azimuth cells.
    double m[4][5]{};const bool north=q[2]>=0;const double zref=north ? 1. : -1.;uint32_t count=0;
    for(uint32_t i=0;i<s.nodes.size();++i){const auto &p=s.nodes[i].direction;if((north ? p[2] : -p[2])<std::sin(84*direction_pi/180))continue;if(!f.boundary[i])return false;const double b[4]={1,p[0],p[1],p[2]-zref};for(int r=0;r<4;++r){for(int c=0;c<4;++c)m[r][c]+=b[r]*b[c];m[r][4]+=b[r]*f.phi[i];}++count;}
    if(count<4)return false;
    for(int k=0;k<4;++k){int pivot=k;for(int j=k+1;j<4;++j)if(std::abs(m[j][k])>std::abs(m[pivot][k]))pivot=j;if(std::abs(m[pivot][k])<1e-16)return false;for(int j=k;j<5;++j)std::swap(m[pivot][j],m[k][j]);const double d=m[k][k];for(int j=k;j<5;++j)m[k][j]/=d;for(int r=0;r<4;++r)if(r!=k){const double a=m[r][k];for(int j=k;j<5;++j)m[r][j]-=a*m[k][j];}}
    value=m[0][4]+m[1][4]*q[0]+m[2][4]*q[1]+m[3][4]*(q[2]-zref);gradient={m[1][4],m[2][4],m[3][4]};const double radial=direction_dot(gradient,q);for(uint32_t k=0;k<3;++k)gradient[k]-=radial*q[k];return true;
  }
 public:
  static CurveSample query(const SphericalStencil &s,const FieldLayer &f,const Direction3 &direction,double epsilon=1e-7) {
    CurveSample out;out.direction=direction;if(!unit_direction(out.direction))return out;
    const auto &q=out.direction;out.cell=s.cell(q);out.alpha=std::atan2(q[1],q[0]);out.beta=std::atan2(q[2],std::hypot(q[0],q[1]));
    if(!f.boundary[out.cell]||f.status!=FieldStatus::Valid)return out;
    const double da=s.layout.azimuth_step,db=s.layout.elevation_step,cb=std::cos(out.beta);
    const int x=int(out.cell%s.layout.azimuth_bins),y=int(out.cell/s.layout.azimuth_bins);
    const double wx=std::clamp((wrapped_angle(out.alpha+direction_pi-(x+.5)*da)+da*.5)/da,0.,1.);
    const double wy=std::clamp((out.beta+direction_pi/2-y*db)/db,0.,1.);
    double hx[2],dx[2],dhx[2],ddx[2],hy[2],dy[2],dhy[2],ddy[2],pa=0,pb=0;
    basis(wx,hx,dx,dhx,ddx);basis(wy,hy,dy,dhy,ddy);
    for(int j=0;j<2;++j)for(int i=0;i<2;++i){const auto v=vertex(s,f,x+i,y+j,1-i,1-j);out.potential+=v.value*hx[i]*hy[j]+v.dx*da*dx[i]*hy[j]+v.dy*db*hx[i]*dy[j];pa+=v.value*dhx[i]*hy[j]/da+v.dx*ddx[i]*hy[j]+v.dy*db*dhx[i]*dy[j]/da;pb+=v.value*hx[i]*dhy[j]/db+v.dx*da*dx[i]*dhy[j]/db+v.dy*hx[i]*ddy[j];}
    const Direction3 east{-std::sin(out.alpha),std::cos(out.alpha),0},north{-std::sin(out.beta)*std::cos(out.alpha),-std::sin(out.beta)*std::sin(out.alpha),cb};
    if(cb>1e-9)for(uint32_t k=0;k<3;++k)out.gradient_B[k]=pa/cb*east[k]+pb*north[k];
    if(std::abs(out.beta)>84*direction_pi/180) {
      double value;Direction3 g;if(!polar(s,f,q,value,g))return out;
      const double t=std::clamp((std::abs(out.beta)*180/direction_pi-84)/2,0.,1.),w=t*t*(3-2*t),dw=3*t*(1-t)*180/direction_pi*(out.beta>=0?1:-1);
      for(uint32_t k=0;k<3;++k)out.gradient_B[k]=(1-w)*out.gradient_B[k]+w*g[k]+dw*(value-out.potential)*north[k];
      out.potential=(1-w)*out.potential+w*value;out.local_chart=true;
    }
    out.norm2=direction_dot(out.gradient_B,out.gradient_B);out.magnitude=std::sqrt(out.norm2);out.valid=std::isfinite(out.norm2)&&std::isfinite(out.potential);
    out.zero_field=out.norm2<=epsilon*epsilon;
    if(cb>1e-9){out.F_alpha=direction_dot(out.gradient_B,east)/cb;out.F_beta=direction_dot(out.gradient_B,north);}else out.F_alpha=out.F_beta=std::numeric_limits<double>::quiet_NaN();
    if(out.magnitude>epsilon){for(uint32_t k=0;k<3;++k)out.unit_B[k]=out.gradient_B[k]/out.magnitude;out.unit_alpha=out.F_alpha/out.magnitude;out.unit_beta=out.F_beta/out.magnitude;}
    return out;
  }
};
} // namespace ego4p0

using namespace ego4p0;
namespace {
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
constexpr const char *names[20]={"direction_x","direction_y","direction_z","gradient_x","gradient_y","gradient_z",
 "unit_x","unit_y","unit_z","alpha","beta","F_alpha","F_beta","norm2","tau","zeta","potential","magnitude","unit_alpha","unit_beta"};
std::array<double,20> values(const CurveSample &s){return {s.direction[0],s.direction[1],s.direction[2],
 s.gradient_B[0],s.gradient_B[1],s.gradient_B[2],s.unit_B[0],s.unit_B[1],s.unit_B[2],
 s.alpha,s.beta,s.F_alpha,s.F_beta,s.norm2,s.tau,s.zeta,s.potential,s.magnitude,s.unit_alpha,s.unit_beta};}
std::array<double,20> maximum{};uint64_t samples=0,invalid=0,polar=0;
void compare(const SphericalStencil &s,const FieldLayer &f,const Direction3 &q,double epsilon=1e-7){
 const auto a=LegacyCartesianDirectionField::query(s,f,q,epsilon),b=FineDirectionField::query(s,f,q,epsilon);
 require(a.valid==b.valid&&a.local_chart==b.local_chart&&a.zero_field==b.zero_field&&a.cell==b.cell,"flags and cell must agree");
 ++samples;invalid+=!a.valid;polar+=a.local_chart;
 const auto old=values(a),now=values(b);
 for(size_t i=0;i<old.size();++i){
  if(std::isnan(old[i])){require(std::isnan(now[i]),"NaN placement unchanged");continue;}
  require(std::isfinite(old[i])==std::isfinite(now[i]),"finite placement unchanged");
  const double error=std::abs(old[i]-now[i]);maximum[i]=std::max(maximum[i],error);
  if(error>2e-13*std::max(1.,std::abs(old[i])))throw std::runtime_error(std::string("field differs: ")+names[i]);
 }
}
uint64_t seed=0xd7f329b6ULL;
double random_unit(){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return double(seed>>11)*(1./9007199254740992.);}
}
int main(){try{
 SphericalStencil stencil;stencil.configure(1.);FieldLayer field;field.allocate(stencil.layout);field.status=FieldStatus::Valid;
 for(size_t i=0;i<field.phi.size();++i){field.phi[i]=random_unit();field.boundary[i]=uint8_t(FieldBoundary::Interior);field.state[i]=uint8_t(CandidateStatus::Available);}
 for(int i=0;i<6000;++i){
  const auto q=sphere_point((2*random_unit()-1)*direction_pi,(2*random_unit()-1)*83.9*direction_pi/180.);
  compare(stencil,field,q);
  if(i<120){const double magnitude=LegacyCartesianDirectionField::query(stencil,field,q).magnitude;
   for(double factor:{1.-1e-12,1.,1.+1e-12})compare(stencil,field,q,magnitude*factor);
  }
 }
 // Preserve the 84--86 degree smooth transition, both caps, seam and poles.
 for(double sign:{-1.,1.})for(double degrees:{84.-1e-9,84.,84.+1e-9,85.,86.-1e-9,86.,86.+1e-9,89.999,90.})
  for(double alpha:{-direction_pi,-1.,0.,1.,direction_pi})compare(stencil,field,sphere_point(alpha,sign*degrees*direction_pi/180.));
 for(int i=0;i<5000;++i){const uint32_t at=uint32_t(random_unit()*field.phi.size());field.boundary[at]=0;field.state[at]=uint8_t(CandidateStatus::Unknown);field.phi[at]=std::numeric_limits<double>::quiet_NaN();}
 for(int i=0;i<3000;++i)compare(stencil,field,sphere_point((2*random_unit()-1)*direction_pi,(2*random_unit()-1)*direction_pi/2));
 require(!FineDirectionField::query(stencil,field,{0,0,1}).valid&&!FineDirectionField::query(stencil,field,{0,0,-1}).valid,"incomplete polar cap stays unknown");
 // A complete constant field has no direction. Stale and invalid queries fail.
 std::fill(field.boundary.begin(),field.boundary.end(),uint8_t(FieldBoundary::Interior));
 std::fill(field.phi.begin(),field.phi.end(),.5);
 for(double beta:{0.,.5,1.4,-1.4})compare(stencil,field,sphere_point(.7,beta));
 compare(stencil,field,{});compare(stencil,field,{std::numeric_limits<double>::quiet_NaN(),0,0});
 field.status=FieldStatus::NoInput;compare(stencil,field,{1,0,0});
 require(invalid>0&&polar>0,"exercise unknown and polar branches");
 std::cout<<std::setprecision(17)<<"PASS intrinsic direction: samples="<<samples<<", invalid="<<invalid<<", polar="<<polar<<'\n';
 for(size_t i=0;i<maximum.size();++i)std::cout<<names[i]<<" max_abs_error="<<maximum[i]<<'\n';
 return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
