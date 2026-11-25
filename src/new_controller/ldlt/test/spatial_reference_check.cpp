#include "new_controller/navigation_io.hpp"
#include <iostream>
using namespace ego4p0;
static void require(bool b,const char*s){if(!b)throw std::runtime_error(s);}
static std::vector<Direction3>arc(double degrees){std::vector<Direction3>q;const int n=std::max(1,int(std::ceil(degrees)));for(int i=0;i<=n;++i)q.push_back(sphere_point(degrees*direction_pi/180*i/n,0));return q;}
int main(int argc,char**argv){try{
  const std::array<double,9>I{1,0,0,0,1,0,0,0,1};SpatialReference r;
  require(r.build({{1,0,0}},{0,0,0},I),"line build");auto far=r.query(20);require(std::abs(far.position_W[0]-20)<1e-12&&far.kappa==0&&r.normal_speed_mps()==12,"unbounded straight tail, no terminal stop");
  if(argc>1)spatial_export(std::filesystem::path(argv[1])/"straight",r,8.);
  for(double angle:{30.,45.,90.,180.}){
    require(r.build(arc(angle),{0,0,0},I),"planar arc build");const double z=angle*direction_pi/180,k=80./144,l=1.2;const double L=z<k*l?2*std::sqrt(z*l/k):z/k+l;
    require(std::abs(r.profile().length-L)<1e-9,"turn length");require(std::abs(r.profile().zeta_end-z)<1e-10,"angular arclength");
    const auto end=r.query(L);require(ref_angle(end.tangent_W,sphere_point(z,0))<1e-9&&std::abs(end.kappa)<1e-12,"turn endpoint tangent and curvature");
    Direction3 independent{};const int N=20000;const double h=L/N;for(int i=0;i<=N;++i){const double a=r.profile().query(i*h)[0];independent=ref_add(independent,Direction3{std::cos(a),std::sin(a),0},(i==0||i==N?1:i%2?4:2)*h/3);}
    require(ref_norm(ref_add(independent,end.position_W,-1))<2e-6,"independent Simpson position");
    for(int i=1;i<100;++i){const double s=L*i/100,eps=2e-5;const auto a=r.query(s-eps),b=r.query(s+eps),c=r.query(s);
      require(ref_norm(ref_add(ref_scale(ref_add(b.position_W,a.position_W,-1),1/(2*eps)),c.tangent_W,-1))<3e-5,"position derivative is tangent");
      require(ref_norm(ref_add(ref_scale(ref_add(b.tangent_W,a.tangent_W,-1),1/(2*eps)),c.curvature_W,-1))<2e-5,"tangent derivative is curvature");
      require(std::abs(ref_norm(c.tangent_W)-1)<1e-12&&std::abs(ref_norm(c.curvature_W)-c.kappa)<1e-10&&c.kappa<=k+1e-12,"unit tangent / bounded curvature");
    }
    r.insert_node(.123456);bool found=false;for(const auto&n:r.nodes())found|=n==.123456;require(found,"explicit connection node");
    if(argc>1)spatial_export(std::filesystem::path(argv[1])/("planar_"+std::to_string(int(angle))),r,L+.1);
    const auto e=r.query(L);SpatialReference continuation;require(continuation.build({e.tangent_W},e.position_W,I)&&continuation.continuous_connection(e),"C2 zero-curvature tail connection");
    std::cout<<"angle "<<angle<<" length "<<r.profile().length<<" time "<<r.profile().length/12<<'\n';
  }
  std::vector<Direction3>spatial;for(int i=0;i<=120;++i)spatial.push_back(sphere_point(i*.009,.15*std::sin(i*.013)));
  require(r.build(spatial,{2,-1,3},I),"3d curve build");for(int i=1;i<40;++i){const double s=r.profile().length*i/40,e=1e-5;auto a=r.query(s-e),b=r.query(s+e),c=r.query(s);require(ref_norm(ref_add(ref_scale(ref_add(b.tangent_W,a.tangent_W,-1),1/(2*e)),c.curvature_W,-1))<1e-4,"3d curvature derivative");}
  if(argc>1)spatial_export(std::filesystem::path(argv[1])/"spatial3d",r,r.profile().length+.1);
  require(NavigationLimits::compensation_input({17,0,0},{15,0,0},.005)&&!NavigationLimits::compensation_input({17.001,0,0},{15,0,0},.005)&&!NavigationLimits::compensation_input({3,0,0},{0,0,0},.005),"compensation 17/450 actual-dt validation");require(!NavigationLimits::compensation_input({1,0,0},{0,0,0},.002),"no clamping a small actual interval up to 5ms");
  require(NavigationLimits::heading_input({10,0,0},{9.5,0,0},.005)&&!NavigationLimits::heading_input({10.01,0,0},{9.5,0,0},.005)&&!NavigationLimits::heading_input({1,0,0},{0,0,0},.005),"heading 10/100 actual-dt validation");
  require(r.build({{1,0,0}},{0,0,0},I),"line rebuild");require(r.normal_attachment({.099,0,0},sphere_point(4.99*direction_pi/180,0),0),"normal attachment accepts bounds");require(!r.normal_attachment({.101,0,0},{1,0,0},0)&&!r.normal_attachment({0,0,0},sphere_point(5.01*direction_pi/180,0),0),"normal attachment rejects out-of-bounds");
  ContinuousAngularPath angular;require(angular.build(arc(3)),"short angular path");DirectionGridLayout grid;grid.configure(1,1);std::vector<uint8_t>available(grid.cells(),uint8_t(CandidateStatus::Available));AngularDomainCertificate certificate;require(certificate.check(angular,grid,available),"whole angular interpolation allowed");const auto hole=grid.cell(1.5*direction_pi/180,0.);available[*hole]=uint8_t(CandidateStatus::Unknown);require(!certificate.check(angular,grid,available),"unknown angular interior between original endpoints rejected");
  const auto mask_builds=certificate.mask_builds();
  require(!certificate.check(angular,grid,available)&&certificate.mask_builds()==mask_builds,"same Omega reuses mask but rechecks curve");
  available[*hole]=uint8_t(CandidateStatus::Available);
  require(certificate.check(angular,grid,available)&&certificate.mask_builds()==mask_builds+1,"changed Omega invalidates cached mask");
  ContinuousAngularPath changed_curve;require(changed_curve.build(arc(8)),"changed curve");
  available[*grid.cell(6.*direction_pi/180,0.)]=uint8_t(CandidateStatus::Unknown);
  require(certificate.check(angular,grid,available),"short curve before remote hole");
  require(!certificate.check(changed_curve,grid,available),"cached mask cannot reuse a different curve proof");
  ContinuousTubeChecker tube;Direction3 obstacle{.025,.2999,0};require(ref_norm(obstacle)>.30&&ref_norm(ref_add(obstacle,{.05,0,0},-1))>.30,"point samples miss obstacle fixture");
  tube.check(r,.1,[&](const Direction3&p,double radius){return ref_norm(ref_add(p,obstacle,-1))<=radius?TubeStatus::PotentialObstacle:TubeStatus::ObservedClear;});require(tube.sections()[0].status==TubeStatus::PotentialObstacle,"continuous interior collision caught");require(tube.interval_status(0,.05)==TubeStatus::PotentialObstacle,"whole interval occupancy query");require(std::abs(tube.sections()[0].cover_radius_m-.325)<1e-9,"tube radius counted once");
  tube.check(r,.1,[](const Direction3&,double){return TubeStatus::Unknown;});for(auto &s:tube.sections())require(s.status==TubeStatus::Unknown,"unknown never free");
  require(tube.interval_status(0,.1)==TubeStatus::Unknown&&tube.interval_status(.2,.3)==TubeStatus::Unknown,"unobserved interval is not clear");tube.invalidate();require(tube.sections().empty(),"occupancy invalidation clears old certificate");
  std::cout<<"PASS continuous reference: 30/45/90/180 lengths; line, 3D, derivatives, attachment, C2 tail joins, inserted node, swept tube interior and Unknown\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
