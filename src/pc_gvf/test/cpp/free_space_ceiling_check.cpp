#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <limits>
using namespace pc_gvf::depth_angular;
void check(bool x,const char* m){if(!x){std::cerr<<m<<'\n';std::exit(1);}}
std::string saved(PaperGuidance& p,const DepthObservation& o,const Eigen::Vector3d& x){std::ostringstream b(std::ios::binary);Eigen::Vector3d q(1,0,0);p.saveReplay(b,o,x,Eigen::Vector3d::Zero(),q,1,.0166666667,&q);return b.str();}
PaperResult replay(std::string b){std::istringstream in(b,std::ios::binary);return PaperGuidance::replay(in);}
int main(){PaperConfig c;c.execution_consistent=true;c.emergency_braking_certificate=true;
c.motion.delay=0;c.motion.max_accel=16;c.motion.brake_accel=16;c.motion.velocity_tau=.1;
c.retain_verified_travel=true;c.retain_certified_volume=true;c.history_uncertainty_rate=0;
c.free_space_ceiling_enabled=true;c.free_space_ceiling_z=20;
Camera cam(24,18,90,68,20);DepthObservation o(cam,std::vector<double>(24*18,0),Eigen::Vector3d(0,0,10),fixedCameraRotation(),1,1);
PaperGuidance p(c);p.setVerifiedSeed(Eigen::Vector3d(0,0,10),40);double r=p.envelopeRadius();Eigen::Vector3d zero=Eigen::Vector3d::Zero(),high(0,0,20-r+.01),safe(0,0,18);
check(p.envelopeKnown(o,Eigen::Vector3d(0,0,20-r),r,1),"tangency rejected");
check(!p.envelopeKnown(o,high,r,1),"seed bypassed ceiling");
check(!p.rememberVerifiedPosition(o,high,1),"reached proof bypassed ceiling");
check(!p.proposalPath(o,{safe,high},1),"capsule bypassed ceiling");
check(p.motionSafe(o,safe,zero,Eigen::Vector3d(1,0,0),1),"safe horizontal rejected");
check(!p.motionSafe(o,Eigen::Vector3d(0,0,20-r-c.restart_clearance-.01),Eigen::Vector3d(0,0,1),zero,1),"emergency ascent tail crossed ceiling");
PaperGuidance unknown(c);check(!unknown.envelopeKnown(o,safe,r,1),"ceiling fabricated free space");
auto off=c;off.free_space_ceiling_enabled=false;PaperGuidance legacy(off);legacy.setVerifiedSeed(Eigen::Vector3d(0,0,10),40);
check(legacy.motionSafe(o,high,zero,Eigen::Vector3d(1,0,0),1),"disabled changed behavior");
auto a=saved(p,o,high),b=saved(legacy,o,high);check(a.substr(0,8)=="EGOPAPRU","U missing");check(!replay(a).accepted,"U lost ceiling");check(replay(b).accepted,"disabled U failed");
size_t offset=0,n=0;for(size_t i=0;i<a.size();++i)if(a[i]!=b[i]){offset=i;++n;}check(n==1,"enabled byte not isolated");
b.erase(offset,1+sizeof(double));b.replace(0,8,"EGOPAPRT");check(replay(b).accepted,"T compatibility failed");
for(double z:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){auto bad=c;bad.free_space_ceiling_z=z;bool threw=false;try{PaperGuidance invalid(bad);}catch(const std::invalid_argument&){threw=true;}check(threw,"nonfinite ceiling accepted");}
std::cout<<"ceiling ball/capsule, full E tail, no unknown promotion, disabled, U/T replay: PASS\n";
}
