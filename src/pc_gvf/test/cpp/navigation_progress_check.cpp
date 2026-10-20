#include "pc_gvf/navigation_progress.hpp"
#include <iostream>
#include <cstdlib>
using namespace pc_gvf::depth_angular;
void check(bool x){if(!x)std::exit(1);}
int main(){NavigationProgress p;Eigen::Vector3d o(0,0,10),q(10,0,0);
check(!p.update(o,q,1));check(!p.update(o,q,2.9));check(p.update(o,q,3.01));check(p.takeSearch(3.01));check(!p.takeSearch(3.1));check(p.takeSearch(4.02));
check(p.update(o+Eigen::Vector3d(.1,0,0),q,3.2));
check(!p.update(o+Eigen::Vector3d(.25,0,0),q,3.3));
check(p.update(o+Eigen::Vector3d(.25,0,0),q,5.4));
check(!p.update(o,Eigen::Vector3d::Zero(),5.5));check(!p.update(o,q,5.6));
check(p.update(o,q,7.7));check(!p.update(o,Eigen::Vector3d(0,10,0),7.8));
check(!p.update(o,Eigen::Vector3d(0,0,1),9));
check(!p.update(o,q,10));check(!p.update(o,q,9));
NavigationProgress loop;check(!loop.update(o,q,1));
check(loop.update(o+Eigen::Vector3d(0,.5,0),q,3.1));
check(loop.update(o-Eigen::Vector3d(.5,0,0),q,3.2));
check(!loop.update(o+Eigen::Vector3d(0,2.1,0),q,3.3));
NavigationProgress alternating;
for(int i=0;i<25;++i) {
    const auto mode=i%2?NavigationProgress::Mode::LOCAL_RECOVERY:NavigationProgress::Mode::NORMAL;
    const bool stalled=alternating.update(o,q,20.+.1*i,mode);
    check(stalled==(i>=20));
}
check(!alternating.update(o,q,23.,NavigationProgress::Mode::VERIFIED_RETURN));
check(!alternating.update(o,q,24.,NavigationProgress::Mode::NORMAL));
check(alternating.update(o,q,26.1,NavigationProgress::Mode::LOCAL_RECOVERY));
std::cout<<"stationary trigger, measured progress, release/reapply, turn, pure vertical, clock: PASS\n";}
