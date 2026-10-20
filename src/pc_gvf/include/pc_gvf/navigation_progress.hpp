#pragma once
#include <Eigen/Core>
#include <cmath>
namespace pc_gvf { namespace depth_angular {
// Measured progress selects a planner; it grants no execution or free space.
struct NavigationProgress {
    enum class Mode { NORMAL, LOCAL_RECOVERY, VERIFIED_RETURN };
    Eigen::Vector3d origin=Eigen::Vector3d::Zero(),reference=Eigen::Vector3d::Zero();
    double stamp=-1.,last_search=-1.;bool searching=false;
    bool update(const Eigen::Vector3d& p,const Eigen::Vector3d& intent,double now,Mode mode=Mode::NORMAL) {
        // Ordinary local recovery can alternate with nominal flight every few
        // frames. Only an explicitly selected verified return gets its own
        // completion window; generic mode changes cannot erase goal stall.
        if(mode==Mode::VERIFIED_RETURN){*this=NavigationProgress();return false;}
        if(!p.allFinite()||!intent.allFinite()||!std::isfinite(now)||
           intent.norm()<1.||intent.head<2>().norm()<.15) {stamp=-1.;last_search=-1.;searching=false;return false;}
        const auto direction=intent.normalized().eval();
        if(stamp<0.||now<stamp||direction.dot(reference)<std::cos(.5)||
           (p-origin).dot(reference)>.20||(p-origin).norm()>2.) {
            origin=p;reference=direction;stamp=now;last_search=-1.;searching=false;
        }
        if(now-stamp>=2.)searching=true;
        return searching;
    }
    bool takeSearch(double now) {
        if(!searching||(last_search>=0.&&now-last_search<1.))return false;
        last_search=now;return true;
    }
};
} }
