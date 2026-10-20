#pragma once
#include <Eigen/Core>
#include <cmath>

namespace pc_gvf { namespace depth_angular {
// A direction proposal only. It cannot authorize free space or execution.
// Capture time is never refreshed by reuse; callers still certify every command.
struct GoalProposalCache {
    Eigen::Vector3d target=Eigen::Vector3d::Zero(), origin=Eigen::Vector3d::Zero();
    Eigen::Vector3d reference=Eigen::Vector3d::Zero();
    double stamp=-1.;
    void save(const Eigen::Vector3d& p,const Eigen::Vector3d& intent,
              const Eigen::Vector3d& direction,double range,double now) {
        stamp=-1.;
        if(!p.allFinite()||!intent.allFinite()||!direction.allFinite()||
           intent.norm()<1e-5||direction.norm()<1e-5||!std::isfinite(now)||
           !std::isfinite(range)||range<=.25)return;
        origin=p;target=p+range*direction.normalized();reference=intent.normalized();stamp=now;
    }
    Eigen::Vector3d proposal(const Eigen::Vector3d& p,const Eigen::Vector3d& intent,double now,double ttl=.15,double max_displacement=.75) const {
        if(stamp<0||!p.allFinite()||!intent.allFinite()||intent.norm()<1e-5||!std::isfinite(now)||
           !std::isfinite(ttl)||ttl<=0||now<stamp||now-stamp>ttl||!std::isfinite(max_displacement)||max_displacement<=0.||(p-origin).norm()>max_displacement||
           intent.normalized().dot(reference)<std::cos(.05))return Eigen::Vector3d::Zero();
        const Eigen::Vector3d delta=target-p;
        if(delta.norm()<=.25||delta.dot(intent)<=1e-6)return Eigen::Vector3d::Zero();
        return intent.norm()*delta.normalized();
    }
};
} }
