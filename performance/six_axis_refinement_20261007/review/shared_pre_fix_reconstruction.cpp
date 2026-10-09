// Review reconstruction: only the two reviewed expressions reverted from fixed snapshot.
#include "pc_gvf/shared_obstacles.hpp"
#include <algorithm>
#include <cmath>
namespace pc_gvf { namespace depth_angular {
SharedObstacleMap::Bounds SharedObstacleMap::bounds(const SharedObstaclePacket& p) {
    Bounds b;
    if(p.tracks.empty())return b;
    b.lower=b.upper=p.tracks.front().point;
    b.earliest=b.latest=p.tracks.front().stamp;
    for(const auto& t:p.tracks) {
        if(!t.point.allFinite()||!t.velocity.allFinite()||!std::isfinite(t.stamp)||
           !std::isfinite(t.radius)||t.radius<0||t.observations<1)return Bounds();
        b.lower=b.lower.cwiseMin(t.point);b.upper=b.upper.cwiseMax(t.point);
        b.velocity=b.velocity.cwiseMax(t.velocity.cwiseAbs());
        b.earliest=std::min(b.earliest,t.stamp);b.latest=std::max(b.latest,t.stamp);
        b.radius=std::max(b.radius,t.radius);
    }
    b.valid=true;return b;
}
void SharedObstacleMap::restore(const SharedObstaclePacket& p) {
    sources_[p.source]=p;bounds_[p.source]=bounds(p);
}
bool SharedObstacleMap::merge(const SharedObstaclePacket& p,double now,const std::string& frame) {
    if(p.source.empty()||p.source.size()>64||p.session.empty()||p.session.size()>64||p.frame!=frame||
       !std::isfinite(now)||!std::isfinite(p.stamp)||p.stamp>now+.02||now-p.stamp>.5||p.tracks.size()>4096)return false;
    // Validate before capacity accounting: malformed or unaligned messages must
    // neither evict valid evidence nor trigger the valid-overcapacity guard.
    for(const auto& t:p.tracks)if(!t.point.allFinite()||t.point.norm()>1e5||!t.velocity.allFinite()||t.velocity.norm()>5.||
        !std::isfinite(t.stamp)||t.stamp>p.stamp+.02||p.stamp-t.stamp>.5||!std::isfinite(t.radius)||t.radius<0||t.radius>2.||t.observations<1)return false;
    for(auto it=sources_.begin();it!=sources_.end();) {
        if(now-it->second.stamp>.5 || it->second.stamp>now+.02) {
            bounds_.erase(it->first);it=sources_.erase(it);
        } else ++it;
    }
    auto old=sources_.find(p.source);
    if(old!=sources_.end()) {
        if(p.stamp<old->second.stamp)return false;
        if(p.session==old->second.session&&p.sequence<=old->second.sequence)return false;
        if(p.session!=old->second.session&&p.stamp<=old->second.stamp)return false;
    } else if(sources_.size()>=8) {
        // An otherwise valid peer may contain a hazard we cannot retain. Keep
        // the bounded memory budget but fail closed for its original lifetime.
        if(p.overloaded||!p.tracks.empty())capacity_blocked_until_=std::max(capacity_blocked_until_,p.stamp+.5);
        return false;
    }
    restore(p);return true;
}
bool SharedObstacleMap::segmentSafe(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    double radius,double now,double begin,double end)const {
    if(!a.allFinite()||!b.allFinite()||!std::isfinite(radius)||radius<0||!std::isfinite(now)||
       !std::isfinite(begin)||!std::isfinite(end)||begin<0||end<begin||now<=capacity_blocked_until_)return false;
    for(const auto& source:sources_) {
        const auto& p=source.second;
        if(now-p.stamp>.5)continue;
        if(p.stamp>now+.02||p.overloaded)return false;
        const auto cached=bounds_.find(source.first);
        if(cached!=bounds_.end()&&cached->second.valid) {
            const auto& box=cached->second;
            if(box.latest>now+.02)return false;
            const double time=std::max(0.,now+end-box.earliest);
            // A superset of every original relative-motion capsule. The worst
            // unassociated speed error (3 m/s), .5 m/s^2 acceleration and all
            // original radii remain here. Overlapping boxes use the exact test.
            const double inflation=radius+box.radius+.03+3.*time+.25*time*time;
            const Eigen::Vector3d reach=box.velocity*time+Eigen::Vector3d::Constant(inflation);
            const double guard=1e-9*(1.+box.lower.cwiseAbs().maxCoeff()+box.upper.cwiseAbs().maxCoeff());
            if(reach.allFinite()&&((a.cwiseMax(b).array()<box.lower.array()-reach.array()-guard).any()||
                                  (a.cwiseMin(b).array()>box.upper.array()+reach.array()+guard).any()))continue;
        }
        if(!obstacleSegmentsSafe(p.tracks,p.overloaded,a,b,radius,now,begin,end))return false;
    }
    return true;
}
} }
