#include "pc_gvf/dynamic_obstacles.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <limits>
namespace pc_gvf { namespace depth_angular {
void DynamicObstacles::ingest(const DepthObservation& o) {
    if(!std::isfinite(o.stamp)||!o.origin.allFinite()||!o.rotation.allFinite()||o.depth.size()!=o.camera.rays().size()) {
        view_overloaded_[o.view]=true;overloaded_=true;return;
    }
    auto it=checked_.find(o.view);
    if(it!=checked_.end()&&o.stamp<it->second.first-.1){tracks_.clear();checked_.clear();view_overloaded_.clear();}
    if(checked_.count(o.view)&&checked_[o.view]==std::make_pair(o.stamp,o.version))return;
    if(checked_.count(o.view)&&o.stamp<=checked_[o.view].first)return;
    checked_[o.view]={o.stamp,o.version};
    std::map<std::tuple<int,int,int>,Eigen::Vector3d> voxels;bool invalid_world_point=false;
    auto add=[&](const Eigen::Vector3d& camera_point) {
        if(!camera_point.allFinite()||camera_point.z()<=0||camera_point.z()>=o.camera.maxDepth()-.01)return;
        const Eigen::Vector3d p=o.origin+o.rotation*camera_point;
        if(!p.allFinite()||p.cwiseAbs().maxCoeff()>1e5){invalid_world_point=true;return;}
        const auto key=std::make_tuple(int(std::floor(p.x()/.3+1e-9)),int(std::floor(p.y()/.3+1e-9)),int(std::floor(p.z()/.3+1e-9)));
        voxels[key]=.3*Eigen::Vector3d(std::get<0>(key)+.5,std::get<1>(key)+.5,std::get<2>(key)+.5);
    };
    if(o.obstacle_points)for(const auto& p:*o.obstacle_points)add(p);
    else for(std::size_t i=0;i<o.depth.size();++i)if(o.depth[i]>0&&std::isfinite(o.depth[i]))
        add(o.camera.rays()[i]*o.depth[i]/o.camera.rays()[i].z());
    if(invalid_world_point){view_overloaded_[o.view]=true;overloaded_=true;return;}
    // Never silently discard excess hazards. The certificate fails closed.
    view_overloaded_[o.view]=voxels.size()>1024;
    overloaded_=false;for(const auto& status:view_overloaded_)overloaded_=overloaded_||status.second;
    if(view_overloaded_[o.view])return;
    std::vector<ObstacleTrack> next;std::vector<bool> used(tracks_.size(),false);
    // Index eligible predictions along their widest world axis. The complete
    // +/- association radius on that axis contains every possible 3-D match.
    // Original-index tie breaking and the exact distance expression preserve
    // the previous association semantics and all motion uncertainty bounds.
    struct Prediction { std::size_t index; Eigen::Vector3d point; };
    std::vector<Prediction> predictions;
    Eigen::Vector3d lower=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d upper=-lower;
    if(tracks_.size()>64)for(std::size_t i=0;i<tracks_.size();++i) {
        const auto& old=tracks_[i];const double dt=o.stamp-old.stamp;
        if(old.view!=o.view||dt<.025||dt>.4)continue;
        const Eigen::Vector3d p=old.point+dt*old.velocity;
        if(!p.allFinite()){overloaded_=true;return;}
        predictions.push_back({i,p});lower=lower.cwiseMin(p);upper=upper.cwiseMax(p);
    }
    Eigen::Index axis=0;if(!predictions.empty())(upper-lower).maxCoeff(&axis);
    const bool indexed=predictions.size()>32;
    if(indexed)std::sort(predictions.begin(),predictions.end(),[axis](const Prediction& a,const Prediction& b) {
        return a.point[axis]<b.point[axis]||(a.point[axis]==b.point[axis]&&a.index<b.index);
    });
    for(const auto& cell:voxels) {
        const auto& p=cell.second;double best=.75*.75;int match=-1;
        auto start=predictions.begin(),finish=predictions.end();
        if(indexed) {
            // This roundoff guard only widens the search interval; the exact
            // historical .75 m Euclidean gate is applied unchanged below.
            const double guard=1e-9*(1.+std::abs(p[axis]));
            start=std::lower_bound(start,finish,p[axis]-.75-guard,[axis](const Prediction& old,double x){return old.point[axis]<x;});
            finish=std::upper_bound(start,finish,p[axis]+.75+guard,[axis](double x,const Prediction& old){return x<old.point[axis];});
        }
        auto consider=[&](std::size_t i) {
            const auto& old=tracks_[i];const double dt=o.stamp-old.stamp;
            if(used[i]||old.view!=o.view||dt<.025||dt>.4)return;
            const double d=(p-old.point-dt*old.velocity).squaredNorm();
            if(d<best||(match>=0&&d==best&&int(i)<match)){best=d;match=int(i);}
        };
        if(indexed)for(auto candidate=start;candidate!=finish;++candidate)consider(candidate->index);
        else for(std::size_t i=0;i<tracks_.size();++i)consider(i);
        ObstacleTrack track;track.point=p;track.stamp=o.stamp;track.view=o.view;
        if(match>=0) {
            used[match]=true;const auto& old=tracks_[match];const double dt=o.stamp-old.stamp;
            track.velocity=clampNorm((p-old.point)/dt,3.);track.observations=old.observations+1;
        }
        next.push_back(track);
    }
    for(std::size_t i=0;i<tracks_.size();++i)if(!used[i]&&o.stamp-tracks_[i].stamp<=.5)next.push_back(tracks_[i]);
    if(next.size()>4096){overloaded_=true;return;}tracks_=std::move(next);
}
bool DynamicObstacles::segmentSafe(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    double radius,double now,double begin,double end) const {
    return obstacleSegmentsSafe(tracks_,overloaded_,a,b,radius,now,begin,end);
}
bool obstacleSegmentsSafe(const std::vector<ObstacleTrack>& tracks,bool overloaded,
    const Eigen::Vector3d& a,const Eigen::Vector3d& b,double radius,double now,double begin,double end) {
    if(overloaded||!a.allFinite()||!b.allFinite()||!std::isfinite(radius)||radius<0||
       !std::isfinite(now)||!std::isfinite(begin)||!std::isfinite(end)||begin<0||end<begin)return false;
    for(const auto& track:tracks) {
        if(!track.point.allFinite()||!track.velocity.allFinite()||!std::isfinite(track.stamp)||
           !std::isfinite(track.radius)||track.radius<0||track.observations<1)return false;
        const double age=now-track.stamp;
        if(age<-.02)return false;
        if(age>.5)continue;
        const double t0=std::max(0.,age+begin),t1=std::max(0.,age+end);
        const Eigen::Vector3d ra=a-track.point-t0*track.velocity,rb=b-track.point-t1*track.velocity;
        const Eigen::Vector3d delta=rb-ra;
        const double u=delta.squaredNorm()>1e-16?std::max(0.,std::min(1.,-ra.dot(delta)/delta.squaredNorm())):0.;
        // Sparse surface coverage, velocity error and bounded acceleration.
        // Unassociated observations use the configured 3 m/s reachability bound.
        const double speed_error=track.observations<2?3.:.35;
        const double bound=radius+track.radius+.03+speed_error*t1+.5*.5*t1*t1;
        if((ra+u*delta).norm()<=bound)return false;
    }
    return true;
}
} }
