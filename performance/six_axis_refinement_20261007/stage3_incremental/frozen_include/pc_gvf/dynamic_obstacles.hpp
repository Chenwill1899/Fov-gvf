#pragma once
#include <Eigen/Core>
#include <vector>
#include <map>
#include <cstdint>
namespace pc_gvf { namespace depth_angular {
struct DepthObservation;
struct ObstacleTrack {
    Eigen::Vector3d point=Eigen::Vector3d::Zero(),velocity=Eigen::Vector3d::Zero();
    double stamp=0, radius=.26; int view=0, observations=1;
};
bool obstacleSegmentsSafe(const std::vector<ObstacleTrack>& tracks,bool overloaded,
    const Eigen::Vector3d& a,const Eigen::Vector3d& b,double radius,double now,double begin,double end);
class DynamicObstacles {
public:
    void ingest(const DepthObservation& observation);
    bool segmentSafe(const Eigen::Vector3d& a,const Eigen::Vector3d& b,double radius,
        double now,double begin,double end) const;
    const std::vector<ObstacleTrack>& tracks() const{return tracks_;}
    bool overloaded() const{return overloaded_;}
    void restore(const std::vector<ObstacleTrack>& tracks,bool overloaded){tracks_=tracks;overloaded_=overloaded;}
private:
    std::vector<ObstacleTrack> tracks_;
    std::map<int,std::pair<double,std::uint64_t>> checked_;
    bool overloaded_=false;
    std::map<int,bool> view_overloaded_;
};
} }
