#pragma once
#include "pc_gvf/dynamic_obstacles.hpp"
#include <string>
#include <map>
#include <limits>
namespace pc_gvf { namespace depth_angular {
struct SharedObstaclePacket {
    std::string source,session,frame;
    std::uint64_t sequence=0;
    double stamp=0;
    bool overloaded=false;
    std::vector<ObstacleTrack> tracks;
};
class SharedObstacleMap {
public:
    bool merge(const SharedObstaclePacket& packet,double now,const std::string& frame);
    bool segmentSafe(const Eigen::Vector3d& a,const Eigen::Vector3d& b,double radius,
        double now,double begin,double end)const;
    const std::map<std::string,SharedObstaclePacket>& sources()const{return sources_;}
    void restore(const SharedObstaclePacket& packet);
    double capacityBlockedUntil()const{return capacity_blocked_until_;}
    void restoreCapacityBlockedUntil(double stamp){capacity_blocked_until_=stamp;}
private:
    struct Bounds {
        Eigen::Vector3d lower=Eigen::Vector3d::Zero(),upper=Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
        double earliest=0.,latest=0.,radius=0.;
        bool valid=false;
    };
    static Bounds bounds(const SharedObstaclePacket& packet);
    std::map<std::string,SharedObstaclePacket> sources_;
    std::map<std::string,Bounds> bounds_;
    double capacity_blocked_until_=-std::numeric_limits<double>::infinity();
};
} }
