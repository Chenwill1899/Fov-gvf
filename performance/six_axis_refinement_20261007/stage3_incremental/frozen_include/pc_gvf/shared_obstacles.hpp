#pragma once
#include "pc_gvf/dynamic_obstacles.hpp"
#include <string>
#include <map>
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
    void restore(const SharedObstaclePacket& packet){sources_[packet.source]=packet;}
private:std::map<std::string,SharedObstaclePacket> sources_;
};
} }
