#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <Eigen/Geometry>
#include <stdexcept>
#include <limits>
using namespace pc_gvf::depth_angular;
int main() {
    Camera camera(48,36,90,68,10.);
    // Equal-distance neighbors visited out of index order must retain the
    // original greedy association (index zero), including the track history.
    DynamicObstacles tied;std::vector<ObstacleTrack> tied_tracks(70);
    for(int i=0;i<70;++i){tied_tracks[i].point=Eigen::Vector3d(20.+i,.15,.15);tied_tracks[i].stamp=1.;}
    tied_tracks[0].point.x()=5.4;tied_tracks[0].observations=7;
    tied_tracks[1].point.x()=5.1;tied_tracks[1].observations=3;tied.restore(tied_tracks,false);
    DepthObservation tied_frame(camera,std::vector<double>(1728,10.),Eigen::Vector3d::Zero(),fixedCameraRotation(),1.1,1);
    tied_frame.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,tied_frame.rotation.transpose()*Eigen::Vector3d(5.25,.15,.15));
    tied.ingest(tied_frame);if(tied.tracks().empty()||tied.tracks().front().observations!=8)return 7;
    // Malformed queries and persisted tracks must never grant free motion.
    const auto zero=Eigen::Vector3d::Zero().eval();
    const double nan=std::numeric_limits<double>::quiet_NaN();
    if(obstacleSegmentsSafe({},false,zero,zero,nan,1.,0.,.2)||
       obstacleSegmentsSafe({},false,zero,zero,.58,1.,0.,nan)||
       obstacleSegmentsSafe({},false,zero,zero,.58,1.,-.1,.2))return 4;
    ObstacleTrack invalid;invalid.stamp=1.;invalid.point.x()=nan;
    if(obstacleSegmentsSafe({invalid},false,zero,zero,.58,1.,0.,.2))return 5;
    DynamicObstacles remote_origin;
    DepthObservation remote(camera,std::vector<double>(1728,10.),Eigen::Vector3d(1e100,0,0),fixedCameraRotation(),1.,1);
    remote.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,Eigen::Vector3d(0,0,1));
    remote_origin.ingest(remote);if(!remote_origin.overloaded())return 6;
    std::cout<<"index_tie=pass malformed_inputs=pass invalid_world_origin=pass\n";
    return 0;
}
