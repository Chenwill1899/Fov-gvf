#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <Eigen/Geometry>
#include <stdexcept>
#include <limits>
using namespace pc_gvf::depth_angular;
int main() {
    Camera camera(48,36,90,68,10.);int before=0,after=0;
    for(int trial=0;trial<40;++trial) {
        PaperConfig old;old.motion.body_radius=.25;old.motion.safety_margin=.02;old.motion.rollout_margin=.02;
        old.depth_uncertainty=.01;old.uncertainty_rate=0.;old.motion.rollout_horizon=.8;old.motion.delay=.1;
        auto cfg=old;cfg.dynamic_obstacles=true;PaperGuidance baseline(old),optimized(cfg);
        for(int k=0;k<4;++k) {
            double t=k*.1;Eigen::Vector3d origin(-2.+.1*k,0,0);
            DepthObservation o(camera,std::vector<double>(48*36,10.),origin,fixedCameraRotation(),t,k+1);
            const Eigen::Vector3d obstacle(.75+.005*trial,1.8-2.*t,.15);
            o.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,o.rotation.transpose()*(obstacle-origin));
            // Rendering the current hit excludes it geometrically. Future
            // obstacle motion is deliberately absent from the depth baseline.
            Eigen::Vector2d pixel;const auto cp=(o.rotation.transpose()*(obstacle-origin)).eval();
            if(camera.pixelFromDirection(cp,&pixel))o.depth[int(std::round(pixel.y()))*48+int(std::round(pixel.x()))]=cp.z();
            optimized.ingestObservation(o);baseline.ingestObservation(o);
            if(k==3) {
                before+=baseline.motionSafe(o,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),t);
                after+=optimized.motionSafe(o,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),t);
            }
        }
    }
    DynamicObstacles ego;
    for(int k=0;k<4;++k) {
        DepthObservation o(camera,std::vector<double>(1728,10.),Eigen::Vector3d(.2*k,0,0),fixedCameraRotation(),k*.1,k+1);
        o.rotation=Eigen::AngleAxisd(.15*k,Eigen::Vector3d::UnitZ()).toRotationMatrix()*o.rotation;
        o.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,o.rotation.transpose()*(Eigen::Vector3d(3,2,0)-o.origin));ego.ingest(o);
    }
    for(const auto& t:ego.tracks())if(t.velocity.norm()>1e-8)throw std::runtime_error("ego motion became object motion");
    DepthObservation malformed(camera,{1.},Eigen::Vector3d::Zero(),fixedCameraRotation(),2.,99);
    ego.ingest(malformed);if(!ego.overloaded())return 2;
    PaperConfig sparse;sparse.dynamic_obstacles=true;sparse.history_duration=60.;
    sparse.history_sample_interval=1.;sparse.history_max_observations=16;
    PaperGuidance recent(sparse);
    for(int k=0;k<9;++k) {
        DepthObservation o(camera,std::vector<double>(1728,10.),Eigen::Vector3d(.02*k,0,0),
            fixedCameraRotation(),k*.1,k+1);
        recent.ingestObservation(o);
    }
    if(recent.retainedObservations()<3) {std::cerr<<"dynamic history expired before its next static-cadence sample\n";return 3;}
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
    std::cout<<"crossing_cases,static_accepted,predictive_accepted,static_world_false_velocity\n40,"<<before<<','<<after<<",0\n";
    return before>20&&after==0?0:1;
}
