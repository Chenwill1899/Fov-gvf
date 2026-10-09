#ifndef PC_GVF_LOCAL_DEPTH_PATH_HPP
#define PC_GVF_LOCAL_DEPTH_PATH_HPP
#include "pc_gvf/paper_guidance.hpp"
namespace pc_gvf { namespace depth_angular {
// Short local path for direction proposals only. Grid visibility and sampled
// obstacles are NOT a full body/motion certificate; execution must still use
// PaperGuidance's observed-volume, transition and complete braking checks.
struct LocalDepthPath {
    bool valid=false;
    Eigen::Vector3d direction=Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> points;
    double length=0.;
};
LocalDepthPath localDepthPath(
    const std::vector<std::shared_ptr<const DepthObservation>>& views,
    const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& goal,double body_envelope,double horizon=8.,double response_time=.44,
    const Eigen::Vector3d* direction_hint=nullptr);
Eigen::Vector3d localPathDirection(const std::vector<Eigen::Vector3d>& path,
    const Eigen::Vector3d& position,double lookahead);
}}
#endif
