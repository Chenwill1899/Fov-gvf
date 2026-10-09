#ifndef PC_GVF_OMNI_DEPTH_HPP
#define PC_GVF_OMNI_DEPTH_HPP

#include "pc_gvf/paper_guidance.hpp"
#include <functional>

namespace pc_gvf { namespace depth_angular {

// A world-frame spherical proposal atlas. It is NOT free-space evidence:
// every selected direction must pass the caller's real-frustum certificate.
struct OmniDepthConfig {
    int azimuth_bins = 180;
    int elevation_bins = 37;
    double horizon = 4.0;
    double radius = .58;
    double max_age = .30;
    double continuity_weight = .12;
    int max_certificates = 24;
    bool include_atlas = false; // Populate even when exact intent passes immediately.
};

struct OmniDepthResult {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> rays;
    std::vector<double> clearance;
    std::vector<std::uint8_t> observed;
    bool valid = false;
    int fresh_views = 0;
    int certificates = 0;
};

// Optical z depth is backprojected with each capture pose and translated to
// the current vehicle origin. Unknown pixels never create occupied or free
// evidence. The observed mask is a visibility diagnostic, not certification.
OmniDepthResult omniDepthProposal(
    const std::vector<std::shared_ptr<const DepthObservation>>& views,
    const Eigen::Vector3d& position, const Eigen::Vector3d& intent,
    const Eigen::Vector3d& previous_direction, double now, double corridor,
    const OmniDepthConfig& config,
    const std::function<bool(const Eigen::Vector3d&)>& certify);

} }
#endif
