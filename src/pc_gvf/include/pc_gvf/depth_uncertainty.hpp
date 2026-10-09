#pragma once
#include "pc_gvf/paper_guidance.hpp"
namespace pc_gvf { namespace depth_angular {
// A lower depth bound, not a calibrated probability: unknown stays unknown.
struct DepthUncertaintyConfig {
    bool enabled = false;
    double sigma0 = .005, sigma_range2 = .001, sigma_multiplier = 3.;
    double pose_bound = .01;
    // Only credit a bound already charged to the final body envelope.
    double reserved_radial_bound = 0.;
    double edge_threshold = .30;
    int edge_radius = 1;
};
// Intrinsics use integer-index pixel centers. Apply before conservative resize.
std::vector<double> conservativeDepth(const std::vector<double>& input,
    int width,int height,double fx,double fy,double cx,double cy,
    const DepthUncertaintyConfig& config);
DepthObservation conservativeObservation(const DepthObservation& input,
    const DepthUncertaintyConfig& config);
} }
