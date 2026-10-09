#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
namespace pc_gvf { namespace depth_angular {
// Suppress small oscillations, but pass deliberate coherent turns through.
// All direction comparisons use raw operator input, never avoidance output.
class OperatorIntent {
public:
    Eigen::Vector3d update(const Eigen::Vector3d& raw,bool enabled) {
        if(!raw.allFinite()||raw.norm()<1e-5) {
            anchor_.setZero();stage_.setZero();previous_.setZero();delta_.setZero();coherent_=0;oscillating_=false;return raw;
        }
        const Eigen::Vector3d direction=raw.normalized();
        if(!enabled||anchor_.norm()<.5) {
            anchor_=stage_=previous_=direction;delta_.setZero();coherent_=0;oscillating_=false;return raw;
        }
        const Eigen::Vector3d change=direction-previous_;
        if(change.norm()>1e-7) {
            if(delta_.norm()>1e-7&&change.dot(delta_)<=.5*change.norm()*delta_.norm())oscillating_=true;
            coherent_=delta_.norm()>1e-7&&change.dot(delta_)>.5*change.norm()*delta_.norm()?
                std::min(3,coherent_+1):1;
            delta_=change;previous_=direction;
        }
        const double angle=std::atan2(anchor_.cross(direction).norm(),anchor_.dot(direction));
        if(angle>.018||coherent_>=3){anchor_=stage_=direction;oscillating_=false;}
        else if(!oscillating_)anchor_=stage_=(.875*anchor_+.125*direction).normalized();
        else {
            // Two poles suppress angular acceleration from small oscillations.
            // alpha=2/9 keeps the low-frequency delay of the former 1/8 pole:
            // 2*(1-alpha)/alpha == (1-1/8)/(1/8) == 7 updates.
            constexpr double alpha=2./9.;
            stage_=((1.-alpha)*stage_+alpha*direction).normalized();
            anchor_=((1.-alpha)*anchor_+alpha*stage_).normalized();
        }
        // A lagging first stage can momentarily push the second stage away
        // from raw even when the pre-update angle was inside the bound.
        // Enforce the contract on the actual output and reset both states.
        if(std::atan2(anchor_.cross(direction).norm(),anchor_.dot(direction))>.018) {
            anchor_=stage_=direction;oscillating_=false;
        }
        return raw.norm()*anchor_;
    }
private:
    Eigen::Vector3d anchor_=Eigen::Vector3d::Zero(),stage_=Eigen::Vector3d::Zero(),previous_=Eigen::Vector3d::Zero(),delta_=Eigen::Vector3d::Zero();
    int coherent_=0;
    bool oscillating_=false;
};
} }
