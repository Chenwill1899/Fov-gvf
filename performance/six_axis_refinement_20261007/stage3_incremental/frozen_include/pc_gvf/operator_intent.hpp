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
            anchor_.setZero();previous_.setZero();delta_.setZero();coherent_=0;return raw;
        }
        const Eigen::Vector3d direction=raw.normalized();
        if(!enabled||anchor_.norm()<.5) {
            anchor_=previous_=direction;delta_.setZero();coherent_=0;return raw;
        }
        const Eigen::Vector3d change=direction-previous_;
        if(change.norm()>1e-7) {
            coherent_=delta_.norm()>1e-7&&change.dot(delta_)>.5*change.norm()*delta_.norm()?
                std::min(3,coherent_+1):1;
            delta_=change;previous_=direction;
        }
        const double angle=std::atan2(anchor_.cross(direction).norm(),anchor_.dot(direction));
        if(angle>.018||coherent_>=3)anchor_=direction;
        else anchor_=(.875*anchor_+.125*direction).normalized();
        return raw.norm()*anchor_;
    }
private:
    Eigen::Vector3d anchor_=Eigen::Vector3d::Zero(),previous_=Eigen::Vector3d::Zero(),delta_=Eigen::Vector3d::Zero();
    int coherent_=0;
};
} }
