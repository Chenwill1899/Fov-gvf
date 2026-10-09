#ifndef PC_GVF_VERIFIED_START_HPP
#define PC_GVF_VERIFIED_START_HPP
#include <Eigen/Core>
#include <string>
namespace pc_gvf { namespace depth_angular {
// Verifies the exact shipped Cloud USD/occupancy pair, then checks the entire
// fixed free ball against occupied cells and the finite map boundary once.
// This API consumes local geometry; it accepts no ROS free-space assertions.
bool certifyDefaultCloudStart(const std::string& scene,const std::string& occupancy,
    const Eigen::Vector3d& center,double radius,std::string* diagnostic);
} }
#endif
