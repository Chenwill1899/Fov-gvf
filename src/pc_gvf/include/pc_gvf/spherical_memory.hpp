#pragma once
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <map>
#include <cstdint>
namespace pc_gvf { namespace depth_angular {
struct DepthObservation;
class SphericalMemory {
public:
    static constexpr double lifetime=4.;
    void ingest(const DepthObservation& observation,bool broadphase = true);
    std::vector<std::shared_ptr<const DepthObservation>> evidence(double now) const;
    std::vector<Eigen::Vector3d> directions(const Eigen::Vector3d& position,double now) const;
    const std::map<int,std::shared_ptr<const DepthObservation>>& slots()const{return slots_;}
    void restore(int bin,std::shared_ptr<const DepthObservation> o){slots_[bin]=std::move(o);}
private:
    std::map<int,std::shared_ptr<const DepthObservation>> slots_;
    std::map<int,std::pair<double,std::uint64_t>> checked_;
};
} }
