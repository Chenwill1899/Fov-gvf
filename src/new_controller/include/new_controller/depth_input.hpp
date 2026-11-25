#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "new_controller/depth_observation_storage.hpp"
#include "new_controller/local_field_storage.hpp"

namespace ego4p0 {
// SPSC: one receiver thread owns writer(), one controller thread owns latest().
// A returned reference remains owned by the controller until its next latest().
// No mutex, allocation, shared_ptr refcount, or image copy in latest().
template<class T> class TripleBuffer {
 public:
  std::array<T, 3> slots;
  T &writer() { return slots[write_]; }
  bool publish() {
    const int old = middle_.exchange(write_ | dirty, std::memory_order_acq_rel);
    write_ = old & index_mask;
    ready_.store(true, std::memory_order_release);
    return (old & dirty) != 0;  // unread middle frame replaced
  }
  const T *latest() {
    if (!ready_.load(std::memory_order_acquire)) return nullptr;
    if (middle_.load(std::memory_order_acquire) & dirty)
      read_ = middle_.exchange(read_, std::memory_order_acq_rel) & index_mask;
    return &slots[read_];
  }
 private:
  static constexpr int dirty = 4, index_mask = 3;
  static_assert(std::atomic<int>::is_always_lock_free && std::atomic<bool>::is_always_lock_free);
  alignas(64) std::atomic<int> middle_{1};
  alignas(64) std::atomic<bool> ready_{false};
  int write_ = 0, read_ = 2;
};

struct DepthRepairStats {
  uint64_t invalid=0, filled=0, unresolved=0;
};
struct DepthFrame {
  std::array<std::vector<float>, 4> D;         // original optical-axis Z, meters
  std::array<std::vector<float>, 4> D_filled;  // independent original-neighbor repair
  std::array<DepthRepairStats,4> repair{};
  std::array<int64_t,4> repair_ns{};
  int64_t preprocessing_ns=0;
  FrameObservationIndex observations;
  LocalFieldData local;
  int64_t t_ns = 0, received_ns = 0, published_ns = 0;
  uint64_t sequence = 0;
  void allocate(size_t pixels) {
    for (auto &d : D) d.resize(pixels, 0.f);
    for (auto &d : D_filled) d.resize(pixels, 0.f);
  }
};
struct BodyState {
  std::array<double, 3> p_W{};
  std::array<double, 3> v_B{};
  std::array<double, 9> R_WB{1,0,0,0,1,0,0,0,1};
  int64_t t_ns = 0;
};
enum class DepthType { OpticalAxisZMeters };
struct CameraCalibration {
  std::string frame;
  std::array<double,9> K{};
  std::array<double,16> T_BC{};  // row major, p_B = T_BC * p_C
  double near_clip_z_m=std::numeric_limits<double>::quiet_NaN(); // absent readback stays unknown
};
// All control inputs, borrowed without allocation/copy. Valid until the reader's
// next TripleBuffer::latest() call. Do not retain image/state pointers across it.
class PixelRayTables;
struct ControllerInputView {
  const DepthFrame *depth;
  const BodyState *state;
  const std::array<CameraCalibration,4> *calibration;
  size_t width,height;
  DepthType depth_type=DepthType::OpticalAxisZMeters;
  const PixelRayTables *pixel_rays=nullptr; // immutable 2-D lookup tables, no world points
  int64_t camera_time_ns(size_t) const { return depth->t_ns; } // exact four-way sync
};
inline BodyState body_state(double x, double y, double z, double w,
                            const std::array<double,3> &v_W, int64_t stamp) {
  const double norm = std::sqrt(x*x+y*y+z*z+w*w);
  if (!std::isfinite(norm) || std::abs(norm-1.)>1e-3)
    throw std::runtime_error("invalid body quaternion");
  x/=norm; y/=norm; z/=norm; w/=norm;
  BodyState s;
  s.R_WB = {1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
            2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
            2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};
  for (int i=0;i<3;++i) {
    if (!std::isfinite(v_W[i])) throw std::runtime_error("nonfinite velocity");
    for (int j=0;j<3;++j) s.v_B[i]+=s.R_WB[j*3+i]*v_W[j];
  }
  s.t_ns=stamp;
  return s;
}

// Only partial groups are retained (bounded). Never mixes camera timestamps.
template<class Message> class FourWaySynchronizer {
 public:
  using Ptr = std::shared_ptr<const Message>;
  using Group = std::array<Ptr,4>;
  uint64_t partial_discarded=0, late_or_duplicate=0;
  bool push(int camera, int64_t stamp, Ptr msg, Group &complete) {
    if (stamp<=last_complete_) { ++late_or_duplicate; return false; }
    auto &g=pending_[stamp];
    if (g[camera]) ++late_or_duplicate;
    g[camera]=std::move(msg);
    if (g[0] && g[1] && g[2] && g[3]) {
      complete=g; last_complete_=stamp;
      auto end=pending_.upper_bound(stamp);
      for(auto it=pending_.begin();it!=end;++it) if(it->first!=stamp) ++partial_discarded;
      pending_.erase(pending_.begin(),end);
      return true;
    }
    while(pending_.size()>4) { pending_.erase(pending_.begin()); ++partial_discarded; }
    return false;
  }
  size_t pending() const { return pending_.size(); }
 private:
  int64_t last_complete_=-1;
  std::map<int64_t,Group> pending_;
};
}  // namespace ego4p0
