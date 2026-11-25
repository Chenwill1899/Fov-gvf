#pragma once
#include <cstdint>
#include <mutex>

namespace ego4p0 {
// Only the receive executor changes the timebase. A ticket covers both the
// three-buffer snapshot and the commit of all coordinator-visible outputs.
class InputEpoch {
 public:
  struct Ticket { uint64_t epoch; int64_t clock_ns; bool accepting; };
  std::unique_lock<std::mutex> lock() { return std::unique_lock<std::mutex>(mutex_); }
  Ticket ticket() const { return current_; }  // caller holds lock or is the sole writer
  void advance(int64_t time) { current_.clock_ns=time; }
  void transition(int64_t time, bool accepting) {
    ++current_.epoch;
    current_.clock_ns=time;
    current_.accepting=accepting;
  }
  bool can_commit(Ticket captured, int64_t oldest, int64_t max_age) const {
    if(captured.epoch!=current_.epoch)return false;
    return oldest<0 || (current_.accepting && oldest<=current_.clock_ns &&
                       current_.clock_ns-oldest<=max_age);
  }
 private:
  std::mutex mutex_;
  Ticket current_{0,-1,true};
};
}
