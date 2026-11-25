#pragma once
#include "new_controller/depth_input.hpp"
#include <cerrno>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <sstream>
#include <system_error>
#include <thread>

namespace ego4p0 {
class CameraSignal {
 public:
  CameraSignal() {
    if (sem_init(&value_, 0, 0) != 0)
      throw std::system_error(errno, std::generic_category(), "camera sem_init");
  }
  CameraSignal(const CameraSignal&) = delete;
  CameraSignal& operator=(const CameraSignal&) = delete;
  ~CameraSignal() { sem_destroy(&value_); }
  void post() noexcept {
    if (sem_post(&value_) != 0) std::terminate();
  }
  void wait() noexcept {
    while (sem_wait(&value_) != 0) {
      if (errno != EINTR) std::terminate();
    }
  }
 private:
  sem_t value_;
};

inline std::vector<int> cpu_siblings(int cpu) {
  std::ifstream stream("/sys/devices/system/cpu/cpu"+std::to_string(cpu)+"/topology/thread_siblings_list");
  std::string list;if(!std::getline(stream,list)) return {cpu};
  std::vector<int> result;std::istringstream input(list);std::string token;
  while(std::getline(input,token,',')) {
    const size_t dash=token.find('-');const int first=std::stoi(token),last=dash==std::string::npos ? first : std::stoi(token.substr(dash+1));
    for(int i=first;i<=last;++i) result.push_back(i);
  }
  return result;
}

class CameraWorkerPool {
 public:
  using Task=void(*)(void*,uint32_t,uint32_t,DepthFrame&);
  CameraWorkerPool()=default;
  CameraWorkerPool(const CameraWorkerPool&)=delete;
  CameraWorkerPool &operator=(const CameraWorkerPool&)=delete;
  ~CameraWorkerPool() {stop();}
  void start(int reader_cpu,void *owner,Task task) {
    stop();cpu_set_t allowed;CPU_ZERO(&allowed);
    if(sched_getaffinity(0,sizeof(allowed),&allowed)) throw std::runtime_error("cannot read camera-worker CPU affinity");
    for(int cpu:cpu_siblings(reader_cpu)) if(cpu>=0 && cpu<CPU_SETSIZE) CPU_CLR(cpu,&allowed);
    std::vector<int> chosen;cpu_set_t used;CPU_ZERO(&used);
    // Prefer four distinct physical cores, leaving logical CPU0 for the coordinator.
    for(int pass=0;pass<3 && chosen.size()<4;++pass) for(int cpu=0;cpu<CPU_SETSIZE && chosen.size()<4;++cpu) {
      if(!CPU_ISSET(cpu,&allowed) || std::find(chosen.begin(),chosen.end(),cpu)!=chosen.end() || (pass==0 && cpu==0)) continue;
      if(pass<2 && CPU_ISSET(cpu,&used)) continue;
      chosen.push_back(cpu);for(int sibling:cpu_siblings(cpu)) if(sibling<CPU_SETSIZE) CPU_SET(sibling,&used);
    }
    if(chosen.size()!=4) throw std::runtime_error("four allowed camera-worker CPUs required outside reader core/siblings");
    owner_=owner;task_=task;stopping_=false;pending_=4;
    try {
      for(uint32_t c=0;c<4;++c) {cpus_[c]=chosen[c];threads_[c]=std::thread([this,c]{loop(c);});}
      std::unique_lock<std::mutex> lock(mutex_);done_.wait(lock,[this]{return pending_==0;});lock.unlock();
      for(uint32_t c=0;c<4;++c) if(affinity_errors_[c] || scheduler_errors_[c]) throw std::runtime_error("camera worker affinity/SCHED_OTHER setup failed");
    } catch(...) {stop();throw;}
  }
  void run(DepthFrame &frame,uint32_t phase) {
    frame_=&frame;phase_=phase;
    for (auto &worker : signals_) worker.ready.post();
    for (auto &worker : signals_) worker.complete.wait();
    // Join the entire group even when one camera fails.
    for (const auto &worker : signals_)
      if (worker.error) std::rethrow_exception(worker.error);
  }
  void stop() noexcept {
    stopping_=true;
    for (size_t i=0; i<threads_.size(); ++i)
      if (threads_[i].joinable()) signals_[i].ready.post();
    for(auto &thread:threads_) if(thread.joinable()) thread.join();
  }
  const std::array<int,4> &cpus() const noexcept {return cpus_;}
  const std::array<int,4> &affinity_errors() const noexcept {return affinity_errors_;}
  const std::array<int,4> &scheduler_errors() const noexcept {return scheduler_errors_;}
  // Change only this pool's already initialized, idle camera threads.
  // The launcher grants a per-process RTPRIO limit; no global settings change.
  void enable_fifo5() {
    for(uint32_t c=0;c<4;++c) {
      if(!threads_[c].joinable()) throw std::runtime_error("camera worker is not initialized");
      sched_param requested{};requested.sched_priority=5;
      scheduler_errors_[c]=pthread_setschedparam(threads_[c].native_handle(),SCHED_FIFO,&requested);
      if(scheduler_errors_[c]) throw std::runtime_error("camera FIFO5 setup failed; use the FIFO launcher with scheduling permission");
    }
    refresh_schedulers();
    for(uint32_t c=0;c<4;++c) if(policies_[c]!=SCHED_FIFO || priorities_[c]!=5)
      throw std::runtime_error("camera FIFO5 readback mismatch");
  }
  void refresh_schedulers() {
    for(uint32_t c=0;c<4;++c) {
      sched_param actual{};
      if(pthread_getschedparam(threads_[c].native_handle(),&policies_[c],&actual))
        throw std::runtime_error("camera scheduler readback failed");
      priorities_[c]=actual.sched_priority;
    }
  }
  const std::array<int,4> &policies() const noexcept {return policies_;}
  const std::array<int,4> &priorities() const noexcept {return priorities_;}
 private:
  void loop(uint32_t camera) {
    const std::string name="depth_cam"+std::to_string(camera);pthread_setname_np(pthread_self(),name.c_str());
    cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpus_[camera],&mask);
    affinity_errors_[camera]=pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask);
    sched_param normal{};scheduler_errors_[camera]=pthread_setschedparam(pthread_self(),SCHED_OTHER,&normal);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!--pending_) done_.notify_one();
    }
    auto &worker=signals_[camera];
    for(;;) {
      worker.ready.wait();
      if(stopping_) return;
      worker.error=nullptr;
      try { task_(owner_,camera,phase_,*frame_); }
      catch (...) { worker.error=std::current_exception(); }
      worker.complete.post();
    }
  }
  std::array<std::thread,4> threads_;
  std::array<int,4> cpus_{{-1,-1,-1,-1}},affinity_errors_{},scheduler_errors_{};
  std::array<int,4> policies_{{-1,-1,-1,-1}},priorities_{{-1,-1,-1,-1}};
  struct alignas(64) Signals {
    CameraSignal ready, complete;
    std::exception_ptr error;
  };
  std::array<Signals,4> signals_;
  std::mutex mutex_;std::condition_variable done_;
  void *owner_=nullptr;Task task_=nullptr;DepthFrame *frame_=nullptr;
  uint32_t phase_=0,pending_=0;
  std::atomic<bool> stopping_{true};
};
} // namespace ego4p0
