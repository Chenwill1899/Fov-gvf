#pragma once
#include "new_controller/harmonic_planner.hpp"
#include "new_controller/depth_camera_workers.hpp"
#include <thread>
namespace ego4p0 {
struct HarmonicEvent {
  uint64_t version=0,sequence=0;int64_t stamp_ns=0,completed_ns=0,total_ns=0,coarse_ns=0,fine_ns=0,curve_ns=0,capture_ns=0;
  double coarse_residual=0,fine_residual=0;uint32_t coarse_iterations=0,fine_iterations=0,coarse_nodes=0,fine_nodes=0,curve_steps=0;
  FieldStatus status=FieldStatus::NoInput;CurveEnd termination=CurveEnd::NoField;bool expanded=false,reused=false;
  uint64_t potential_version=0,curve_version=0;
  int64_t component_ns=0,coarse_domain_ns=0,fine_domain_ns=0,coarse_cache_ns=0,coarse_assembly_ns=0,coarse_iteration_ns=0,coarse_residual_ns=0,coarse_store_ns=0,fine_cache_ns=0,fine_assembly_ns=0,fine_iteration_ns=0,fine_residual_ns=0,fine_store_ns=0,query_ns=0,segment_ns=0;
  uint32_t refined_nodes=0;double angular_length=0;
  FieldStatus coarse_status=FieldStatus::NoInput;
  bool coarse_graph_reused=false;int64_t coarse_graph_cache_ns=0;
  int64_t coarse_topology_ns=0,coarse_cpu_ns=0,fine_cpu_ns=0;

};
class HarmonicWorker {
 public:
  ~HarmonicWorker(){stop();}
  void start(const std::array<CameraCalibration,4> &cal,uint32_t w,uint32_t h,int reader_cpu,size_t event_capacity,const HarmonicConfig &config={}) {
    stop();planner_.configure(cal,w,h,config);
    for(auto &s:depth_.slots)planner_.allocate(s,w,h);
    for(auto &s:result_.slots)planner_.allocate(s);
    events_.reserve(event_capacity);events_.clear();stopping_=false;failed_=false;initialized_=false;
    cpu_set_t allowed;CPU_ZERO(&allowed);if(sched_getaffinity(0,sizeof(allowed),&allowed))throw std::runtime_error("field affinity inventory failed");
    for(int cpu:cpu_siblings(reader_cpu))if(cpu>=0&&cpu<CPU_SETSIZE)CPU_CLR(cpu,&allowed);
    if(!CPU_COUNT(&allowed))throw std::runtime_error("no field CPU outside reader core");
    allowed_=allowed;
    thread_=std::thread([this]{loop();});
    while(!initialized_.load(std::memory_order_acquire))std::this_thread::yield();
    if(failed_) {stop();throw std::runtime_error("harmonic worker affinity/scheduler setup failed");}
  }
  // Called only by the single ROS receiving thread, after new-group processing.
  void submit_depth(const DepthFrame &f) {auto &d=depth_.writer();d.capture(f);if(d.ready)depth_.publish();}
  // Called only by the 2ms thread. Trivial scalar copy, no mutex or image copy.
  void submit_intent(const HarmonicIntent &i) {intent_.writer()=i;intent_.publish();}
  const HarmonicResult *latest(){return result_.latest();} // one 2ms consumer
  void stop(){stopping_.store(true,std::memory_order_release);if(thread_.joinable())thread_.join();}
  bool failed()const{return failed_.load(std::memory_order_acquire);}
  const std::string &error()const{return error_;} // after stop only
  int policy()const{return policy_;}int priority()const{return priority_;}
  const std::vector<HarmonicEvent> &events()const{return events_;} // after stop
 private:
  void loop() {
    pthread_setname_np(pthread_self(),"ego4p0_fields");sched_param p{};
    const int affinity=pthread_setaffinity_np(pthread_self(),sizeof(allowed_),&allowed_),scheduler=pthread_setschedparam(pthread_self(),SCHED_OTHER,&p);
    pthread_getschedparam(pthread_self(),&policy_,&p);priority_=p.sched_priority;
    if(affinity||scheduler) {error_="field affinity or SCHED_OTHER failed";failed_.store(true,std::memory_order_release);initialized_.store(true,std::memory_order_release);return;}
    initialized_.store(true,std::memory_order_release);
    uint64_t last_sequence=0,last_geometry=0,last_configuration=0,version=0;int64_t last_stamp=0;HarmonicIntent last_intent;bool had_intent=false;
    try {
      while(!stopping_.load(std::memory_order_acquire)) {
        const auto *d=depth_.latest();const auto *i=intent_.latest();
        if(d&&i&&(d->sequence!=last_sequence||d->stamp_ns!=last_stamp||d->geometry!=last_geometry||d->configuration!=last_configuration||!had_intent||!HarmonicPlanner::same_intent(*i,last_intent))) {
          auto &r=result_.writer();planner_.build(*d,*i,r);r.version=++version;result_.publish();
          last_sequence=d->sequence;last_stamp=d->stamp_ns;last_geometry=d->geometry;last_configuration=d->configuration;last_intent=*i;had_intent=true;
          if(events_.size()==events_.capacity())throw std::runtime_error("field event capacity exceeded");
          events_.push_back({r.version,r.sequence,r.stamp_ns,r.completed_ns,r.total_ns,r.coarse.solve_ns,r.fine.solve_ns,r.curve_ns,r.capture_ns,r.coarse.residual,r.fine.residual,r.coarse.iterations,r.fine.iterations,r.coarse.nodes,r.fine.nodes,r.integration_steps,r.status,r.termination,r.expanded,r.reused});
          auto &e=events_.back();e.potential_version=r.field_version;e.curve_version=r.curve_version;e.component_ns=r.component_ns;e.coarse_domain_ns=r.coarse.domain_ns;e.fine_domain_ns=r.fine.domain_ns;e.coarse_cache_ns=r.coarse.cache_ns;e.coarse_assembly_ns=r.coarse.assembly_ns;e.coarse_iteration_ns=r.coarse.iteration_ns;e.coarse_residual_ns=r.coarse.residual_ns;e.coarse_store_ns=r.coarse.store_ns;e.fine_cache_ns=r.fine.cache_ns;e.fine_assembly_ns=r.fine.assembly_ns;e.fine_iteration_ns=r.fine.iteration_ns;e.fine_residual_ns=r.fine.residual_ns;e.fine_store_ns=r.fine.store_ns;e.query_ns=r.query_ns;e.segment_ns=r.segment_ns;e.refined_nodes=r.adaptive.refined_nodes;e.angular_length=r.angular_length;e.coarse_status=r.coarse.status;e.coarse_topology_ns=r.coarse_topology_ns;e.coarse_cpu_ns=r.coarse.solve_cpu_ns;e.fine_cpu_ns=r.fine.solve_cpu_ns;e.coarse_graph_reused=r.coarse_graph_reused;e.coarse_graph_cache_ns=r.coarse_graph_cache_ns;

        } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch(const std::exception &e){error_=e.what();failed_.store(true,std::memory_order_release);}
  }
  HarmonicPlanner planner_;TripleBuffer<HarmonicDepthInput>depth_;TripleBuffer<HarmonicIntent>intent_;TripleBuffer<HarmonicResult>result_;
  std::thread thread_;std::atomic<bool>stopping_{true},failed_{false},initialized_{false};cpu_set_t allowed_{};
  int policy_=0,priority_=0;std::string error_;std::vector<HarmonicEvent>events_;
};
} // namespace ego4p0
