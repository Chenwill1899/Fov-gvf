#include "new_controller/depth_input.hpp"
#include "new_controller/depth_preprocess.hpp"
#include "new_controller/depth_observation_io.hpp"
#include "new_controller/depth_size_query.hpp"
#include "new_controller/depth_local_io.hpp"
#include "new_controller/desired_direction.hpp"
#include "new_controller/harmonic_io.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <unistd.h>
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif

namespace fs=std::filesystem;
using namespace ego4p0;
using Image=sensor_msgs::msg::Image;
static int64_t mono_ns() { timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000000LL+t.tv_nsec; }
static int64_t thread_cpu_ns() { timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1000000000LL+t.tv_nsec; }
static inline void cpu_relax() {
#if defined(__x86_64__) || defined(__i386__)
  _mm_pause();
#else
  std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}
template<class T> int64_t stamp_ns(const T &m) { return int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec; }
struct Options {
  HarmonicConfig field;
  fs::path output;
  std::string run_id, mode="reference", duration_clock="sim";
  std::string desired_source="velocity";double goal_speed_mps=BaseConfig::max_speed_mps;
  double duration=.5, timeout=300;
  double azimuth_resolution_deg=1.,elevation_resolution_deg=1.;
  int cpu=0, fifo=5, spin_us=0;
  int64_t timer_slack_ns=0;bool lock_memory=false,scheduler_gate=false;
};
struct Sample {
  int64_t scheduled=0,begin=0,clock=0,stamp=0,state_stamp=0,published=0;
  int64_t read_ns=0,work_ns=0;
  uint64_t sequence=0,skipped=0;
  std::array<float,4> probe{};
  double ray_probe=0.;
  float pixel_query_z=std::numeric_limits<float>::quiet_NaN(),region_min_z=std::numeric_limits<float>::quiet_NaN();
  uint32_t region_valid=0,region_total=0,region_tiles=0,region_pixels_scanned=0,direction_sources=0;
  uint64_t field_version=0,field_sequence=0;int64_t field_completed_ns=0;
  FieldStatus field_status=FieldStatus::NoInput;CurveEnd curve_end=CurveEnd::NoField;bool field_compatible=false;
  DesiredMotion desired;uint32_t local_excluded_blocks=0,local_excluded_cells=0,local_available_cells=0,local_unknown_cells=0;
};
struct ImageEvent { int64_t stamp=0,wall=0;int camera=0; };
struct FrameEvent {
  int64_t stamp=0,received=0,published=0,clock=0;uint64_t seq=0;bool overwritten=false;
  int64_t raw_copy_ns=0,preprocess_ns=0;
  std::array<DepthRepairStats,4> repair{};
  std::array<int64_t,4> repair_ns{};
  int64_t direction_ns=0,region_ns=0;
  uint32_t direction_sources=0,unobserved_cells=0;
  int64_t shared_projection_local_max_ns=0,local_merge_ns=0;uint32_t local_excluded_blocks=0,local_excluded_cells=0,local_available_cells=0,local_unknown_cells=0;
};

class DepthReader:public rclcpp::Node {
 public:
  explicit DepthReader(const Options &o):Node("ego4p0_cpp_depth_input"),o_(o) {
    std::ofstream(o.output/"base_config.json")<<BaseConfig::json<<'\n';
    auto c=YAML::LoadFile((o.output/"simulation.sensors.json").string());
    if(c["run_id"].as<std::string>()!=o.run_id || c["depth_type"].as<std::string>()!="optical_axis_z_m")
      throw std::runtime_error("calibration run or depth type mismatch");
    width_=c["render_width"].as<size_t>();height_=c["render_height"].as<size_t>();pixels_=width_*height_;
    if(width_<8 || height_<8 || width_>4096 || height_>4096 || c["cameras"].size()!=4)
      throw std::runtime_error("invalid four camera calibration dimensions");
    const std::array<std::string,4> names{"Front","Left","Back","Right"};
    for(int i=0;i<4;++i) {
      auto v=c["cameras"][i];auto &a=cal_[i];
      if(v["name"].as<std::string>()!=names[i]) throw std::runtime_error("wrong camera order");
      a.frame=v["frame_id"].as<std::string>();
      a.K={v["fx"].as<double>(),0,v["cx"].as<double>(),0,v["fy"].as<double>(),v["cy"].as<double>(),0,0,1};
      if(v["near_clip_z_m"]) a.near_clip_z_m=v["near_clip_z_m"].as<double>();
      for(double x:a.K) if(!std::isfinite(x)) throw std::runtime_error("nonfinite K");
      if(a.K[0]<=0 || a.K[4]<=0) throw std::runtime_error("invalid focal length");
      for(int r=0;r<3;++r) {
        for(int k=0;k<3;++k) a.T_BC[r*4+k]=v["robot_from_optical_rotation"][r][k].as<double>();
        a.T_BC[r*4+3]=v["camera_position_robot_m"][r].as<double>();
      }
      a.T_BC[15]=1;
      for(double x:a.T_BC) if(!std::isfinite(x)) throw std::runtime_error("nonfinite T_BC");
      for(int r=0;r<3;++r) for(int k=0;k<3;++k) {
        double dot=0;for(int j=0;j<3;++j) dot+=a.T_BC[j*4+r]*a.T_BC[j*4+k];
        if(std::abs(dot-(r==k?1.:0.))>1e-6) throw std::runtime_error("nonorthogonal T_BC");
      }
      const auto &t=a.T_BC;
      const double det=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);
      if(std::abs(det-1)>1e-6) throw std::runtime_error("reflected T_BC");
    }
    for(auto &slot:images_.slots) slot.allocate(pixels_);
    first_frame_.allocate(pixels_);copy_frame_.allocate(pixels_);
    const int64_t rays_begin=mono_ns();ray_tables_.configure(cal_,width_,height_);
    ray_build_ns_=mono_ns()-rays_begin;
    observation_builder_.configure(cal_,width_,height_,o_.azimuth_resolution_deg,o_.elevation_resolution_deg,16,3,o_.cpu>=0 ? o_.cpu : 5,true,true,true,true);
    for(auto &slot:images_.slots) observation_builder_.allocate(slot);
    observation_builder_.allocate(first_frame_);
    if(o_.azimuth_resolution_deg!=1. || o_.elevation_resolution_deg!=1.)throw std::runtime_error("two-level harmonic planner requires 1 degree input grid");
    field_worker_.start(cal_,width_,height_,o_.cpu>=0 ? o_.cpu : 5,size_t(o.timeout/BaseConfig::control_period_s)+1024,o_.field);
    observation_builder_.refresh_worker_schedulers();
    samples_.resize(size_t(o.timeout/BaseConfig::control_period_s)+1024); // prefault timing storage before the loop
    const size_t events=size_t(o.duration*c["requested_camera_hz"].as<double>(60.)*2)+4096;
    frames_.reserve(events);received_.reserve(events*4);
    auto qos=rclcpp::SensorDataQoS().keep_last(2);
    const std::array<std::string,4> roots{"/sim/depth","/sim/depth_left","/sim/depth_back","/sim/depth_right"};
    for(int i=0;i<4;++i) {
      image_subs_[i]=create_subscription<Image>(roots[i]+"/image_raw",qos,[this,i](Image::ConstSharedPtr m) {
        try { receive(i,std::move(m)); } catch(const std::exception &e) { fail(e.what()); }
      });
      info_subs_[i]=create_subscription<sensor_msgs::msg::CameraInfo>(roots[i]+"/camera_info",qos,
        [this,i](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          try {
            if(info_ready_[i] && m->k==last_k_[i] && m->d==last_d_[i] && m->width==width_ && m->height==height_ && m->header.frame_id==cal_[i].frame && m->distortion_model=="plumb_bob") return;
            if(m->width!=width_ || m->height!=height_ || m->header.frame_id!=cal_[i].frame || m->distortion_model!="plumb_bob" || m->d.size()!=5)
              throw std::runtime_error("CameraInfo shape/frame/distortion mismatch");
            for(size_t j=0;j<9;++j)
              if(!std::isfinite(m->k[j]) || std::abs(m->k[j]-cal_[i].K[j])>1e-5*(1+std::abs(cal_[i].K[j])))
                throw std::runtime_error("CameraInfo K differs from actual USD projection");
            for(auto x:m->d) if(!std::isfinite(x)||std::abs(x)>1e-12) throw std::runtime_error("nonzero distortion");
            last_k_[i]=m->k;last_d_[i]=m->d;info_ready_[i]=true;++info_validations_[i];
          }catch(const std::exception &e){fail(e.what());}
        });
    }
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/sim/odom",qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      try {
        if(m->header.frame_id!="world" || m->child_frame_id!="base_link") throw std::runtime_error("odometry frame mismatch");
        const auto &q=m->pose.pose.orientation;const auto &v=m->twist.twist.linear;
        auto &s=states_.writer();s=body_state(q.x,q.y,q.z,q.w,{v.x,v.y,v.z},stamp_ns(*m));
        const auto &p=m->pose.pose.position;s.p_W={p.x,p.y,p.z};
        for(double x:s.p_W) if(!std::isfinite(x)) throw std::runtime_error("nonfinite position");
        states_.publish();
      }catch(const std::exception &e){fail(e.what());}
    });
    intent_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/human_intent",qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        if(o_.desired_source!="velocity") return;
        if(m->header.frame_id!="world" && m->header.frame_id!="base_link") {fail("intent frame must be world or base_link");return;}
        DesiredInput in;in.mode=m->header.frame_id=="world" ? DesiredInputMode::WorldVelocity : DesiredInputMode::BodyVelocity;
        in.vector={m->twist.linear.x,m->twist.linear.y,m->twist.linear.z};in.stamp_ns=stamp_ns(*m);
        desired_.writer()=in;desired_.publish();
      });
    goal_sub_=create_subscription<geometry_msgs::msg::PointStamped>("/controller/goal",qos,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        if(o_.desired_source!="goal") return;
        if(m->header.frame_id!="world") {fail("goal frame must be world");return;}
        DesiredInput in;in.mode=DesiredInputMode::WorldGoal;in.vector={m->point.x,m->point.y,m->point.z};
        in.goal_speed_mps=o_.goal_speed_mps;in.stamp_ns=stamp_ns(*m);desired_.writer()=in;desired_.publish();
      });
    tangent_sub_=create_subscription<geometry_msgs::msg::Vector3Stamped>("/controller/reference_tangent",qos,
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) {
        if(m->header.frame_id!="base_link"){fail("reference tangent must be base_link");return;}
        Direction3 q{m->vector.x,m->vector.y,m->vector.z};
        if(!unit_direction(q)){fail("reference tangent must be finite and nonzero");return;}
        tangent_.writer()=q;tangent_.publish();
      });
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",qos,[this](rosgraph_msgs::msg::Clock::ConstSharedPtr m){
      clock_.store(int64_t(m->clock.sec)*1000000000LL+m->clock.nanosec,std::memory_order_release);
    });
  }
  void fail(const std::string &message) { error_message_=message;fatal_.store(true,std::memory_order_release); }
  void run() {
    pthread_setname_np(pthread_self(),"ego4p2_5ms");
    if(o_.cpu>=0) {
      cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(o_.cpu,&mask);
      affinity_result_=pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask);
    }
    // Optional administrator rendezvous, entirely before the timed loop.
    // The supervisor raises only this verified reader process' RT limit;
    // Isaac and the launcher keep their original limits.
    if(o_.scheduler_gate) {
      std::ofstream(o_.output/"reader_scheduler_ready")<<getpid()<<'\n';
      const int64_t gate_deadline=mono_ns()+15000000000LL;
      while(!fs::exists(o_.output/"reader_scheduler_authorized")) {
        if(!rclcpp::ok() || mono_ns()>gate_deadline) {loop_error_="reader scheduler authorization timeout";return;}
        timespec pause{0,1000000};nanosleep(&pause,nullptr);
      }
    }
    if(o_.fifo==5) observation_builder_.enable_fifo5_workers();
    observation_builder_.refresh_worker_schedulers();
    if(o_.fifo>0) {sched_param param{};param.sched_priority=o_.fifo;fifo_result_=pthread_setschedparam(pthread_self(),SCHED_FIFO,&param);}
    int policy=0;sched_param actual{};pthread_getschedparam(pthread_self(),&policy,&actual);
    actual_policy_=policy;actual_priority_=actual.sched_priority;
    if(fifo_result_ || policy!=(o_.fifo?SCHED_FIFO:SCHED_OTHER) || actual.sched_priority!=o_.fifo) throw std::runtime_error("reader scheduling readback failed");
    timer_slack_before_=prctl(PR_GET_TIMERSLACK);
    if(o_.timer_slack_ns>0 && prctl(PR_SET_TIMERSLACK,static_cast<unsigned long>(o_.timer_slack_ns))!=0) timer_slack_error_=errno;
    timer_slack_after_=prctl(PR_GET_TIMERSLACK);
    if(o_.lock_memory && mlockall(MCL_CURRENT)!=0) memory_lock_error_=errno;
    getrusage(RUSAGE_THREAD,&usage_before_);
    run_cpu_start_=thread_cpu_ns();run_wall_start_=mono_ns();
    const int64_t period=BaseConfig::period_ns,started=run_wall_start_;int64_t next=started+period;
    std::ofstream(o_.output/"reader_ready")<<o_.run_id<<'\n';
    while(rclcpp::ok()&&!fatal_.load(std::memory_order_acquire)) {
      const int64_t sleep_until=next-o_.spin_us*1000LL;
      if(mono_ns()<sleep_until) {
        timespec t{sleep_until/1000000000LL,sleep_until%1000000000LL};
        int ret;
        do{ret=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);}while(ret==EINTR && rclcpp::ok());
        if(ret && ret!=EINTR) {loop_error_="clock_nanosleep failed";break;}
      }
      while(mono_ns()<next) cpu_relax();
      const int64_t begin=mono_ns();
      const auto *frame=images_.latest();const auto *body=states_.latest();
      const int64_t clock=clock_.load(std::memory_order_acquire);
      if(begin-started>int64_t(o_.timeout*1e9)) {loop_error_="wall timeout";break;}
      if(frame && body && clock>=0) {
        if(!start_wall_) {start_wall_=begin;start_clock_=clock;first_state_=*body;}
        if(clock<start_clock_) {loop_error_="clock moved backwards";break;}
        if(used_>=samples_.size()) {loop_error_="preallocated timing capacity exceeded";break;}
        ControllerInputView input{frame,body,&cal_,width_,height_};
        // Both original and filled images belong to the same protected slot;
        // camera-optical rays are immutable throughout all timed reads.
        input.pixel_rays=&ray_tables_;
        const DepthFrame *consumed=input.depth;
        if(o_.mode=="copy") {
          for(int i=0;i<4;++i) {
            std::memcpy(copy_frame_.D[i].data(),frame->D[i].data(),pixels_*sizeof(float));
            std::memcpy(copy_frame_.D_filled[i].data(),frame->D_filled[i].data(),pixels_*sizeof(float));
          }
          consumed=&copy_frame_;
        }
        Sample &s=samples_[used_++];
        for(int i=0;i<4;++i) s.probe[i]=consumed->D_filled[i][pixels_/2];
        s.ray_probe=input.pixel_rays->at(0,width_/2,height_/2).d[2]; // no table generation in this loop
        const DepthObservationView queries(*input.depth);
        const auto pixel=queries.pixel(0,width_/2,height_/2);
        const auto region=queries.region(0,{0,0,static_cast<int>(width_),static_cast<int>(height_)});
        s.pixel_query_z=pixel.z.value_or(std::numeric_limits<float>::quiet_NaN());
        s.region_min_z=region.z.value_or(std::numeric_limits<float>::quiet_NaN());
        s.region_valid=region.valid_pixels;s.region_total=region.total_pixels;s.region_tiles=region.tiles_used;s.region_pixels_scanned=region.pixels_scanned;
        s.direction_sources=frame->observations.source_count;
        const auto *intent=desired_.latest();s.desired=desired_motion(intent ? *intent : DesiredInput{},*body);
        s.local_excluded_blocks=frame->local.excluded_blocks;s.local_excluded_cells=frame->local.excluded_cells;s.local_available_cells=frame->local.available_cells;s.local_unknown_cells=frame->local.unknown_cells;
        HarmonicIntent field_intent;field_intent.desired=s.desired;field_intent.state_stamp_ns=body->t_ns;field_intent.R_WB=body->R_WB;field_intent.p_W=body->p_W;
        const auto *tangent=tangent_.latest();field_intent.q0=tangent ? *tangent : body->v_B;
        if(!unit_direction(field_intent.q0))field_intent.q0=s.desired.d_desired;
        field_worker_.submit_intent(field_intent);
        const auto *field=field_worker_.latest();
        if(field) {
          s.field_version=field->version;s.field_sequence=field->sequence;s.field_completed_ns=field->completed_ns;s.field_status=field->status;s.curve_end=field->termination;
          s.field_compatible=field->sequence==frame->sequence && field->stamp_ns==frame->t_ns && field->configuration==frame->observations.configuration_id && field->geometry==frame->local.geometry_id && field->parameter_revision==o_.field.revision && HarmonicPlanner::same_intent(field->intent,field_intent);
        }
        if(field_worker_.failed()){loop_error_="harmonic worker failed";break;}
        const int64_t read_end=mono_ns();
        s.scheduled=next;s.begin=begin;s.clock=clock;s.stamp=frame->t_ns;s.state_stamp=body->t_ns;
        s.sequence=frame->sequence;s.published=frame->published_ns;s.read_ns=read_end-begin;
        last_frame_=frame;last_state_=*body;end_clock_=clock;end_wall_=begin;
        const int64_t finish=mono_ns();s.work_ns=finish-begin;
        next+=period;
        if(finish>=next) {s.skipped=uint64_t((finish-next)/period+1);next+=s.skipped*period;}
        const bool done=o_.duration_clock=="wall" ? begin-start_wall_>=int64_t(o_.duration*1e9) : clock-start_clock_>=int64_t(o_.duration*1e9);
        if(done) {completed_=true;break;}
      } else {
        ++waiting_ticks_;next+=period;
        const int64_t now=mono_ns();if(now>=next) next+=((now-next)/period+1)*period;
      }
    }
    run_wall_end_=mono_ns();run_cpu_end_=thread_cpu_ns();getrusage(RUSAGE_THREAD,&usage_after_);
  }
  void write_results() {
    field_worker_.stop();save_harmonic_events(o_.output,field_worker_);
    if(const auto *field=field_worker_.latest())save_harmonic(o_.output,*field);
    if(field_worker_.failed())loop_error_="harmonic worker: "+field_worker_.error();
    std::ofstream f(o_.output/"reads.csv");
    f<<"read_index,scheduled_ns,wall_monotonic_ns,clock_ns,sim_elapsed_s,interval_ms,read_ms,work_ms,wake_lateness_ms,skipped_slots,sequence,new_frame,t1_ns,t2_ns,t3_ns,t4_ns,depth_age_ms,buffer_age_ms,state_t_ns,state_age_ms,probe_D1_filled,probe_D2_filled,probe_D3_filled,probe_D4_filled,probe_ray_dz,pixel_query_z,region_min_z,region_valid_pixels,region_total_pixels,region_tiles_used,region_pixels_scanned,direction_sources,desired_x,desired_y,desired_z,desired_speed_mps,desired_valid,local_excluded_blocks,local_excluded_cells,local_available_cells,local_unknown_cells,field_version,field_sequence,field_age_ms,field_status,curve_termination,field_compatible\n"<<std::setprecision(12);
    uint64_t prev=0;
    for(size_t i=0;i<used_;++i) {
      const auto &s=samples_[i];f<<i<<','<<s.scheduled<<','<<s.begin<<','<<s.clock<<','<<(s.clock-start_clock_)/1e9<<',';
      if(i) f<<(s.begin-samples_[i-1].begin)/1e6;
      f<<','<<s.read_ns/1e6<<','<<s.work_ns/1e6<<','<<(s.begin-s.scheduled)/1e6<<','<<s.skipped<<','<<s.sequence<<','<<(s.sequence!=prev);
      for(int j=0;j<4;++j) f<<','<<s.stamp;
      f<<','<<(s.clock-s.stamp)/1e6<<','<<(s.begin-s.published)/1e6<<','<<s.state_stamp<<','<<(s.clock-s.state_stamp)/1e6;
      for(float z:s.probe) f<<','<<z;
      f<<','<<s.ray_probe<<','<<s.pixel_query_z<<','<<s.region_min_z<<','<<s.region_valid<<','<<s.region_total<<','<<s.region_tiles<<','<<s.region_pixels_scanned<<','<<s.direction_sources;
      for(double x:s.desired.d_desired) f<<','<<x;
      f<<','<<s.desired.v_desired<<','<<s.desired.valid<<','<<s.local_excluded_blocks<<','<<s.local_excluded_cells<<','<<s.local_available_cells<<','<<s.local_unknown_cells<<','<<s.field_version<<','<<s.field_sequence<<','<<(s.field_completed_ns ? (s.begin-s.field_completed_ns)/1e6 : -1.)<<','<<field_status_name(s.field_status)<<','<<curve_end_name(s.curve_end)<<','<<s.field_compatible<<'\n';prev=s.sequence;
    }
    std::ofstream events(o_.output/"frames.csv");events<<"sequence,image_stamp_ns,received_ns,published_ns,clock_ns,overwrote_unread,raw_copy_ms,repair_max_ms";
    for(int i=1;i<=4;++i) events<<",invalid"<<i<<",filled"<<i<<",unresolved"<<i<<",repair"<<i<<"_ms";
    events<<",camera_work_ms,index_merge_ms,direction_sources,unobserved_cells,shared_projection_local_max_ms,local_merge_ms,local_excluded_blocks,local_excluded_cells,local_available_cells,local_unknown_cells";
    events<<'\n'<<std::setprecision(12);
    for(const auto &e:frames_) {
      events<<e.seq<<','<<e.stamp<<','<<e.received<<','<<e.published<<','<<e.clock<<','<<e.overwritten<<','<<e.raw_copy_ns/1e6<<','<<e.preprocess_ns/1e6;
      for(int i=0;i<4;++i) events<<','<<e.repair[i].invalid<<','<<e.repair[i].filled<<','<<e.repair[i].unresolved<<','<<e.repair_ns[i]/1e6;
      events<<','<<e.direction_ns/1e6<<','<<e.region_ns/1e6<<','<<e.direction_sources<<','<<e.unobserved_cells
        <<','<<e.shared_projection_local_max_ns/1e6<<','<<e.local_merge_ns/1e6<<','<<e.local_excluded_blocks<<','<<e.local_excluded_cells<<','<<e.local_available_cells<<','<<e.local_unknown_cells<<'\n';
    }
    std::ofstream recv(o_.output/"received.csv");recv<<"camera,image_stamp_ns,wall_monotonic_ns\n";
    for(const auto &e:received_) recv<<e.camera<<','<<e.stamp<<','<<e.wall<<'\n';
    if(last_frame_) {
      save_depth(o_.output/"first_depth.bin",first_frame_.D);save_depth(o_.output/"last_depth.bin",last_frame_->D);
      save_depth(o_.output/"first_filled_depth.bin",first_frame_.D_filled);save_depth(o_.output/"last_filled_depth.bin",last_frame_->D_filled);
      save_observation_index(o_.output,"first",first_frame_);save_observation_index(o_.output,"last",*last_frame_);
      save_local_field(o_.output,"first",first_frame_,observation_builder_.local_geometry());
      save_local_field(o_.output,"last",*last_frame_,observation_builder_.local_geometry());
    }
    save_rays();
    std::ofstream meta(o_.output/"reader_meta.json");
    auto array=[&meta](const auto &values){meta<<'[';bool first=true;for(auto v:values){if(!first)meta<<',';meta<<v;first=false;}meta<<']';};
    auto body=[&](const BodyState &s){meta<<"{\"t_ns\":"<<s.t_ns<<",\"p_W\":";array(s.p_W);meta<<",\"v_B\":";array(s.v_B);meta<<",\"R_WB\":";array(s.R_WB);meta<<'}';};
    meta<<std::setprecision(17)<<"{\n\"version\":\"ego4p0_cpp_depth_v2\",\"run_id\":"<<std::quoted(o_.run_id)
      <<",\"base_config\":"<<BaseConfig::json<<",\"base_config_sha256\":"<<std::quoted(BaseConfig::sha256)
      <<",\"requested_period_ns\":"<<BaseConfig::period_ns<<",\"requested_read_hz\":"<<1./BaseConfig::control_period_s<<""
      <<",\"ok\":"<<(completed_&&!fatal_&&loop_error_.empty()?"true":"false")
      <<",\"error\":"<<std::quoted(error_message_.empty()?loop_error_:error_message_)
      <<",\"read_mode\":"<<std::quoted(o_.mode)<<",\"duration_clock\":"<<std::quoted(o_.duration_clock)
      <<",\"requested_duration_s\":"<<o_.duration<<",\"width\":"<<width_<<",\"height\":"<<height_
      <<",\"reads\":"<<used_<<",\"waiting_ticks\":"<<waiting_ticks_
      <<",\"start_clock_ns\":"<<start_clock_<<",\"end_clock_ns\":"<<end_clock_
      <<",\"measured_sim_duration_s\":"<<(end_clock_-start_clock_)/1e9
      <<",\"measured_wall_duration_s\":"<<(end_wall_-start_wall_)/1e9
      <<",\"complete_sets\":"<<frames_.size()<<",\"partial_discarded\":"<<sync_.partial_discarded
      <<",\"late_or_duplicate_images\":"<<sync_.late_or_duplicate<<",\"pending_partial_sets\":"<<sync_.pending()
      <<",\"overwritten_unread_sets\":"<<overwritten_<<",\"received_images\":";array(received_counts_);
    meta<<",\"intrinsic_validations\":";array(info_validations_);
    meta<<",\"reader_cpu_requested\":"<<o_.cpu<<",\"affinity_result\":"<<affinity_result_
      <<",\"fifo_requested\":"<<o_.fifo<<",\"fifo_result\":"<<fifo_result_
      <<",\"actual_scheduler_policy\":"<<actual_policy_<<",\"actual_priority\":"<<actual_priority_
      <<",\"scheduler_gate_requested\":"<<(o_.scheduler_gate?"true":"false")
      <<",\"timer_slack_requested_ns\":"<<o_.timer_slack_ns<<",\"timer_slack_before_ns\":"<<timer_slack_before_
      <<",\"timer_slack_actual_ns\":"<<timer_slack_after_<<",\"timer_slack_error\":"<<timer_slack_error_
      <<",\"lock_memory_requested\":"<<(o_.lock_memory?"true":"false")<<",\"memory_lock_error\":"<<memory_lock_error_
      <<",\"reader_run_wall_ns\":"<<(run_wall_end_-run_wall_start_)<<",\"reader_thread_cpu_ns\":"<<(run_cpu_end_-run_cpu_start_)
      <<",\"reader_voluntary_switches\":"<<(usage_after_.ru_nvcsw-usage_before_.ru_nvcsw)
      <<",\"reader_involuntary_switches\":"<<(usage_after_.ru_nivcsw-usage_before_.ru_nivcsw)
      <<",\"reader_minor_faults\":"<<(usage_after_.ru_minflt-usage_before_.ru_minflt)
      <<",\"reader_major_faults\":"<<(usage_after_.ru_majflt-usage_before_.ru_majflt)
      <<",\"spin_us\":"<<o_.spin_us<<",\"first_image_stamp_ns\":"<<first_frame_.t_ns
      <<",\"last_image_stamp_ns\":"<<(last_frame_?last_frame_->t_ns:0)<<",\"first_state\":";body(first_state_);
    meta<<",\"last_state\":";body(last_state_);
    meta<<",\"depth_preprocessing\":{\"enabled\":true,\"location\":\"camera_workers_before_scan\",\"repair_timing_scope\":\"max_per_camera_elapsed_overlapping_camera_work\",\"fused_block_region_scan\":true,\"neighborhood\":\"original_3x3_in_bounds_finite_positive\",\"unresolved_preserved\":true"
      <<",\"ray_table_builds\":"<<ray_tables_.builds()<<",\"ray_table_build_ns\":"<<ray_build_ns_
      <<",\"ray_table_bytes\":"<<ray_tables_.bytes()<<",\"ray_dtype\":\"float64_le\",\"ray_shape\":[4,"<<height_<<','<<width_<<",3],\"ray_frame\":\"camera_optical\",\"point_cloud_generated\":false,\"environment_map_generated\":false}";
    meta<<",\"direction_depth_queries\":{\"enabled\":true,\"azimuth_resolution_deg\":"<<o_.azimuth_resolution_deg
        <<",\"elevation_resolution_deg\":"<<o_.elevation_resolution_deg<<",\"azimuth_bins\":"<<observation_builder_.layout().azimuth_bins
        <<",\"elevation_bins\":"<<observation_builder_.layout().elevation_bins
        <<",\"query_tile_edge\":16,\"block_edge\":3,\"block_capacity_per_camera\":"<<first_frame_.observations.blocks_per_camera
        <<",\"block_mean_input\":\"full_resolution_filled_finite_positive\",\"coverage_scope\":\"block_mean_center_directions\""
        <<",\"rotated_ray_table_builds\":"<<observation_builder_.table_builds()<<",\"index_builds\":"<<observation_builder_.builds()
        <<",\"reused_groups\":"<<observation_builder_.reused_groups()<<",\"boundary_lookup\":true,\"worker_cpus\":";
    array(observation_builder_.worker_cpus());meta<<",\"worker_affinity_errors\":";array(observation_builder_.worker_affinity_errors());
    meta<<",\"worker_scheduler_errors\":";array(observation_builder_.worker_scheduler_errors());
    meta<<",\"worker_policies\":";array(observation_builder_.worker_policies());meta<<",\"worker_priorities\":";array(observation_builder_.worker_priorities());meta<<",\"worker_scheduler\":\""<<(o_.fifo==5?"SCHED_FIFO":"SCHED_OTHER")<<"\",\"timing_scope\":\"camera_work_wall_then_index_merge; per_camera_kernels_overlap\",\"cross_camera_depth_minimum\":false,\"per_pixel_validity_mask\":false,\"free_domain_decision\":false}";
    meta<<",\"local_field_preparation\":{\"enabled\":true,\"radius_m\":"<<ControllerLimits::radius_m<<",\"local_range_m\":"<<ControllerLimits::local_range_m<<",\"raw_required_for_clearance\":true,\"initial_clearance_basis_supplied\":false,\"current_position_checked\":false,\"straight_distance_image_generated\":false,\"geometry_table_builds\":"<<observation_builder_.local_geometry().builds()
      <<",\"planning_model\":\"body_origin_virtual_omnidirectional\",\"candidate_domain_constructed\":true,\"approximation_authorized\":true,\"translation_ignored\":true,\"physical_near_clip_checked\":false,\"direction_lookup_precomputed\":true,\"distance_bin_m\":0.05,\"shared_projection_includes_local_refinement\":true,\"free_endpoint_index_generated\":false,\"source_boundary_generated\":false,\"solver_domain_constructed\":true,\"block_exclusion_enabled\":true,\"hot_near_pixel_transforms\":0,\"local_merge_follows_index_merge\":true}"
      <<",\"desired_motion\":{\"source\":"<<std::quoted(o_.desired_source)<<",\"goal_speed_mps\":"<<o_.goal_speed_mps<<",\"max_speed_mps\":"<<BaseConfig::max_speed_mps<<",\"period_s\":"<<BaseConfig::control_period_s<<",\"max_reference_acceleration_mps2\":"<<BaseConfig::reference_command_acceleration_mps2<<",\"reference_integration_implemented\":true}"
      <<",\"harmonic_planner\":{\"enabled\":true,\"coarse_degrees\":3,\"fine_degrees\":1,\"radius_m\":"<<ControllerLimits::radius_m<<",\"local_range_m\":"<<ControllerLimits::local_range_m<<",\"separate_worker\":true,\"worker_policy\":"<<field_worker_.policy()<<",\"worker_priority\":"<<field_worker_.priority()<<",\"positive_gradient\":true,\"adaptive_coarse\":true,\"coarse_interpolation\":\"constant_FV_prolongation\",\"normalized_arc_length_integration\":true,\"residual_tolerance\":"<<o_.field.residual_tolerance<<",\"gradient_epsilon\":"<<o_.field.gradient_epsilon<<",\"max_iterations\":"<<o_.field.max_iterations<<",\"boundary_cap_deg\":"<<o_.field.boundary_cap_deg<<",\"corridor_layers\":"<<o_.field.corridor_deg<<",\"fine_boundary_mode\":\""<<(o_.field.fine_neumann_roi?"local_corridor_neumann":"coarse_dirichlet_coupling")<<"\",\"max_step_deg\":"<<o_.field.max_step_deg<<",\"min_step_deg\":"<<o_.field.min_step_deg<<",\"max_curve_steps\":"<<o_.field.max_curve_steps<<",\"parameter_revision\":"<<o_.field.revision<<",\"q0_source\":\"reference_tangent_else_body_velocity_else_desired_when_stationary\",\"final_execution_check_implemented\":false}"
      <<",\"depth_type\":\"optical_axis_z_m\",\"float_format\":\"float32_le\",\"frame_buffers\":3,\"max_partial_sets\":4,\"image_history_retained\":false,\"control_loop_allocates_image_buffers\":false,\"motion_publishers_created\":0}\n";
    meta.close();std::ofstream(o_.output/"reader_done")<<o_.run_id<<'\n';
  }
  bool success() const { return completed_&&!fatal_&&loop_error_.empty(); }
 private:
  void receive(int camera,Image::ConstSharedPtr m) {
    const int64_t now=mono_ns();++received_counts_[camera];
    if(received_.size()==received_.capacity()) throw std::runtime_error("receive metadata capacity exceeded");
    received_.push_back({stamp_ns(*m),now,camera});
    if(m->encoding!="32FC1" || m->width!=width_ || m->height!=height_ || m->step<width_*4 || m->data.size()!=height_*m->step || m->header.frame_id!=cal_[camera].frame)
      throw std::runtime_error("depth image encoding/dimensions/stride/frame mismatch");
    if(!std::all_of(info_ready_.begin(),info_ready_.end(),[](bool b){return b;})) return;
    FourWaySynchronizer<Image>::Group group;
    if(!sync_.push(camera,stamp_ns(*m),m,group)) return;
    auto &f=images_.writer();f.t_ns=stamp_ns(*m);f.received_ns=now;f.sequence=++sequence_;
    const int64_t copy_start=mono_ns();
    for(int i=0;i<4;++i) {
      const auto &image=*group[i];auto *dst=f.D[i].data();
      if(!image.is_bigendian && image.step==width_*4) std::memcpy(dst,image.data.data(),pixels_*4);
      else for(size_t y=0;y<height_;++y) for(size_t x=0;x<width_;++x) {
        uint32_t bits;std::memcpy(&bits,image.data.data()+y*image.step+x*4,4);
        if(image.is_bigendian) bits=__builtin_bswap32(bits);
        std::memcpy(dst+y*width_+x,&bits,4);
      }
    }
    const int64_t copy_ns=mono_ns()-copy_start;
    observation_builder_.process(f,ray_tables_);
    if(sequence_==1) {
      for(int i=0;i<4;++i) {
        std::memcpy(first_frame_.D[i].data(),f.D[i].data(),pixels_*4);
        std::memcpy(first_frame_.D_filled[i].data(),f.D_filled[i].data(),pixels_*4);
      }
      first_frame_.t_ns=f.t_ns;
      first_frame_.sequence=f.sequence;first_frame_.observations=f.observations;first_frame_.local=f.local;
    }
    field_worker_.submit_depth(f);
    f.published_ns=mono_ns();const auto published=f.published_ns;
    const bool replaced=images_.publish();overwritten_+=replaced;
    if(frames_.size()==frames_.capacity()) throw std::runtime_error("frame metadata capacity exceeded");
    frames_.push_back({stamp_ns(*m),now,published,clock_.load(),sequence_,replaced,copy_ns,f.preprocessing_ns,f.repair,f.repair_ns,
                      f.observations.camera_work_ns,f.observations.merge_ns,f.observations.source_count,
                      f.observations.layout.cells()-f.observations.observed_cells,*std::max_element(f.local.shared_projection_ns.begin(),f.local.shared_projection_ns.end()),
                      f.local.merge_ns,f.local.excluded_blocks,f.local.excluded_cells,f.local.available_cells,f.local.unknown_cells});
  }
  void save_depth(const fs::path &p,const std::array<std::vector<float>,4> &depth) {
    std::ofstream f(p,std::ios::binary);for(const auto &d:depth) f.write(reinterpret_cast<const char*>(d.data()),d.size()*4);
    if(!f) throw std::runtime_error("cannot write depth snapshot");
  }
  void save_rays() {
    std::ofstream b(o_.output/"pixel_rays_b.bin",std::ios::binary),d(o_.output/"pixel_rays_d.bin",std::ios::binary);
    for(size_t i=0;i<4;++i) for(const auto &ray:ray_tables_.camera(i)) {
      b.write(reinterpret_cast<const char*>(ray.b.data()),3*sizeof(double));
      d.write(reinterpret_cast<const char*>(ray.d.data()),3*sizeof(double));
    }
    if(!b || !d) throw std::runtime_error("cannot write pixel ray tables");
  }
  Options o_;size_t width_=0,height_=0,pixels_=0,used_=0;
  std::array<CameraCalibration,4> cal_;
  PixelRayTables ray_tables_;int64_t ray_build_ns_=0;
  FrameObservationBuilder observation_builder_;
  HarmonicWorker field_worker_;TripleBuffer<Direction3> tangent_;
  std::array<bool,4> info_ready_{};
  std::array<std::array<double,9>,4> last_k_{};
  std::array<std::vector<double>,4> last_d_;
  std::array<uint64_t,4> received_counts_{},info_validations_{};
  TripleBuffer<DepthFrame> images_;TripleBuffer<BodyState> states_;
  TripleBuffer<DesiredInput> desired_;
  FourWaySynchronizer<Image> sync_;
  DepthFrame first_frame_,copy_frame_;const DepthFrame *last_frame_=nullptr;
  BodyState first_state_,last_state_;
  std::vector<Sample> samples_;std::vector<ImageEvent> received_;std::vector<FrameEvent> frames_;
  std::atomic<int64_t> clock_{-1};std::atomic<bool> fatal_{false};
  std::string error_message_,loop_error_;uint64_t sequence_=0,overwritten_=0,waiting_ticks_=0;
  int64_t start_clock_=0,end_clock_=0,start_wall_=0,end_wall_=0;
  int affinity_result_=0,fifo_result_=0,actual_policy_=0,actual_priority_=0;bool completed_=false;
  int timer_slack_error_=0,memory_lock_error_=0;long timer_slack_before_=0,timer_slack_after_=0;
  int64_t run_wall_start_=0,run_wall_end_=0,run_cpu_start_=0,run_cpu_end_=0;
  rusage usage_before_{},usage_after_{};
  std::array<rclcpp::Subscription<Image>::SharedPtr,4> image_subs_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr,4> info_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr intent_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr tangent_sub_;
};

int main(int argc,char **argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--print-config") {std::cout<<BaseConfig::json<<'\n';return 0;}
    Options o;
    for(int i=1;i<argc;++i) {
      std::string key=argv[i];if(i+1>=argc) throw std::runtime_error("missing argument value");std::string value=argv[++i];
      if(key=="--output") o.output=value;else if(key=="--run-id") o.run_id=value;
      else if(key=="--duration") o.duration=std::stod(value);else if(key=="--wall-timeout") o.timeout=std::stod(value);
      else if(key=="--read-mode") o.mode=value;else if(key=="--duration-clock") o.duration_clock=value;
      else if(key=="--reader-cpu") o.cpu=std::stoi(value);else if(key=="--fifo-priority") o.fifo=std::stoi(value);
      else if(key=="--spin-us") o.spin_us=std::stoi(value);
      else if(key=="--field-tolerance") o.field.residual_tolerance=std::stod(value);
      else if(key=="--field-max-iterations") {int n=std::stoi(value);if(n<=0||n>100000)throw std::runtime_error("invalid field iteration limit");o.field.max_iterations=uint32_t(n);}
      else if(key=="--field-corridor-deg") o.field.corridor_deg=std::stod(value);
      else if(key=="--field-cap-deg") o.field.boundary_cap_deg=std::stod(value);
      else if(key=="--field-step-deg") o.field.max_step_deg=std::stod(value);
      else if(key=="--field-fine-boundary") {
        if(value=="local_neumann")o.field.fine_neumann_roi=true;
        else if(value=="coarse_coupling")o.field.fine_neumann_roi=false;
        else throw std::runtime_error("invalid fine boundary mode");
      }
      else if(key=="--desired-source") o.desired_source=value;
      else if(key=="--goal-speed-mps") o.goal_speed_mps=std::stod(value);
      else if(key=="--azimuth-resolution-deg") o.azimuth_resolution_deg=std::stod(value);
      else if(key=="--elevation-resolution-deg") o.elevation_resolution_deg=std::stod(value);
      else if(key=="--timer-slack-ns") o.timer_slack_ns=std::stoll(value);
      else if(key=="--lock-memory") o.lock_memory=std::stoi(value)!=0;
      else if(key=="--scheduler-gate") o.scheduler_gate=std::stoi(value)!=0;else throw std::runtime_error("unknown argument: "+key);
    }
    if(o.output.empty()||o.run_id.empty()||!std::isfinite(o.duration)||o.duration<=0||!std::isfinite(o.timeout)||o.timeout<o.duration||o.timeout>900
       ||(o.mode!="reference"&&o.mode!="copy")||(o.duration_clock!="sim"&&o.duration_clock!="wall")||o.cpu>=CPU_SETSIZE||o.cpu< -1||o.spin_us<0||o.spin_us>1000||(o.fifo!=0&&o.fifo!=5)||o.timer_slack_ns<0||o.timer_slack_ns>1000000
       ||(o.desired_source!="velocity" && o.desired_source!="goal" && o.desired_source!="none") || !std::isfinite(o.goal_speed_mps) || o.goal_speed_mps<0 || o.goal_speed_mps>BaseConfig::max_speed_mps)
      throw std::runtime_error("invalid reader options");
    rclcpp::init(0,nullptr);
    auto node=std::make_shared<DepthReader>(o);
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    std::thread rx([&]{try{pthread_setname_np(pthread_self(),"ego4p0_receive");executor.spin();}catch(const std::exception &e){node->fail(e.what());}});
    try {node->run();} catch(const std::exception &e) {node->fail(e.what());} executor.cancel();rx.join();node->write_results();const bool ok=node->success();
    rclcpp::shutdown();return ok?0:2;
  }catch(const std::exception &e){std::cerr<<"depth_reader: "<<e.what()<<'\n';return 3;}
}
