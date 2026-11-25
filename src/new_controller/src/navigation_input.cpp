#include "new_controller/navigation_io.hpp"
#include "new_controller/rolling_navigation.hpp"
#include "input_epoch.hpp"
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <atomic>
#include <cerrno>
#include <time.h>
#include <cstring>
#include <iostream>
#include <map>
#include <thread>
using namespace ego4p0;namespace fs=std::filesystem;
template<class M>static int64_t message_stamp(const M&m){return int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;}
struct DesiredPacket {Direction3 velocity{};std::string frame;int64_t stamp=0;bool valid=false;uint64_t epoch=0;};
struct StampedState {BodyState value;uint64_t epoch=0;};
class NavigationInputNode:public rclcpp::Node {
 public:
  using Image=sensor_msgs::msg::Image;
  NavigationInputNode(NavigationPipeline&p,const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h,uint64_t calibration_version,const std::string &session,uint64_t first_generation):Node("ego4p2_navigation_input"),cal_(cal),w_(w),h_(h),calibration_version_(calibration_version),session_(session),source_(first_generation){
    for(auto &o:groups_.slots)p.allocate(o);
    if(!session.empty() && (session.size()!=32 || session.find_first_not_of("0123456789abcdef")!=std::string::npos))
      throw std::invalid_argument("source session must be a fresh 32-character lowercase hex UUID");
    if(first_generation>UINT32_MAX || (first_generation && session.empty()))throw std::invalid_argument("invalid source generation");
    bind_inputs(source_);
    clock_sub_=subscribe_clock(source_);
    if(!session_.empty())next_clock_=subscribe_clock(source_+1);
  }
  InputEpoch timebase;
 private:
  std::string prefix(uint64_t source) const {
    return session_.empty()?"":"/ego4p3/s_"+session_+"/e"+std::to_string(source);
  }
  void bind_inputs(uint64_t source){
    auto qos=rclcpp::SensorDataQoS().keep_last(8);std::array<std::string,4>roots{"/sim/depth","/sim/depth_left","/sim/depth_back","/sim/depth_right"};
    for(uint32_t c=0;c<4;++c){
      images_[c]=create_subscription<Image>(prefix(source)+roots[c]+"/image_raw",qos,[this,c,source](Image::ConstSharedPtr m){safe([&]{
      if(source!=source_ || !timebase.ticket().accepting)return;
        if(m->encoding!="32FC1"||m->width!=w_||m->height!=h_||m->step!=w_*4||m->data.size()!=size_t(w_)*h_*4||m->is_bigendian||m->header.frame_id!=cal_[c].frame)throw std::runtime_error("depth payload/frame mismatch");
        FourWaySynchronizer<Image>::Group complete;const int64_t t=message_stamp(*m);if(!epoch_stamp_valid(t)||t<=image_stamp_[c])return;
        image_stamp_[c]=t;
        if(sync_.push(c,t,std::move(m),complete)){pending_[t]=std::move(complete);while(pending_.size()>4){pending_.erase(pending_.begin());++discarded_;}drain();}
      });});
      infos_[c]=create_subscription<sensor_msgs::msg::CameraInfo>(prefix(source)+roots[c]+"/camera_info",qos,[this,c,source](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){safe([&]{
      if(source!=source_ || !timebase.ticket().accepting)return;
        if(m->width!=w_||m->height!=h_||m->header.frame_id!=cal_[c].frame||m->distortion_model!="plumb_bob"||m->d.size()!=5)throw std::runtime_error("calibration dimensions/frame/model changed");
        for(int j=0;j<9;++j)if(!std::isfinite(m->k[j])||std::abs(m->k[j]-cal_[c].K[j])>1e-8*(1+std::abs(cal_[c].K[j])))throw std::runtime_error("calibration changed: reload metadata with a new calibration version");
        for(double d:m->d)if(!std::isfinite(d)||std::abs(d)>1e-12)throw std::runtime_error("unsupported distorted calibration");
        if(!session_.empty() && !epoch_stamp_valid(message_stamp(*m)))return;
        info_ready_[c]=true;drain();
      });});
    }
    odom_=create_subscription<nav_msgs::msg::Odometry>(prefix(source)+"/sim/odom",qos,[this,source](nav_msgs::msg::Odometry::ConstSharedPtr m){safe([&]{
      if(source!=source_ || !timebase.ticket().accepting)return;
      if(m->header.frame_id!="world"||m->child_frame_id!="base_link")throw std::runtime_error("odometry coordinate contract mismatch");
      if(!epoch_stamp_valid(message_stamp(*m)) || message_stamp(*m)<=last_state_)return;
      last_state_=message_stamp(*m);
      const auto&q=m->pose.pose.orientation;const auto&v=m->twist.twist.linear;auto s=body_state(q.x,q.y,q.z,q.w,{v.x,v.y,v.z},message_stamp(*m));const auto&a=m->pose.pose.position;s.p_W={a.x,a.y,a.z};if(!NavigationPipeline::valid_state(s))throw std::runtime_error("invalid odometry state");
      states_.writer()={s,timebase.ticket().epoch};states_.publish();poses_[s.t_ns]=s;while(poses_.size()>32)poses_.erase(poses_.begin());drain();
    });});
    intent_=create_subscription<geometry_msgs::msg::TwistStamped>(prefix(source)+"/human_intent",qos,[this,source](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){safe([&]{
      if(source!=source_ || !timebase.ticket().accepting)return;
      if(m->header.frame_id!="world"&&m->header.frame_id!="base_link")throw std::runtime_error("desired velocity must name world or base_link");
      if(!epoch_stamp_valid(message_stamp(*m)) || message_stamp(*m)<=last_intent_)return;
      last_intent_=message_stamp(*m);
      auto&d=desired_.writer();d.velocity={m->twist.linear.x,m->twist.linear.y,m->twist.linear.z};for(double x:d.velocity)if(!std::isfinite(x))throw std::runtime_error("nonfinite desired velocity");d.frame=m->header.frame_id;d.stamp=message_stamp(*m);d.epoch=timebase.ticket().epoch;d.valid=true;desired_.publish();
    });});
  }
  using Clock=rosgraph_msgs::msg::Clock;
  rclcpp::Subscription<Clock>::SharedPtr subscribe_clock(uint64_t source){
    return create_subscription<Clock>(prefix(source)+"/clock",rclcpp::SensorDataQoS().keep_last(8),
      [this,source](Clock::ConstSharedPtr msg){safe([&]{
        const int64_t time=int64_t(msg->clock.sec)*1000000000LL+msg->clock.nanosec;
        if(time<0 || source<source_)return;
        const bool next=source>source_;
        {
          auto guard=timebase.lock();
          const auto prior=timebase.ticket();
          if(!next && (!prior.accepting || time>=prior.clock_ns)){
            if(prior.accepting)timebase.advance(time);
            return;
          }
          // A rewind on the same topic cannot distinguish queued old samples.
          // Only a clock on the next source-owned namespace can re-open it.
          timebase.transition(time,next);
          discarded_+=sync_.partial_discarded+sync_.pending()+pending_.size();
          sync_={};pending_.clear();poses_.clear();info_ready_.fill(false);image_stamp_.fill(-1);
          last_state_=last_intent_=last_group_stamp_=-1;
        }
        if(next){
          source_=source;
          bind_inputs(source);
          clock_sub_=std::move(next_clock_);
          next_clock_=subscribe_clock(source+1);
        }
      });});
  }
 public:
  NavigationObservation*latest(uint64_t epoch){auto *o=groups_.latest_owned();return o&&o->receive_epoch==epoch?o:nullptr;}
  const BodyState*state(uint64_t epoch){auto *s=states_.latest();return s&&s->epoch==epoch?&s->value:nullptr;}
  const DesiredPacket*desired(uint64_t epoch){auto *d=desired_.latest();return d&&d->epoch==epoch?d:nullptr;}
  bool failed()const{return failed_.load(std::memory_order_acquire);}
  const std::string&error()const{return error_;}uint64_t complete()const{return sequence_;}uint64_t discarded()const{return discarded_+sync_.partial_discarded;}
 private:
  bool epoch_stamp_valid(int64_t stamp)const{
    const auto current=timebase.ticket();
    if(session_.empty() && current.epoch==0)return stamp>=0;
    // Scoped packets may precede their matching clock callback. They remain
    // ineligible for planning until validation against the captured clock.
    return current.accepting && stamp>=0 && current.clock_ns>=0 &&
      stamp>=current.clock_ns-RollingNavigator::max_age_ns &&
      stamp<=current.clock_ns+RollingNavigator::max_age_ns;
  }
  template<class F>void safe(F f){if(failed())return;try{f();}catch(const std::exception&e){error_=e.what();failed_.store(true,std::memory_order_release);}}
  void drain(){
    if(!std::all_of(info_ready_.begin(),info_ready_.end(),[](bool x){return x;}))return;
    for(auto it=pending_.begin();it!=pending_.end();){if(it->first<=last_group_stamp_){it=pending_.erase(it);++discarded_;continue;}auto pose=poses_.find(it->first);if(pose==poses_.end()){++it;continue;}auto&o=groups_.writer();
      for(uint32_t c=0;c<4;++c)std::memcpy(o.depth.D[c].data(),it->second[c]->data.data(),size_t(w_)*h_*sizeof(float));
      o.depth.sequence=++sequence_;o.depth.t_ns=it->first;o.depth.received_ns=field_now_ns();o.identity={sequence_,calibration_version_,sequence_,it->first,{it->first,it->first,it->first,it->first},15};o.observation_pose=pose->second;o.observation_pose_valid=true;o.receive_epoch=timebase.ticket().epoch;last_group_stamp_=it->first;groups_.publish();it=pending_.erase(it);
    }
  }
  std::array<CameraCalibration,4>cal_;uint32_t w_,h_;uint64_t calibration_version_;FourWaySynchronizer<Image>sync_;std::map<int64_t,FourWaySynchronizer<Image>::Group>pending_;std::map<int64_t,BodyState>poses_;
  TripleBuffer<NavigationObservation>groups_;TripleBuffer<StampedState>states_;TripleBuffer<DesiredPacket>desired_;std::array<bool,4>info_ready_{};
  std::array<int64_t,4>image_stamp_{{-1,-1,-1,-1}};
  std::array<rclcpp::Subscription<Image>::SharedPtr,4>images_;std::array<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr,4>infos_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr intent_;rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_,next_clock_;
  std::string session_;uint64_t source_=0;int64_t last_state_=-1,last_intent_=-1,last_group_stamp_=-1;
  std::atomic<bool>failed_{false};std::string error_;uint64_t sequence_=0,discarded_=0;
};
int main(int argc,char**argv){try{
  fs::path calibration,output;std::string session;double duration=.2,timeout=60;bool normal=false,track_fixed=false,track_rolling=false;uint64_t expected_groups=0,first_generation=0;
  for(int i=1;i<argc;++i){const std::string a=argv[i];auto value=[&](){if(i+1==argc)throw std::invalid_argument("missing CLI value");return std::string(argv[++i]);};
    if(a=="--print-config"){std::cout<<NavigationConfig::json<<'\n';return 0;}else if(a=="--source-generation")first_generation=std::stoull(value());else if(a=="--source-session")session=value();else if(a=="--calibration")calibration=value();else if(a=="--output")output=value();else if(a=="--duration-sim")duration=std::stod(value());else if(a=="--wall-timeout")timeout=std::stod(value());else if(a=="--correctness-normal")normal=true;else if(a=="--track-fixed")track_fixed=true;else if(a=="--track-rolling")track_rolling=true;else if(a=="--expect-groups")expected_groups=std::stoull(value());else throw std::invalid_argument("unknown option "+a);
  }
  if((track_fixed&&track_rolling)||calibration.empty()||output.empty()||!std::isfinite(duration)||duration<=0||!std::isfinite(timeout)||timeout<=0||timeout>600||!fs::create_directory(output))throw std::invalid_argument("new output, calibration and finite bounded duration required");
  uint32_t w,h;const auto cal=navigation_calibration(calibration,w,h);const uint64_t calibration_version=navigation_calibration_version(cal,w,h);NavigationPipeline pipeline;pipeline.configure(cal,w,h,calibration_version);NavigationObservation missing;pipeline.allocate(missing);
  rclcpp::init(0,nullptr);auto node=std::make_shared<NavigationInputNode>(pipeline,cal,w,h,calibration_version,session,first_generation);rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread receive([&](){pthread_setname_np(pthread_self(),"ego4p2_receive");try{executor.spin();}catch(...){}});
  try{
    pthread_setname_np(pthread_self(),"ego4p2_coord");cpu_set_t cpu;CPU_ZERO(&cpu);CPU_SET(0,&cpu);if(pthread_setaffinity_np(pthread_self(),sizeof(cpu),&cpu))throw std::runtime_error("coordinator CPU0 affinity");if(!normal)pipeline.enable_fifo5();
    int policy=0;sched_param priority{};pthread_getschedparam(pthread_self(),&policy,&priority);if(policy!=(normal?SCHED_OTHER:SCHED_FIFO)||priority.sched_priority!=(normal?0:5))throw std::runtime_error("coordinator scheduler mismatch");pipeline.builder().refresh_worker_schedulers();
    std::ofstream metadata(output/"preprocess_execution.json");metadata<<"{\"coordinator_scheduler\":"<<policy<<",\"coordinator_priority\":"<<priority.sched_priority<<",\"coordinator_cpu\":0,\"worker_policies\":[";for(int i=0;i<4;++i){if(i)metadata<<',';metadata<<pipeline.builder().worker_policies()[i];}metadata<<"],\"worker_priorities\":[";for(int i=0;i<4;++i){if(i)metadata<<',';metadata<<pipeline.builder().worker_priorities()[i];}metadata<<"],\"worker_cpus\":[";for(int i=0;i<4;++i){if(i)metadata<<',';metadata<<pipeline.builder().worker_cpus()[i];}metadata<<"],\"algorithm_threads\":5,\"ROS_receive_policy\":0,\"period_ns\":5000000,\"motion_publishers\":0,\"secondary_summary_in_camera_threads\":true,\"odom_velocity_source\":\"Isaac_publishRawVelocities_world\",\"production_input_chain\":true,\"reference_kind\":\"spatial_reference\",\"spatial_reference_enabled\":true,\"continuous_tube_check_enabled\":true}\n";metadata.close();
    std::ofstream(output/"configuration.json")<<NavigationConfig::json<<'\n';
    std::ofstream events(output/"navigation_events.csv");events<<"scheduled_ns,start_ns,finish_ns,sim_ns,group,reference_version,invalid,angular_ready,spatial_ready,spatial_ms,occupancy_ms,compute_ms,skipped_releases,epoch"<<','<<navigation_detail_header()<<'\n';
    std::optional<RollingNavigator> rolling;
    RollingActuatorState rolling_actuator;
    std::ofstream rolling_events;
    uint64_t rolling_ready=0,rolling_protected=0,rolling_resets=0,rolling_plans=0,rolling_commits=0;
    if(track_rolling){
      rolling.emplace(pipeline);
      rolling_events.open(output/"rolling_events.csv");
      if(!rolling_events)throw std::runtime_error("rolling events output");
      rolling_events<<std::setprecision(17)<<"state_ns,mode,reason,version,global_s,certified_end,new_observation,planned,continued,committed,cover_balls,planning_ns,certification_ns,control_ns,cycle_ns,plant_ux,plant_uy,plant_uz,epoch\n";
    }
    std::optional<FixedReferenceTracker> tracker;
    std::ofstream tracking;
    if(track_fixed){
      tracker.emplace();
      tracking.open(output/"fixed_tracking_events.csv");
      if(!tracking)throw std::runtime_error("tracking events output");
      tracking<<std::setprecision(17)<<"state_ns,fixed_geometry_version,dt_s,status,outer_updated,tracking_ns,s_m,error_bound_m,fresh_clear_prefix,desired_vx,desired_vy,desired_vz,plant_ux,plant_uy,plant_uz,epoch\n";
    }
    int64_t tracking_stamp=-1,fixed_observation_stamp=-1;
    uint64_t fixed_geometry=0,tracking_ready=0;
    double clear_prefix=0.;
    const auto started=field_now_ns();auto release=started+BaseConfig::period_ns;int64_t initial=-1;uint64_t valid=0,spatial_ready=0;bool ended=false;
    std::ofstream lifecycle(output/"epoch_events.csv");
    lifecycle<<"event,captured_epoch,commit_epoch,captured_ns,commit_ns,group,accepting\n";
    uint64_t active_epoch=0,epoch_resets=0,discarded_results=0;
    bool have_shadow_input=false,exportable=false;
    InputEpoch::Ticket last_commit{};
    int64_t last_oldest=-1;
    while(rclcpp::ok()){
      timespec t{release/1000000000LL,release%1000000000LL};int e;do{e=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);}while(e==EINTR);if(e)throw std::runtime_error("release clock failed");const auto begin=field_now_ns();
      if(node->failed())throw std::runtime_error(node->error());
      auto snapshot_lock=node->timebase.lock();
      const auto ticket=node->timebase.ticket();
      auto*o=node->latest(ticket.epoch);const auto*s=node->state(ticket.epoch);const auto*d=node->desired(ticket.epoch);
      snapshot_lock.unlock();
      if(active_epoch!=ticket.epoch){
        if(rolling){rolling->reset_epoch();++rolling_resets;}else pipeline.reset_epoch();
        if(tracker)tracker.emplace();
        fixed_geometry=0;clear_prefix=0.;tracking_stamp=fixed_observation_stamp=-1;
        rolling_actuator={};have_shadow_input=false;initial=-1;
        lifecycle<<"reset,"<<active_epoch<<','<<ticket.epoch<<",-1,"<<ticket.clock_ns<<",0,"<<ticket.accepting<<'\n';
        active_epoch=ticket.epoch;++epoch_resets;
      }
      NavigationState state;state.current_simulation_ns=ticket.clock_ns;
      if(s){state.actual=*s;state.actual_valid=true;}
      DesiredMotion desired;if(d&&d->valid&&s&&o&&d->stamp<=ticket.clock_ns&&ticket.clock_ns-d->stamp<=RollingNavigator::max_age_ns){auto world=d->frame=="world"?d->velocity:nav_rotate(s->R_WB,d->velocity);const double speed=nav_norm(world);auto direction=nav_unrotate(o->observation_pose.R_WB,world);desired={direction,std::min(speed,BaseConfig::max_speed_mps),true,unit_direction(direction)};desired.d_desired=direction;}
      RollingTick rolling_tick;
      if(rolling){
        if(!have_shadow_input&&s)rolling_actuator={nav_rotate(s->R_WB,s->v_B),{}};
        rolling_tick=rolling->tick(o,state,desired,rolling_actuator);
        if(rolling_tick.mode==RollingMode::Tracking){
          rolling_actuator={rolling_tick.command.plant_input_W,rolling_tick.command.omega_W};
          have_shadow_input=true;
        }else if(rolling_tick.mode!=RollingMode::RepeatedState){
          // The ROS path is a diagnostic shadow. Its initial actuator state
          // is seeded from measured velocity when there is no active session.
          rolling_actuator={s?nav_rotate(s->R_WB,s->v_B):Direction3{},{}};
          have_shadow_input=false;
        }

      }
      const auto &out=rolling?pipeline.output():pipeline.process(o?*o:missing,state,desired,false,true);

      TrackingCommand command;
      bool track_record=false,fresh_clear=false;
      double physics_dt=0.;
      int64_t tracking_ns=0;
      constexpr auto max_age_ns=int64_t(NavigationConfig::max_observation_age_sim_s*1e9);
      if(tracker && fixed_geometry && (!desired.valid || !out.input_ready || ticket.clock_ns-fixed_observation_stamp>max_age_ns)){
        tracker.emplace();fixed_geometry=0;clear_prefix=0.;
      }
      if(tracker&&s&&s->t_ns>=0&&state.current_simulation_ns>=s->t_ns){
        const bool fresh=state.current_simulation_ns-s->t_ns<=max_age_ns;
        const auto track_begin=field_now_ns();
        if(!fixed_geometry&&s->t_ns!=tracking_stamp&&fresh&&out.spatial_ready&&desired.valid&&
           std::abs(desired.v_desired-BaseConfig::max_speed_mps)<=FixedReferenceTracker::attachment_speed_tolerance_mps){
          const auto velocity=nav_rotate(s->R_WB,s->v_B);
          command.status=tracker->attach(pipeline.reference(),s->p_W,velocity,0.,out.checked_horizon_m,velocity,{});
          track_record=s->t_ns!=tracking_stamp;
          tracking_stamp=s->t_ns;
          if(command.status==TrackingStatus::Ready){
            fixed_geometry=out.geometry_version;
            fixed_observation_stamp=out.observation.simulation_stamp_ns;
            for(const auto &section:pipeline.tube().sections()){
              if(section.status!=TubeStatus::ObservedClear)break;
              clear_prefix=section.s1;
            }
            track_record=false;
          }
        }else if(tracker->attached()&&(!fresh||s->t_ns!=tracking_stamp)){
          physics_dt=(s->t_ns-tracking_stamp)*1e-9;
          const auto velocity=nav_rotate(s->R_WB,s->v_B);
          // An odometry gap can span several configured physics steps.
          const bool single_step=physics_dt<=BaseConfig::physics_dt_s+1e-12;
          command=tracker->step(s->p_W,velocity,fresh&&single_step?physics_dt:0.);
          tracking_stamp=s->t_ns;
          track_record=true;
          if(command.status==TrackingStatus::Ready){
            fresh_clear=state.current_simulation_ns-fixed_observation_stamp<=max_age_ns&&
              command.s+BaseConfig::max_speed_mps*physics_dt<=clear_prefix;
            if(!fresh_clear){command={};tracker.emplace();fixed_geometry=0;clear_prefix=0.;}
          }
        }
        tracking_ns=field_now_ns()-track_begin;
      }
      const auto finish=field_now_ns();
      const auto skipped=std::max<int64_t>(0,(finish-release)/BaseConfig::period_ns);
      auto commit_lock=node->timebase.lock();
      const bool uses_input=(rolling_tick.mode==RollingMode::Tracking || ((!rolling || rolling_tick.new_observation) && out.input_ready)) ||
                            (track_record && command.status==TrackingStatus::Ready);
      int64_t oldest=uses_input&&s&&o&&d?std::min({s->t_ns,o->depth.t_ns,d->stamp}):-1;
      if(fresh_clear)oldest=std::min(oldest,fixed_observation_stamp);
      if(!node->timebase.can_commit(ticket,oldest,max_age_ns)){
        ++discarded_results;exportable=false;
        const auto live=node->timebase.ticket();
        lifecycle<<"discard,"<<ticket.epoch<<','<<live.epoch<<','<<ticket.clock_ns<<','<<live.clock_ns
                 <<','<<(o?o->identity.group:0)<<','<<live.accepting<<'\n';
        if(active_epoch==node->timebase.ticket().epoch){
          if(rolling)rolling->interlock();
          if(tracker)tracker.emplace();
          fixed_geometry=0;clear_prefix=0.;fixed_observation_stamp=tracking_stamp=-1;
          have_shadow_input=false;
        }
        release+=(skipped+1)*BaseConfig::period_ns;
        if(finish-started>int64_t(timeout*1e9))break;
        continue;
      }
      last_commit=ticket;last_oldest=oldest;
      exportable=!rolling || rolling_tick.new_observation || rolling_tick.mode==RollingMode::Tracking;
      if(!rolling||rolling_tick.new_observation){valid+=out.angular_ready;spatial_ready+=out.spatial_ready;}
      if(rolling){
        rolling_ready+=rolling_tick.mode==RollingMode::Tracking;
        rolling_protected+=rolling_tick.mode==RollingMode::Protection;
        rolling_plans+=rolling_tick.planned;rolling_commits+=rolling_tick.committed;
      }
      if(track_record){
        tracking_ready+=command.status==TrackingStatus::Ready;
        tracking<<tracking_stamp<<','<<fixed_geometry<<','<<physics_dt<<','<<tracking_status_name(command.status)
          <<','<<command.outer_updated<<','<<tracking_ns<<','<<command.s<<','<<command.cross_track_m<<','<<fresh_clear;
        for(double value:command.desired_W)tracking<<','<<value;
        for(double value:command.plant_input_W)tracking<<','<<value;
        tracking<<','<<ticket.epoch<<'\n';
      }
      if(rolling&&(rolling_tick.mode!=RollingMode::RepeatedState||rolling_tick.new_observation)){
        rolling_events<<state.actual.t_ns<<','<<rolling_mode_name(rolling_tick.mode)<<','<<int(rolling_tick.reason)
          <<','<<rolling_tick.reference_version<<','<<rolling_tick.global_s<<','<<rolling_tick.certified_end
          <<','<<rolling_tick.new_observation<<','<<rolling_tick.planned<<','<<rolling_tick.continued
          <<','<<rolling_tick.committed<<','<<rolling_tick.cover_balls<<','<<rolling_tick.planning_ns
          <<','<<rolling_tick.certification_ns<<','<<rolling_tick.control_ns<<','<<(finish-begin);
        for(double value:rolling_tick.command.plant_input_W)rolling_events<<','<<value;
        rolling_events<<','<<ticket.epoch<<'\n';
      }
      if(!rolling||rolling_tick.new_observation){
      events<<release<<','<<begin<<','<<finish<<','<<state.current_simulation_ns<<','<<out.observation.group<<','<<out.reference_version<<','<<navigation_invalid_name(out.invalid)<<','<<out.angular_ready<<','<<out.spatial_ready<<','<<out.spatial_ns/1e6<<','<<out.occupancy_ns/1e6<<','<<(finish-begin)/1e6<<','<<skipped<<','<<ticket.epoch;navigation_detail_csv(events,pipeline);events<<'\n';
      }
      release+=(skipped+1)*BaseConfig::period_ns;
      if(initial<0&&out.input_ready)initial=state.current_simulation_ns;
      if(initial>=0&&state.current_simulation_ns-initial>=int64_t(duration*1e9)){ended=true;break;}
      if(expected_groups&&out.observation.group>=expected_groups&&(rolling?out.input_ready:out.angular_ready)){ended=true;break;}
      if(finish-started>int64_t(timeout*1e9))break;
    }
    executor.cancel();receive.join();if(exportable&&!node->failed()&&node->timebase.can_commit(last_commit,last_oldest,RollingNavigator::max_age_ns))navigation_export(output/"last",pipeline);std::ofstream summary(output/"summary.json");summary<<"{\"source_session\":\""<<session<<"\",\"source_quarantined\":"<<!node->timebase.ticket().accepting<<",\"epoch_resets\":"<<epoch_resets<<",\"discarded_results\":"<<discarded_results<<",\"complete_depth_groups\":"<<node->complete()<<",\"discarded_groups\":"<<node->discarded()<<",\"angular_ready_records\":"<<valid<<",\"spatial_ready_records\":"<<spatial_ready<<",\"completed_window\":"<<ended<<",\"fixed_tracking_ready_records\":"<<tracking_ready<<",\"rolling_ready_records\":"<<rolling_ready<<",\"rolling_protected_records\":"<<rolling_protected<<",\"rolling_resets\":"<<rolling_resets<<",\"rolling_plans\":"<<rolling_plans<<",\"rolling_commits\":"<<rolling_commits<<",\"motion_publishers\":0}\n";rclcpp::shutdown();return ended?0:3;
  }catch(...){executor.cancel();receive.join();rclcpp::shutdown();throw;}
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
