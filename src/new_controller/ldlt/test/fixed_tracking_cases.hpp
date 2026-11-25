#pragma once
#include "new_controller/fixed_reference_tracker.hpp"
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace ego4p0::tracking_fixture {

inline void require(bool value,const std::string &message) {
  if(!value)throw std::runtime_error(message);
}

inline const std::array<double,9> identity{1,0,0,0,1,0,0,0,1};

struct Case {
  const char *name;
  double turn_degrees,lateral_m,heading_degrees,physics_step_s;
  bool spatial=false;
};

inline const std::array<Case,6> cases{{
  {"straight",0.,0.,0.,.005},
  {"straight_limit",0.,.10,5.,.005},
  {"turn90",90.,.10,5.,.005},
  {"turn180",180.,-.10,-5.,.005},
  {"spatial",105.,.10,-5.,.005,true},
  {"split_physics",90.,-.10,5.,.0025}
}};

inline void build(SpatialReference &ref,const Case &c) {
  std::vector<Direction3> q;
  const int n=std::max(1,int(std::ceil(c.turn_degrees)));
  for(int i=0;i<=n;++i) {
    const double f=double(i)/n;
    const double alpha=c.turn_degrees*direction_pi/180.*f;
    const double beta=c.spatial?.22*std::sin(2*direction_pi*f):0.;
    q.push_back(sphere_point(alpha,beta));
  }
  require(ref.build(q,{0,0,0},identity),"fixture spatial reference");
}

// Independent transcription of the Isaac velocity response, including the
// zero-input braking branch and acceleration cap.
inline Direction3 model_velocity(const Direction3 &v,const Direction3 &u,double dt) {
  const auto difference=nav_add(u,v,-1.);
  const double length=nav_norm(difference);
  double scale=nav_norm(u)<1e-8?1.:std::min(1.,dt/BaseConfig::plant_tau_s);
  if(length>1e-12)scale=std::min(scale,BaseConfig::plant_acceleration_mps2*dt/length);
  return nav_add(v,difference,scale);
}

inline void verify_step(const TrackingCommand &out,const Direction3 &v,const Direction3 &actual_next,
                        const Direction3 &previous_input,const Direction3 &previous_omega,double dt) {
  require(out.status==TrackingStatus::Ready,tracking_status_name(out.status));
  require(std::abs(nav_norm(out.desired_W)-12.)<1e-10&&
          std::abs(nav_norm(actual_next)-12.)<1e-10,"constant speed");
  require(nav_norm(nav_add(actual_next,out.next_velocity_W,-1.))<1e-10,"plant inverse");
  require(nav_norm(nav_add(actual_next,v,-1.))<=150.*dt+1e-11,"plant acceleration");
  require(NavigationLimits::compensation_input(out.plant_input_W,previous_input,dt),"input limits");
  require(NavigationLimits::heading_input(out.omega_W,previous_omega,dt),"angular limits");
}

// Validate geometric deviation with an independent scalar distance search.
inline double independent_deviation(const SpatialReference &ref,const Direction3 &position,double s) {
  double lo=std::max(0.,s-.12),hi=s+.18;
  auto distance=[&](double at){return nav_norm(nav_add(position,ref.position(at).position,-1.));};
  for(int i=0;i<24;++i) {
    const double a=lo+(hi-lo)/3.,b=hi-(hi-lo)/3.;
    if(distance(a)<distance(b))hi=b;else lo=a;
  }
  return std::min({distance(lo),distance((lo+hi)/2.),distance(hi)});
}

inline void run(const std::filesystem::path &csv_path,int repeats,bool periodic,bool independent=false) {
  std::ofstream csv;
  if(!csv_path.empty()) {
    csv.open(csv_path);
    if(!csv)throw std::runtime_error("cannot open tracking CSV");
    csv<<std::setprecision(17)<<"case,repeat,step,physics_dt_s,outer_updated,wall_ns,cpu_ns,s_m,error_bound_m,desired_speed,actual_speed,input_speed,input_slew,angular_rate,angular_acceleration,plant_acceleration\n";
  }
  for(const auto &c:cases) {
    SpatialReference ref;
    build(ref,c);
    const double duration=std::max(.5,ref.profile().length/12.+.16);
    const int steps=int(std::ceil(duration/c.physics_step_s));
    FixedReferenceTracker tracker;
    const auto start=ref.query(0.);
    const auto normal=nav_cross(Direction3{0,0,1},start.tangent_W);
    auto transverse=normal;
    require(unit_direction(transverse),"fixture perpendicular");
    for(int repeat=0;repeat<repeats;++repeat) {
      auto position=nav_add(start.position_W,transverse,c.lateral_m);
      const double angle=c.heading_degrees*direction_pi/180.;
      auto velocity=nav_scale(nav_add(nav_scale(start.tangent_W,std::cos(angle)),transverse,std::sin(angle)),12.);
      auto previous_input=velocity;
      Direction3 previous_omega{};
      require(tracker.attach(ref,position,velocity,0.,ref.profile().length+8.,previous_input,previous_omega)==TrackingStatus::Ready,
              std::string(c.name)+" attachment");
      auto release=field_now_ns()+BaseConfig::period_ns;
      int outer_count=0;
      double max_error=0.,end_s=0.;
      for(int k=0;k<steps;++k) {
        if(periodic) {
          const timespec t{release/1000000000LL,release%1000000000LL};
          int error;
          do{error=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);}while(error==EINTR);
          require(error==0,"tracking periodic release");
        }
        const auto cpu_begin=field_thread_cpu_ns();
        const auto begin=field_now_ns();
        const auto command=tracker.step(position,velocity,c.physics_step_s);
        const auto end=field_now_ns();
        const auto cpu=field_thread_cpu_ns()-cpu_begin;
        if(command.status!=TrackingStatus::Ready)
          throw std::runtime_error(std::string(c.name)+" step "+std::to_string(k)+" "+tracking_status_name(command.status));
        const auto next=model_velocity(velocity,command.plant_input_W,c.physics_step_s);
        verify_step(command,velocity,next,previous_input,previous_omega,c.physics_step_s);
        max_error=std::max(max_error,command.cross_track_m);
        end_s=command.s;
        outer_count+=command.outer_updated;
        if(independent)
          require(independent_deviation(ref,position,command.s)<.20,std::string(c.name)+" independent tracking error");
        if(csv.is_open())csv<<c.name<<','<<repeat<<','<<k<<','<<c.physics_step_s<<','<<command.outer_updated<<','
          <<end-begin<<','<<cpu<<','<<command.s<<','<<command.cross_track_m<<','<<nav_norm(command.desired_W)<<','
          <<nav_norm(next)<<','<<nav_norm(command.plant_input_W)<<','
          <<nav_norm(nav_add(command.plant_input_W,previous_input,-1.))/c.physics_step_s<<','
          <<nav_norm(command.omega_W)<<','<<nav_norm(nav_add(command.omega_W,previous_omega,-1.))/c.physics_step_s<<','
          <<nav_norm(nav_add(next,velocity,-1.))/c.physics_step_s<<'\n';
        previous_input=command.plant_input_W;
        previous_omega=command.omega_W;
        velocity=next;
        position=nav_add(position,velocity,c.physics_step_s);
        const auto period=int64_t(std::llround(c.physics_step_s*1e9));
        release+=(1+std::max<int64_t>(0,(end-release)/period))*period;
      }
      require(max_error<=.20&&end_s>=ref.profile().length,std::string(c.name)+" finish");
      require(outer_count==(steps+(c.physics_step_s<.005?1:0))/(c.physics_step_s<.005?2:1),
              std::string(c.name)+" outer/inner release count");
    }
  }
  if(csv_path.empty())return;
  if(!csv)throw std::runtime_error("tracking CSV write failed");
}

} // namespace ego4p0::tracking_fixture
