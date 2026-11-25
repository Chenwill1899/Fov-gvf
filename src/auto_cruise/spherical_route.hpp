#pragma once

#include "new_controller/navigation_config.hpp"
#include "new_controller/position_flowbox.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ego4p0::cruise {
using spherical::Bearing;
using spherical::Radial;

inline constexpr double dt=BaseConfig::physics_dt_s;
inline constexpr double speed_limit=BaseConfig::max_speed_mps;
inline constexpr double beta=dt/BaseConfig::plant_tau_s;
inline constexpr double emergency=BaseConfig::plant_acceleration_mps2;

inline Radial limit(Radial value,double maximum) {
  value.r=std::min(value.r,maximum);
  return value;
}

inline Radial read_radial(double az,double el,double range) {
  if(!std::isfinite(az) || !std::isfinite(el) || !std::isfinite(range) ||
     std::abs(el)>spherical::pi/2. || range<0.)
    throw std::runtime_error("invalid spherical direction or range");
  return {{spherical::wrap(az),el},range};
}

inline Bearing steer(Radial velocity,Bearing target,double maximum_angle) {
  if(velocity.r==0.)return target;
  auto displacement=spherical::log(velocity.dir,target);
  const double length=std::hypot(displacement.east,displacement.north);
  if(length<=maximum_angle)return target;
  return spherical::exp(velocity.dir,{displacement.east*maximum_angle/length,
                                      displacement.north*maximum_angle/length});
}

inline Radial predicted_velocity(Radial current,Radial input) {
  if(input.r<1e-8)return {current.dir,std::max(0.,current.r-emergency*dt)};
  return add(current,limit(scale(add(input,current,-1.),beta),emergency*dt));
}

inline bool within_input_limits(Radial value,Radial previous,double max_value,double max_rate) {
  return value.r<=max_value+1e-12 && distance(value,previous)<=max_rate*dt+1e-12;
}

struct CruiseRoute {
  struct Motion {Radial p,v,u,w;};

  void load(const std::filesystem::path &path) {
    std::ifstream input(path);
    load(input);
  }
  void load(std::istream &input) {
    if(!points_.empty())throw std::logic_error("guide already loaded");
    std::string line;
    std::getline(input,line);
    if(!line.empty() && line.back()=='\r')line.pop_back();
    const bool radial_format=line=="az,el,range";
    if(!radial_format && line!="x,y,z")throw std::runtime_error("unrecognized guide coordinate header");
    while(std::getline(input,line)){
      std::replace(line.begin(),line.end(),',',' ');
      std::istringstream fields(line);
      double first,second,third;
      if(!(fields>>first>>second>>third) || !std::isfinite(first) || !std::isfinite(second) || !std::isfinite(third))
        throw std::runtime_error("invalid guide input");
      const auto point=radial_format ? read_radial(first,second,third) :
                                       spherical::from_xyz(first,second,third);
      if(points_.empty()){
        arc_.push_back(0.);
      }else{
        const auto segment=add(point,points_.back(),-1.);
        if(segment.r<1e-8)continue;
        segments_.push_back(segment);
        arc_.push_back(arc_.back()+segment.r);
      }
      points_.push_back(point);
    }
    if(points_.size()<2)throw std::runtime_error("missing geometric guide");
    build_sections();
  }

  // The foot of the perpendicular lies at rho*cos(theta) along a segment.
  // Updating a preview cursor never changes the measured progress cursor.
  double project(Radial position,size_t &cursor) const {
    const size_t begin=cursor>2?cursor-2:0, end=std::min(segments_.size(),cursor+10);
    double nearest=std::numeric_limits<double>::infinity(),progress=arc_[cursor];
    for(size_t i=begin;i<end;++i){
      const auto relative=add(position,points_[i],-1.);
      const Radial segment=segments_[i];
      const double along=std::clamp(relative.r*cosine(relative.dir,segment.dir),0.,segment.r);
      const double error=distance(relative,{segment.dir,along});
      if(error<nearest){nearest=error;cursor=i;progress=arc_[i]+along;}
    }
    return progress;
  }

  struct Reference { Radial position, derivative, curvature; };

  double length() const {return arc_.back();}

  Reference reference(double progress) const {
    const double s=std::clamp(progress,0.,length());
    auto next=std::upper_bound(sections_.begin(),sections_.end(),s,
      [](double value,const Section &piece){return value<piece.s;});
    const auto &piece=*std::prev(next);
    const double h=s-piece.s;
    const auto derivative=add(piece.first,piece.second,h);
    const auto position=add(piece.position,add(piece.first,piece.second,.5*h),h);
    if(derivative.r<1e-8)return {position,{},{}};
    // Curvature is the normal part of d²q/ds² divided by |dq/ds|².
    const auto normal=add(piece.second,{derivative.dir,1.},
                           -piece.second.r*cosine(derivative.dir,piece.second.dir));
    return {position,derivative,scale(normal,1./(derivative.r*derivative.r))};
  }

  struct Tracking {
    PositionFlowBox box;
    PositionFlowBox::Coordinates coordinates{};
    Bearing tangent;
    double station_scale;
    bool valid;
  };

  Tracking tracking(Radial position,double progress,bool terminal=false) const {
    const auto ref=reference(progress);
    const auto delta=add(position,ref.position,-1.);
    auto tangent=ref.derivative.dir;
    // Recover longitudinally after passing the endpoint. The same transverse
    // section and feedback apply when travelling in the opposite direction.
    if(terminal && progress>=length() && delta.r*cosine(delta.dir,tangent)>0.)
      tangent=spherical::opposite(tangent);
    Tracking track{PositionFlowBox(tangent,ref.curvature),{},tangent,ref.derivative.r,false};
    track.valid=ref.derivative.r>=1e-8 && track.box.coordinates(delta,track.coordinates);
    return track;
  }

  struct Actuation {Radial v,u,w;};
  Actuation cruise_command(const Motion &state,const Tracking &track) const {
    const auto guidance=track.box.guidance(track.coordinates,state.v,speed_limit,NavigationConfig::lateral_gain_per_s);
    const auto rate=limit(guidance.angular_velocity(state.v.dir,NavigationConfig::heading_gain_per_s),
                          NavigationConfig::reference_turn_acceleration_mps2/speed_limit);
    const auto omega=add(state.w,limit(add(rate,state.w,-1.),40.*dt));
    const auto heading=spherical::rotate_about(omega.dir,state.v.dir,omega.r*dt);
    const Radial velocity{heading,speed_limit};
    const auto input=add(state.v,add(velocity,state.v,-1.),1./beta);
    return {velocity,input,omega};
  }

  Motion next(const Motion &state,const Tracking &track) const {
    const auto command=cruise_command(state,track);
    return {add(state.p,command.v,dt),command.v,command.u,command.w};
  }

  size_t current_segment=0;
 private:
  // Integrate each change of polyline tangent uniformly over a short station
  // interval. Overlapping intervals add linearly. The resulting piecewise
  // quadratic has a continuous tangent and preserves the two route endpoints.
  struct Section { double s; Radial position,first,second; };
  void build_sections() {
    struct Event {double s; Radial change;};
    std::vector<Event> events;
    for(size_t i=1;i<segments_.size();++i){
      const double span=std::min({NavigationConfig::reference_min_radius_m,arc_[i],length()-arc_[i]});
      const auto change=scale(spherical::add({segments_[i].dir,1.},{segments_[i-1].dir,1.},-1.),.5/span);
      events.push_back({arc_[i]-span,change});
      events.push_back({arc_[i]+span,scale(change,-1.)});
    }
    std::sort(events.begin(),events.end(),[](const Event &a,const Event &b){return a.s<b.s;});
    sections_.push_back({0.,points_.front(),{segments_.front().dir,1.},{}});
    for(const auto &event:events){
      const Section last=sections_.back();
      const double h=event.s-last.s;
      if(h>0.){
        sections_.push_back({event.s,add(last.position,add(last.first,last.second,.5*h),h),
                             add(last.first,last.second,h),last.second});
      }
      sections_.back().second=add(sections_.back().second,event.change);
    }
  }
  std::vector<Section> sections_;
  std::vector<Radial> points_,segments_;
  std::vector<double> arc_;
};
} // namespace ego4p0::cruise
