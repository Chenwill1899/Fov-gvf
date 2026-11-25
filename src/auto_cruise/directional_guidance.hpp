#pragma once

#include "new_controller/spherical_field.hpp"
#include "spherical_route.hpp"

namespace ego4p0::cruise {

// Return only a direction in the current body frame. If the start is already
// in the exit cap, an arc entirely in the verified domain may finish the turn.
inline std::optional<Bearing> field_step_heading(const HarmonicResult &field,Bearing goal,double budget) {
  if(field.termination!=CurveEnd::Exit || !field.curve_size || field.curve_size>field.curve.size())
    return std::nullopt;
  const auto &first=field.curve.front();
  const Bearing start{first.alpha,first.beta};
  if(field.curve_size>1){
    Bearing chosen{field.curve[1].alpha,field.curve[1].beta};
    // Follow the available turn angle for this tick. Each shortcut chord
    // stays in the fine field; the actuator separately limits its own slew.
    for(uint32_t j=2;j<field.curve_size && field.curve[j-1].zeta<budget;++j){
      const Bearing next{field.curve[j].alpha,field.curve[j].beta};
      CurveEvidence proof;
      if(!angular_arc_clear(field.fine.layout,field.fine,start,next,proof))break;
      chosen=next;
    }
    return chosen;
  }
  const auto cell=field.fine.layout.cell(goal.az,goal.el);
  CurveEvidence evidence;
  if(cell && FieldBoundary(field.fine.boundary[*cell])==FieldBoundary::Exit &&
     angular_arc_clear(field.fine.layout,field.fine,start,goal,evidence))return goal;
  return start;
}

inline double required_free_travel(double speed) {
  return (2.*dt)*speed + speed*speed/(2.*emergency);
}

// All bearings here are expressed in the yaw frame of the original field.
// Its curve supplies steering only. The caller must certify the realized
// velocity against the new four-camera observation before issuing motion.
inline std::optional<Bearing> continue_field_heading(const HarmonicResult &field,Bearing current,
                                                     Bearing goal,double budget,uint32_t &cursor) {
  if(field.termination!=CurveEnd::Exit || !field.curve_size || field.curve_size>field.curve.size() ||
     cursor>=field.curve_size)return std::nullopt;
  auto point=[&](uint32_t i){return Bearing{field.curve[i].alpha,field.curve[i].beta};};
  const auto &fine=field.fine;
  CurveEvidence evidence;
  if(field.curve_size==1 || cursor+1==field.curve_size){
    const auto cell=fine.layout.cell(goal.az,goal.el);
    if(cell && FieldBoundary(fine.boundary[*cell])==FieldBoundary::Exit &&
       angular_arc_clear(fine.layout,fine,current,goal,evidence))return goal;
    return std::nullopt;
  }
  double nearest=spherical::angle(current,point(cursor));
  while(cursor+1<field.curve_size){
    const double next=spherical::angle(current,point(cursor+1));
    if(next>nearest)break;
    nearest=next;++cursor;
  }
  // A displaced heading must attach again to a newly solved curve.
  if(nearest>field.curve_max_step_deg*spherical::pi/180.+1e-9)return std::nullopt;
  if(cursor+1==field.curve_size)return std::nullopt;
  uint32_t chosen=cursor+1;
  if(!angular_arc_clear(fine.layout,fine,current,point(chosen),evidence))return std::nullopt;
  for(uint32_t j=chosen+1;j<field.curve_size && field.curve[j-1].zeta-field.curve[cursor].zeta<budget;++j){
    if(!angular_arc_clear(fine.layout,fine,current,point(j),evidence))break;
    chosen=j;
  }
  return point(chosen);
}
}
