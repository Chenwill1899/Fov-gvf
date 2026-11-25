#pragma once
#define EGO4P2_LOCAL_FINE_PROFILE 1
#include "new_controller/depth_input.hpp"
#include "new_controller/desired_direction.hpp"
#include "new_controller/image_hierarchy.hpp"
#include "new_controller/spherical_geometry.hpp"
#include <chrono>
#include <ctime>

namespace ego4p0 {
using Direction3=std::array<double,3>;
inline int64_t field_now_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline int64_t field_thread_cpu_ns() {
  timespec time{};
  if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&time))throw std::runtime_error("thread CPU clock unavailable");
  return int64_t(time.tv_sec)*1000000000LL+time.tv_nsec;
}
inline double direction_dot(const Direction3 &a,const Direction3 &b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline bool unit_direction(Direction3 &q) {double n=std::hypot(q[0],q[1],q[2]);if(!std::isfinite(n)||n<1e-9) return false;for(auto &x:q)x/=n;return true;}
inline double direction_angle(const Direction3 &a,const Direction3 &b) {return std::acos(std::clamp(direction_dot(a,b),-1.,1.));}
inline Direction3 sphere_point(double a,double b) {return {std::cos(b)*std::cos(a),std::cos(b)*std::sin(a),std::sin(b)};}
inline double wrapped_angle(double a) {return std::remainder(a,2*direction_pi);}

enum class FieldStatus:uint8_t {NoInput,Valid,NeedsRefinement,SolveFailed,Idle,TopologyReady};
enum class FieldBoundary:uint8_t {Outside,Interior,Low,Exit,Coupling};
enum class CurveEnd:uint8_t {NoField,Exit,Stagnation,NeedsExpansion,Blocked,StepLimit,NoDirection};
inline const char *field_status_name(FieldStatus s) {switch(s){case FieldStatus::Valid:return "valid";case FieldStatus::NeedsRefinement:return "needs_refinement";case FieldStatus::SolveFailed:return "solve_failed";case FieldStatus::Idle:return "idle";case FieldStatus::TopologyReady:return "topology_ready";default:return "no_input";}}
inline const char *curve_end_name(CurveEnd s) {switch(s){case CurveEnd::Exit:return "exit";case CurveEnd::Stagnation:return "stagnation";case CurveEnd::NeedsExpansion:return "needs_expansion";case CurveEnd::Blocked:return "blocked";case CurveEnd::StepLimit:return "step_limit";case CurveEnd::NoDirection:return "no_direction";default:return "no_field";}}
struct HarmonicConfig {
  double residual_tolerance=1e-7,gradient_epsilon=1e-7;
  double boundary_cap_deg=3.,corridor_deg=3.,max_step_deg=.8,min_step_deg=.00001;
  uint32_t max_iterations=1600,max_curve_steps=2048;
  // Optional legacy coarse-Dirichlet coupling remains selectable.
  bool fine_neumann_roi=true;
  // Diagnostic numerical oracle for the otherwise topology-only local mode.
  // The coupled mode always solves its coarse potential regardless of this flag.
  bool force_coarse_solve=false;
  // Diagnostic oracle: retain topology-only mode but rebuild and validate its
  // graph on every call, without reading or storing the graph-build cache.
  bool force_coarse_rebuild=false;
  uint64_t revision=4;
  void validate() const {
    if(!std::isfinite(residual_tolerance)||residual_tolerance<=0||!std::isfinite(gradient_epsilon)||gradient_epsilon<0||
       !std::isfinite(boundary_cap_deg)||boundary_cap_deg<=0||boundary_cap_deg>30||!std::isfinite(corridor_deg)||corridor_deg<1||corridor_deg>180||
       !std::isfinite(max_step_deg)||max_step_deg<=0||max_step_deg>1||!std::isfinite(min_step_deg)||min_step_deg<=0||min_step_deg>max_step_deg||
       !max_iterations||!max_curve_steps) throw std::invalid_argument("invalid harmonic configuration");
  }
};
// Separate ROS-producer -> field-worker SPSC snapshot. No borrowed DepthFrame.
struct HarmonicDepthInput {
  std::array<std::vector<ImageLevelSummary>,4> coarse_image;
  std::vector<uint8_t> fine_state,cameras;
  uint32_t columns=0,rows=0,image_width=0,image_height=0;
  uint32_t first_block_edge=4,second_block_edge=3,first_block_columns=0,first_block_rows=0;
  uint64_t sequence=0,configuration=0,geometry=0;
  int64_t stamp_ns=0,capture_ns=0;
  bool ready=false;
  void allocate(uint32_t w,uint32_t h,const DirectionGridLayout &fine,uint32_t first_edge=4,uint32_t second_edge=3) {
    hierarchy_.configure(w,h,first_edge,second_edge);
    image_width=w;image_height=h;first_block_edge=first_edge;second_block_edge=second_edge;
    first_block_columns=hierarchy_.first_columns;first_block_rows=hierarchy_.first_rows;columns=hierarchy_.columns;rows=hierarchy_.rows;
    for(auto &v:coarse_image)v.resize(size_t(columns)*rows);
    fine_state.resize(fine.cells());cameras.resize(fine.cells());ready=false;
  }
  const ImageHierarchyGeometry &image_hierarchy()const{return hierarchy_;}
  void capture(const DepthFrame &f) {
    const int64_t begin=field_now_ns();const auto &o=f.observations;ready=false;capture_ns=0;
    if(!hierarchy_.matches(image_width,image_height,first_block_edge,second_block_edge)||!f.local.ready||!f.local.virtual_center||!o.ready||f.local.configuration_id!=o.configuration_id||f.local.sequence!=f.sequence||o.sequence!=f.sequence||f.local.stamp_ns!=f.t_ns||o.stamp_ns!=f.t_ns||o.block_edge!=first_block_edge||o.width!=image_width||o.height!=image_height||o.block_columns!=hierarchy_.first_columns||o.block_rows!=hierarchy_.first_rows||fine_state.size()!=f.local.node_state.size()||cameras.size()!=f.local.observation_cameras.size()||columns!=hierarchy_.columns||rows!=hierarchy_.rows||first_block_columns!=hierarchy_.first_columns||first_block_rows!=hierarchy_.first_rows) return;
    const bool prepared=o.second_ready;
    if(prepared&&(o.second_columns!=columns||o.second_rows!=rows))return;
    for(uint32_t c=0;c<4;++c) {
      if(o.blocks[c].size()!=hierarchy_.first_to_second.size()||coarse_image[c].size()!=size_t(columns)*rows)return;
      if(prepared&&o.second_level[c].size()!=coarse_image[c].size())return;
      if(!prepared)std::fill(coarse_image[c].begin(),coarse_image[c].end(),ImageLevelSummary{});
      for(uint32_t k=0;k<o.blocks[c].size();++k) {
        const auto &v=o.blocks[c][k];const auto &placement=hierarchy_.first_to_second[k];auto &summary=coarse_image[c][placement.coarse];
        if(v.valid_pixels>placement.pixels||v.raw_valid_pixels>placement.pixels||(v.valid_pixels&&(!std::isfinite(v.mean_z)||!(v.mean_z>0))))return;
        if(prepared)continue;
        if(!summary.valid)summary.mean_z=0.;
        if(v.valid_pixels){summary.mean_z+=double(v.mean_z)*v.valid_pixels;summary.valid+=v.valid_pixels;}
        summary.raw_valid+=v.raw_valid_pixels;summary.total+=placement.pixels;
        if(std::isfinite(v.min_rho)&&v.min_rho>0&&(!std::isfinite(summary.min_rho)||v.min_rho<summary.min_rho))summary.min_rho=v.min_rho;
        if(std::isfinite(v.raw_min_rho)&&v.raw_min_rho>0&&(!std::isfinite(summary.raw_min_rho)||v.raw_min_rho<summary.raw_min_rho))summary.raw_min_rho=v.raw_min_rho;
      }
      if(prepared){
        for(uint32_t k=0;k<o.second_level[c].size();++k){const auto&v=o.second_level[c][k];const uint32_t span=first_block_edge*second_block_edge,x=k%columns*span,y=k/columns*span,expected=(std::min(x+span,image_width)-x)*(std::min(y+span,image_height)-y);
          if(v.total!=expected||v.valid>expected||v.raw_valid>expected||(v.valid&&(!std::isfinite(v.mean_z)||v.mean_z<=0)))return;}
        std::copy(o.second_level[c].begin(),o.second_level[c].end(),coarse_image[c].begin());
      }else for(auto &summary:coarse_image[c])summary.mean_z=summary.valid ? summary.mean_z/summary.valid : std::numeric_limits<double>::quiet_NaN();
    }
    std::copy(f.local.node_state.begin(),f.local.node_state.end(),fine_state.begin());
    std::copy(f.local.observation_cameras.begin(),f.local.observation_cameras.end(),cameras.begin());
    sequence=f.sequence;stamp_ns=f.t_ns;configuration=o.configuration_id;geometry=f.local.geometry_id;ready=true;capture_ns=field_now_ns()-begin;
  }
 private:
  ImageHierarchyGeometry hierarchy_;
};
struct AngularIntent { spherical::Bearing start, goal; };
struct HarmonicIntent {
  DesiredMotion desired;
  Direction3 q0{};
  uint64_t coordinate_revision=0;
  int64_t state_stamp_ns=0;
  // Exact R/position are part of cache invalidation for world-frame intent.
  std::array<double,9> R_WB{};
  Direction3 p_W{};
  std::optional<AngularIntent> bearings;
};
struct FieldLayer {
  DirectionGridLayout layout;
  std::vector<double> phi;
  std::vector<uint8_t> boundary,state,cameras;
  FieldStatus status=FieldStatus::NoInput;
  double residual=0.;uint32_t iterations=0,nodes=0;
  int64_t domain_ns=0,solve_ns=0,solve_cpu_ns=0,cache_ns=0,assembly_ns=0,iteration_ns=0,residual_ns=0,store_ns=0;
  bool reused=false,warm_started=false,constant_boundary=false;
  void allocate(const DirectionGridLayout &l) {layout=l;phi.assign(l.cells(),.5);boundary.resize(l.cells());state.resize(l.cells());cameras.resize(l.cells());}
};
struct AdaptiveCoarseField {
  std::vector<uint32_t> owner,offset,neighbor,representative;
  std::vector<double> weight,phi;
  std::vector<uint8_t> boundary,refined;
  uint32_t nodes=0,edges=0,refined_nodes=0,raw_edges=0;int64_t edge_merge_ns=0;
  void allocate(uint32_t n) {owner.resize(n);offset.resize(n+1);neighbor.resize(4*n);weight.resize(4*n);representative.resize(n);phi.resize(n);boundary.resize(n);refined.resize(n);}
};
struct CurveSample {
  Direction3 direction{},gradient_B{},unit_B{};
  double alpha=0,beta=0,F_alpha=0,F_beta=0,norm2=0,tau=0,zeta=0,potential=0,magnitude=0,unit_alpha=0,unit_beta=0;
  uint32_t cell=0;bool valid=false,local_chart=false,zero_field=true;
};
struct HarmonicResult {
  FieldLayer coarse,fine;
  // TopologyReady/reused describe a graph certificate, never a solved phi.
  bool coarse_topology_only=false;
  int64_t coarse_topology_ns=0;
  const char *coarse_topology_failure="none";
  bool coarse_graph_reused=false;
  // Exact-key checks and snapshot copies; included in coarse.domain_ns.
  int64_t coarse_graph_cache_ns=0;
  AdaptiveCoarseField adaptive;
  std::array<std::vector<ImageLevelSummary>,4> coarse_image;
  uint32_t image_columns=0,image_rows=0,image_width=0,image_height=0,image_first_block_edge=4,image_second_block_edge=3;
  std::vector<CurveSample> curve;
  FieldStatus status=FieldStatus::NoInput;CurveEnd termination=CurveEnd::NoField;
  uint64_t version=0,sequence=0,configuration=0,geometry=0,parameter_revision=0;
  uint64_t field_version=0,curve_version=0;
  int64_t stamp_ns=0,completed_ns=0,total_ns=0,curve_ns=0,capture_ns=0,component_ns=0,query_ns=0,segment_ns=0,field_ns=0;
  int64_t cpu_ns=0,reset_ns=0,export_ns=0,finalize_ns=0;
  double angular_length=0,anchor_potential=0;Direction3 exit_direction{};
  HarmonicIntent intent;
  uint32_t curve_size=0,integration_steps=0,gradient_queries=0,fine_attempts=0,segment_checks=0;
  uint8_t curve_failure_kind=0;uint32_t curve_failure_cell=UINT32_MAX,curve_rejected_trials=0;double curve_min_accepted_step_rad=0.;
  bool expanded=false,coarse_mixed=false,exit_clipped=false,reused=false;
  bool fine_neumann_roi=false;double fine_corridor_deg=0.,curve_max_step_deg=0.;
  void allocate(const DirectionGridLayout &c,const DirectionGridLayout &f,uint32_t max_steps) {coarse.allocate(c);fine.allocate(f);adaptive.allocate(f.cells());curve.resize(max_steps+1);}
};
} // namespace ego4p0
