#pragma once
#include "new_controller/harmonic_curve.hpp"
#include "new_controller/spherical_field.hpp"
#include "new_controller/harmonic_adaptive.hpp"
#include "new_controller/harmonic_topology.hpp"
#include "new_controller/harmonic_graph_cache.hpp"
#include "new_controller/depth_virtual_panorama.hpp"
namespace ego4p0 {
class HarmonicPlanner {
 public:
  void configure(const std::array<CameraCalibration,4> &cal,uint32_t w,uint32_t h,const HarmonicConfig &config={},uint32_t first_block_edge=4,bool angles_only=false) {
    config.validate();config_=config;image_geometry_.configure(w,h,first_block_edge,3);image_columns_=image_geometry_.columns;image_rows_=image_geometry_.rows;coarse_.configure(3.,angles_only);fine_.configure(1.,angles_only);
    coarse_sources_.configure(cal,w,h,coarse_.layout,image_geometry_.pixel_span());
    coarse_solver_.configure(fine_,coarse_);fine_solver_.configure(fine_);if(external_fine_)external_fine_->configure(static_cast<uint32_t>(fine_.nodes.size()));
    coarse_topology_.configure(static_cast<uint32_t>(fine_.nodes.size()));
    coarse_graph_cache_.configure(static_cast<uint32_t>(fine_.nodes.size()),static_cast<uint32_t>(coarse_.nodes.size()));
    const auto n=fine_.nodes.size();component_nodes_.reserve(n);roi_nodes_.reserve(n);cached_roi_nodes_.reserve(n);component_.resize(n);roi_.resize(n);distance_.resize(n);parent_.resize(n);queue_.resize(n);coarse_component_.resize(n);coarse_parent_.resize(n);cached_roi_.resize(n);
    cached_curve_.resize(config_.max_curve_steps+1);previous_=false;curve_cached_=roi_cached_=false;
  }
  void allocate(HarmonicDepthInput &d,uint32_t w,uint32_t h)const {if(w!=image_geometry_.width||h!=image_geometry_.height)throw std::invalid_argument("input image dimensions differ from configured planner");d.allocate(w,h,fine_.layout,image_geometry_.first_edge,image_geometry_.second_edge);}
  const ImageHierarchyGeometry &image_hierarchy()const{return image_geometry_;}
  const VirtualPanoramaTables &coarse_image_sources()const{return coarse_sources_;}
  void allocate(HarmonicResult &r)const {r.allocate(coarse_.layout,fine_.layout,config_.max_curve_steps);r.image_columns=image_columns_;r.image_rows=image_rows_;r.image_width=image_geometry_.width;r.image_height=image_geometry_.height;r.image_first_block_edge=image_geometry_.first_edge;r.image_second_block_edge=image_geometry_.second_edge;for(auto &v:r.coarse_image)v.resize(image_columns_*image_rows_);}
  void invalidate_cache(){previous_=false;curve_cached_=roi_cached_=false;coarse_solver_.invalidate();coarse_topology_.invalidate();coarse_graph_cache_.invalidate();fine_solver_.invalidate();if(external_fine_)external_fine_->invalidate();}
  void set_trial_solvers(SparseTrialMode mode,bool use_fine=false){coarse_solver_.set_trial_solver(mode);if(use_fine){external_fine_=std::make_unique<TrialFieldSolver>(mode);external_fine_->configure(static_cast<uint32_t>(fine_.nodes.size()));}else external_fine_.reset();invalidate_cache();}
  void set_native_preconditioner(CoarsePreconditioner mode){coarse_solver_.clear_trial_solver();external_fine_.reset();coarse_solver_.set_preconditioner(mode);invalidate_cache();}
  const SparseTrialStats &coarse_trial_stats()const{return coarse_solver_.trial_stats();}
  const SparseTrialStats &fine_trial_stats()const{static const SparseTrialStats empty;return external_fine_ ? fine_trial_totals_ : empty;}
  const SphericalStencil &fine_stencil()const {return fine_;}
  const CoarsePreconditionerStats &preconditioner_stats()const {return coarse_solver_.preconditioner_stats();}
  const SphericalStencil &coarse_stencil()const {return coarse_;}
  const HarmonicConfig &config()const {return config_;}
  static bool same_bearings(const HarmonicIntent &a,const HarmonicIntent &b) {
    if(!a.bearings || !b.bearings)return !a.bearings && !b.bearings;
    return a.bearings->start.az==b.bearings->start.az && a.bearings->start.el==b.bearings->start.el &&
           a.bearings->goal.az==b.bearings->goal.az && a.bearings->goal.el==b.bearings->goal.el;
  }
  static bool same_intent(const HarmonicIntent &a,const HarmonicIntent &b) {
    return same_bearings(a,b) && a.q0==b.q0 && a.desired.d_desired==b.desired.d_desired && a.desired.v_desired==b.desired.v_desired && a.desired.valid==b.desired.valid && a.desired.has_direction==b.desired.has_direction && a.coordinate_revision==b.coordinate_revision && a.R_WB==b.R_WB && a.p_W==b.p_W;
  }
  // Translation places the eventual spatial reference, but is not a variable
  // in the body-frame S^2 boundary-value problem. Keep the full public identity
  // for worker scheduling and use this narrower key only for angular reuse.
  static bool same_angular_intent(const HarmonicIntent &a,const HarmonicIntent &b) {
    return same_bearings(a,b) && a.q0==b.q0 && a.desired.d_desired==b.desired.d_desired &&
      a.desired.v_desired==b.desired.v_desired && a.desired.valid==b.desired.valid &&
      a.desired.has_direction==b.desired.has_direction &&
      a.coordinate_revision==b.coordinate_revision && a.R_WB==b.R_WB;
  }
  void build(const HarmonicDepthInput &input,const HarmonicIntent &intent,HarmonicResult &out) {
    const auto cpu_begin=field_thread_cpu_ns();
    const auto begin=field_now_ns();coarse_solver_.clear_trial_stats();fine_trial_totals_={};if(external_fine_)external_fine_->clear_reported_stats();
    out.intent=intent;out.sequence=input.sequence;out.stamp_ns=input.stamp_ns;out.configuration=input.configuration;out.geometry=input.geometry;out.parameter_revision=config_.revision;out.fine_neumann_roi=config_.fine_neumann_roi;out.fine_corridor_deg=config_.corridor_deg;out.curve_max_step_deg=config_.max_step_deg;out.image_width=input.image_width;out.image_height=input.image_height;out.image_first_block_edge=input.first_block_edge;out.image_second_block_edge=input.second_block_edge;
    out.capture_ns=input.capture_ns;out.curve_size=out.integration_steps=out.gradient_queries=out.fine_attempts=out.segment_checks=0;
    out.curve_ns=out.query_ns=out.segment_ns=out.component_ns=out.field_ns=0;
    out.cpu_ns=out.reset_ns=out.export_ns=out.finalize_ns=0;
    out.adaptive.edge_merge_ns=0;out.angular_length=out.anchor_potential=0;out.exit_direction={};
    out.expanded=out.coarse_mixed=out.exit_clipped=out.reused=false;out.adaptive.nodes=out.adaptive.edges=out.adaptive.refined_nodes=0;
    out.coarse_topology_only=config_.fine_neumann_roi&&!config_.force_coarse_solve;
    out.coarse_topology_ns=0;out.coarse_topology_failure="none";
    out.coarse_graph_reused=false;out.coarse_graph_cache_ns=0;
    out.field_version=out.curve_version=0;out.curve_failure_kind=0;out.curve_failure_cell=UINT32_MAX;out.curve_rejected_trials=0;out.curve_min_accepted_step_rad=0.;
    reset_layer(out.coarse);reset_layer(out.fine);out.status=FieldStatus::NoInput;out.termination=CurveEnd::NoField;
    if(out.coarse_topology_only){
      out.coarse.residual=std::numeric_limits<double>::quiet_NaN();
      std::fill(out.coarse.phi.begin(),out.coarse.phi.end(),std::numeric_limits<double>::quiet_NaN());
    }
    out.reset_ns=field_now_ns()-begin;
    auto done=[&]{out.cpu_ns=field_thread_cpu_ns()-cpu_begin;out.total_ns=field_now_ns()-begin;out.completed_ns=field_now_ns();++out.version;};
    if(!input.ready||!image_geometry_.matches(input.image_width,input.image_height,input.first_block_edge,input.second_block_edge)||input.first_block_columns!=image_geometry_.first_columns||input.first_block_rows!=image_geometry_.first_rows||input.fine_state.size()!=fine_.layout.cells()||input.cameras.size()!=fine_.layout.cells()||out.fine.phi.size()!=fine_.layout.cells()||out.fine.state.size()!=fine_.layout.cells()||out.coarse.phi.size()!=coarse_.layout.cells()||std::any_of(input.coarse_image.begin(),input.coarse_image.end(),[&](const auto &v){return v.size()!=size_t(input.columns)*input.rows;})||std::any_of(out.coarse_image.begin(),out.coarse_image.end(),[&](const auto &v){return v.size()!=size_t(image_columns_)*image_rows_;})||input.columns!=image_columns_||input.rows!=image_rows_){invalidate_cache();done();return;}
    for(uint32_t c=0;c<4;++c)std::copy(input.coarse_image[c].begin(),input.coarse_image[c].end(),out.coarse_image[c].begin());
    Direction3 q=intent.q0,desired=intent.desired.d_desired;
    bool valid_directions=false;
    if(intent.bearings && fine_.angular_storage){
      const auto &b=*intent.bearings;
      valid_directions=fine_.layout.cell(b.start.az,b.start.el).has_value() &&
                       fine_.layout.cell(b.goal.az,b.goal.el).has_value();
    }else if(!intent.bearings && !fine_.angular_storage){
      valid_directions=unit_direction(q) && unit_direction(desired);
    }
    if(!intent.desired.valid || !intent.desired.has_direction || !valid_directions){
      out.status=FieldStatus::Idle;out.termination=CurveEnd::NoDirection;coarse_graph_cache_.invalidate();done();return;
    }
    if(!previous_||input.geometry!=last_geometry_||input.configuration!=last_configuration_||input.first_block_edge!=last_first_block_edge_||input.second_block_edge!=last_second_block_edge_||input.image_width!=last_image_width_||input.image_height!=last_image_height_||intent.coordinate_revision!=last_intent_.coordinate_revision||intent.R_WB!=last_intent_.R_WB){coarse_solver_.invalidate();coarse_topology_.invalidate();coarse_graph_cache_.invalidate();fine_solver_.invalidate();if(external_fine_)external_fine_->invalidate();curve_cached_=false;roi_cached_=false;}
    const bool identical_intent=previous_&&same_angular_intent(intent,last_intent_);
    previous_=true;last_intent_=intent;last_geometry_=input.geometry;last_configuration_=input.configuration;last_first_block_edge_=input.first_block_edge;last_second_block_edge_=input.second_block_edge;last_image_width_=input.image_width;last_image_height_=input.image_height;
    const auto components=field_now_ns();std::copy(input.fine_state.begin(),input.fine_state.end(),out.fine.state.begin());std::copy(input.cameras.begin(),input.cameras.end(),out.fine.cameras.begin());
    const uint32_t start=intent.bearings ? *fine_.layout.cell(intent.bearings->start.az,intent.bearings->start.el) : fine_.cell(q);
    if(input.fine_state[start]!=uint8_t(CandidateStatus::Available)){out.status=out.fine.status=FieldStatus::NeedsRefinement;curve_cached_=roi_cached_=false;coarse_graph_cache_.invalidate();out.component_ns=field_now_ns()-components;done();return;}
    const auto count=flood(fine_,input.fine_state,start,component_,parent_);
    const auto requested=intent.bearings ? *fine_.layout.cell(intent.bearings->goal.az,intent.bearings->goal.el) : fine_.cell(desired);
    uint32_t target=requested,low=start;
    if(intent.bearings)fine_.prepare_goal(intent.bearings->goal);
    component_nodes_.clear();
    for(uint32_t i=0;i<component_.size();++i)if(component_[i])component_nodes_.push_back(i);
    if(!component_[requested]){
      out.exit_clipped=true;
      double best=-2.;
      for(uint32_t i:component_nodes_){
        const auto score=intent.bearings?fine_.goal_score(i):direction_dot(fine_.nodes[i].direction,desired);
        if(score>best){best=score;target=i;}
      }
    }
    double worst=2.;
    for(uint32_t i:component_nodes_){const auto score=fine_.node_cosine(i,target);if(score<worst){worst=score;low=i;}}
    out.component_ns=field_now_ns()-components;
    if(count<2){out.status=FieldStatus::NeedsRefinement;curve_cached_=roi_cached_=false;coarse_graph_cache_.invalidate();done();return;}
    // Preserve the exact adaptive graph and Exit labels. The default local
    // Neumann fine problem does not consume coarse potentials, so only its
    // topology is certified. Coupled mode and the diagnostic oracle still
    // execute the original numerical coarse solve.
    const auto coarse_domain=field_now_ns();build_coarse_state(input,out);
    const double coarse_cap=config_.boundary_cap_deg*direction_pi/180;
    const bool graph_cache_enabled=out.coarse_topology_only&&!config_.force_coarse_rebuild;
    if(graph_cache_enabled){
      const auto cache_begin=field_now_ns();
      out.coarse_graph_reused=coarse_graph_cache_.restore(component_,component_nodes_,out.coarse.state,low,target,coarse_cap,out.adaptive);
      out.coarse_graph_cache_ns=field_now_ns()-cache_begin;
    }
    if(!out.coarse_graph_reused)coarse_solver_.build(component_,component_nodes_,out.coarse,low,target,coarse_cap,out.adaptive);
    out.coarse.domain_ns=field_now_ns()-coarse_domain;
    bool coarse_ready=false;
    if(out.coarse_topology_only){
      if(out.coarse_graph_reused){
        // Exact builder inputs restored a private, already validated graph.
        // This is the same graph certificate that validate() would reuse.
        out.coarse.status=FieldStatus::TopologyReady;out.coarse.nodes=out.adaptive.nodes;
        out.coarse.reused=true;coarse_ready=true;
      }else{
        std::fill_n(out.adaptive.phi.begin(),out.adaptive.nodes,std::numeric_limits<double>::quiet_NaN());
        const auto topology_begin=field_now_ns();
        coarse_ready=coarse_topology_.validate(out.adaptive,component_,component_nodes_,out.coarse);
        out.coarse_topology_ns=field_now_ns()-topology_begin;
        out.coarse_topology_failure=coarse_topology_.error();
        if(coarse_ready&&graph_cache_enabled){
          const auto cache_begin=field_now_ns();
          coarse_graph_cache_.remember_validated(component_,component_nodes_,out.coarse.state,low,target,coarse_cap,out.adaptive);
          const auto cache_ns=field_now_ns()-cache_begin;
          out.coarse_graph_cache_ns+=cache_ns;out.coarse.domain_ns+=cache_ns;
        }
      }
    }else{
      const auto solve_cpu_begin=field_thread_cpu_ns();
      coarse_solver_.solve(out.adaptive,out.coarse,config_);
      out.coarse.solve_cpu_ns=field_thread_cpu_ns()-solve_cpu_begin;
      coarse_ready=out.coarse.status==FieldStatus::Valid;
    }
    if(!coarse_ready){out.status=FieldStatus::SolveFailed;curve_cached_=roi_cached_=false;coarse_graph_cache_.invalidate();out.field_ns=field_now_ns()-begin;done();return;}
    const auto export_begin=field_now_ns();
    export_regular(out);
    out.export_ns=field_now_ns()-export_begin;
    const auto local_domain=field_now_ns();
    if(out.coarse.reused&&roi_cached_&&cached_roi_[start]&&cached_roi_[target]){
      roi_=cached_roi_;roi_nodes_=cached_roi_nodes_;
    }
    else {
      std::fill(roi_.begin(),roi_.end(),0);uint32_t v=target;
      while(true){roi_[v]=1;if(v==start)break;v=parent_[v];}
      // Follow increasing coarse potential once; include its aggregate supports.
      if(!config_.fine_neumann_roi){
      std::fill(coarse_component_.begin(),coarse_component_.end(),0);auto node=out.adaptive.owner[start];
      for(uint32_t k=0;k<out.adaptive.nodes;++k){coarse_component_[node]=1;if(FieldBoundary(out.adaptive.boundary[node])==FieldBoundary::Exit)break;uint32_t next=node;double maximum=out.adaptive.phi[node];for(uint32_t e=out.adaptive.offset[node];e<out.adaptive.offset[node+1];++e){const auto j=out.adaptive.neighbor[e];if(!coarse_component_[j]&&out.adaptive.phi[j]>maximum){maximum=out.adaptive.phi[j];next=j;}}if(next==node)break;node=next;}
      for(uint32_t i=0;i<roi_.size();++i)if(component_[i]&&coarse_component_[out.adaptive.owner[i]])roi_[i]=1;
      }
      expand_roi(uint32_t(std::ceil(config_.corridor_deg)));
    }
    make_fine(out);out.fine.domain_ns=field_now_ns()-local_domain;
    for(uint32_t attempt=0;attempt<3;++attempt) {
      const auto old_solve=out.fine.solve_ns,old_solve_cpu=out.fine.solve_cpu_ns,old_iteration=out.fine.iteration_ns,old_cache=out.fine.cache_ns,old_assembly=out.fine.assembly_ns,old_residual=out.fine.residual_ns,old_store=out.fine.store_ns;
      const auto old_iterations=out.fine.iterations;
      const auto solve_cpu_begin=field_thread_cpu_ns();
      if(external_fine_){external_fine_->solve_layer(fine_,out.fine,config_);accumulate_fine_trial_stats(external_fine_->stats(),out.fine_attempts==0);}else fine_solver_.solve(out.fine,config_);++out.fine_attempts;
      out.fine.solve_cpu_ns=old_solve_cpu+field_thread_cpu_ns()-solve_cpu_begin;
      out.fine.solve_ns+=old_solve;out.fine.iteration_ns+=old_iteration;out.fine.cache_ns+=old_cache;out.fine.assembly_ns+=old_assembly;out.fine.residual_ns+=old_residual;out.fine.store_ns+=old_store;out.fine.iterations+=old_iterations;
      if(out.fine.status!=FieldStatus::Valid){out.status=FieldStatus::SolveFailed;curve_cached_=roi_cached_=false;coarse_graph_cache_.invalidate();out.field_ns=field_now_ns()-begin-out.curve_ns;done();return;}
      if(!attempt&&out.fine.reused&&out.coarse.reused&&identical_intent&&curve_cached_) {
        std::copy_n(cached_curve_.begin(),cached_curve_size_,out.curve.begin());out.curve_size=cached_curve_size_;out.termination=cached_termination_;out.angular_length=cached_length_;out.anchor_potential=cached_anchor_potential_;out.exit_direction=cached_exit_;out.integration_steps=out.curve_size?out.curve_size-1:0;out.reused=true;break;
      }
      const auto old_curve=out.curve_ns,old_query=out.query_ns,old_segment=out.segment_ns;const auto old_queries=out.gradient_queries,old_checks=out.segment_checks;
      if(intent.bearings) integrate_bearings(fine_,config_,out);
      else integrator_.integrate(fine_,config_,out);out.curve_ns+=old_curve;out.query_ns+=old_query;out.segment_ns+=old_segment;out.gradient_queries+=old_queries;out.segment_checks+=old_checks;
      if(out.termination==CurveEnd::Exit||attempt==2)break;
#if EGO4P0_CURVE_GUARD
      if(!FineCurveIntegrator::expansion_allowed(out))break;
#endif
      const auto expand_begin=field_now_ns();expand_roi(uint32_t(std::ceil(config_.corridor_deg)));out.expanded=true;make_fine(out);out.fine.domain_ns+=field_now_ns()-expand_begin;
    }
    const auto finalize_begin=field_now_ns();
    out.field_ns=field_now_ns()-begin-out.curve_ns;
    out.status=out.termination==CurveEnd::Exit ? FieldStatus::Valid : FieldStatus::NeedsRefinement;
    if(out.status!=FieldStatus::Valid)coarse_graph_cache_.invalidate();
    if(!out.coarse.reused||!out.fine.reused)++field_version_;
    if(!out.reused)++curve_version_;
    out.field_version=field_version_;out.curve_version=curve_version_;
    roi_cached_=out.fine.status==FieldStatus::Valid;if(roi_cached_){cached_roi_=roi_;cached_roi_nodes_=roi_nodes_;}
    curve_cached_=out.status==FieldStatus::Valid;cached_curve_size_=out.curve_size;cached_termination_=out.termination;cached_length_=out.angular_length;cached_anchor_potential_=out.anchor_potential;cached_exit_=out.exit_direction;
    if(curve_cached_)std::copy_n(out.curve.begin(),out.curve_size,cached_curve_.begin());
    out.finalize_ns=field_now_ns()-finalize_begin;
    done();
  }
 private:
  void accumulate_fine_trial_stats(const SparseTrialStats &s,bool first) {
    fine_trial_totals_.validation_ns+=s.validation_ns;fine_trial_totals_.conversion_ns+=s.conversion_ns;fine_trial_totals_.import_ns+=s.import_ns;fine_trial_totals_.analysis_ns+=s.analysis_ns;fine_trial_totals_.factor_ns+=s.factor_ns;fine_trial_totals_.solve_ns+=s.solve_ns;fine_trial_totals_.verify_ns+=s.verify_ns;fine_trial_totals_.total_ns+=s.total_ns;fine_trial_totals_.iterations+=s.iterations;
    fine_trial_totals_.unknowns=s.unknowns;fine_trial_totals_.nonzeros=s.nonzeros;fine_trial_totals_.factor_nonzeros=s.factor_nonzeros;fine_trial_totals_.residual=s.residual;fine_trial_totals_.error=s.error;
    fine_trial_totals_.ok=first ? s.ok : fine_trial_totals_.ok&&s.ok;fine_trial_totals_.symbolic_reused=first ? s.symbolic_reused : fine_trial_totals_.symbolic_reused&&s.symbolic_reused;fine_trial_totals_.numeric_reused=first ? s.numeric_reused : fine_trial_totals_.numeric_reused&&s.numeric_reused;
  }
  static void reset_layer(FieldLayer &l) {std::fill(l.boundary.begin(),l.boundary.end(),0);std::fill(l.state.begin(),l.state.end(),0);std::fill(l.cameras.begin(),l.cameras.end(),0);std::fill(l.phi.begin(),l.phi.end(),.5);l.status=FieldStatus::NoInput;l.iterations=l.nodes=0;l.residual=0;l.domain_ns=l.solve_ns=l.solve_cpu_ns=l.cache_ns=l.assembly_ns=l.iteration_ns=l.residual_ns=l.store_ns=0;l.reused=l.warm_started=l.constant_boundary=false;}
  uint32_t flood(const SphericalStencil &s,const std::vector<uint8_t> &state,uint32_t start,std::vector<uint8_t> &mask,std::vector<uint32_t> &parent) {
    std::fill(mask.begin(),mask.end(),0);uint32_t head=0,tail=0;queue_[tail++]=start;mask[start]=1;parent[start]=start;
    while(head<tail) {const auto i=queue_[head++];for(auto j:s.nodes[i].neighbor)if(j!=UINT32_MAX&&!mask[j]&&state[j]==uint8_t(CandidateStatus::Available)){mask[j]=1;parent[j]=i;queue_[tail++]=j;}}
    return tail;
  }
  void expand_roi(uint32_t radius) {
    uint32_t head=0,tail=0;std::fill(distance_.begin(),distance_.end(),UINT32_MAX);
    for(uint32_t i=0;i<roi_.size();++i)if(roi_[i]) {queue_[tail++]=i;distance_[i]=0;}
    while(head<tail) {auto i=queue_[head++];if(distance_[i]>=radius)continue;for(auto j:fine_.nodes[i].neighbor)if(j!=UINT32_MAX&&component_[j]&&distance_[j]==UINT32_MAX){roi_[j]=1;distance_[j]=distance_[i]+1;queue_[tail++]=j;}}
    roi_nodes_.assign(queue_.begin(),queue_.begin()+tail);
    std::sort(roi_nodes_.begin(),roi_nodes_.end());
  }
  void build_coarse_state(const HarmonicDepthInput &d,HarmonicResult &out) {
    const auto nf=fine_.layout.azimuth_bins,nc=coarse_.layout.azimuth_bins;
    for(uint32_t i=0;i<coarse_.nodes.size();++i) {
      const auto a=i%nc,b=i/nc;bool all=true,any=false,near=false;uint8_t cameras=0;
      for(uint32_t y=b*3;y<(b+1)*3;++y)for(uint32_t x=a*3;x<(a+1)*3;++x) {const auto id=y*nf+x;all&=d.fine_state[id]==uint8_t(CandidateStatus::Available);any|=d.fine_state[id]==uint8_t(CandidateStatus::Available);near|=d.fine_state[id]==uint8_t(CandidateStatus::Unavailable);cameras|=d.cameras[id];}
      bool complete=false;const auto &sources=coarse_sources_.node(i);
      for(uint32_t c=0;c<4;++c)if(sources.blocks[c]!=UINT32_MAX) {
        const auto &s=d.coarse_image[c][sources.blocks[c]];
        near|=std::isfinite(s.min_rho)&&s.min_rho<=ControllerLimits::local_range_m;
        complete|=s.total && s.raw_valid==s.total && std::isfinite(s.raw_min_rho)&&s.raw_min_rho>ControllerLimits::local_range_m+ControllerLimits::radius_m;
      }
      out.coarse.state[i]=uint8_t(near ? CandidateStatus::Unavailable : all&&complete ? CandidateStatus::Available : CandidateStatus::Unknown);out.coarse.cameras[i]=cameras;
      out.coarse_mixed|=any&&(!all||!complete||near);
    }
  }
  void set_boundaries(const SphericalStencil &s,const std::vector<uint8_t> &domain,uint32_t low,uint32_t target,FieldLayer &l)const {
    const double separation=direction_angle(s.nodes[low].direction,s.nodes[target].direction);
    const double cap=std::min(config_.boundary_cap_deg*direction_pi/180.,separation*.24),threshold=std::cos(cap);
    for(uint32_t i=0;i<domain.size();++i)if(domain[i]) {
      l.boundary[i]=uint8_t(FieldBoundary::Interior);
      if(i==target||direction_dot(s.nodes[i].direction,s.nodes[target].direction)>=threshold){l.boundary[i]=uint8_t(FieldBoundary::Exit);l.phi[i]=1.;}
      else if(i==low||direction_dot(s.nodes[i].direction,s.nodes[low].direction)>=threshold){l.boundary[i]=uint8_t(FieldBoundary::Low);l.phi[i]=0.;}
    }
  }
  void export_regular(HarmonicResult &out)const {
    const auto nf=fine_.layout.azimuth_bins,nc=coarse_.layout.azimuth_bins;
    for(uint32_t i=0;i<coarse_.nodes.size();++i){const auto x=i%nc*3,y=i/nc*3,node=out.adaptive.owner[y*nf+x];if(node==UINT32_MAX||out.adaptive.refined[node])continue;bool same=true;for(uint32_t dy=0;dy<3;++dy)for(uint32_t dx=0;dx<3;++dx)same&=out.adaptive.owner[(y+dy)*nf+x+dx]==node;if(same){out.coarse.phi[i]=out.adaptive.phi[node];out.coarse.boundary[i]=out.adaptive.boundary[node];}}
  }
  void make_fine(HarmonicResult &out)const {
    if(config_.fine_neumann_roi){make_local_fine(out);return;}
    for(uint32_t i:roi_nodes_) {
      const auto node=out.adaptive.owner[i];out.fine.boundary[i]=out.adaptive.boundary[node];out.fine.phi[i]=out.adaptive.phi[node];
      if(FieldBoundary(out.fine.boundary[i])!=FieldBoundary::Interior)continue;
      for(auto j:fine_.nodes[i].neighbor)if(j!=UINT32_MAX&&component_[j]&&!roi_[j]){out.fine.boundary[i]=uint8_t(FieldBoundary::Coupling);break;}
      // Constant prolongation is a defined FV interpolation I(phi_c), using
      // only the observed support containing this cell, never unknown neighbors.
    }
  }
  // The coarse graph still validates the connected component. The reduced
  // fine problem is grounded on the local entry rim and the existing Exit;
  // all artificial side faces are homogeneous Neumann. It is intentionally
  // a different fine operator, not an approximate reuse of the coupled field.
  void make_local_fine(HarmonicResult &out)const {
    uint32_t exit=UINT32_MAX,low=UINT32_MAX;double best=-2.,worst=2.;
    for(uint32_t i:roi_nodes_) {
      const auto node=out.adaptive.owner[i];
      const bool is_exit=FieldBoundary(out.adaptive.boundary[node])==FieldBoundary::Exit;
      out.fine.boundary[i]=uint8_t(is_exit?FieldBoundary::Exit:FieldBoundary::Interior);
      out.fine.phi[i]=is_exit?1.:.5;
      if(is_exit){const auto score=(out.intent.bearings?fine_.goal_score(i):direction_dot(fine_.nodes[i].direction,out.intent.desired.d_desired));if(score>best){best=score;exit=i;}}
    }
    if(exit==UINT32_MAX)return; // Original solver validation rejects missing Exit.
    for(uint32_t i:roi_nodes_)if(FieldBoundary(out.fine.boundary[i])!=FieldBoundary::Exit) {
      const auto score=fine_.node_cosine(i,exit);
      if(score<worst){worst=score;low=i;}
    }
    // The aligned case may have a whole ROI inside Exit. Ground a different
    // observed rim node while retaining the actual anchor as Exit.
    if(low==UINT32_MAX)for(uint32_t i:roi_nodes_)if(i!=(out.intent.bearings ? *fine_.layout.cell(out.intent.bearings->start.az,out.intent.bearings->start.el) : fine_.cell(out.intent.q0))) {
      const auto score=fine_.node_cosine(i,exit);
      if(score<worst){worst=score;low=i;}
    }
    if(low!=UINT32_MAX){out.fine.boundary[low]=uint8_t(FieldBoundary::Low);out.fine.phi[low]=0.;}
  }
  uint32_t image_columns_=0,image_rows_=0;ImageHierarchyGeometry image_geometry_;
  uint32_t last_first_block_edge_=0,last_second_block_edge_=0,last_image_width_=0,last_image_height_=0;
  HarmonicConfig config_;SphericalStencil coarse_,fine_;VirtualPanoramaTables coarse_sources_;
  CoarseTopologyValidator coarse_topology_;
  ValidatedCoarseGraphCache coarse_graph_cache_;
  AdaptiveCoarseSolver coarse_solver_;HarmonicLinearSolver fine_solver_;FineCurveIntegrator integrator_;std::unique_ptr<TrialFieldSolver> external_fine_;SparseTrialStats fine_trial_totals_;
  std::vector<uint8_t>component_,coarse_component_,roi_,cached_roi_;std::vector<uint32_t>queue_,parent_,coarse_parent_,distance_,component_nodes_,roi_nodes_,cached_roi_nodes_;
  HarmonicIntent last_intent_;uint64_t last_geometry_=0,last_configuration_=0;bool previous_=false,curve_cached_=false,roi_cached_=false;
  std::vector<CurveSample>cached_curve_;uint32_t cached_curve_size_=0;CurveEnd cached_termination_=CurveEnd::NoField;double cached_length_=0,cached_anchor_potential_=0;Direction3 cached_exit_{};
  uint64_t field_version_=0,curve_version_=0;
};
} // namespace ego4p0
