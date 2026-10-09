#include "pc_gvf/paper_guidance.hpp"
#include <istream>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <cmath>
namespace pc_gvf { namespace depth_angular {
namespace {
// Versioned local binary format: scalar fields, not compiler struct padding.
// Bounded lengths reject truncated or oversized files before allocation.
struct Archive {
    std::istream* in=nullptr; std::ostream* out=nullptr;
    bool memory_metadata=true;
    explicit Archive(std::istream& s):in(&s){}
    explicit Archive(std::ostream& s):out(&s){}
    template<class T> void number(T& x) {
        static_assert(std::is_arithmetic<T>::value,"numeric archive only");
        if(in)in->read(reinterpret_cast<char*>(&x),sizeof x);
        else out->write(reinterpret_cast<const char*>(&x),sizeof x);
        if((in&&!*in)||(out&&!*out))throw std::runtime_error("incomplete replay stream");
    }
    void number(bool& x) {
        unsigned char b=x?1:0;number(b);
        if(b>1)throw std::runtime_error("invalid replay boolean");
        if(in)x=b;
    }
    template<class T> void eigen(T& x) { for(int i=0;i<x.size();++i)number(x.data()[i]); }
    template<class T> void numbers(std::vector<T>& x) {
        std::uint32_t n=static_cast<std::uint32_t>(x.size());number(n);
        if(n>2000000)throw std::runtime_error("oversized replay vector");
        if(in)x.resize(n);
        for(auto& v:x)number(v);
    }
    void string(std::string& x) {
        std::vector<unsigned char> bytes(x.begin(),x.end());numbers(bytes);
        if(in)x.assign(bytes.begin(),bytes.end());
    }
    void camera(Camera& c) {
        int w=c.width(),h=c.height();double hf=c.horizontalFovDeg(),vf=c.verticalFovDeg(),d=c.maxDepth();
        double fx=c.fx(),fy=c.fy(),cx=c.cx(),cy=c.cy();
        number(w);number(h);number(hf);number(vf);number(d);number(fx);number(fy);number(cx);number(cy);
        if(in) {
            if(w<2||h<2||w>1024||h>1024)throw std::runtime_error("invalid replay camera");
            c=Camera(w,h,hf,vf,d);c.setIntrinsics(fx,fy,cx,cy);
        }
    }
    void observation(DepthObservation& o,int nesting=0) {
        if(nesting>2)throw std::runtime_error("nested replay observation");
        camera(o.camera);numbers(o.depth);eigen(o.origin);eigen(o.rotation);
        number(o.stamp);number(o.version);number(o.view);number(o.revoked);
        if(memory_metadata){number(o.memory_uncertainty_rate);if(!std::isfinite(o.memory_uncertainty_rate)||o.memory_uncertainty_rate<0)throw std::runtime_error("invalid memory uncertainty");}
        // Raw hits have already been ingested at capture time. The replayed
        // step consumes the resulting revoked state and conservative depths.
        std::uint32_t n=static_cast<std::uint32_t>(o.supporting_views.size());number(n);
        if(n>256)throw std::runtime_error("too many supporting views");
        if(in)o.supporting_views.clear();
        for(std::uint32_t i=0;i<n;++i) {
            if(in) {
                auto view=std::make_shared<DepthObservation>(Camera(),std::vector<double>(),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),0);
                observation(*view,nesting+1);o.supporting_views.push_back(view);
            } else observation(const_cast<DepthObservation&>(*o.supporting_views[i]),nesting+1);
        }
    }
    void balls(std::vector<std::pair<Eigen::Vector3d,double>>& v) {
        std::uint32_t n=static_cast<std::uint32_t>(v.size());number(n);
        if(n>20000)throw std::runtime_error("too many certificates");
        if(in)v.resize(n);
        for(auto& b:v){eigen(b.first);number(b.second);}
    }
};
}
struct PaperReplayCodec {
    static void config(Archive& io,PaperConfig& c,bool modern=true,bool teleop=true,bool preview=true,bool direct=true,bool model=true,bool response=true,bool history_capacity=true,bool certificate_memory=true,bool feedforward=true,bool command_accel=true,bool local_repair=true,bool six_axis=true) {
        io.number(c.motion.body_radius);
        io.number(c.motion.safety_margin);
        io.number(c.motion.reference_speed);
        io.number(c.motion.max_accel);
        io.number(c.motion.brake_accel);
        io.number(c.motion.velocity_tau);
        io.number(c.motion.delay);
        io.number(c.motion.planning_horizon);
        io.number(c.motion.control_dt);
        io.number(c.motion.dynamics_dt);
        io.number(c.motion.rollout_dt);
        io.number(c.motion.rollout_horizon);
        io.number(c.motion.rollout_margin);
        io.number(c.motion.max_direction_rate);
        io.number(c.motion.horizontal_only);
        io.number(c.motion.continuous_harmonic_guidance);
        io.number(c.motion.harmonic_gradient_radius_cells);
        io.number(c.motion.harmonic_gradient_lookahead_cells);
        io.number(c.motion.goal_tolerance);
        io.number(c.motion.clearance_reward);
        io.number(c.motion.hysteresis_weight);
        io.number(c.motion.deterministic_left_bias);
        io.number(c.motion.convergence_distance);
        io.number(c.motion.convergence_margin);
        io.number(c.motion.convergence_length);
        io.number(c.motion.convergence_max_angle);
        io.number(c.motion.source_radius_cells);
        io.number(c.motion.goal_radius_cells);
        io.number(c.motion.field_tolerance);
        io.number(c.motion.field_max_iterations);
        io.number(c.motion.depth_point_stride);
        io.number(c.motion.cone_chunk_size);
        io.number(c.field_interval);
        io.number(c.observation_timeout);
        io.number(c.history_duration);
        io.number(c.retain_verified_travel);
        io.number(c.history_sample_interval);
        io.number(c.history_uncertainty_rate);
        io.number(c.depth_uncertainty);
        io.number(c.uncertainty_rate);
        io.number(c.max_speed);
        io.number(c.max_vertical_speed);
        io.number(c.transverse_gain);
        io.number(c.chart_speed);
        io.number(c.minimum_lookahead);
        io.number(c.minimum_gradient);
        io.number(c.section_radius);
        io.number(c.max_return_arc);
        io.number(c.command_change_angle);
        io.number(c.coarse_factor);
        io.number(c.fine_width);
        io.number(c.fine_height);
        io.number(c.feedback);
        io.number(c.continuation);
        io.number(c.coarse_fine);
        io.number(c.continuous_certificates);
        io.number(c.adaptive_lookahead);
        io.number(c.adaptive_grid);
        io.number(c.certificate_resolution);
        if(modern)io.number(c.restart_clearance);
        if(teleop){io.number(c.intent_guard);io.number(c.smooth_speed);}
        if(preview)io.number(c.response_preview);
        if(direct)io.number(c.certified_direct);
        if(model){io.number(c.model_velocity_shaping);io.number(c.proposal_jerk_limit);}
        if(response)io.number(c.proposal_response_time);
        if(history_capacity)io.number(c.history_max_observations);
        if(certificate_memory)io.number(c.retain_certified_volume);
        if(feedforward)io.number(c.proposal_feedforward);
        if(command_accel)io.number(c.proposal_command_accel);
        if(local_repair)io.number(c.local_history_repair);
        if(six_axis){io.number(c.dynamic_obstacles);io.number(c.incremental_field);io.number(c.spherical_memory);io.number(c.shared_obstacles);}
    }
    static void tracks(Archive& io,std::vector<ObstacleTrack>& tracks) {
        std::uint32_t count=tracks.size();io.number(count);if(count>4096)throw std::runtime_error("too many replay obstacle tracks");
        if(io.in)tracks.resize(count);
        for(auto& t:tracks) {
            io.eigen(t.point);io.eigen(t.velocity);io.number(t.stamp);io.number(t.radius);io.number(t.view);io.number(t.observations);
            if(!t.point.allFinite()||!t.velocity.allFinite()||!std::isfinite(t.stamp)||!std::isfinite(t.radius)||t.radius<0||t.observations<1)
                throw std::runtime_error("invalid replay obstacle track");
        }
    }
    static void packet(Archive& io,SharedObstaclePacket& p) {
        io.string(p.source);io.string(p.session);io.string(p.frame);io.number(p.sequence);io.number(p.stamp);io.number(p.overloaded);tracks(io,p.tracks);
    }
    static void sixState(Archive& io,PaperGuidance& p) {
        auto values=p.dynamic_.tracks();tracks(io,values);bool overload=p.dynamic_.overloaded();io.number(overload);
        if(io.in)p.dynamic_.restore(values,overload);
        std::uint32_t n=p.spherical_.slots().size();io.number(n);if(n>48)throw std::runtime_error("oversized spherical cache");
        if(io.in)for(std::uint32_t i=0;i<n;++i) {
            int bin;io.number(bin);if(bin<0||bin>=288)throw std::runtime_error("invalid spherical bin");
            auto o=std::make_shared<DepthObservation>(Camera(),std::vector<double>(),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),0);
            io.observation(*o);p.spherical_.restore(bin,o);
        } else for(const auto& slot:p.spherical_.slots()) {int bin=slot.first;io.number(bin);auto o=*slot.second;io.observation(o);}
        n=p.shared_.sources().size();io.number(n);if(n>8)throw std::runtime_error("too many shared robots");
        if(io.in)for(std::uint32_t i=0;i<n;++i){SharedObstaclePacket packet_value;packet(io,packet_value);p.shared_.restore(packet_value);}
        else for(const auto& source:p.shared_.sources()){auto value=source.second;packet(io,value);}
    }
    static void state(Archive& io,PaperGuidance& p,bool modern=true,bool reference_intent=true,bool recovery=true,bool acceleration=true,bool feedforward=true) {
        std::uint32_t n=static_cast<std::uint32_t>(p.history_.size());io.number(n);
        if(n>256)throw std::runtime_error("too many historical views");
        if(io.in)p.history_.clear();
        for(std::uint32_t i=0;i<n;++i) {
            if(io.in) {
                auto o=std::make_shared<DepthObservation>(Camera(),std::vector<double>(),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),0);
                io.observation(*o);p.history_.push_back(o);
            } else io.observation(const_cast<DepthObservation&>(*p.history_[i]));
        }
        io.balls(p.seeds_);io.balls(p.verified_travel_);io.balls(p.pending_motion_);
        if(modern)io.balls(p.reached_proofs_);
        n=static_cast<std::uint32_t>(p.verified_tubes_.size());io.number(n);
        if(n>256)throw std::runtime_error("too many tubes");
        if(io.in)p.verified_tubes_.resize(n);
        for(auto& tube:p.verified_tubes_){io.eigen(tube.a);io.eigen(tube.b);io.number(tube.radius);}
        io.eigen(p.last_reached_);io.number(p.last_reached_time_);io.number(p.seed_initialized_);
        io.number(p.revoked_tubes_);io.number(p.revoked_balls_);io.number(p.expired_views_);io.number(p.evicted_views_);
        io.number(p.maximum_clearance_);io.string(p.build_reason_);io.number(p.required_prefix_);
        io.number(p.free_directions_);io.number(p.grid_refined_);io.number(p.target_holding_);
        io.eigen(p.diagnostic_state_);io.eigen(p.diagnostic_rate_);io.number(p.diagnostic_time_);io.number(p.diagnostic_valid_);
        io.eigen(p.previous_command_);io.eigen(p.previous_intent_);
        if(reference_intent)io.eigen(p.reference_intent_);
        else if(io.in)p.reference_intent_=p.previous_intent_; // Legacy best available baseline.
        io.number(p.last_update_);io.number(p.coupling_error_);io.number(p.field_lookahead_);
        auto& f=p.field_;io.number(f.width);io.number(f.height);io.eigen(f.offset);io.number(f.spacing);
        io.numbers(f.blocked);io.numbers(f.potential);io.numbers(f.boundary_extension);
        io.eigen(f.source);io.eigen(f.goal);io.number(f.valid);io.number(f.residual);
        if(f.width<0||f.height<0||f.width>1024||f.height>1024||
            f.blocked.size()!=static_cast<std::size_t>(f.width*f.height))throw std::runtime_error("invalid replay field");
        auto& b=p.box_;io.eigen(b.anchor);io.eigen(b.tangent);io.number(b.section_value);io.number(b.valid);
        bool has=bool(p.observation_);io.number(has);
        if(io.in) {
            p.observation_.reset();
            if(has)p.observation_.reset(new DepthObservation(Camera(),{},Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),0));
        }
        if(has)io.observation(*p.observation_);
        n=static_cast<std::uint32_t>(p.reference_world_.size());io.number(n);
        if(n>20000)throw std::runtime_error("too many reference points");
        if(io.in)p.reference_world_.resize(n);
        for(auto& point:p.reference_world_)io.eigen(point);
        if(recovery){io.number(p.recovery_cursor_);if(p.recovery_cursor_<0||p.recovery_cursor_>147)throw std::runtime_error("invalid recovery cursor");}
        if(acceleration)io.eigen(p.proposal_acceleration_);
        if(feedforward){io.eigen(p.proposal_reference_);io.number(p.proposal_reference_valid_);}
    }
};
void PaperGuidance::saveReplay(std::ostream& out,const DepthObservation& observation,
    const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,double now,double dt,const Eigen::Vector3d* proposal) const {
    out.write("EGOPAPRM",8);Archive io(out);
    // Output archives never mutate any scalar or collection.
    PaperReplayCodec::config(io,const_cast<PaperConfig&>(cfg_));
    PaperReplayCodec::state(io,const_cast<PaperGuidance&>(*this));
    PaperReplayCodec::sixState(io,const_cast<PaperGuidance&>(*this));
    io.eigen(const_cast<Eigen::Vector3d&>(field_position_));
    double capacity_until=shared_.capacityBlockedUntil();io.number(capacity_until);
    auto o=observation;io.observation(o);
    auto p=position,v=velocity,q=intent;io.eigen(p);io.eigen(v);io.eigen(q);io.number(now);io.number(dt);
    bool proposed=proposal!=nullptr;io.number(proposed);if(proposed){auto value=*proposal;io.eigen(value);}
}
PaperResult PaperGuidance::replay(std::istream& in,std::ostream* audit,bool enable_incremental_for_ablation) {
    char magic[8]={};in.read(magic,8);
    const std::string version(magic,8);
    if(version!="EGOPAPR3"&&version!="EGOPAPR4"&&version!="EGOPAPR5"&&version!="EGOPAPR6"&&version!="EGOPAPR7"&&version!="EGOPAPR8"&&version!="EGOPAPR9"&&version!="EGOPAPRA"&&version!="EGOPAPRB"&&version!="EGOPAPRC"&&version!="EGOPAPRD"&&version!="EGOPAPRE"&&version!="EGOPAPRF"&&version!="EGOPAPRG"&&version!="EGOPAPRH"&&version!="EGOPAPRI"&&version!="EGOPAPRJ"&&version!="EGOPAPRK"&&version!="EGOPAPRL"&&version!="EGOPAPRM")throw std::runtime_error("unsupported replay format");
    Archive io(in);io.memory_metadata=version>="EGOPAPRK";const bool modern=version!="EGOPAPR3";
    PaperConfig config;PaperReplayCodec::config(io,config,modern,version>="EGOPAPR6",version>="EGOPAPR7",version>="EGOPAPR8",version>="EGOPAPRC",version>="EGOPAPRD",version>="EGOPAPRE",version>="EGOPAPRF",version>="EGOPAPRG",version>="EGOPAPRH",version>="EGOPAPRI",version>="EGOPAPRJ");
    if(enable_incremental_for_ablation)config.incremental_field=true;
    PaperGuidance p(config);
    PaperReplayCodec::state(io,p,modern,version>="EGOPAPR5",version>="EGOPAPR9",version>="EGOPAPRB",version>="EGOPAPRG");
    if(version>="EGOPAPRJ")PaperReplayCodec::sixState(io,p);
    if(version>="EGOPAPRL")io.eigen(p.field_position_);
    else if(p.observation_)p.field_position_=p.observation_->origin;
    if(version>="EGOPAPRM") {
        double capacity_until;io.number(capacity_until);
        if(std::isnan(capacity_until)||capacity_until==std::numeric_limits<double>::infinity())
            throw std::runtime_error("invalid shared capacity horizon");
        p.shared_.restoreCapacityBlockedUntil(capacity_until);
    }
    DepthObservation o(Camera(),{},Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),0);io.observation(o);
    Eigen::Vector3d position,velocity,intent;double now,dt;
    io.eigen(position);io.eigen(velocity);io.eigen(intent);io.number(now);io.number(dt);
    bool proposed=false;Eigen::Vector3d proposal=Eigen::Vector3d::Zero();
    if(version>="EGOPAPRA"){io.number(proposed);if(proposed)io.eigen(proposal);}
    if(audit) {
        DepthObservation evidence=o;
        evidence.supporting_views.insert(evidence.supporting_views.end(),p.history_.begin(),p.history_.end());
        *audit<<"time "<<now<<" position "<<position.transpose()<<" velocity "<<velocity.transpose()
              <<" intent "<<intent.transpose()<<" envelope "<<p.envelopeRadius()<<'\n';
        for(double margin:{0.,.005,.01,.02,.03})
            *audit<<"known_body_plus "<<margin<<' '<<p.envelopeKnown(evidence,position,p.envelopeRadius()+margin,now)<<'\n';
        *audit<<"evidence history "<<p.history_.size()<<" current "<<o.supporting_views.size()
              <<" pending "<<p.pending_motion_.size()<<" reached "<<p.reached_proofs_.size()
              <<" previous_command "<<p.previous_command_.transpose()<<" proposal "<<proposal.transpose()<<'\n';
        for(const auto& view:evidence.supporting_views)
            *audit<<"view "<<view->view<<" stamp "<<view->stamp<<" revoked "<<view->revoked<<" origin "<<view->origin.transpose()<<'\n';
        for(double scale:{0.,.5,1.}) {
            const Eigen::Vector3d command=scale*p.previous_command_;
            const bool current=p.motionSafe(evidence,position,velocity,command,now);
            const auto size=p.reached_proofs_.size();
            p.reached_proofs_.insert(p.reached_proofs_.end(),p.pending_motion_.begin(),p.pending_motion_.end());
            const bool saved=p.motionSafe(evidence,position,velocity,command,now);
            p.reached_proofs_.resize(size);
            *audit<<"motion_scale "<<scale<<" current "<<current<<" saved_certificate "<<saved<<'\n';
        }
        // Diagnostic samples locate missing evidence; they never authorize motion.
        if(intent.norm()>1e-9)for(double advance:{.01,.10,.30}) {
            const Eigen::Vector3d center=position+advance*intent.normalized();
            int missing=0,total=0;
            for(int latitude=-90;latitude<=90;latitude+=15)for(int longitude=0;longitude<360;longitude+=15) {
                const double a=latitude*std::acos(-1.)/180.,b=longitude*std::acos(-1.)/180.;
                const Eigen::Vector3d offset=p.envelopeRadius()*Eigen::Vector3d(std::cos(a)*std::cos(b),std::cos(a)*std::sin(b),std::sin(a));
                ++total;
                if(!p.envelopeKnown(evidence,center+offset,0.,now)) {
                    if(missing<3)*audit<<"uncovered_surface "<<advance<<" offset "<<offset.transpose()<<'\n';
                    ++missing;
                }
            }
            *audit<<"surface_sample_missing "<<advance<<' '<<missing<<'/'<<total<<'\n';
        }
        *audit<<"yaw_deg,pitch_deg,intent_cosine,prefix_m\n";
        const double radians=std::acos(-1.)/180.;
        for(int pitch:{-60,-30,0,30,60})for(int yaw=-180;yaw<180;yaw+=15) {
            Eigen::Vector3d ray(std::cos(pitch*radians)*std::cos(yaw*radians),
                std::cos(pitch*radians)*std::sin(yaw*radians),std::sin(pitch*radians));
            *audit<<yaw<<','<<pitch<<','<<(intent.norm()>0?ray.dot(intent.normalized()):0.)<<','
                <<p.directionalClearance(evidence,position,ray,now,.5)<<'\n';
        }
    }
    return proposed?p.stepWithProposal(o,position,velocity,intent,proposal,now,dt):p.step(o,position,velocity,intent,now,dt);
}
} }
