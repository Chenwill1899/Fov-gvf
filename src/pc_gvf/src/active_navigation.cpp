#include "pc_gvf/paper_guidance.hpp"
#include "pc_gvf/proof_query_memo.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace pc_gvf { namespace depth_angular {
namespace {
using V = Eigen::Vector3d;
constexpr double pi=3.14159265358979323846;
double segmentDistance(const V& p,const V& a,const V& b) {
    const V d=b-a;
    return (p-a-(d.squaredNorm()>1e-16?std::max(0.,std::min(1.,(p-a).dot(d)/d.squaredNorm())):0.)*d).norm();
}
}
std::vector<CertifiedTube> PaperGuidance::executionPath(const V& position,const V& velocity,
    const V& command,bool emergency) const {
    std::vector<CertifiedTube> path;
    if(!position.allFinite()||!velocity.allFinite()||!command.allFinite())return path;
    const double dt=cfg_.execution_dt;
    V p=position,v=velocity;
    const double transition=cfg_.motion.delay+(emergency?0.:cfg_.motion.rollout_horizon);
    const int n=std::min(10000,static_cast<int>(std::ceil((transition+
        std::max({v.norm(),command.norm(),previous_command_.norm()})/
        std::min(cfg_.motion.brake_accel,cfg_.motion.max_accel)+1.)/dt)));
    for(int k=0;k<n;++k) {
        const double t=k*dt;
        // Same update as manual_control_math.velocity_response + p += dt*v_new.
        // Transport delay holds the last published target, rather than freezing velocity.
        const V target=t<cfg_.motion.delay?previous_command_:(t<transition?command:V::Zero().eval());
        V delta=target-v;
        const bool braking=target.norm()<1e-8;
        const double scale=braking?1.:std::min(1.,dt/cfg_.motion.velocity_tau);
        delta=clampNorm(scale*delta,(braking?cfg_.motion.brake_accel:cfg_.motion.max_accel)*dt);
        v+=delta;
        const V next=p+dt*v;
        path.push_back({p,next,envelopeRadius()+cfg_.restart_clearance});
        p=next;
        if(t>=transition&&v.norm()<1e-8)return path;
    }
    return {}; // An unbounded/non-stopping rollout cannot authorize execution.
}
void PaperGuidance::cancelActiveNavigation() {
    active_.assist_active=false;active_.previous_intent.setZero();
    active_.certified_stamp=-1.;
    active_.paused=true;
    // Keep anchor, accounting and recovery requirement across release/restart.
    confirmPublishedCommand(V::Zero());
}
void PaperGuidance::confirmExecutionOverride(const DepthObservation& input,const V& position,
    const V& velocity,const V& prior_published,const V& command,double now) {
    // The previous interval was charged before solving. Do not undo that debt
    // or the episode's initial volume; classify only the NEXT executed stop.
    if(active_.episode_active&&command.isZero(1e-10)) {
        bool known=false;
        if(position.allFinite()&&velocity.allFinite()&&prior_published.allFinite()&&
           now>=input.stamp-.02&&now-input.stamp<=cfg_.observation_timeout) {
            DepthObservation evidence=input;
            evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
            previous_command_=prior_published;
            const auto stop=executionPath(position,velocity,V::Zero(),true);
            known=!stop.empty();
            for(const auto& segment:stop)
                if(!segmentKnown(evidence,segment.a,segment.b,envelopeRadius(),now)){known=false;break;}
        }
        active_.last_unknown=!known;
    }
    confirmPublishedCommand(command);
}
bool PaperGuidance::episodeObservationProgress(const DepthObservation& evidence,
    const V& position,double now) const {
    if(active_.observed_targets!=0)return true;
    if(!active_.critical_targets.empty()||!active_.episode_origin_valid||
       (position-active_.episode_origin).norm()<=.2)return false;
    // Sparse surface witnesses can all be known although the continuous
    // restart shell is unresolved. In that case an empty witness list cannot
    // ever set observed_targets. Require a stronger replacement: NEW measured
    // views alone must cover the entire reliable interior footprint. No seed,
    // reached/predicted tube, old keyframe or merely changed timestamp grants it.
    DepthObservation fresh=evidence;
    fresh.revoked=true;fresh.supporting_views.clear();
    auto add=[&](const DepthObservation& view) {
        if(!view.revoked&&view.stamp>active_.episode_stamp+.02&&
           now>=view.stamp-.02&&
           std::none_of(fresh.supporting_views.begin(),fresh.supporting_views.end(),[&](const auto& old) {
               return old->view==view.view&&old->stamp==view.stamp&&old->version==view.version;
           }))fresh.supporting_views.push_back(std::make_shared<DepthObservation>(view));
    };
    if(std::any_of(evidence.depth.begin(),evidence.depth.end(),[](double d){return std::isfinite(d)&&d>0.;}))add(evidence);
    for(const auto& view:evidence.supporting_views)if(view)add(*view);
    if(fresh.supporting_views.empty())return false;
    PaperGuidance measured_only(cfg_);
    return measured_only.envelopeKnown(fresh,position,envelopeRadius()+.04,now);
}
bool PaperGuidance::requestVerifiedRetreat(const DepthObservation& input,const V& position,double now) {
    if(!activeNavigationEnabled()||!position.allFinite()||now<input.stamp-.02||
       now-input.stamp>cfg_.observation_timeout)return false;
    DepthObservation evidence=input;
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    struct ReturnPoint {V position;double score;};
    std::vector<ReturnPoint> choices;
    auto consider=[&](const V& p) {
        const double d=(p-position).norm();
        if(d<.12||d>cfg_.probe_anchor_range||
           std::any_of(choices.begin(),choices.end(),[&](const auto& old){return (p-old.position).norm()<.01;}))return;
        // Back away far enough to leave a local nomination trap. A closer
        // reliable point is still usable when there is no longer return.
        choices.push_back({p,std::abs(d-std::min(1.5,cfg_.probe_anchor_range))});
    };
    if(active_.anchor_valid)consider(active_.anchor);
    for(auto it=verified_travel_.rbegin();it!=verified_travel_.rend();++it)
        if(it->second>=envelopeRadius()+.04)consider(it->first);
    for(const auto& seed:seeds_)consider(seed.first);
    std::stable_sort(choices.begin(),choices.end(),[](const auto& a,const auto& b){return a.score<b.score;});
    V destination=V::Zero();bool found=false;
    // Bound fresh proof work; saved footprint size only nominates a point.
    for(std::size_t i=0;i<std::min<std::size_t>(12,choices.size());++i)
        if(envelopeKnown(evidence,choices[i].position,envelopeRadius()+.04,now)) {
            destination=choices[i].position;found=true;break;
        }
    if(!found)return false;
    active_.anchor=destination;active_.anchor_valid=true;active_.anchor_stamp=input.stamp;
    active_.recovering=true;active_.verified_retreat=true;
    active_.certified_stamp=-1.;
    // Invalidate only nomination/shaping state. Preserve the actual published
    // target during its transport delay, every geometric proof and risk debt.
    const V published=previous_command_;
    reset();confirmPublishedCommand(published);
    return true;
}
bool PaperGuidance::activeNavigationRequested(const DepthObservation& o,const V& position,
    const V& intent,double now) {
    if(!activeNavigationEnabled())return false;
    const bool vertical=cfg_.assisted_vertical&&std::abs(intent.z())>.05&&intent.head<2>().norm()<.15;
    if(!vertical)active_.assist_active=false;
    if(vertical){active_.boundary_inward=0;return true;}
    // This mode selection grants no motion. Its body/trajectory checks run in
    // activeStep; do not repeat three expensive union proofs before a boundary
    // decision that is independent of them (the node also queries this mode).
    const double boundary_band=active_.boundary_inward?.35:.15;
    const int boundary=intent.head<2>().norm()<.15&&!active_.boundary_inward?0:
        (cfg_.free_space_ceiling_enabled&&position.z()+envelopeRadius()+boundary_band>cfg_.free_space_ceiling_z?-1:
        (cfg_.free_space_floor_enabled&&position.z()-envelopeRadius()-boundary_band<cfg_.free_space_floor_z?1:0));
    if(boundary){active_.boundary_inward=boundary;active_.recovering=true;return true;}
    DepthObservation evidence=o;
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    const bool known=envelopeKnown(evidence,position,envelopeRadius(),now);
    const bool restart_known=known&&envelopeKnown(evidence,position,envelopeRadius()+cfg_.restart_clearance,now);
    // Admission and reliable recovery destinations have different jobs:
    // retain the larger interior margin for an anchor, not the minimal
    // execution-start proof at a stopping rim.
    const bool interior=restart_known&&envelopeKnown(evidence,position,envelopeRadius()+.04,now);
    // A known body can still be parked at the outward nomination limit, with
    // its anchor equal to itself. Enter inward recovery BEFORE the generic
    // anchor-completion rule; otherwise that rule cancels it every frame.
    if(active_.boundary_inward) {
        active_.boundary_inward=0;
        // A completed boundary retreat must not return toward its old high
        // anchor. Only a freshly proved reliable interior can replace it;
        // the exploration origin and accumulated debt remain untouched.
        if(interior) {
            active_.anchor=position;active_.anchor_valid=true;active_.anchor_stamp=o.stamp;
            active_.recovering=false;active_.verified_retreat=false;
        }
    }
    // Finish a return only at its still-proved interior destination. Do
    // this before choosing the front-end, so a completed return cannot be
    // reclassified by the exploration comfort buffer every following frame.
    if(active_.recovering&&interior&&active_.anchor_valid&&
       (position-active_.anchor).norm()<.1){active_.recovering=false;active_.verified_retreat=false;}
    if(active_.recovering)return true;
    if(interior&&!active_.episode_active) {
        active_.anchor=position;active_.anchor_stamp=o.stamp;active_.anchor_valid=true;
    }
    if(active_.episode_active&&restart_known) {
        // Observation progress must also be collected during certified normal
        // flight, otherwise the first new view after handoff would be lost.
        for(const auto& view:o.supporting_views)
            if(view&&!view->revoked&&view->stamp>(active_.episode_origin_valid?active_.episode_stamp:active_.anchor_stamp)+.02&&
               now>=view->stamp-.02&&now-view->stamp<=cfg_.observation_timeout)
                for(std::size_t i=0;i<active_.critical_targets.size();++i)
                    if(observedEnvelope(*view,active_.critical_targets[i],.025))
                        active_.observed_targets|=std::uint64_t(1)<<i;
        // Re-enter the ordinary obstacle-avoidance front-end once the reached
        // body has an interior proof. Keep the episode and its cumulative
        // budgets; known motion does not replenish an exploration allowance.
        // This predicate is queried more than once per frame. Charge only
        // measured translation/time since the last distinct state update.
        if(now<active_.last_time)return true;
        if(active_.last_time>=0.&&now>active_.last_time) {
            const double travel=(position-active_.last_position).norm();
            if(active_.last_unknown||!segmentKnown(evidence,active_.last_position,position,envelopeRadius(),now)) {
                active_.unknown_distance+=travel;
                active_.unknown_volume+=pi*envelopeRadius()*envelopeRadius()*travel;
                active_.unknown_exposure+=now-active_.last_time;
                if(!active_.paused)active_.unknown_time+=now-active_.last_time;
            }
        }
        active_.last_position=position;active_.last_time=now;
        active_.last_unknown=false;active_.paused=false;
        if(interior&&active_.episode_origin_valid&&
           (position-active_.episode_origin).norm()>.2&&episodeObservationProgress(evidence,position,now)) {
            active_.anchor=position;active_.anchor_stamp=o.stamp;
            active_.episode_active=false;active_.recovering=false;active_.verified_retreat=false;active_.episode_origin_valid=false;
            active_.unknown_distance=active_.unknown_time=active_.unknown_volume=0.;
            active_.unknown_exposure=0.;active_.unknown_volume_started=false;
            active_.critical_targets.clear();active_.observed_targets=0;
        }
        // Certified known-space translation may move the recovery destination
        // without renewing the still-open episode. Keep its observation epoch
        // until new measured information closes that episode above.
        if(interior) {
            active_.anchor=position;active_.anchor_valid=true;
            if(!active_.episode_active)active_.anchor_stamp=o.stamp;
        }
        return false;
    }
    // The executor's first swept segment includes restart clearance. An
    // unproved initial execution ball makes every ordinary nominal command
    // inadmissible, even when the smaller present body is known. Assess the
    // bounded recovery permit before rebuilding an impossible angular field.
    return active_.episode_active||!(cfg_.execution_consistent?restart_known:known);
}
PaperResult PaperGuidance::activeStep(const DepthObservation& input,const V& position,
    const V& velocity,const V& intent,double now,double dt) {
    PaperResult out;
    bool reused_nomination=false;
    auto finish=[&](const std::string& status) {
        out.status=status;out.probe_episode=active_.episode;
        out.unknown_distance=active_.unknown_distance;out.unknown_time=active_.unknown_time;
        out.unknown_volume=active_.unknown_volume;out.recovery_anchor=active_.anchor;
        out.unknown_exposure=active_.unknown_exposure;
        if(!out.accepted){out.navigation_mode="BRAKE";out.command.setZero();}
        if(out.accepted&&out.authorization=="CERTIFIED"&&out.command.norm()>1e-5&&
           !reused_nomination&&intent.head<2>().norm()>=.15) {
            active_.certified_command=out.command;active_.certified_position=position;
            active_.certified_intent=intent.normalized();active_.certified_anchor=active_.anchor;
            active_.certified_stamp=now;active_.certified_recovering=active_.recovering;
            active_.certified_boundary_inward=active_.boundary_inward;
        }
        confirmPublishedCommand(out.command);cache_enabled_=false;return out;
    };
    if(!position.allFinite()||!velocity.allFinite()||!intent.allFinite()||
       !std::isfinite(now)||!std::isfinite(dt)||dt<=0.)return finish("ACTIVE_INVALID_STATE");
    if(intent.norm()<1e-5){cancelActiveNavigation();return finish("ZERO_INTENT");}
    if(now<input.stamp-.02||now-input.stamp>cfg_.observation_timeout) {
        cancelActiveNavigation();return finish("ACTIVE_STALE_DEPTH");
    }
    DepthObservation evidence=input;
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    for(auto& c:envelope_caches_)c.clear();
    for(auto& policy:segment_proof_memos_)for(auto& memo:policy)memo.clear();
    cache_enabled_=true;
    const double radius=envelopeRadius(), area=pi*radius*radius;
    const bool body_known=envelopeKnown(evidence,position,radius,now);
    rememberVerifiedPosition(evidence,position,now);
    // Charge measured unknown motion, not each receding-horizon prediction.
    // A cancelled/held stick never renews this episode's budget.
    if(active_.episode_active&&active_.last_time>=0.) {
        if(now<active_.last_time)return finish("ACTIVE_CLOCK_REVERSED");
        const double travel=(position-active_.last_position).norm();
        if(active_.last_unknown||!segmentKnown(evidence,active_.last_position,position,radius,now)) {
            active_.unknown_distance+=travel;
            active_.unknown_volume+=area*travel;
            active_.unknown_exposure+=now-active_.last_time;
            if(!active_.paused)active_.unknown_time+=now-active_.last_time;
        }
    }
    active_.last_position=position;active_.last_time=now;
    active_.paused=false;
    active_.last_unknown=!body_known;
    // An anchor must still be provable. Native-hit revocation applies as usual.
    if(active_.anchor_valid&&!envelopeKnown(evidence,active_.anchor,radius+.04,now))active_.anchor_valid=false;
    const bool interior=envelopeKnown(evidence,position,radius+.04,now);
    const double inward_z=active_.boundary_inward?double(active_.boundary_inward):
        (cfg_.free_space_ceiling_enabled&&position.z()+radius+.15>cfg_.free_space_ceiling_z?-1.:
        (cfg_.free_space_floor_enabled&&position.z()-radius-.15<cfg_.free_space_floor_z?1.:0.));
    if(inward_z!=0.&&!interior)active_.recovering=true;
    // Replenishment requires actual translation AND new full-body evidence with
    // interior margin; frame versions/timestamps alone do not count as information.
    bool fresh=false;
    for(const auto& view:input.supporting_views)
        if(view&&!view->revoked&&view->stamp>(active_.episode_origin_valid?active_.episode_stamp:active_.anchor_stamp)+.02&&
           now>=view->stamp-.02&&now-view->stamp<=cfg_.observation_timeout)
            for(std::size_t i=0;i<active_.critical_targets.size();++i)
                if(!(active_.observed_targets&(std::uint64_t(1)<<i))&&
                   observedEnvelope(*view,active_.critical_targets[i],.025)) {
                    fresh=true;active_.observed_targets|=std::uint64_t(1)<<i;
                }
    if(interior&&(!active_.anchor_valid||(!active_.episode_active&&!active_.recovering)||
       (active_.episode_origin_valid&&(position-active_.episode_origin).norm()>.2&&episodeObservationProgress(evidence,position,now)))) {
        active_.anchor=position;active_.anchor_stamp=input.stamp;active_.anchor_valid=true;
        if(active_.episode_active&&active_.episode_origin_valid&&
           (position-active_.episode_origin).norm()>.2&&episodeObservationProgress(evidence,position,now)) {
            active_.episode_active=false;active_.recovering=false;active_.verified_retreat=false;active_.episode_origin_valid=false;
            active_.unknown_distance=active_.unknown_time=active_.unknown_volume=0.;
            active_.unknown_exposure=0.;
            active_.unknown_volume_started=false;
            active_.critical_targets.clear();
            active_.observed_targets=0;
        }
    }
    if(!active_.anchor_valid) {
        double best=cfg_.probe_anchor_range;
        for(const auto& seed:seeds_) {
            const double d=(position-seed.first).norm();
            if(d<best&&envelopeKnown(evidence,seed.first,radius+.04,now)) {
                best=d;active_.anchor=seed.first;active_.anchor_valid=true;active_.anchor_stamp=input.stamp;
            }
        }
    }
    // A sub-millimetre direct anchor can retain the body but lose the
    // interior room required to restart after a new view. Pick another still
    // verified reached footprint; changing the destination never replenishes
    // this episode's distance/time/volume budgets or observation progress.
    if(!active_.anchor_valid||(active_.recovering&&
       ((position-active_.anchor).norm()<.1&&!interior))||
       (active_.anchor_valid&&(position-active_.anchor).norm()>cfg_.probe_anchor_range)) {
        double best=cfg_.probe_anchor_range;
        V replacement=V::Zero();bool found=false;
        auto consider=[&](const std::pair<V,double>& ball) {
            const double distance=(position-ball.first).norm();
            if(distance<.12||distance>=best)return;
            if(!envelopeKnown(evidence,ball.first,radius+.04,now))return;
            best=distance;replacement=ball.first;found=true;
        };
        for(const auto& ball:verified_travel_)consider(ball);
        for(const auto& ball:seeds_)consider(ball);
        if(found){active_.anchor=replacement;active_.anchor_valid=true;active_.anchor_stamp=input.stamp;}
    }
    if(!active_.recovering&&intent.head<2>().norm()>=.15&&velocity.norm()>.03&&velocity.dot(intent)<-1e-6)
        return finish("INTENT_REVERSAL_BRAKE");
    const bool vertical=cfg_.assisted_vertical&&std::abs(intent.z())>.05&&intent.head<2>().norm()<.15;
    if(!vertical)active_.assist_active=false; // explicit horizontal input takes over
    if(vertical&&(!active_.assist_active||active_.previous_intent.z()*intent.z()<=0.)) {
        active_.assist_active=true;active_.center=position+V(cfg_.assist_radius,0,0);
        active_.progress_position=position;active_.progress_time=now;
    }
    if(vertical&&(std::abs(position.z()-active_.progress_position.z())>.08||fresh)) {
        active_.progress_position=position;active_.progress_time=now;
    }
    if(inward_z!=0.&&!vertical)active_.recovering=true;
    else if(active_.recovering&&interior&&(position-active_.anchor).norm()<.1){active_.recovering=false;active_.verified_retreat=false;}
    active_.previous_intent=intent;reference_intent_=intent.normalized();

    // A complete fresh motion/emergency certificate already covers every
    // swept body. Horizontal certified motion does not need the occupied
    // risk-scoring grid or speculative visibility scores. Keep an independent
    // raw-hit current-body guard before this fast path; native obstacle hits
    // inside the actual envelope always veto it, including contradictory data.
    if(active_.boundary_inward)active_.recovering=true;
    if(!vertical&&body_known) {
        bool occupied_body=false;
        auto conflict=[&](const DepthObservation& view) {
            if(view.revoked||!view.obstacle_points||now<view.stamp-.02||
               (!(cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.)&&
                now-view.stamp>std::max(cfg_.observation_timeout,cfg_.history_duration)))return false;
            const V local=view.rotation.transpose()*(position-view.origin);
            const bool rigid=(view.rotation.transpose()*view.rotation-Eigen::Matrix3d::Identity()).norm()<1e-10;
            for(const auto& raw:*view.obstacle_points) {
                if(rigid&&(raw-local).squaredNorm()>(radius+1e-8)*(radius+1e-8))continue;
                if((view.origin+view.rotation*raw-position).norm()<=radius)return true;
            }
            return false;
        };
        occupied_body=conflict(input);
        for(const auto& view:evidence.supporting_views)
            if(view&&conflict(*view)){occupied_body=true;break;}
        if(occupied_body)return finish("ACTIVE_KNOWN_OBSTACLE");
        if(active_.certified_boundary_inward==active_.boundary_inward&&active_.certified_stamp>=0.&&
           now>=active_.certified_stamp&&now-active_.certified_stamp<=.3&&
           (position-active_.certified_position).norm()<=.75&&
           active_.certified_recovering==active_.recovering&&
           (!active_.recovering||active_.boundary_inward||(active_.anchor-active_.certified_anchor).norm()<.1)&&
           (!active_.boundary_inward||active_.certified_command.z()*active_.boundary_inward>=0.)&&
           active_.certified_intent.dot(intent.normalized())>=std::cos(cfg_.command_change_angle)) {
            std::vector<std::pair<V,double>> prefix;
            if(certifyAndRetainMotion(evidence,position,velocity,active_.certified_command,now,&prefix)) {
                pending_motion_=std::move(prefix);out.command=active_.certified_command;out.accepted=true;
                out.authorization="CERTIFIED";out.navigation_mode=active_.recovering?"RECOVER":"NORMAL";
                for(const auto& segment:executionPath(position,velocity,out.command))out.preview_path.push_back(segment.b);
                reused_nomination=true;
                return finish("ACTIVE_CACHED_CERTIFIED");
            }
            active_.certified_stamp=-1.;
        }
        if(active_.recovering&&inward_z!=0.) {
            V lateral=active_.anchor_valid?(active_.anchor-position).eval():(-intent).eval();
            lateral.z()=0.;
            if(lateral.norm()>.01)lateral=.12*lateral.normalized();
            else lateral.setZero();
            // Before re-entering the old forward target, regain vertical room
            // inside the finite workspace. This is a complete motion/brake
            // certificate, and may retreat against the operator's direction.
            for(double sideways:{1.,0.})for(double scale:{1.,.5,.25}) {
                V command=scale*(sideways*lateral+V(0,0,inward_z*.24));
                command=clampNorm(command,cfg_.max_speed);
                if(std::abs(command.z())>cfg_.max_vertical_speed)command*=cfg_.max_vertical_speed/std::abs(command.z());
                std::vector<std::pair<V,double>> prefix;
                if(!certifyAndRetainMotion(evidence,position,velocity,command,now,&prefix))continue;
                pending_motion_=std::move(prefix);out.command=command;out.accepted=true;
                out.authorization="CERTIFIED";out.navigation_mode="RECOVER";
                for(const auto& s:executionPath(position,velocity,command))out.preview_path.push_back(s.b);
                // Drop the nomination that drove into this boundary. Keep
                // geometry, reached proofs and cumulative exploration state;
                // finish() below confirms the actually selected command.
                reset();
                return finish("ACTIVE_CERTIFIED_INWARD");
            }
        }
        std::vector<V> strict;
        auto reenter=[&]() {
            // A fresh, fully certified short forward motion can hand execution
            // back to the normal policy without finishing a needless return.
            // This never grants unknown execution or renews an episode budget.
            const V reentry=std::min(cfg_.probe_speed,intent.norm())*intent.normalized();
            if(active_.boundary_inward&&reentry.z()*active_.boundary_inward<0.)return false;
            std::vector<std::pair<V,double>> reentry_prefix;
            if(certifyAndRetainMotion(evidence,position,velocity,reentry,now,&reentry_prefix)) {
                active_.recovering=false;active_.verified_retreat=false;pending_motion_=std::move(reentry_prefix);
                out.command=reentry;out.accepted=true;out.authorization="CERTIFIED";out.navigation_mode="NORMAL";
                for(const auto& segment:executionPath(position,velocity,reentry))out.preview_path.push_back(segment.b);
                return true;
            }
            return false;
        };
        if(active_.recovering) {
            if(!active_.verified_retreat&&reenter())return finish("ACTIVE_CERTIFIED_REENTRY");
            if(active_.anchor_valid&&(active_.anchor-position).norm()>.002) {
                const V delta=active_.anchor-position;
                const V command=std::min(cfg_.probe_speed,delta.norm()/std::max(.4,4*cfg_.motion.velocity_tau))*delta.normalized();
                for(double scale:{1.,.5,.25,.125})strict.push_back(scale*command);
            }
        } else for(double scale:{1.,.5,.25,.125})strict.push_back(scale*clampNorm(intent,cfg_.max_speed));
        for(auto command:strict) {
            if(active_.boundary_inward&&command.z()*active_.boundary_inward<0.)continue;
            if(std::abs(command.z())>cfg_.max_vertical_speed)command*=cfg_.max_vertical_speed/std::abs(command.z());
            std::vector<std::pair<V,double>> prefix;
            if(!certifyAndRetainMotion(evidence,position,velocity,command,now,&prefix))continue;
            pending_motion_=std::move(prefix);out.command=command;out.accepted=true;
            out.authorization="CERTIFIED";out.navigation_mode=active_.recovering?"RECOVER":"NORMAL";
            for(const auto& segment:executionPath(position,velocity,command))out.preview_path.push_back(segment.b);
            return finish(active_.recovering?"ACTIVE_CERTIFIED_RECOVER":"ACTIVE_CERTIFIED");
        }
        // A direct anchor line can be unproved even though a nearby lateral
        // motion has a complete certificate. Search a bounded horizontal fan
        // before treating that local geometry as an unknown-space problem.
        // These are certified repositioning moves, never risk-budget renewal.
        if(active_.recovering&&active_.anchor_valid) {
            V base=active_.anchor-position;base.z()=0.;
            if(base.norm()<.02){base=-intent;base.z()=0.;}
            if(base.norm()>.02) {
                base.normalize();const V side(-base.y(),base.x(),0.);
                for(double scale:{1.,.5,.25,.125})for(double degrees:{30.,-30.,60.,-60.,90.,-90.,120.,-120.,180.}) {
                    const double angle=degrees*pi/180.;
                    const V command=scale*cfg_.probe_speed*(std::cos(angle)*base+std::sin(angle)*side);
                    const auto path=executionPath(position,velocity,command);
                    if(path.empty()||(path.back().b-active_.anchor).norm()>cfg_.probe_anchor_range)continue;
                    std::vector<std::pair<V,double>> prefix;
                    if(!certifyAndRetainMotion(evidence,position,velocity,command,now,&prefix))continue;
                    pending_motion_=std::move(prefix);out.command=command;out.accepted=true;
                    out.authorization="CERTIFIED";out.navigation_mode="RECOVER";
                    for(const auto& segment:path)out.preview_path.push_back(segment.b);
                    return finish("ACTIVE_CERTIFIED_SIDESTEP");
                }
            }
        }
        if(active_.recovering&&active_.verified_retreat&&reenter())return finish("ACTIVE_CERTIFIED_REENTRY");
    }
    // Build occupied evidence from real native hits (all pixels), plus retained
    // conservative depth surfaces. Zero/invalid depth is UNKNOWN, never occupied.
    // Local voxel representatives expand the collision radius by their cell diagonal.
    const double cell=.08, voxel_margin=std::sqrt(3.)*cell;
    struct HitCell {V representative;std::vector<V> points;};
    std::map<std::tuple<int,int,int>,HitCell> occupied;
    auto add=[&](const V& p) {
        if(p.allFinite()&&(p-position).norm()<cfg_.probe_anchor_range+radius+1.) {
            const auto key=std::make_tuple(int(std::floor(p.x()/cell)),int(std::floor(p.y()/cell)),int(std::floor(p.z()/cell)));
            auto found=occupied.find(key);
            if(found==occupied.end())found=occupied.emplace(key,HitCell{p,{}}).first;
            found->second.points.push_back(p);
        }
    };
    std::vector<const DepthObservation*> views;
    views.push_back(&input);
    for(const auto& o:evidence.supporting_views)if(o)views.push_back(o.get());
    for(const auto* o:views) {
        if(o->revoked||now<o->stamp-.02||
           (!(cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.)&&
            now-o->stamp>std::max(cfg_.observation_timeout,cfg_.history_duration)))continue;
        if(o->obstacle_points) {
            const V local=o->rotation.transpose()*(position-o->origin);
            const double reach=cfg_.probe_anchor_range+radius+1.;
            const bool rigid=(o->rotation.transpose()*o->rotation-Eigen::Matrix3d::Identity()).norm()<1e-10;
            for(const auto& p:*o->obstacle_points) {
                if(rigid&&(p-local).squaredNorm()>(reach+1e-8)*(reach+1e-8))continue;
                add(o->origin+o->rotation*p);
            }
        }
        // Current native hits are exact; resized depth can be a min-pooled proxy
        // at another ray. Use proxies only when raw hits are unavailable.
        else for(int y=0;y<o->camera.height();++y)for(int x=0;x<o->camera.width();++x) {
            const std::size_t i=y*o->camera.width()+x;
            if(i>=o->depth.size())continue;
            const double d=o->depth[i];const V ray=o->camera.ray(x,y);
            if(std::isfinite(d)&&d>0&&d<o->camera.maxDepth()-1e-6)
                add(o->origin+o->rotation*(d/ray.z()*ray));
        }
    }
    auto hardSafe=[&](const std::vector<CertifiedTube>& path,bool exit_buffer=false) {
        if(path.empty())return false;
        double t=0.;
        for(const auto& s:path) {
            if(!withinWorkingBounds(s.a,s.b,s.radius))return false;
            if(cfg_.free_space_floor_enabled&&std::min(s.a.z(),s.b.z())-s.radius<cfg_.free_space_floor_z)return false;
            if(cfg_.free_space_ceiling_enabled&&std::max(s.a.z(),s.b.z())+s.radius>cfg_.free_space_ceiling_z)return false;
            if(cfg_.dynamic_obstacles&&!dynamic_.segmentSafe(s.a,s.b,s.radius,now,t,t+cfg_.execution_dt))return false;
            if(cfg_.shared_obstacles&&!shared_.segmentSafe(s.a,s.b,s.radius,now,t,t+cfg_.execution_dt))return false;
            t+=cfg_.execution_dt;
        }
        const auto bounds=capsulePathBounds(path);
        const double comfort=exit_buffer?0.:cfg_.probe_clearance;
        for(const auto& entry:occupied) {
                // The representative is a broad phase only. A cell diagonal
                // covers all merged hits; resolve possible overlaps using all
                // original points rather than treating that bound as a wall.
                const auto& bucket=entry.second;
                if(!bounds.mayTouch(bucket.representative,comfort+voxel_margin))continue;
                for(const auto& point:bucket.points) {
                if(!bounds.mayTouch(point,comfort))continue;
                for(const auto& s:path) {
                const double clearance=segmentDistance(point,s.a,s.b);
                const double hard_radius=s.radius;
                if(clearance<=hard_radius)return false;
                // Ordinary probing retains the optional 8cm comfort buffer.
                // Bounded recovery uses no EXTRA comfort floor: the complete
                // hard radius above (including restart clearance) stays strict.
                // A 1cm additional floor blocked millimetre-clear recovery paths.
                if(!exit_buffer&&clearance<=hard_radius+cfg_.probe_clearance)return false;
                }
                }
            }
        return true;
    };
    if(!hardSafe({{position,position,radius}})) {
        if(!hardSafe({{position,position,radius}},true))return finish("ACTIVE_KNOWN_OBSTACLE");
        active_.recovering=true;
    }
    // Local accounting only; no observation or motion commitment changes here.
    CapsuleProofMemo accounting_proofs;
    auto pathUnknown=[&](const std::vector<CertifiedTube>& path) {
        if(path.empty())return std::make_pair(0.,false);
        const V a=path.front().a,b=path.back().b;
        double length=0.,radius=0.,bulge=0.;
        for(const auto& s:path) {
            length+=(s.b-s.a).norm();radius=std::max(radius,s.radius);
            bulge=std::max({bulge,segmentDistance(s.a,a,b),segmentDistance(s.b,a,b)});
        }
        // One enclosing capsule covers EVERY piece of the modeled curve.
        // A failed proof charges the entire path as unknown, a conservative
        // upper bound. Re-proving each 60Hz piece for every probe/return used
        // to consume the control deadline before a safe return was published.
        const double bound=radius+bulge+(bulge>0.?1e-10:0.);
        const bool known=accounting_proofs.evaluate(a,b,bound,
            [&]{return envelope_cache_revisions_;},
            [&]{return segmentKnown(evidence,a,b,bound,now);});
        return std::make_pair(known?0.:length,!known);
    };
    // Missing near-cap targets. Visibility is a proposal score, never free evidence.
    const double sign=intent.z()<0?-1.:1.;
    for(int i=0;i<9;++i) {
        const double angle=2*pi*i/8.;
        const V target=position+V(i? .35*std::cos(angle):0.,i?.35*std::sin(angle):0.,sign*(radius+.08));
        if(!envelopeKnown(evidence,target,.025,now))out.observation_targets.push_back(target);
    }
    if(!vertical) {
        out.observation_targets.clear();
        for(int x:{-1,0,1})for(int y:{-1,0,1})for(int z:{-1,0,1}) {
            const V d(x,y,z);if(d.norm()<.5)continue;
            const V target=position+(radius+.08)*d.normalized();
            if(!envelopeKnown(evidence,target,.025,now))out.observation_targets.push_back(target);
        }
    }
    auto visibility=[&](const V& displacement) {
        double visible=0.;
        for(const auto& target:out.observation_targets) {
            bool seen=false;
            for(const auto& view:input.supporting_views)if(view&&!view->revoked&&
                now-view->stamp<=cfg_.observation_timeout&&now>=view->stamp-.02) {
                const V origin=view->origin+displacement;
                const V ray=view->rotation.transpose()*(target-origin);Eigen::Vector2d uv;
                if(ray.z()<=.03||ray.z()>view->camera.maxDepth()||
                   !view->camera.pixelFromDirection(ray,&uv)||!view->camera.inside(uv,1.))continue;
                bool occluded=false;
                for(const auto& entry:occupied) {
                    const auto& bucket=entry.second;
                    if(segmentDistance(bucket.representative,origin,target)>=.12+voxel_margin)continue;
                    for(const auto& point:bucket.points)if((point-origin).norm()+.12<(target-origin).norm()&&
                        segmentDistance(point,origin,target)<.12){occluded=true;break;}
                    if(occluded)break;
                }
                if(!occluded){seen=true;break;}
            }
            visible+=seen;
        }
        return out.observation_targets.empty()?0.:visible/out.observation_targets.size();
    };
    struct Candidate { V command;bool recovery=false;double gain=0.,score=0.; };
    std::vector<Candidate> candidates;
    auto candidate=[&](V cmd,bool recovery=false) {
        cmd=clampNorm(cmd,cfg_.max_speed);
        if(std::abs(cmd.z())>cfg_.max_vertical_speed)cmd*=cfg_.max_vertical_speed/std::abs(cmd.z());
        const double look=std::max(2.,2*cfg_.assist_radius/std::max(.1,cfg_.assist_speed));
        const double gain=visibility(look*cmd);
        double progress=cmd.dot(intent.normalized());
        candidates.push_back({cmd,recovery,gain,progress+1.2*gain-.12*(cmd-previous_command_).norm()});
    };
    candidate(clampNorm(intent,cfg_.max_speed));
    if(!vertical)for(double scale:{.5,.25,.125})candidate(scale*clampNorm(intent,cfg_.max_speed));
    if(vertical) {
        V radial=position-active_.center;radial.z()=0.;
        if(radial.norm()<.05)radial=V(-1,0,0);else radial.normalize();
        const V tangent(-radial.y(),radial.x(),0.);
        for(double turn:{active_.turn_sign,-active_.turn_sign})for(double lift:{.5,0.}) {
            V direction=turn*tangent-.8*((position-active_.center).head<2>().norm()-cfg_.assist_radius)*radial;
            direction=clampNorm(direction,cfg_.assist_speed);direction.z()=sign*lift*std::min(std::abs(intent.z()),cfg_.max_vertical_speed);
            candidate(direction);
        }
        for(int i=0;i<8;++i) {
            const double a=2*pi*i/8.;
            candidate(V(cfg_.assist_speed*std::cos(a),cfg_.assist_speed*std::sin(a),sign*.2));
        }
    }
    // Recovery need not satisfy the original intent dot-product gate. It still
    // needs fresh observations, hard checks and the same risk budget.
    if(active_.anchor_valid&&(position-active_.anchor).norm()>.002) {
        const V delta=active_.anchor-position;
        candidate(std::min(cfg_.probe_speed,delta.norm()/std::max(.4,4*cfg_.motion.velocity_tau))*delta.normalized(),true);
        if(inward_z!=0.) {
            V lateral=delta;lateral.z()=0.;
            if(lateral.norm()<=.01){lateral=-intent;lateral.z()=0.;}
            if(lateral.norm()>.01)lateral.normalize();
            if(lateral.norm()>.01) {
                const V side(-lateral.y(),lateral.x(),0.);
                for(double degrees:{0.,30.,-30.,60.,-60.,90.,-90.}) {
                    const double angle=degrees*pi/180.;
                    candidate(cfg_.probe_speed*(std::cos(angle)*lateral+std::sin(angle)*side),true);
                }
            }
            for(double lift:{1.,.5})candidate(cfg_.probe_speed*
                (lateral+V(0,0,inward_z*lift)).normalized(),true);
            candidate(V(0,0,inward_z*cfg_.probe_vertical_speed),true);
        }
    }
    std::stable_sort(candidates.begin(),candidates.end(),[&](const Candidate& a,const Candidate& b){
        if(active_.boundary_inward) {
            const bool a_inward=a.command.z()*active_.boundary_inward>1e-6;
            const bool b_inward=b.command.z()*active_.boundary_inward>1e-6;
            if(a_inward!=b_inward)return a_inward;
        }
        return a.score>b.score;
    });
    // All strict candidates are tried before accepting any unknown execution.
    for(const auto& c:candidates) {
        if(active_.recovering&&!c.recovery)continue;
        if(active_.boundary_inward&&c.command.z()*active_.boundary_inward<0.)continue;
        if(c.recovery&&!active_.recovering&&body_known)continue;
        if(vertical&&!c.recovery&&c.command.z()*sign<.01&&c.gain<=0.)continue;
        // A safe lateral orbit that repeatedly promises visibility but supplies
        // no measured height/information progress must not starve exploration.
        if(vertical&&!c.recovery&&now-active_.progress_time>1.5&&c.command.z()*sign<.08)continue;
        std::vector<std::pair<V,double>> prefix;
        if(certifyAndRetainMotion(evidence,position,velocity,c.command,now,&prefix)) {
            pending_motion_=prefix;out.command=c.command;out.accepted=true;out.authorization="CERTIFIED";
            out.navigation_mode=c.recovery?"RECOVER":(vertical?"ASSIST":"NORMAL");
            out.expected_visibility=c.gain;
            for(const auto& s:executionPath(position,velocity,c.command))out.preview_path.push_back(s.b);
            return finish(c.recovery?"ACTIVE_CERTIFIED_RECOVER":(vertical?"ASSIST_CERTIFIED":"ACTIVE_CERTIFIED"));
        }
    }
    if(!cfg_.bounded_exploration)return finish("ACTIVE_NO_CERTIFIED_PATH");
    bool usable_sensor=false;
    for(const auto& view:input.supporting_views)if(view&&!view->revoked&&now>=view->stamp-.02&&
        now-view->stamp<=cfg_.observation_timeout)
        for(double d:view->depth)if(std::isfinite(d)&&d>0.){usable_sensor=true;break;}
    if(!usable_sensor)return finish("PROBE_NO_USABLE_DEPTH");
    if(!active_.anchor_valid)return finish("PROBE_NO_ANCHOR");
    const auto emergency=executionPath(position,velocity,V::Zero(),true);
    if(!hardSafe(emergency,active_.recovering)) {
        // The stop rollout includes restart clearance, so its extra comfort
        // shell may overlap while the present body shell does not. Recovery
        // still checks full rollout/body radii; only its optional extra buffer
        // is removed, never the hard restart envelope.
        if(!hardSafe(emergency,true))return finish("PROBE_UNSAFE_BRAKE");
        active_.recovering=true;
    }
    // Every projected unknown cost below is nonnegative. No risk candidate can
    // replenish an already exceeded measured budget. All fully certified paths
    // were still tried above; do not rebuild the same impossible risk accounting.
    if(active_.unknown_distance>cfg_.probe_distance_budget ||
       active_.unknown_time>cfg_.probe_time_budget ||
       active_.unknown_volume>cfg_.probe_volume_budget)
        return finish("PROBE_BUDGET_EXHAUSTED");
    // Projected costs reserve nominal motion, emergency brake and the direct
    // return corridor. Unknown initial body volume is charged once per episode.
    bool budget_rejected=false;
    for(auto c:candidates) {
        if(active_.recovering&&!c.recovery)continue;
        // A measured obstacle can block immediate inward descent/ascent while
        // the restart shell is unproved. Bounded horizontal parallax may clear
        // it, but no candidate may undo the boundary height retreat.
        if(active_.boundary_inward&&c.command.z()*active_.boundary_inward < -1e-8)continue;
        c.command=clampNorm(c.command,cfg_.probe_speed);
        // At an unobserved body cap, gain parallax before adding more height.
        // Otherwise a diagonal probe keeps moving its own blind cap upward.
        if(vertical&&!c.recovery)c.command.z()=0.;
        // First stop outward momentum before attempting the return direction.
        if(c.recovery&&velocity.dot(active_.anchor-position)<-1e-6)c.command.setZero();
        if(std::abs(c.command.z())>cfg_.probe_vertical_speed)c.command*=cfg_.probe_vertical_speed/std::abs(c.command.z());
        c.gain=visibility(std::max(2.,2*cfg_.assist_radius/std::max(.1,cfg_.assist_speed))*c.command);
        if(!c.recovery&&c.gain<=0.)continue;
        const auto path=executionPath(position,velocity,c.command);
        if(path.empty()||!hardSafe(path,c.recovery))continue;
        const V end=path.back().b;
        if((end-active_.anchor).norm()>cfg_.probe_anchor_range)continue;
        std::vector<CertifiedTube> back;
        const int pieces=std::max(1,static_cast<int>(std::ceil((active_.anchor-end).norm()/.05)));
        for(int i=0;i<pieces;++i)back.push_back({end+(active_.anchor-end)*(double(i)/pieces),
            end+(active_.anchor-end)*(double(i+1)/pieces),radius+cfg_.restart_clearance});
        if(!hardSafe(back,c.recovery))continue;
        const auto a=pathUnknown(path),b=pathUnknown(back),e=pathUnknown(emergency);
        // Exploration reserves a command-switch/turn allowance in addition to
        // the geometric return. Recovery may consume this reserved allowance.
        const double turn_time=2.*(cfg_.motion.rollout_horizon+cfg_.motion.velocity_tau+2.*cfg_.execution_dt);
        const double reserve=c.recovery?0.:cfg_.probe_speed*turn_time;
        const double distance=active_.unknown_distance+std::max(a.first,e.first)+b.first+reserve;
        const double initial=active_.unknown_volume_started?0.:
            ((!body_known||a.second||b.second||e.second)?4*pi*radius*radius*radius/3.:0.);
        const double volume=active_.unknown_volume+initial+area*(std::max(a.first,e.first)+b.first+reserve);
        const double time=active_.unknown_time+std::max(a.second?path.size()*cfg_.execution_dt:0.,
            e.second?emergency.size()*cfg_.execution_dt:0.)+
            (b.second?b.first/std::max(.05,cfg_.probe_speed):0.)+(c.recovery?0.:turn_time);
        if(distance>cfg_.probe_distance_budget||volume>cfg_.probe_volume_budget||time>cfg_.probe_time_budget) {
            budget_rejected=true;continue;
        }
        if(!active_.episode_active) {
            ++active_.episode;active_.episode_active=true;
            active_.episode_origin=position;active_.episode_origin_valid=true;active_.episode_stamp=now;
            active_.critical_targets=out.observation_targets;
            // Sparse outer witnesses can all be known while a thin continuous
            // restart shell is not. Save actual unresolved INNER shell balls,
            // rather than opening an event with no attainable progress bit.
            if(active_.critical_targets.empty()) {
                const double shell=radius+cfg_.restart_clearance;
                std::vector<V> missing;
                for(int latitude=-90;latitude<=90;latitude+=15)
                    for(int longitude=0;longitude<360;longitude+=30) {
                        const double a=latitude*pi/180.,b=longitude*pi/180.;
                        const V target=position+shell*V(std::cos(a)*std::cos(b),std::cos(a)*std::sin(b),std::sin(a));
                        if(!withinWorkingBounds(target,target,.025)||
                           (cfg_.free_space_floor_enabled&&target.z()-.025<cfg_.free_space_floor_z)||
                           (cfg_.free_space_ceiling_enabled&&target.z()+.025>cfg_.free_space_ceiling_z))continue;
                        bool occupied_target=false;
                        const int ix=int(std::floor(target.x()/cell)),iy=int(std::floor(target.y()/cell)),iz=int(std::floor(target.z()/cell));
                        for(int dx=-1;dx<=1&&!occupied_target;++dx)for(int dy=-1;dy<=1&&!occupied_target;++dy)for(int dz=-1;dz<=1&&!occupied_target;++dz) {
                            const auto hit=occupied.find(std::make_tuple(ix+dx,iy+dy,iz+dz));
                            if(hit!=occupied.end())for(const V& point:hit->second.points)
                                if((point-target).squaredNorm()<=.04*.04){occupied_target=true;break;}
                        }
                        if(!occupied_target&&!envelopeKnown(evidence,target,.025,now)&&
                           std::none_of(missing.begin(),missing.end(),[&](const V& old){return (old-target).norm()<.01;}))missing.push_back(target);
                    }
                // Spread the bounded witness set over the shell; do not fill
                // all 16 slots from one permanently occluded/occupied cap.
                while(!missing.empty()&&active_.critical_targets.size()<16) {
                    auto best=missing.begin();double separation=-1.;
                    for(auto it=missing.begin();it!=missing.end();++it) {
                        double nearest=std::numeric_limits<double>::infinity();
                        for(const auto& target:active_.critical_targets)nearest=std::min(nearest,(*it-target).squaredNorm());
                        if(nearest>separation){separation=nearest;best=it;}
                    }
                    active_.critical_targets.push_back(*best);missing.erase(best);
                }
            }
        }
        active_.unknown_volume+=initial;
        if(initial>0.)active_.unknown_volume_started=true;
        active_.recovering=c.recovery;out.command=c.command;out.accepted=true;
        active_.last_unknown=true; // Evidence arriving next frame cannot erase past exposure.
        out.authorization="EXPLORATORY";out.navigation_mode=c.recovery?"RECOVER":"PROBE";
        out.expected_visibility=c.gain;
        for(const auto& s:path)out.preview_path.push_back(s.b);
        // No pending balls/tubes are retained from a risk-accepted motion.
        return finish(c.recovery?"EXPLORATORY_RECOVER":"EXPLORATORY_PROBE");
    }
    active_.recovering=true;
    return finish(budget_rejected?"PROBE_BUDGET_EXHAUSTED":"PROBE_NO_USEFUL_PATH");
}
} }
