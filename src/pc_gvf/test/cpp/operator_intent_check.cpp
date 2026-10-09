#include "pc_gvf/operator_intent.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include "operator_lag_fixture.hpp"
using namespace pc_gvf::depth_angular;
int main(){
    double variation[2]={},progress[2]={};Camera c(32,24,90,68,20.);
    for(int mode=0;mode<2;++mode){
        OperatorIntent input;PaperConfig cfg;cfg.smooth_speed=true;cfg.proposal_command_accel=2.4;cfg.proposal_jerk_limit=6.;
        cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;cfg.observation_timeout=.3;cfg.motion.rollout_margin=.02;cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.1;cfg.observation_timeout=.3;cfg.motion.max_accel=1.2;cfg.motion.brake_accel=1.2;cfg.motion.delay=.2;cfg.motion.rollout_horizon=.2;PaperGuidance planner(cfg);
        Eigen::Vector3d p(2,0,0),v=Eigen::Vector3d::Zero(),last=Eigen::Vector3d::Zero();
        for(int k=0;k<600;++k){double a=k==0?0:.008*std::sin(k*1.7);Eigen::Vector3d raw(std::cos(a),std::sin(a),0);
            auto q=input.update(raw,mode);DepthObservation o(c,std::vector<double>(768,20.),p-Eigen::Vector3d(2,0,0),fixedCameraRotation(),k*.02,k+1);
            auto result=planner.stepWithProposal(o,p,v,q,q,k*.02,.02);
            if(!result.accepted)return 2;
            if(k>100)variation[mode]+=(result.command-last).norm();last=result.command;
            v+=.02*clampNorm((result.command-v)/.22,1.2);p+=.02*v;
        }progress[mode]=p.x()-2;
        if(input.update(Eigen::Vector3d::Zero(),mode).norm()!=0)return 3;
        input.update(Eigen::Vector3d::UnitX(),mode);
        if((input.update(Eigen::Vector3d::UnitY(),mode)-Eigen::Vector3d::UnitY()).norm()>1e-10)return 4;
        double max_error=0;
        for(int k=0;k<100;++k){double a=k*.002;Eigen::Vector3d q(std::cos(a),std::sin(a),0);auto out=input.update(q,mode);
            max_error=std::max(max_error,std::atan2(out.cross(q).norm(),out.dot(q)));}
        if(max_error>.018001)return 5;
        // A coherent slow turn should pass raw direction after three changes;
        // repeating sensor samples must not fabricate additional turn evidence.
        OperatorIntent deliberate;deliberate.update(Eigen::Vector3d::UnitX(),true);
        for(int k=1;k<=50;++k) {
            const double a=.002*k;const Eigen::Vector3d raw(std::cos(a),std::sin(a),0);
            auto actual=deliberate.update(raw,true);
            if(k>=3&&(actual-raw).norm()>1e-10)return 6;
            deliberate.update(raw,true);
        }
    }
    // Real control ticks can repeat an input. Exercise oscillation -> deliberate
    // three-dimensional turn -> neutral, preserving speed and immediate bypass.
    OperatorIntent transitions;
    for(int k=0;k<120;++k) {
        Eigen::Vector3d raw(1.,.006*std::sin(1.7*k),.006*std::sin(1.9*k));raw.normalize();raw*=.8;
        auto filtered=transitions.update(raw,true);
        if(std::abs(filtered.norm()-.8)>1e-12)return 7;
        if(std::atan2(filtered.cross(raw).norm(),filtered.dot(raw))>.018001)return 8;
        if(k%3==0)transitions.update(raw,true);
    }
    const Eigen::Vector3d emergency(-.6,0.,.3);
    for(int k=0;k<5;++k)if((transitions.update(emergency,true)-emergency).norm()>1e-12)return 9;
    if(transitions.update(Eigen::Vector3d::Zero(),true).norm()!=0)return 10;
    if((transitions.update(Eigen::Vector3d(.2,.1,0),true)-Eigen::Vector3d(.2,.1,0)).norm()>1e-12)return 11;
    if((transitions.update(Eigen::Vector3d(.3,.2,.1),false)-Eigen::Vector3d(.3,.2,.1)).norm()>1e-12)return 12;
    OperatorIntent bounded;bounded.update(Eigen::Vector3d::UnitX(),true);
    for(double angle:kOperatorLagAngles) {
        const Eigen::Vector3d raw(std::cos(angle),std::sin(angle),0.);
        const auto out=bounded.update(raw,true);
        if(std::atan2(out.cross(raw).norm(),out.dot(raw))>.018000000001)return 13;
    }
    std::cout<<"baseline_command_total_variation,assisted_total_variation,baseline_progress_m,assisted_progress_m,release_delay_frames,large_turn_delay_frames\n"
        <<variation[0]<<','<<variation[1]<<','<<progress[0]<<','<<progress[1]<<",0,0\n";
    return variation[1]<variation[0]*.5&&progress[1]>=progress[0]-.01?0:1;
}
