from pathlib import Path
r=Path(__file__).resolve().parents[3]
p=r/'src/pc_gvf/include/pc_gvf/paper_guidance.hpp';s=p.read_text();needle='    PaperResult stepWithProposal('
s=s.replace(needle,'    Eigen::Vector3d incrementalGoalProposal(const Eigen::Vector3d& position,\n        const Eigen::Vector3d& intent,double now) const;\n'+needle);p.write_text(s)
p=r/'src/pc_gvf/src/paper_guidance.cpp';s=p.read_text()
a=s.index('    bool reused_goal=false;',s.index('PaperResult PaperGuidance::stepWithProposal'))
b=s.index('    if(!position.allFinite()',a)
old=s[a:b]
condition_start=old.index('    if(cfg_.incremental_field')
condition_end=old.index(' {',condition_start)
condition=old[condition_start:condition_end].replace('&&proposal.allFinite()&&proposal.norm()<1e-5','')
body=old[condition_end:]
body=body.replace('proposal=std::min(intent.norm(),cfg_.max_speed)*delta.normalized();reused_goal=true;','return std::min(intent.norm(),cfg_.max_speed)*delta.normalized();')
method='Eigen::Vector3d PaperGuidance::incrementalGoalProposal(const Eigen::Vector3d& position,\n    const Eigen::Vector3d& intent,double now) const {\n'+condition+body+'    return Eigen::Vector3d::Zero();\n}\n'
s=s[:a]+'    bool reused_goal=false;\n    if(proposal.allFinite()&&proposal.norm()<1e-5) {\n        const auto cached=incrementalGoalProposal(position,intent,now);\n        if(cached.norm()>1e-5){proposal=cached;reused_goal=true;}\n    }\n'+s[b:]
s=s.replace('PaperResult PaperGuidance::stepWithProposal',method+'PaperResult PaperGuidance::stepWithProposal',1);p.write_text(s)
p=r/'src/pc_gvf/src/depth_angular_controller_node.cpp';s=p.read_text()
s=s.replace('            operator_assistance_=declare_parameter<bool>("operator_assistance",false);','            operator_assistance_=declare_parameter<bool>("operator_assistance",false);\n            incremental_frontend_=declare_parameter<bool>("paper_incremental_frontend",true);')
s=s.replace('        double omni_corridor=0.;','        double omni_corridor=0.;\n        // After a compute watchdog, the last completed field already supplies\n        // a short-lived world target. Skip repeated expensive candidate search,\n        // then require the same current full-motion/braking proof below.\n        const bool incremental_frontend=incremental_frontend_&&incremental_recovery_pending_&&\n            paper_->incrementalGoalProposal(position,intent,stamp.seconds()).norm()>1e-5;\n')
s=s.replace('if(omni_depth_enabled_&&intent.norm()>1e-5) {','if(!incremental_frontend&&omni_depth_enabled_&&intent.norm()>1e-5) {')
s=s.replace('if(paper_depth_proposal_&&!omni_depth_enabled_&&intent.norm()>1e-5)', 'if(!incremental_frontend&&paper_depth_proposal_&&!omni_depth_enabled_&&intent.norm()>1e-5)')
s=s.replace('if(proposal.norm()<1e-5)\n            memory_proposal=', 'if(!incremental_frontend&&proposal.norm()<1e-5)\n            memory_proposal=')
s=s.replace('const bool use_proposal=paper_depth_proposal_||memory_proposal.norm()>1e-5;', 'const bool use_proposal=paper_depth_proposal_||incremental_frontend||memory_proposal.norm()>1e-5;')
s=s.replace('        command_.setZero();\n        if (paper_) paper_->reset();','        command_.setZero();\n        incremental_recovery_pending_=false;\n        if (paper_) paper_->reset();')
s=s.replace('        const PaperResult replay_result=result;','        const PaperResult replay_result=result;\n        incremental_recovery_pending_=incremental_frontend&&result.status=="INCREMENTAL_GOAL_CERTIFIED";')
s=s.replace('            result.command.setZero();result.accepted=false;result.status="COMPUTE_DEADLINE";','            incremental_recovery_pending_=paper_->field().valid;\n            result.command.setZero();result.accepted=false;result.status="COMPUTE_DEADLINE";')
s=s.replace('    OperatorIntent operator_intent_;','    bool incremental_frontend_=true,incremental_recovery_pending_=false;\n    OperatorIntent operator_intent_;')
s=s.replace('            << ",\\"operator_assistance\\":"','            << ",\\"incremental_frontend\\":" << (incremental_frontend?"true":"false")\n            << ",\\"operator_assistance\\":"')
p.write_text(s)
p=r/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py';s=p.read_text().replace('                "paper_incremental_field":','                "paper_incremental_frontend": os.environ.get("FOV_GVF_INCREMENTAL_FRONTEND", "1") == "1",\n                "paper_incremental_field":');p.write_text(s)
print('Recovery fast frontend applied; inspect diff before build')

p=r/'src/pc_gvf/src/depth_angular_controller_node.cpp';s=p.read_text();needle='        if(!incremental_frontend&&omni_depth_enabled_&&intent.norm()>1e-5) {'
s=s.replace(needle,'        if(incremental_frontend)RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"INCREMENTAL_FRONTEND active=1");\n'+needle);p.write_text(s)
