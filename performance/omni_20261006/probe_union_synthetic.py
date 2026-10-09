#!/usr/bin/env python3
"""Analytically covered six-ball union vs deliberate holes, offline only."""
from pathlib import Path
import json,os,subprocess,sys
out=Path(__file__).resolve().parent;root=out.parents[1];work=Path('/tmp/ego1p3_union_cert_diagnostic')
source=work/'make_synthetic.cpp'
source.write_text(r'''
#include "pc_gvf/paper_guidance.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
using namespace pc_gvf::depth_angular;
int main(int argc,char** argv){
 if(argc!=2)return 2;
 const Eigen::Vector3d center(.37,-.28,1.5);
 PaperConfig cfg;cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;
 cfg.motion.rollout_margin=.02;cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.1;cfg.observation_timeout=.3;
 Camera camera(8,6,90.,68.,10.);
 DepthObservation evidence(camera,std::vector<double>(48,0.),center,Eigen::Matrix3d::Identity(),1.);
 for(int millimetres:{4,6,8,10,15,20,-2}) {
  PaperGuidance p(cfg);const double radius=p.envelopeRadius(),offset=.4;
  const double critical=std::sqrt(radius*radius+offset*offset-2.*radius*offset/std::sqrt(3.));
  std::vector<Eigen::Vector3d> centers;
  for(int axis=0;axis<3;++axis)for(int sign:{-1,1}){auto q=center;q[axis]+=sign*offset;centers.push_back(q);}
  p.setVerifiedNeighborhood(centers,critical+.001*millimetres);
  const auto name=std::string(argv[1])+"/synthetic_"+std::to_string(millimetres)+".bin";
  std::ofstream file(name,std::ios::binary);const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  p.saveReplay(file,evidence,center,zero,Eigen::Vector3d(.1,0,0),1.,.02,&zero);
  std::cout<<millimetres<<" "<<p.envelopeKnown(evidence,center,radius,1.)<<" "<<critical<<"\n";
 }
}
''')
exe=work/'make_synthetic'
command=['g++','-std=c++17','-O2','-fopenmp','-I/usr/include/eigen3','-I'+str(root/'src/pc_gvf/include'),str(source),'/tmp/fov_gvf_ego1p3_isaac_build/pc_gvf/libpc_gvf_depth_angular_core.a','-lcrypto','-o',str(exe)]
if '--reuse' not in sys.argv:
 with (out/'union_synthetic_compile.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
 with (out/'union_synthetic_old.txt').open('w') as log:subprocess.run([str(exe),str(work)],stdout=log,stderr=subprocess.STDOUT,check=True)
rows=[]
for margin in [4,6,8,10,15,20,-2]:
 for level in [5,7,8]:
  env=dict(os.environ,EGO_DIAG_LEVEL=str(level),EGO_DIAG_BUDGET='1024')
  r=subprocess.run([str(work/'variable_replay'),str(work/f'synthetic_{margin}.bin'),'--audit'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env,check=True,timeout=60)
  rows.append(dict(margin_mm=margin,level=level,budget=1024,known_body=[line for line in r.stdout.splitlines() if line.startswith('known_body_plus')]))
(out/'union_synthetic_results.json').write_text(json.dumps(rows,indent=2)+'\n')
for r in rows:print(r['margin_mm'],r['level'],r['known_body'][0])
