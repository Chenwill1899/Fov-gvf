from pathlib import Path
r=Path(__file__).resolve().parents[3]
p=r/'src/pc_gvf/src/paper_guidance.cpp';s=p.read_text();old='        cfg_.history_duration=std::min(.5,cfg_.history_duration);';assert old in s
s=s.replace(old,old+"\n        // Four cameras need several recent poses inside the 0.5s horizon.\n        // Static Cloud's 1s ingress cadence otherwise expires before refresh.\n        cfg_.history_sample_interval=std::min(.1,cfg_.history_sample_interval);\n        if(cfg_.history_max_observations>=4)cfg_.history_max_observations=std::max(32,cfg_.history_max_observations);")
p.write_text(s)
p=r/'src/pc_gvf/test/cpp/dynamic_obstacles_check.cpp';s=p.read_text();needle='    std::cout<<"crossing_cases';assert needle in s
s=s.replace(needle,'    PaperConfig sparse;sparse.dynamic_obstacles=true;sparse.history_duration=60.;\n    sparse.history_sample_interval=1.;sparse.history_max_observations=16;\n    PaperGuidance recent(sparse);\n    for(int k=0;k<9;++k) {\n        DepthObservation o(camera,std::vector<double>(1728,10.),Eigen::Vector3d(.02*k,0,0),\n            fixedCameraRotation(),k*.1,k+1);\n        recent.ingestObservation(o);\n    }\n    if(recent.retainedObservations()<3) {std::cerr<<"dynamic history expired before its next static-cadence sample\\n";return 3;}\n'+needle);p.write_text(s)
