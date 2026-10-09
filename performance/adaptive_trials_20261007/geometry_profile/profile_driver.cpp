#include "pc_gvf/paper_guidance.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
int main(int argc,char** argv){if(argc<2)return 2;for(int i=0;i<20;++i){std::ifstream in(argv[1],std::ios::binary);auto r=pc_gvf::depth_angular::PaperGuidance::replay(in);std::cout<<r.status<<' '<<r.accepted<<'\n';}return 0;}
