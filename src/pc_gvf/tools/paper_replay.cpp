#include "pc_gvf/paper_guidance.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc,char** argv) {
    if(argc<2||argc>4){std::cerr<<"usage: paper_replay SNAPSHOT.bin [--audit] [--incremental-field]\n";return 2;}
    bool audit=false,incremental=false;
    for(int i=2;i<argc;++i) {
        const std::string option(argv[i]);
        if(option=="--audit")audit=true;
        else if(option=="--incremental-field")incremental=true;
        else {std::cerr<<"unknown option: "<<option<<'\n';return 2;}
    }
    try {
        std::ifstream in(argv[1],std::ios::binary);
        const auto r=pc_gvf::depth_angular::PaperGuidance::replay(in,audit?&std::cerr:nullptr,incremental);
        std::cout<<std::setprecision(17)<<"{\"status\":\""<<r.status<<"\",\"reason\":\""<<r.build_reason
            <<"\",\"accepted\":"<<(r.accepted?"true":"false")<<",\"command\":["<<r.command.x()<<','<<r.command.y()<<','<<r.command.z()
            <<"],\"required_prefix\":"<<r.required_prefix<<",\"best_prefix\":"<<r.best_prefix
            <<",\"free_directions\":"<<r.free_directions<<",\"refined\":"<<(r.grid_refined?"true":"false")
            <<",\"command_changed\":"<<(r.command_changed?"true":"false")
            <<",\"intent_change_angle\":"<<r.intent_change_angle
            <<",\"reset_reason\":\""<<r.reset_reason<<"\",\"continued\":"<<(r.continued?"true":"false")<<"}\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
