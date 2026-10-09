#include "pc_gvf/local_depth_path.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
using namespace pc_gvf::depth_angular;
void check(bool condition,const char* message){if(!condition){std::cerr<<message<<'\n';std::exit(1);}}
int main(){
    Camera camera(64,48,90,68,10);
    auto view=std::make_shared<DepthObservation>(camera,std::vector<double>(64*48,10.),Eigen::Vector3d(.22,0,0),fixedCameraRotation(),1);
    auto clear=localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d(20,0,0),.58);
    check(clear.valid&&(clear.direction-Eigen::Vector3d::UnitX()).norm()<1e-10,"clear space lost straight shortest proposal");
    const Eigen::Vector3d old_hint=Eigen::Vector3d(1,0,1).normalized();
    auto recaptured=localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
        Eigen::Vector3d(20,0,0),.58,8.,.44,&old_hint);
    check(recaptured.valid&&(recaptured.direction-Eigen::Vector3d::UnitX()).norm()<1e-10,
        "old detour hint survived a clear operator direction");
    view->depth.assign(64*48,0.);
    check(!localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d(20,0,0),.58).valid,"unknown image generated an observed local route");
    view->depth.assign(64*48,3.);
    check(!localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d(20,0,0),.58).valid,"full blocking plane was crossed by local route");
    // A finite box in the center leaves a visible overhead corridor. Render
    // its front face by exact rays; side rays are unobstructed to max range.
    view->depth.assign(64*48,10.);
    for(int y=0;y<48;++y)for(int x=0;x<64;++x){
        const Eigen::Vector3d point=view->origin+view->rotation*(2.78/camera.ray(x,y).z()*camera.ray(x,y));
        if(std::abs(point.y())<.9&&std::abs(point.z())<.5)view->depth[y*64+x]=2.78;
    }
    auto route=localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d(20,0,0),.58);
    check(route.valid&&route.direction.dot(Eigen::Vector3d::UnitX())>0&&std::abs(route.direction.z())>.05,"visible short vertical detour not proposed");
    auto approaching=localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d(2,0,0),Eigen::Vector3d(20,0,0),.58);
    check(approaching.valid&&approaching.direction.x()<route.direction.x(),
        "measured approach velocity failed to anticipate the finite obstacle");
    check(!localDepthPath({view},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
        Eigen::Vector3d(20,0,0),.58,8.,-1.).valid,"invalid lag horizon accepted");
    const Eigen::Vector3d shift(2,3,-1);const auto rotation=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    view->origin=shift+rotation*view->origin;view->rotation=rotation*view->rotation;
    auto transformed=localDepthPath({view},shift,Eigen::Vector3d::Zero(),shift+rotation*Eigen::Vector3d(20,0,0),.58);
    check(transformed.valid&&(transformed.direction-rotation*route.direction).norm()<1e-8,"local route changed under world yaw/translation");
    std::vector<Eigen::Vector3d> bend{Eigen::Vector3d(0,0,0),Eigen::Vector3d(2,0,0),Eigen::Vector3d(2,2,0),Eigen::Vector3d(4,2,0)};
    auto tracked=localPathDirection(bend,Eigen::Vector3d(1,0,0),2.);
    check((tracked-Eigen::Vector3d(1,1,0).normalized()).norm()<1e-12,"world-path arc lookahead drifted with the vehicle");
    auto tracked_next=localPathDirection(bend,Eigen::Vector3d(1.01,0,0),2.);
    check((tracked_next-tracked).norm()<.02,"continuous path tracking introduced a grid-sized target jump");
    auto blind=std::make_shared<DepthObservation>(*view);blind->depth.assign(64*48,0.);
    check(localDepthPath({blind,view},shift,Eigen::Vector3d::Zero(),shift+rotation*Eigen::Vector3d(20,0,0),.58).valid,"retained depth was not used for occluded route proposal");
    view->revoked=true;
    check(!localDepthPath({blind,view},shift,Eigen::Vector3d::Zero(),shift+rotation*Eigen::Vector3d(20,0,0),.58).valid,"revoked history authorized a local route proposal");
    std::cout<<"local depth-only route proposals: PASS\n";
}
