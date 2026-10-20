#include "pc_gvf/verified_start.hpp"
#include <openssl/evp.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
namespace pc_gvf { namespace depth_angular {
namespace {
std::string digest(const std::string& path) {
    std::ifstream stream(path,std::ios::binary);if(!stream)return {};
    EVP_MD_CTX* ctx=EVP_MD_CTX_new();if(!ctx)return {};
    bool ok=EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr)==1;
    char buffer[65536];
    while(ok&&stream) {stream.read(buffer,sizeof(buffer));ok=EVP_DigestUpdate(ctx,buffer,stream.gcount())==1;}
    unsigned char bytes[EVP_MAX_MD_SIZE];unsigned int count=0;
    ok=ok&&!stream.bad()&&EVP_DigestFinal_ex(ctx,bytes,&count)==1;EVP_MD_CTX_free(ctx);
    if(!ok)return {};
    std::ostringstream text;text<<std::hex<<std::setfill('0');
    for(unsigned int i=0;i<count;++i)text<<std::setw(2)<<static_cast<unsigned int>(bytes[i]);
    return text.str();
}
}
bool certifyDefaultCloudStart(const std::string& scene,const std::string& occupancy,
    const Eigen::Vector3d& center,double radius,std::string* diagnostic) {
    auto fail=[&](const char* why){if(diagnostic)*diagnostic=why;return false;};
    if(!center.allFinite()||!std::isfinite(radius)||radius<=0)return fail("invalid free-ball geometry");
    const auto scene_hash=digest(scene), occupancy_hash=digest(occupancy);
    const std::string original_scene="43f085ddc46a4391dcd743488af77cf08850a9e279b42252bd744d39057b35b2";
    const std::string original_occupancy="567695099c3b518f37bcdd17e22f4238622ed3e3e980020399ad0890c9069e2a";
    Eigen::Vector3i size(400,300,50);
    // Self-contained unified map: all geometry and colors are inside this USD.
    // Pin its complete bytes together with the full 1200x300x50 occupancy grid.
    if(scene_hash=="cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca"&&
       occupancy_hash=="e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8") {
        size.x()=1200;
    } else if(scene_hash==original_scene&&occupancy_hash==original_occupancy) {
        // Original frozen map.
    } else if((scene_hash=="ae05c3f8b8a9bf55c9ca17654a47ea3c331bc3daa2e516600bf61b5b3a0acd69"||
               scene_hash=="4c080a85a6bf73fd972e2024e3e3a84def7bc4c81b20ca66f1a271d2cba4eb17")&&
              occupancy_hash=="e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8") {
        // The derived layer references the exact original mesh. Verify that
        // dependency too: a matching small wrapper alone proves no geometry.
        const auto slash=scene.find_last_of('/');
        const auto base=(slash==std::string::npos?std::string("."):scene.substr(0,slash))+
            "/../ego_swarm_cloud/ego_swarm_cloud_navigation.usd";
        if(digest(base)!=original_scene)
            return fail("derived Cloud dependency mismatch; initialization refused");
        // The appearance-only layer is also pinned: a referenced USD layer
        // could otherwise introduce geometry as well as vertex colors.
        if(scene_hash=="4c080a85a6bf73fd972e2024e3e3a84def7bc4c81b20ca66f1a271d2cba4eb17") {
            const auto colors=(slash==std::string::npos?std::string("."):scene.substr(0,slash))+
                "/height_colors.usdc";
            const auto colors_hash=digest(colors);
            // Keep the previous exact palette for archived appearance runs.
            if(colors_hash!="decb259e9e786874e0dc5c2d154ca4394ba47acd43d207dad5ef30a4c6045e9e"&&
               colors_hash!="fea2e740be1dcd4a64884c4d202f6bd6bb89ddbc8a2e766be44f2d706b51801a")
                return fail("derived Cloud color dependency mismatch; initialization refused");
        }
        size.x()=1200;
    } else return fail("scene/occupancy provenance mismatch; initialization refused");
    const Eigen::Vector3d origin(-80,-60,0);const double resolution=.4;
    for(int axis=0;axis<3;++axis)
        if(center[axis]-radius<=origin[axis]||center[axis]+radius>=origin[axis]+resolution*size[axis])
            return fail("initial free ball extends outside verified map");
    std::ifstream input(occupancy,std::ios::binary);std::vector<unsigned char> cells(size.prod());
    input.read(reinterpret_cast<char*>(cells.data()),cells.size());
    if(input.gcount()!=static_cast<std::streamsize>(cells.size()))return fail("occupancy read failed");
    // Include an additional half-cell on every side to cover cell alignment and
    // the voxel mesh representation; never certify through an occupied voxel.
    Eigen::Vector3i lo,hi;
    for(int axis=0;axis<3;++axis) {
        lo[axis]=std::max(0,static_cast<int>(std::floor((center[axis]-origin[axis]-radius)/resolution))-2);
        hi[axis]=std::min(size[axis]-1,static_cast<int>(std::ceil((center[axis]-origin[axis]+radius)/resolution))+2);
    }
    for(int x=lo.x();x<=hi.x();++x)for(int y=lo.y();y<=hi.y();++y)for(int z=lo.z();z<=hi.z();++z) {
        if(!cells[(x*size.y()+y)*size.z()+z])continue;
        const Eigen::Vector3d cell=origin+resolution*(Eigen::Vector3d(x,y,z)+Eigen::Vector3d::Constant(.5));
        const Eigen::Vector3d distance=((center-cell).cwiseAbs()-Eigen::Vector3d::Constant(resolution)).cwiseMax(0);
        if(distance.norm()<=radius)return fail("occupied geometry intersects initial free ball");
    }
    if(diagnostic)*diagnostic="verified fixed Cloud free ball; USD and occupancy SHA-256 matched";
    return true;
}
} }
