#include "pc_gvf/depth_angular_core.hpp"

#include "fixture_reader.hpp"
#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using pc_gvf::depth_angular::AngularSolution;
using pc_gvf::depth_angular::Camera;
using pc_gvf::depth_angular::GuidanceResult;
using pc_gvf::depth_angular::SimConfig;
using pc_gvf::depth_angular::computeGuidance;
using pc_gvf_test::Fixture;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "guidance_composition_check: " << message << '\n';
        std::exit(1);
    }
}

bool close(double actual, double expected, double absolute, double relative)
{
    return std::abs(actual - expected) <=
        absolute + relative * std::abs(expected);
}

Eigen::Vector2d vector2(const std::vector<double>& values)
{
    if (values.size() != 2) {
        throw std::runtime_error("expected two fixture values");
    }
    return Eigen::Vector2d(values[0], values[1]);
}

Eigen::Vector3d vector3(const std::vector<double>& values)
{
    if (values.size() != 3) {
        throw std::runtime_error("expected three fixture values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Matrix3d matrix3(const std::vector<double>& values)
{
    if (values.size() != 9) {
        throw std::runtime_error("expected nine fixture values");
    }
    Eigen::Matrix3d result;
    result <<
        values[0], values[1], values[2],
        values[3], values[4], values[5],
        values[6], values[7], values[8];
    return result;
}

Camera fixtureCamera(const Fixture& fixture)
{
    Camera camera(
        fixture.integer("camera.width"),
        fixture.integer("camera.height"),
        fixture.scalar("camera.hfov_deg"),
        fixture.scalar("camera.vfov_deg"),
        fixture.scalar("camera.max_depth"));
    camera.setIntrinsics(
        fixture.scalar("camera.fx"), fixture.scalar("camera.fy"),
        fixture.scalar("camera.cx"), fixture.scalar("camera.cy"));
    return camera;
}

SimConfig fixtureConfig(const Fixture& fixture)
{
    SimConfig config;
    config.body_radius = fixture.scalar("cfg.body_radius");
    config.safety_margin = fixture.scalar("cfg.safety_margin");
    config.reference_speed = fixture.scalar("cfg.reference_speed");
    config.max_accel = fixture.scalar("cfg.max_accel");
    config.brake_accel = fixture.scalar("cfg.brake_accel");
    config.velocity_tau = fixture.scalar("cfg.velocity_tau");
    config.delay = fixture.scalar("cfg.delay");
    config.planning_horizon = fixture.scalar("cfg.planning_horizon");
    config.control_dt = fixture.scalar("cfg.control_dt");
    config.dynamics_dt = fixture.scalar("cfg.dynamics_dt");
    config.rollout_dt = fixture.scalar("cfg.rollout_dt");
    config.rollout_horizon = fixture.scalar("cfg.rollout_horizon");
    config.rollout_margin = fixture.scalar("cfg.rollout_margin");
    config.max_direction_rate = fixture.scalar("cfg.max_direction_rate");
    config.goal_tolerance = fixture.scalar("cfg.goal_tolerance");
    config.clearance_reward = fixture.scalar("cfg.clearance_reward");
    config.hysteresis_weight = fixture.scalar("cfg.hysteresis_weight");
    config.deterministic_left_bias =
        fixture.scalar("cfg.deterministic_left_bias");
    config.convergence_distance = fixture.scalar("cfg.convergence_distance");
    config.convergence_margin = fixture.scalar("cfg.convergence_margin");
    config.convergence_length = fixture.scalar("cfg.convergence_length");
    config.convergence_max_angle = fixture.scalar("cfg.convergence_max_angle");
    config.source_radius_cells = fixture.integer("cfg.source_radius_cells");
    config.goal_radius_cells = fixture.integer("cfg.goal_radius_cells");
    config.field_tolerance = fixture.scalar("cfg.field_tolerance");
    config.field_max_iterations = fixture.integer("cfg.field_max_iterations");
    config.depth_point_stride = fixture.integer("cfg.depth_point_stride");
    config.cone_chunk_size = fixture.integer("cfg.cone_chunk_size");
    return config;
}

void compareVector(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double absolute,
    double relative,
    bool allow_nan,
    const std::string& context)
{
    require(actual.size() == expected.size(), "shape mismatch: " + context);
    for (std::size_t index = 0; index < actual.size(); ++index) {
        if (allow_nan && std::isnan(expected[index])) {
            require(std::isnan(actual[index]),
                    "NaN layout mismatch: " + context + " index " +
                        std::to_string(index));
        } else {
            require(close(actual[index], expected[index], absolute, relative),
                    "numeric mismatch: " + context + " index " +
                        std::to_string(index));
        }
    }
}

void checkFixture(const std::string& directory, const std::string& name)
{
    const Fixture fixture = Fixture::load(directory + "/" + name + ".fixture");
    const Camera camera = fixtureCamera(fixture);
    const std::vector<double> depth = fixture.numbers("input.depth");
    const Eigen::Vector3d position = vector3(fixture.numbers("input.position_w"));
    const Eigen::Vector3d velocity = vector3(fixture.numbers("input.velocity_w"));
    const Eigen::Vector3d goal = vector3(fixture.numbers("input.goal_w"));
    const Eigen::Vector2d previous = vector2(fixture.numbers("input.q_previous"));
    const Eigen::Matrix3d rotation = matrix3(fixture.numbers("input.R_wc"));

    Eigen::Vector2d previous_goal = Eigen::Vector2d::Zero();
    const Eigen::Vector2d* previous_goal_pointer = nullptr;
    if (fixture.value("input.q_goal_previous") != "none") {
        previous_goal = vector2(fixture.numbers("input.q_goal_previous"));
        previous_goal_pointer = &previous_goal;
    }
    Eigen::Vector3d reference_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_direction = Eigen::Vector3d::Zero();
    const Eigen::Vector3d* reference_origin_pointer = nullptr;
    const Eigen::Vector3d* reference_direction_pointer = nullptr;
    if (fixture.value("input.reference_origin_w") != "none") {
        reference_origin = vector3(fixture.numbers("input.reference_origin_w"));
        reference_direction = vector3(
            fixture.numbers("input.reference_direction_w"));
        reference_origin_pointer = &reference_origin;
        reference_direction_pointer = &reference_direction;
    }

    const GuidanceResult result = computeGuidance(
        depth, camera, fixtureConfig(fixture), position, velocity, goal,
        previous, previous_goal_pointer, rotation, reference_origin_pointer,
        reference_direction_pointer);
    const AngularSolution& solution = result.solution;

    compareVector(solution.depth, depth, 0.0, 0.0, true, name + " depth");
    compareVector(
        solution.free_distance, fixture.numbers("expected.free_distance"),
        1.0e-9, 1.0e-9, false, name + " free distance");
    const std::vector<double> expected_mask =
        fixture.numbers("expected.planning_mask");
    require(solution.planning_mask.size() == expected_mask.size(),
            "planning-mask shape mismatch: " + name);
    for (std::size_t index = 0; index < expected_mask.size(); ++index) {
        require(solution.planning_mask[index] ==
                    static_cast<std::uint8_t>(expected_mask[index]),
                "planning-mask mismatch: " + name + " pixel " +
                    std::to_string(index));
    }
    compareVector(
        solution.potential, fixture.numbers("expected.potential"),
        1.0e-6, 1.0e-6, true, name + " potential");

    require((solution.reference_pixel -
             vector2(fixture.numbers("expected.q_ref"))).norm() <= 1.0e-6,
            "reference pixel mismatch: " + name);
    require((solution.source_pixel -
             vector2(fixture.numbers("expected.q_source"))).norm() <= 1.0e-6,
            "source pixel mismatch: " + name);
    require((solution.goal_pixel -
             vector2(fixture.numbers("expected.q_goal"))).norm() <= 1.0e-6,
            "goal pixel mismatch: " + name);
    require((solution.command_pixel -
             vector2(fixture.numbers("expected.q_cmd"))).norm() <= 1.0e-6,
            "command pixel mismatch: " + name);
    require(solution.field_valid ==
                static_cast<bool>(fixture.integer("expected.field_valid")),
            "field validity mismatch: " + name);
    require((result.returned_goal_pixel -
             vector2(fixture.numbers("expected.q_goal_returned"))).norm() <= 1.0e-6,
            "returned goal mismatch: " + name);
    require((result.command_world -
             vector3(fixture.numbers("expected.command_w"))).norm() <= 1.0e-6,
            "world command mismatch: " + name);
}

}  // namespace

int main()
{
    const std::string directory = PC_GVF_FIXTURE_DIR;
    const Fixture manifest = Fixture::load(directory + "/MANIFEST.txt");
    const std::vector<std::string> cases = manifest.strings("cases");
    require(static_cast<int>(cases.size()) == manifest.integer("case_count"),
            "manifest case count must match its case list");
    for (const std::string& name : cases) {
        checkFixture(directory, name);
    }
    // A stale detour goal must yield to the continuous reference in a clear
    // angular segment; a newly observed wall must still block direct capture.
    Camera camera(64,48,90,68,10);SimConfig recapture;recapture.recapture_reference=true;
    recapture.body_radius=.58;recapture.safety_margin=0;recapture.reference_speed=2.;
    recapture.planning_horizon=3.;recapture.control_dt=.02;
    std::vector<double> clear_depth(64*48,10.);
    Eigen::Vector2d previous(camera.cx()+5,camera.cy()),held(camera.cx()+8,camera.cy());
    auto direct=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,8),previous,&held,Eigen::Matrix3d::Identity(),
        nullptr,nullptr,nullptr,nullptr,1,true,true);
    require((direct.returned_goal_pixel-Eigen::Vector2d(camera.cx(),camera.cy())).norm()<1e-12,
        "clear continuous intent retained obsolete detour goal");
    for(int y=0;y<48;++y)for(int x=29;x<35;++x)clear_depth[y*64+x]=3.;
    auto blocked=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,8),previous,&held,Eigen::Matrix3d::Identity());
    require((blocked.returned_goal_pixel-Eigen::Vector2d(camera.cx(),camera.cy())).norm()>1.,
        "reference recapture crossed observed wall");
    int checked_goals=0;
    auto certify_right=[&](const Eigen::Vector2d& goal){++checked_goals;return goal.x()>camera.cx()+3.;};
    std::fill(clear_depth.begin(),clear_depth.end(),10.);
    auto constrained=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,8),previous,&held,Eigen::Matrix3d::Identity(),
        nullptr,nullptr,nullptr,nullptr,1,true,true,nullptr,nullptr,true,certify_right);
    require(checked_goals>0&&checked_goals<=12,"goal certificate exceeded bounded query budget");
    require(!constrained.solution.field_valid||constrained.returned_goal_pixel.x()>camera.cx()+3.,
        "direction proposal ignored failed goal certificate");
    auto no_goal=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,8),previous,&held,Eigen::Matrix3d::Identity(),
        nullptr,nullptr,nullptr,nullptr,1,true,true,nullptr,nullptr,true,[](const Eigen::Vector2d&){return false;});
    require(!no_goal.solution.field_valid,"all rejected goals still produced a valid candidate field");
    // A finite waypoint ends the route: a wall beyond the destination must
    // not turn the vehicle away from an already clear arrival corridor.
    std::fill(clear_depth.begin(),clear_depth.end(),5.);recapture.finite_goal=true;
    auto nearby_goal=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,2),previous,&held,Eigen::Matrix3d::Identity());
    require(nearby_goal.solution.field_valid&&
        (nearby_goal.returned_goal_pixel-Eigen::Vector2d(camera.cx(),camera.cy())).norm()<1e-12,
        "obstacle beyond finite goal caused needless avoidance");
    std::fill(clear_depth.begin(),clear_depth.end(),1.);
    auto before_goal=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,2),previous,&held,Eigen::Matrix3d::Identity());
    require(!before_goal.solution.field_valid,"finite goal ignored a wall before arrival");
    std::fill(clear_depth.begin(),clear_depth.end(),10.);
    auto target_only=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,2),previous,&held,Eigen::Matrix3d::Identity(),
        nullptr,nullptr,nullptr,nullptr,1,true,true,nullptr,nullptr,true,{},true);
    require(target_only.goal_valid&&!target_only.solution.field_valid,"goal-only mode fabricated a harmonic field");
    require((target_only.command_world-Eigen::Vector3d(0,0,2)).norm()<1e-10,"goal-only proposal changed continuous target direction");
    // Rectangular image coordinates must not penalize a physically smaller
    // vertical turn more than a larger horizontal turn.
    Camera metric_camera(81,41,120,45,10);
    pc_gvf::depth_angular::BinaryMask metric_mask(81*41,0);
    for(int y=16;y<=24;++y)for(int x=36;x<=44;++x)metric_mask[y*81+x]=1;
    SimConfig metric_cfg;metric_cfg.clearance_reward=0.;metric_cfg.hysteresis_weight=0.;metric_cfg.deterministic_left_bias=0.;
    const Eigen::Vector2d center(40,20);
    auto pixel_goal=pc_gvf::depth_angular::chooseSafeGoal(center,center,metric_mask,81,41,nullptr,metric_cfg);
    auto angular_goal=pc_gvf::depth_angular::chooseSafeGoal(center,center,metric_mask,81,41,nullptr,metric_cfg,&metric_camera);
    require(pixel_goal.valid&&angular_goal.valid,"angle-metric fixture has no free component");
    require(std::abs(angular_goal.goal.x()-center.x())<1e-12&&std::abs(pixel_goal.goal.y()-center.y())<1e-12,
        "goal choice used image aspect ratio instead of real ray angle");
    // Once a direction has the requested additional clearance, an ever
    // wider opening must not keep pulling the target farther from q.
    metric_cfg.clearance_reward=1.;
    auto unbounded=pc_gvf::depth_angular::chooseSafeGoal(center,center,metric_mask,81,41,nullptr,metric_cfg,&metric_camera);
    metric_cfg.goal_clearance_cap=2.;
    auto capped=pc_gvf::depth_angular::chooseSafeGoal(center,center,metric_mask,81,41,nullptr,metric_cfg,&metric_camera);
    require((capped.goal-center).norm()<(unbounded.goal-center).norm(),"clearance reward still attracts unbounded detours");
    const int cu=static_cast<int>(capped.goal.x()),cv=static_cast<int>(capped.goal.y());
    require(!metric_mask[cv*81+cu],"clearance cap selected an occupied direction");
    // A plane at 3 m from the body is 2.78 m from a camera mounted
    // 22 cm forward. Check physical body clearance under arbitrary rigid
    // transforms and 10 cm motion since acquisition, including native hits.
    Camera origin_camera(65,49,90,68,10);
    SimConfig origin_cfg;origin_cfg.body_radius=.4;origin_cfg.safety_margin=.1;
    origin_cfg.depth_point_stride=1;
    std::vector<double> plane_depth(65*49,2.78);
    const Eigen::Matrix3d origin_rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    const Eigen::Vector3d captured_body(3,1,2);
    const Eigen::Vector3d sensor_origin=captured_body+origin_rotation*Eigen::Vector3d(0,0,.22);
    const auto native_plane=pc_gvf::depth_angular::backprojectObstaclePoints(plane_depth,origin_camera,1);
    for(double motion:{0.,.1})for(bool native:{false,true}) {
        const Eigen::Vector3d body=captured_body+origin_rotation*Eigen::Vector3d(0,0,motion);
        const Eigen::Vector3d target=body+origin_rotation*Eigen::Vector3d(0,0,8);
        auto centered=computeGuidance(plane_depth,origin_camera,origin_cfg,body,Eigen::Vector3d::Zero(),target,
            Eigen::Vector2d(32,24),nullptr,origin_rotation,nullptr,nullptr,nullptr,nullptr,1,true,false,
            nullptr,native?&native_plane:nullptr,true,{},true,&sensor_origin);
        require(std::abs(centered.solution.free_distance[24*65+32]-(2.5-motion))<1e-10,
            "camera/body origin or acquisition-motion compensation changed physical clearance");
    }
    pc_gvf::depth_angular::BinaryMask strip(81*41,0);
    for(int y=0;y<41;++y)for(int x=38;x<=42;++x)strip[y*81+x]=1;
    for(int x=0;x<81;++x)strip[x]=strip[40*81+x]=1;
    for(int y=0;y<41;++y)strip[y*81]=strip[y*81+80]=1;
    SimConfig continuous;continuous.clearance_reward=.018;continuous.hysteresis_weight=0.;
    continuous.deterministic_left_bias=0.;continuous.subpixel_goal=true;
    auto fractional=pc_gvf::depth_angular::chooseSafeGoal(Eigen::Vector2d(40,20.2),Eigen::Vector2d(20,20),strip,81,41,nullptr,continuous);
    auto fractional_next=pc_gvf::depth_angular::chooseSafeGoal(Eigen::Vector2d(40,20.3),Eigen::Vector2d(20,20),strip,81,41,nullptr,continuous);
    require(fractional.valid&&fractional_next.valid&&std::abs(fractional.goal.y()-20.2)<1e-8&&
        std::abs(fractional_next.goal.y()-20.3)<1e-8,"candidate quantized fractional intent into integer goal steps");
    recapture.subpixel_goal=true;recapture.recapture_reference=false;
    Eigen::Vector2d fractional_held(40.2,25.3);
    auto retained_fraction=computeGuidance(clear_depth,camera,recapture,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,2),previous,&fractional_held,Eigen::Matrix3d::Identity(),
        nullptr,nullptr,nullptr,nullptr,1,true,true,nullptr,nullptr,true,{},true);
    require(retained_fraction.goal_valid&&(retained_fraction.returned_goal_pixel-fractional_held).norm()<1e-12,
        "held target discarded continuous subpixel direction");
    SimConfig projection=recapture;projection.finite_goal=false;projection.recapture_reference=true;
    std::fill(clear_depth.begin(),clear_depth.end(),10.);
    for(int y=0;y<48;++y)for(int x=29;x<35;++x)clear_depth[y*64+x]=3.;
    auto projected_goal=[&](double margin) {
        projection.goal_projection_clearance=margin;
        return computeGuidance(clear_depth,camera,projection,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
            Eigen::Vector3d(0,0,8),previous,nullptr,Eigen::Matrix3d::Identity(),nullptr,nullptr,nullptr,nullptr,
            1,true,false,nullptr,nullptr,true,[](const Eigen::Vector2d&){return true;},true);
    };
    const auto wide_goal=projected_goal(0),closer_goal=projected_goal(2);
    const Eigen::Vector2d q_pixel(camera.cx(),camera.cy());
    require(wide_goal.goal_valid&&closer_goal.goal_valid&&
        (closer_goal.returned_goal_pixel-q_pixel).norm()<(wide_goal.returned_goal_pixel-q_pixel).norm(),
        "safe detour projection did not recover operator progress");
    require((closer_goal.returned_goal_pixel-q_pixel).dot(wide_goal.returned_goal_pixel-q_pixel)>0,
        "detour projection switched the selected obstacle side");
    std::cout << "guidance_composition_check: passed " << cases.size()
              << " complete Python fixtures\n";
    return 0;
}
