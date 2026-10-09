#include "pc_gvf/depth_angular_core.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include "pc_gvf/local_depth_path.hpp"
#include "pc_gvf/verified_start.hpp"
#include <deque>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pc_gvf_msgs/msg/position_command.hpp>
#include <rclcpp/create_timer.hpp>
#include <thread>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

namespace pc_gvf {
namespace depth_angular {

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

struct TimingSeries
{
    void add(double value)
    {
        if (std::isfinite(value) && value >= 0.0) {
            values.push_back(value);
            sum += value;
            maximum = std::max(maximum, value);
        }
    }

    double mean() const
    {
        return values.empty() ? 0.0 : sum / values.size();
    }

    double percentile(double fraction) const
    {
        if (values.empty()) {
            return 0.0;
        }
        std::vector<double> ordered = values;
        std::sort(ordered.begin(), ordered.end());
        const std::size_t index = static_cast<std::size_t>(std::nearbyint(
            fraction * static_cast<double>(ordered.size() - 1)));
        return ordered[std::min(index, ordered.size() - 1)];
    }

    std::vector<double> values;
    double sum = 0.0;
    double maximum = 0.0;
};
}

class DepthAngularControllerNode : public rclcpp::Node
{
public:
    DepthAngularControllerNode()
    : Node("depth_angular_controller"),
      intent_time_(0, 0, get_clock()->get_clock_type()),
      active_goal_time_(0, 0, get_clock()->get_clock_type()),
      last_guidance_time_(0, 0, get_clock()->get_clock_type())
    {
        frame_id_ = declare_parameter<std::string>("frame_id", "world");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
        depth_topic_ = declare_parameter<std::string>(
            "depth_topic", "/sim/depth/image_raw");
        camera_info_topic_ = declare_parameter<std::string>(
            "camera_info_topic", "/sim/depth/camera_info");
        horizontal_360_enabled_ = declare_parameter<bool>(
            "horizontal_360_enabled", false);
        depth_topics_ = {{
            depth_topic_,
            declare_parameter<std::string>(
                "left_depth_topic", "/sim/depth_left/image_raw"),
            declare_parameter<std::string>(
                "back_depth_topic", "/sim/depth_back/image_raw"),
            declare_parameter<std::string>(
                "right_depth_topic", "/sim/depth_right/image_raw")}};
        camera_info_topics_ = {{
            camera_info_topic_,
            declare_parameter<std::string>(
                "left_camera_info_topic", "/sim/depth_left/camera_info"),
            declare_parameter<std::string>(
                "back_camera_info_topic", "/sim/depth_back/camera_info"),
            declare_parameter<std::string>(
                "right_camera_info_topic", "/sim/depth_right/camera_info")}};
        camera_yaw_offsets_ = {0.0, 0.5 * kPi, kPi, -0.5 * kPi};
        intent_topic_ = declare_parameter<std::string>(
            "human_intent_topic", "/human_intent");
        command_topic_ = declare_parameter<std::string>(
            "cmd_topic", "/position_cmd");
        fixed_forward_intent_ = declare_parameter<bool>(
            "fixed_forward_intent", false);
        intent_timeout_ = std::max(
            0.05, declare_parameter<double>("human_intent_timeout", 0.5));
        max_depth_age_ = std::max(
            0.05, declare_parameter<double>("max_depth_age", 0.3));
        target_width_ = std::max(
            8, static_cast<int>(declare_parameter<std::int64_t>("angular_width", 48)));
        target_height_ = std::max(
            8, static_cast<int>(declare_parameter<std::int64_t>("angular_height", 36)));
        obstacle_clear_frames_ = std::max(
            1, std::min(255, static_cast<int>(declare_parameter<std::int64_t>(
                "obstacle_clear_frames", 3))));
        const auto contract=declare_parameter<std::string>("paper_depth_contract", "unknown_nonfinite");
        if(contract!="unknown_nonfinite"&&contract!="isaac_rendered_z")throw std::invalid_argument("unknown depth contract");
        isaac_depth_contract_=contract=="isaac_rendered_z";
        max_depth_ = std::max(
            0.2, declare_parameter<double>("max_depth", 10.0));
        forward_lookahead_ = std::max(
            1.0, declare_parameter<double>("forward_lookahead", 8.0));
        use_fixed_goal_ = declare_parameter<bool>("use_fixed_goal", false);
        fixed_goal_ = Eigen::Vector3d(
            declare_parameter<double>("goal_x", 7.2),
            declare_parameter<double>("goal_y", 0.0),
            declare_parameter<double>("goal_z", 1.2));
        use_reference_line_ = declare_parameter<bool>("use_reference_line", true);
        stop_at_goal_ = declare_parameter<bool>("stop_at_goal", false);
        goal_tolerance_ = std::max(
            0.05, declare_parameter<double>("goal_tolerance", 0.3));
        command_accel_limit_ = std::max(
            0.1, declare_parameter<double>("command_accel_limit", 4.0));
        goal_hold_time_ = std::max(
            0.0, declare_parameter<double>("safe_goal_hold_time", 0.35));
        max_vertical_speed_ = std::max(
            0.0, declare_parameter<double>("max_vertical_speed", 0.0));
        max_reference_speed_ = std::max(
            0.05, declare_parameter<double>("max_speed", 2.0));
        command_yaw_ = declare_parameter<double>("command_yaw", 0.0);
        field_radius_ = std::max(
            0.2, declare_parameter<double>("field_radius", 1.5));
        field_publish_rate_ = std::max(
            1.0, declare_parameter<double>("field_publish_rate", 15.0));
        performance_report_interval_ = std::max(
            1.0, declare_parameter<double>("performance_report_interval", 5.0));
        performance_log_path_ = declare_parameter<std::string>(
            "performance_log_path", "");
        performance_run_id_ = declare_parameter<std::string>(
            "performance_run_id", "unspecified");
        const double direction_reanchor_degrees = declare_parameter<double>(
            "direction_reanchor_deg", 15.0);
        reanchor_cosine_ = std::cos(direction_reanchor_degrees * kPi / 180.0);

        config_.reference_speed = declare_parameter<double>("speed", 1.0);
        config_.max_accel = command_accel_limit_;
        config_.max_direction_rate = std::max(
            0.05, declare_parameter<double>("max_direction_rate", 1.60));
        config_.continuous_harmonic_guidance = declare_parameter<bool>(
            "continuous_harmonic_guidance", true);
        config_.harmonic_gradient_radius_cells = std::max(
            1, static_cast<int>(declare_parameter<std::int64_t>(
                "harmonic_gradient_radius_cells", 3)));
        config_.harmonic_gradient_lookahead_cells = std::max(
            0.25, declare_parameter<double>(
                "harmonic_gradient_lookahead_cells", 2.5));
        config_.hysteresis_weight = std::max(
            0.0, declare_parameter<double>("safe_goal_hysteresis_weight", 0.40));
        config_.body_radius = declare_parameter<double>("body_radius", 0.25);
        config_.safety_margin = declare_parameter<double>("safety_margin", 0.175);
        config_.rollout_margin = declare_parameter<double>("rollout_margin", 0.10);
        config_.deterministic_left_bias = declare_parameter<double>("safe_goal_left_bias", 0.012);
        config_.goal_projection_clearance=std::max(0.,declare_parameter<double>("paper_goal_projection_clearance",0.));
        config_.goal_clearance_cap=std::max(0.,declare_parameter<double>("safe_goal_clearance_cap",0.));
        config_.clearance_reward = std::max(0.0, declare_parameter<double>("safe_goal_clearance_reward", 0.018));
        config_.planning_horizon = declare_parameter<double>(
            "planning_horizon", 3.0);
        config_.horizontal_only = horizontal_360_enabled_;

        paper_enabled_ = declare_parameter<bool>("paper_enabled", false);
        if (paper_enabled_) {
            chart_width_=std::max(8,static_cast<int>(declare_parameter<std::int64_t>("paper_chart_width",24)));
            chart_height_=std::max(8,static_cast<int>(declare_parameter<std::int64_t>("paper_chart_height",18)));
            paper_depth_proposal_=declare_parameter<bool>("paper_depth_proposal",false);
            paper_local_path_proposal_=declare_parameter<bool>("paper_local_path_proposal",false);
            paper_subpixel_proposal_=declare_parameter<bool>("paper_subpixel_proposal",false);
            paper_vehicle_origin_proposal_=declare_parameter<bool>("paper_vehicle_origin_proposal",false);
            paper_intent_corridor_=declare_parameter<bool>("paper_intent_corridor",false);
            paper_angular_goal_cost_=declare_parameter<bool>("paper_angular_goal_cost",false);
            sampled_proposal_=declare_parameter<bool>("paper_sampled_proposal",false);
            native_proposal_stride_=std::max(0,static_cast<int>(declare_parameter<int>("paper_native_proposal_stride",0)));
            PaperConfig paper_config;
            paper_config.motion = config_;
            paper_config.motion.brake_accel=std::min(config_.brake_accel,config_.max_accel);
            paper_config.motion.delay=declare_parameter<double>("paper_pipeline_delay", .10);
            paper_config.motion.rollout_horizon=declare_parameter<double>("paper_rollout_horizon", .80);
            paper_config.max_speed = max_reference_speed_;
            paper_config.max_vertical_speed = max_vertical_speed_;
            paper_config.observation_timeout = max_depth_age_;
            paper_config.retain_verified_travel=declare_parameter<bool>("paper_static_verified_travel", false);
            paper_config.retain_certified_volume=declare_parameter<bool>("paper_retain_certified_volume", false);
            paper_config.history_duration = declare_parameter<double>("paper_observation_history", 0.0);
            paper_config.history_max_observations=static_cast<int>(declare_parameter<int>("paper_history_max_observations",32));
            paper_config.history_sample_interval=declare_parameter<double>("paper_history_sample_interval", .5);
            paper_config.history_uncertainty_rate=declare_parameter<double>("paper_history_uncertainty_rate", .1);
            paper_config.uncertainty_rate=declare_parameter<double>("paper_observation_uncertainty_rate",.1);
            const bool static_scene=declare_parameter<bool>("paper_static_scene",false);
            if(paper_config.uncertainty_rate==0. && (!static_scene||!isaac_depth_contract_))
                throw std::invalid_argument("zero temporal inflation requires explicit static Isaac depth contract");

            seed_scene_=declare_parameter<std::string>("paper_seed_scene", "");
            seed_occupancy_=declare_parameter<std::string>("paper_seed_occupancy", "");
            seed_radius_=declare_parameter<double>("paper_seed_radius", 2.5);
            paper_config.field_interval = declare_parameter<double>("paper_field_interval", 0.05);
            paper_config.command_change_angle = declare_parameter<double>("paper_command_change_angle", 0.05);
            paper_config.intent_guard=declare_parameter<bool>("paper_intent_guard",true);
            paper_config.smooth_speed=declare_parameter<bool>("paper_smooth_speed",true);
            paper_config.model_velocity_shaping=declare_parameter<bool>("paper_model_velocity_shaping",false);
            paper_config.proposal_jerk_limit=declare_parameter<double>("paper_proposal_jerk_limit",8.);
            paper_config.proposal_response_time=declare_parameter<double>("paper_proposal_response_time",.22);
            paper_config.response_preview=declare_parameter<bool>("paper_response_preview",false);
            paper_config.certified_direct=declare_parameter<bool>("paper_certified_direct",false);
            paper_config.transverse_gain = declare_parameter<double>("paper_transverse_gain", 1.0);
            paper_config.minimum_lookahead=declare_parameter<double>("paper_minimum_lookahead", .50);
            paper_config.chart_speed = declare_parameter<double>("paper_chart_speed", 32.0);
            paper_config.feedback = declare_parameter<bool>("paper_feedback", true);
            paper_config.continuation = declare_parameter<bool>("paper_continuation", true);
            paper_config.coarse_fine = declare_parameter<bool>("paper_coarse_fine", true);
            paper_config.continuous_certificates=declare_parameter<bool>("paper_continuous_certificates",true);
            paper_config.adaptive_lookahead=declare_parameter<bool>("paper_adaptive_lookahead",false);
            paper_config.adaptive_grid=declare_parameter<bool>("paper_adaptive_grid",false);
            paper_config.certificate_resolution=declare_parameter<double>("paper_certificate_resolution",.005);
            paper_config.restart_clearance=declare_parameter<double>("paper_restart_clearance",0.);
            replay_directory_=declare_parameter<std::string>("paper_replay_directory", "");
            paper_ = std::make_unique<PaperGuidance>(paper_config);
            paper_diagnostics_ = create_publisher<std_msgs::msg::String>("~/paper_diagnostics", 10);
        }
        const auto camera_offset = declare_parameter<std::vector<double>>(
            "camera_offset", {0.22, 0.0, 0.02});
        if (camera_offset.size() != 3) throw std::invalid_argument("camera_offset must have 3 values");
        camera_offset_ = Eigen::Vector3d(camera_offset[0], camera_offset[1], camera_offset[2]);

        command_publisher_ = create_publisher<pc_gvf_msgs::msg::PositionCommand>(
            command_topic_, 10);
        rclcpp::QoS latched_qos(1);
        latched_qos.transient_local();
        status_publisher_ = create_publisher<std_msgs::msg::String>(
            "~/status", latched_qos);
        field_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pc_gvf/angular_field", latched_qos);

        // This group is serviced by its own stable executor thread. A
        // controller callback must not migrate between OpenMP worker pools.
        sensor_callback_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group=sensor_callback_group_;
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS().keep_last(1),
            [this](nav_msgs::msg::Odometry::SharedPtr message) {
                std::lock_guard<std::mutex> lock(mutex_);
                odometry_ = message;
                if (!odometry_history_.empty() &&
                    messageTime(message->header.stamp) < messageTime(odometry_history_.back()->header.stamp))
                    odometry_history_.clear();
                odometry_history_.push_back(std::move(message));
                while (odometry_history_.size() > 300) odometry_history_.pop_front();
            }, sensor_options);
        const std::size_t view_count = horizontal_360_enabled_ ? 4 : 1;
        for (std::size_t index = 0; index < view_count; ++index) {
            depth_times_[index] = rclcpp::Time(
                0, 0, get_clock()->get_clock_type());
            camera_info_subscriptions_[index] =
                create_subscription<sensor_msgs::msg::CameraInfo>(
                    camera_info_topics_[index], rclcpp::SensorDataQoS().keep_last(1),
                    [this, index](sensor_msgs::msg::CameraInfo::SharedPtr message) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        camera_infos_[index] = std::move(message);
                    }, sensor_options);
            depth_subscriptions_[index] =
                create_subscription<sensor_msgs::msg::Image>(
                    depth_topics_[index], rclcpp::SensorDataQoS().keep_last(1),
                    [this, index](sensor_msgs::msg::Image::SharedPtr message) {
                        depthCallback(std::move(message), index);
                    }, sensor_options);
        }
        intent_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            intent_topic_, 10,
            std::bind(&DepthAngularControllerNode::intentCallback, this,
                      std::placeholders::_1), sensor_options);
        timer_ = rclcpp::create_timer(
            this, get_clock(), rclcpp::Duration::from_seconds(0.02),
            std::bind(&DepthAngularControllerNode::control, this));

        RCLCPP_INFO(
            get_logger(),
            "C++ controller commands publish on %s; horizontal views=%zu",
            command_publisher_->get_topic_name(), view_count);
        performance_start_wall_ = std::chrono::steady_clock::now();
        last_performance_report_wall_ = performance_start_wall_;
    }

    ~DepthAngularControllerNode() override
    {
        reportPerformance(true);
    }

    rclcpp::CallbackGroup::SharedPtr sensorCallbackGroup() const {return sensor_callback_group_;}

private:
    static double finiteOrZero(double value)
    {
        if (std::isnan(value)) {
            return 0.0;
        }
        if (std::isinf(value)) {
            return std::signbit(value)
                ? std::numeric_limits<double>::lowest()
                : std::numeric_limits<double>::max();
        }
        return value;
    }

    static bool vectorFinite(const Eigen::Vector3d& value)
    {
        return value.allFinite();
    }

    static Eigen::Vector2d directionPixel(
        const Camera& camera,
        const Eigen::Matrix3d& rotation_camera_from_world,
        const Eigen::Vector3d& direction_world)
    {
        const Eigen::Vector3d direction_camera =
            rotation_camera_from_world * direction_world;
        Eigen::Vector2d pixel;
        if (camera.pixelFromDirection(direction_camera, &pixel)) {
            return pixel;
        }
        if (direction_camera.z() > kEpsilon) {
            pixel = Eigen::Vector2d(
                camera.fx() * direction_camera.x() / direction_camera.z() +
                    camera.cx(),
                camera.fy() * direction_camera.y() / direction_camera.z() +
                    camera.cy());
        } else {
            pixel = Eigen::Vector2d(
                direction_camera.x() >= 0.0 ? camera.width() - 2.0 : 1.0,
                camera.cy());
        }
        pixel.x() = std::max(
            1.0, std::min(pixel.x(), camera.width() - 2.0));
        pixel.y() = std::max(
            1.0, std::min(pixel.y(), camera.height() - 2.0));
        return pixel;
    }

    rclcpp::Time messageTime(const builtin_interfaces::msg::Time& stamp)
    {
        const rclcpp::Time converted(stamp, get_clock()->get_clock_type());
        return converted.nanoseconds() == 0 ? get_clock()->now() : converted;
    }

    double age(const rclcpp::Time& now, const rclcpp::Time& older, bool present) const
    {
        return present
            ? (now - older).seconds()
            : std::numeric_limits<double>::infinity();
    }

    static float readFloat32(const std::uint8_t* bytes, bool big_endian)
    {
        std::uint32_t bits;
        if (big_endian) {
            bits = (static_cast<std::uint32_t>(bytes[0]) << 24) |
                (static_cast<std::uint32_t>(bytes[1]) << 16) |
                (static_cast<std::uint32_t>(bytes[2]) << 8) |
                static_cast<std::uint32_t>(bytes[3]);
        } else {
            bits = static_cast<std::uint32_t>(bytes[0]) |
                (static_cast<std::uint32_t>(bytes[1]) << 8) |
                (static_cast<std::uint32_t>(bytes[2]) << 16) |
                (static_cast<std::uint32_t>(bytes[3]) << 24);
        }
        float result;
        std::memcpy(&result, &bits, sizeof(result));
        return result;
    }

    static std::uint16_t readUint16(const std::uint8_t* bytes, bool big_endian)
    {
        if (big_endian) {
            return static_cast<std::uint16_t>(
                (static_cast<std::uint16_t>(bytes[0]) << 8) |
                static_cast<std::uint16_t>(bytes[1]));
        }
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[0]) |
            (static_cast<std::uint16_t>(bytes[1]) << 8));
    }

    std::vector<double> decodeAndResizeDepth(
        const sensor_msgs::msg::Image& message, bool conservative = true) const
    {
        std::size_t item_size;
        double scale;
        if (message.encoding == "32FC1") {
            item_size = 4;
            scale = 1.0;
        } else if (message.encoding == "16UC1") {
            item_size = 2;
            scale = 0.001;
        } else {
            throw std::invalid_argument(
                "depth encoding must be 32FC1 or 16UC1");
        }
        if (message.step < message.width * item_size ||
            message.step % item_size != 0 ||
            message.data.size() < static_cast<std::size_t>(message.height) * message.step) {
            throw std::invalid_argument("invalid depth row step or data size");
        }

        if (paper_enabled_ && conservative) {
            std::vector<double> raw(message.width * message.height);
            for (std::size_t v = 0; v < message.height; ++v) {
                for (std::size_t u = 0; u < message.width; ++u) {
                    const auto* bytes = message.data.data() + v * message.step + u * item_size;
                    raw[v * message.width + u] = message.encoding == "32FC1"
                        ? static_cast<double>(readFloat32(bytes, message.is_bigendian != 0))
                        : scale * readUint16(bytes, message.is_bigendian != 0);
                }
            }
            return conservativeDepthResize(raw, message.width, message.height,
                target_width_, target_height_, max_depth_, isaac_depth_contract_);
        }
        std::vector<double> resized(
            static_cast<std::size_t>(target_width_) * target_height_);
        for (int target_v = 0; target_v < target_height_; ++target_v) {
            const int source_v = static_cast<int>(std::nearbyint(
                static_cast<double>(target_v) * (message.height - 1) /
                std::max(1, target_height_ - 1)));
            for (int target_u = 0; target_u < target_width_; ++target_u) {
                const int source_u = static_cast<int>(std::nearbyint(
                    static_cast<double>(target_u) * (message.width - 1) /
                    std::max(1, target_width_ - 1)));
                const std::uint8_t* bytes = message.data.data() +
                    static_cast<std::size_t>(source_v) * message.step +
                    static_cast<std::size_t>(source_u) * item_size;
                double value = message.encoding == "32FC1"
                    ? static_cast<double>(readFloat32(bytes, message.is_bigendian != 0))
                    : scale * readUint16(bytes, message.is_bigendian != 0);
                if (!std::isfinite(value) || value < 0.0) {
                    value = 0.0;
                } else if (value > max_depth_) {
                    value = max_depth_;
                }
                resized[static_cast<std::size_t>(target_v) * target_width_ +
                    target_u] = value;
            }
        }
        return resized;
    }

    void depthCallback(
        sensor_msgs::msg::Image::SharedPtr message,
        std::size_t view_index)
    {
        sensor_msgs::msg::CameraInfo::SharedPtr information;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            information = camera_infos_[view_index];
        }
        if (!information || information->width != message->width ||
            information->height != message->height) {
            return;
        }
        try {
            std::vector<double> depth = decodeAndResizeDepth(*message);
            std::vector<double> proposal_depth=paper_depth_proposal_&&sampled_proposal_?decodeAndResizeDepth(*message,false):std::vector<double>();
            std::shared_ptr<Camera> camera = std::make_shared<Camera>(
                target_width_, target_height_,
                90.0, 68.0,
                max_depth_);
            const double scale_x =
                static_cast<double>(target_width_) / message->width;
            const double scale_y =
                static_cast<double>(target_height_) / message->height;
            camera->setIntrinsics(
                information->k[0] * scale_x,
                information->k[4] * scale_y,
                (information->k[2] + (paper_enabled_&&isaac_depth_contract_?0.0:0.5)) * scale_x - 0.5,
                (information->k[5] + (paper_enabled_&&isaac_depth_contract_?0.0:0.5)) * scale_y - 0.5);
            auto hits=std::make_shared<std::vector<Eigen::Vector3d>>();
            if(paper_enabled_) {
                hits->reserve(message->width*message->height);
                const std::size_t item_size=message->encoding=="32FC1"?4:2;
                const double pixel_offset=isaac_depth_contract_?.5:0.0;
                for(std::size_t v=0;v<message->height;++v)for(std::size_t u=0;u<message->width;++u) {
                    const auto* bytes=message->data.data()+v*message->step+u*item_size;
                    const double d=item_size==4?static_cast<double>(readFloat32(bytes,message->is_bigendian)):
                        .001*readUint16(bytes,message->is_bigendian);
                    if(!std::isfinite(d)||d<=0||d>=max_depth_)continue;
                    hits->emplace_back(d*(u+pixel_offset-information->k[2])/information->k[0],
                        d*(v+pixel_offset-information->k[5])/information->k[4],d);
                }
            }
            const rclcpp::Time stamp = messageTime(message->header.stamp);
            std::lock_guard<std::mutex> lock(mutex_);
            depths_[view_index] = std::move(depth);
            proposal_depths_[view_index]=std::move(proposal_depth);
            obstacle_points_[view_index]=hits;
            cameras_[view_index] = std::move(camera);
            depth_times_[view_index] = stamp;
            has_depth_times_[view_index] = true;
            ++depth_versions_[view_index];
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "%s", error.what());
        }
    }

    void intentCallback(geometry_msgs::msg::TwistStamped::SharedPtr message)
    {
        if (fixed_forward_intent_) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        intent_ = Eigen::Vector3d(
            message->twist.linear.x,
            message->twist.linear.y,
            message->twist.linear.z);
        intent_time_ = get_clock()->now();
        has_intent_ = true;
    }

    void resetPlanningHistory()
    {
        cached_local_path_=LocalDepthPath();local_path_time_=-1.;
        has_previous_pixel_ = false;
        has_previous_goal_pixel_ = false;
        has_previous_direction_world_ = false;
        has_previous_goal_direction_world_ = false;
        has_reference_direction_ = false;
        has_active_goal_time_ = false;
        previous_potential_.clear();
        planning_mask_state_.clear();
        planning_clear_counts_.clear();
        last_hysteresis_depth_version_ =
            std::numeric_limits<std::uint64_t>::max();
        last_field_version_ = std::numeric_limits<std::uint64_t>::max();
    }

    void publishCommand(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity)
    {
        pc_gvf_msgs::msg::PositionCommand message;
        message.header.stamp = stamp;
        message.header.frame_id = frame_id_;
        message.position.x = position.x();
        message.position.y = position.y();
        message.position.z = position.z();
        message.velocity.x = velocity.x();
        message.velocity.y = velocity.y();
        message.velocity.z = velocity.z();
        message.yaw = command_yaw_;
        message.trajectory_flag =
            pc_gvf_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
        const auto publish_start = std::chrono::steady_clock::now();
        command_publisher_->publish(message);
        const auto publish_end = std::chrono::steady_clock::now();
        publish_call_ms_.add(std::chrono::duration<double, std::milli>(
            publish_end - publish_start).count());
        callback_to_publish_ms_.add(std::chrono::duration<double, std::milli>(
            publish_end - current_control_wall_start_).count());
        if (has_last_command_publish_wall_) {
            command_interval_ms_.add(std::chrono::duration<double, std::milli>(
                publish_end - last_command_publish_wall_).count());
            command_ros_interval_ms_.add(std::max(
                0.0, (stamp - last_command_stamp_).seconds() * 1000.0));
        } else {
            first_command_publish_wall_ = publish_end;
            last_performance_report_wall_ = publish_end;
        }
        last_command_publish_wall_ = publish_end;
        last_command_stamp_ = stamp;
        has_last_command_publish_wall_ = true;
        ++command_frame_count_;
        maybeReportPerformance(publish_end);
    }

    void maybeReportPerformance(
        const std::chrono::steady_clock::time_point& now)
    {
        if (std::chrono::duration<double>(
                now - last_performance_report_wall_).count() >=
            performance_report_interval_) {
            reportPerformance(false);
            last_performance_report_wall_ = now;
        }
    }

    void reportPerformance(bool final)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::max(
            1.0e-9, std::chrono::duration<double>(
                now - performance_start_wall_).count());
        const double active_elapsed = has_last_command_publish_wall_
            ? std::max(1.0e-9, std::chrono::duration<double>(
                last_command_publish_wall_ - first_command_publish_wall_).count())
            : elapsed;
        const double command_wall_fps = command_frame_count_ > 1
            ? (command_frame_count_ - 1) / active_elapsed : 0.0;
        const double command_ros_fps = command_ros_interval_ms_.mean() > 0.0
            ? 1000.0 / command_ros_interval_ms_.mean() : 0.0;
        const double guidance_wall_fps = guidance_frame_count_ > 1 &&
            has_guidance_wall_span_
            ? (guidance_frame_count_ - 1) /
                std::max(1.0e-9, std::chrono::duration<double>(
                    last_guidance_wall_ - first_guidance_wall_).count())
            : 0.0;
        RCLCPP_INFO(
            get_logger(),
            "[PERF CTRL%s] run=%s elapsed=%.2fs callbacks=%llu "
            "commands=%llu guidance=%llu command_fps(ros/wall)=%.2f/%.2f "
            "guidance_wall_fps=%.2f "
            "avoid_ms(mean/p50/p95/max)=%.3f/%.3f/%.3f/%.3f "
            "cycle_ms(mean/p95/max)=%.3f/%.3f/%.3f "
            "depth_to_cmd_ms(mean/p95/max)=%.3f/%.3f/%.3f "
            "callback_to_publish_ms(mean/p95/max)=%.3f/%.3f/%.3f",
            final ? " FINAL" : "", performance_run_id_.c_str(), elapsed,
            static_cast<unsigned long long>(control_callback_count_),
            static_cast<unsigned long long>(command_frame_count_),
            static_cast<unsigned long long>(guidance_frame_count_),
            command_ros_fps, command_wall_fps, guidance_wall_fps,
            avoidance_compute_ms_.mean(), avoidance_compute_ms_.percentile(0.50),
            avoidance_compute_ms_.percentile(0.95), avoidance_compute_ms_.maximum,
            command_interval_ms_.mean(), command_interval_ms_.percentile(0.95),
            command_interval_ms_.maximum, depth_to_command_ms_.mean(),
            depth_to_command_ms_.percentile(0.95), depth_to_command_ms_.maximum,
            callback_to_publish_ms_.mean(),
            callback_to_publish_ms_.percentile(0.95),
            callback_to_publish_ms_.maximum);
        if(paper_enabled_)RCLCPP_INFO(get_logger(),"EVIDENCE_COST ingest(mean/p95/max)=%.3f/%.3f/%.3f record=%.3f/%.3f/%.3f ms",
            evidence_ingest_ms_.mean(),evidence_ingest_ms_.percentile(.95),evidence_ingest_ms_.maximum,
            evidence_record_ms_.mean(),evidence_record_ms_.percentile(.95),evidence_record_ms_.maximum);
        if(paper_enabled_)RCLCPP_INFO(get_logger(),"EVIDENCE_PHASE per_view_mean_ms balls=%.3f tubes=%.3f history=%.3f eviction=%.3f",
            evidence_phase_ms_[0].mean(),evidence_phase_ms_[1].mean(),evidence_phase_ms_[2].mean(),evidence_phase_ms_[3].mean());
        if (!final || performance_log_path_.empty()) {
            return;
        }
        std::ostringstream record;
        record << std::fixed << std::setprecision(3)
               << "\n#### Controller — " << performance_run_id_ << "\n\n"
               << "- Wall duration: " << elapsed << " s\n"
               << "- Timer callbacks / command frames / guidance frames: "
               << control_callback_count_ << " / " << command_frame_count_
               << " / " << guidance_frame_count_ << "\n"
               << "- Command FPS (ROS simulation time / active wall time): "
               << command_ros_fps << " / " << command_wall_fps << " Hz\n"
               << "- Guidance FPS (active wall time): " << guidance_wall_fps
               << " Hz\n"
               << "- Command interval in ROS time mean / P50 / P95 / max: "
               << command_ros_interval_ms_.mean() << " / "
               << command_ros_interval_ms_.percentile(0.50) << " / "
               << command_ros_interval_ms_.percentile(0.95) << " / "
               << command_ros_interval_ms_.maximum << " ms\n"
               << "- Avoidance compute mean / P50 / P95 / max: "
               << avoidance_compute_ms_.mean() << " / "
               << avoidance_compute_ms_.percentile(0.50) << " / "
               << avoidance_compute_ms_.percentile(0.95) << " / "
               << avoidance_compute_ms_.maximum << " ms\n"
               << "- Command interval mean / P50 / P95 / max: "
               << command_interval_ms_.mean() << " / "
               << command_interval_ms_.percentile(0.50) << " / "
               << command_interval_ms_.percentile(0.95) << " / "
               << command_interval_ms_.maximum << " ms\n"
               << "- Latest-depth stamp to command mean / P50 / P95 / max: "
               << depth_to_command_ms_.mean() << " / "
               << depth_to_command_ms_.percentile(0.50) << " / "
               << depth_to_command_ms_.percentile(0.95) << " / "
               << depth_to_command_ms_.maximum << " ms (ROS simulation time)\n"
               << "- Control callback to publish mean / P50 / P95 / max: "
               << callback_to_publish_ms_.mean() << " / "
               << callback_to_publish_ms_.percentile(0.50) << " / "
               << callback_to_publish_ms_.percentile(0.95) << " / "
               << callback_to_publish_ms_.maximum << " ms (steady clock)\n"
               << "- DDS publish call mean / P95 / max: "
               << publish_call_ms_.mean() << " / "
               << publish_call_ms_.percentile(0.95) << " / "
               << publish_call_ms_.maximum << " ms (steady clock)\n";
        std::ofstream output(performance_log_path_, std::ios::app);
        if (output) {
            output << record.str();
        } else {
            RCLCPP_WARN(get_logger(), "cannot append performance log: %s",
                        performance_log_path_.c_str());
        }
    }

    void setStatus(const std::string& value)
    {
        if (value == last_status_) {
            return;
        }
        std_msgs::msg::String message;
        message.data = value;
        status_publisher_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", value.c_str());
        last_status_ = value;
    }

    void stop(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& position,
        const std::string& status)
    {
        command_.setZero();
        if (paper_) paper_->reset();
        has_last_guidance_time_ = false;
        publishCommand(stamp, position, command_);
        setStatus(status);
    }

    static geometry_msgs::msg::Point point(const Eigen::Vector3d& value)
    {
        geometry_msgs::msg::Point result;
        result.x = value.x();
        result.y = value.y();
        result.z = value.z();
        return result;
    }

    static std_msgs::msg::ColorRGBA color(
        double red, double green, double blue, double alpha = 1.0)
    {
        std_msgs::msg::ColorRGBA result;
        result.r = static_cast<float>(red);
        result.g = static_cast<float>(green);
        result.b = static_cast<float>(blue);
        result.a = static_cast<float>(alpha);
        return result;
    }

    visualization_msgs::msg::Marker marker(
        const rclcpp::Time& stamp,
        const std::string& marker_namespace,
        int marker_id,
        int marker_type) const
    {
        visualization_msgs::msg::Marker result;
        result.header.stamp = stamp;
        result.header.frame_id = frame_id_;
        result.ns = marker_namespace;
        result.id = marker_id;
        result.type = marker_type;
        result.action = visualization_msgs::msg::Marker::ADD;
        result.pose.orientation.w = 1.0;
        return result;
    }

    static void potentialGradient(
        const AngularSolution& solution,
        int width,
        int height,
        std::vector<double>* gradient_x,
        std::vector<double>* gradient_y)
    {
        const std::size_t size = static_cast<std::size_t>(width) * height;
        gradient_x->assign(size, 0.0);
        gradient_y->assign(size, 0.0);
        std::vector<double> filled = solution.potential;
        std::vector<std::pair<int, int>> finite_pixels;
        finite_pixels.reserve(size);
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                const std::size_t index = static_cast<std::size_t>(v) * width + u;
                if (std::isfinite(filled[index])) {
                    finite_pixels.emplace_back(u, v);
                }
            }
        }
        if (finite_pixels.empty()) {
            return;
        }
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                const std::size_t index = static_cast<std::size_t>(v) * width + u;
                if (std::isfinite(filled[index])) {
                    continue;
                }
                int best_distance = std::numeric_limits<int>::max();
                std::size_t best_index = 0;
                for (const auto& pixel : finite_pixels) {
                    const int du = u - pixel.first;
                    const int dv = v - pixel.second;
                    const int distance = du * du + dv * dv;
                    if (distance < best_distance) {
                        best_distance = distance;
                        best_index = static_cast<std::size_t>(pixel.second) * width +
                            pixel.first;
                    }
                }
                filled[index] = filled[best_index];
            }
        }
        const auto value = [&filled, width](int u, int v) {
            return filled[static_cast<std::size_t>(v) * width + u];
        };
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                const std::size_t index = static_cast<std::size_t>(v) * width + u;
                if (solution.planning_mask[index] != 0) {
                    continue;
                }
                const double dx = u == 0
                    ? value(1, v) - value(0, v)
                    : (u == width - 1
                        ? value(width - 1, v) - value(width - 2, v)
                        : 0.5 * (value(u + 1, v) - value(u - 1, v)));
                const double dy = v == 0
                    ? value(u, 1) - value(u, 0)
                    : (v == height - 1
                        ? value(u, height - 1) - value(u, height - 2)
                        : 0.5 * (value(u, v + 1) - value(u, v - 1)));
                (*gradient_x)[index] = -dx;
                (*gradient_y)[index] = -dy;
            }
        }
    }

    void publishField(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& position,
        const Eigen::Matrix3d& rotation_world_from_camera,
        const Camera& camera,
        const AngularSolution& solution)
    {
        constexpr int step = 3;
        const auto project = [&](const Eigen::Vector2d& pixel) -> Eigen::Vector3d {
            return position + rotation_world_from_camera *
                (field_radius_ * camera.rayFromPixel(pixel));
        };
        visualization_msgs::msg::MarkerArray output;

        auto unsafe = marker(
            stamp, "fov_unsafe", 0, visualization_msgs::msg::Marker::POINTS);
        unsafe.scale.x = 0.045;
        unsafe.scale.y = 0.045;
        unsafe.color = color(1.0, 0.12, 0.05, 0.55);
        for (int v = 0; v < camera.height(); v += step) {
            for (int u = 0; u < camera.width(); u += step) {
                if (solution.planning_mask[
                        static_cast<std::size_t>(v) * camera.width() + u] != 0) {
                    unsafe.points.push_back(point(project(Eigen::Vector2d(u, v))));
                }
            }
        }
        output.markers.push_back(std::move(unsafe));

        std::vector<double> gradient_x;
        std::vector<double> gradient_y;
        if (solution.field_valid) {
            potentialGradient(
                solution, camera.width(), camera.height(),
                &gradient_x, &gradient_y);
        } else {
            const std::size_t size = static_cast<std::size_t>(camera.width()) *
                camera.height();
            gradient_x.assign(size, 0.0);
            gradient_y.assign(size, 0.0);
        }
        auto quiver = marker(
            stamp, "angular_gvf", 0,
            visualization_msgs::msg::Marker::LINE_LIST);
        quiver.scale.x = 0.012;
        quiver.color = color(1.0, 1.0, 1.0, 1.0);
        const double pulse_time = stamp.seconds();
        const auto positiveModulo = [](double value) {
            value = std::fmod(value, 1.0);
            return value < 0.0 ? value + 1.0 : value;
        };
        const auto addSegment = [&quiver](
            const Eigen::Vector3d& first,
            const Eigen::Vector3d& second,
            const std_msgs::msg::ColorRGBA& first_color,
            const std_msgs::msg::ColorRGBA& second_color) {
            quiver.points.push_back(point(first));
            quiver.points.push_back(point(second));
            quiver.colors.push_back(first_color);
            quiver.colors.push_back(second_color);
        };
        for (int v = 1; v < camera.height() - 1; v += step) {
            for (int u = 1; u < camera.width() - 1; u += step) {
                const std::size_t index =
                    static_cast<std::size_t>(v) * camera.width() + u;
                if (solution.planning_mask[index] != 0) {
                    continue;
                }
                double flow_u = gradient_x[index];
                double flow_v = gradient_y[index];
                double magnitude = std::hypot(flow_u, flow_v);
                if (magnitude <= 1.0e-8) {
                    flow_u = solution.goal_pixel.x() - u;
                    flow_v = solution.goal_pixel.y() - v;
                    magnitude = std::hypot(flow_u, flow_v);
                }
                if (magnitude <= 1.0e-8) {
                    continue;
                }
                Eigen::Vector2d end_pixel(
                    u + 2.0 * flow_u / magnitude,
                    v + 2.0 * flow_v / magnitude);
                end_pixel.x() = std::max(
                    0.0, std::min(static_cast<double>(camera.width() - 1),
                                  end_pixel.x()));
                end_pixel.y() = std::max(
                    0.0, std::min(static_cast<double>(camera.height() - 1),
                                  end_pixel.y()));
                const Eigen::Vector3d first = project(Eigen::Vector2d(u, v));
                Eigen::Vector3d tangent = project(end_pixel) - first;
                const double tangent_norm = tangent.norm();
                if (tangent_norm <= 1.0e-8) {
                    continue;
                }
                tangent /= tangent_norm;
                const double safe_speed = brakingSpeed(
                    solution.free_distance[index], config_.reference_speed, config_);
                const double speed_ratio = std::max(
                    0.0, std::min(1.0, safe_speed /
                        std::max(config_.reference_speed, 1.0e-6)));
                const double shaft = 0.18 * field_radius_ * speed_ratio;
                if (shaft <= 1.0e-4) {
                    continue;
                }
                const Eigen::Vector3d last = first + shaft * tangent;
                const auto base_color = color(
                    1.0 - speed_ratio, speed_ratio, 0.2, 0.85);
                const double pulse = positiveModulo(
                    static_cast<double>(u) / camera.width() -
                    2.5 * pulse_time);
                constexpr int subdivisions = 5;
                for (int subdivision = 0; subdivision < subdivisions;
                     ++subdivision) {
                    const double s0 = static_cast<double>(subdivision) /
                        subdivisions;
                    const double s1 = static_cast<double>(subdivision + 1) /
                        subdivisions;
                    const double d0 = positiveModulo(s0 - pulse + 0.5) - 0.5;
                    const double d1 = positiveModulo(s1 - pulse + 0.5) - 0.5;
                    const double alpha0 = base_color.a *
                        (0.12 + 0.88 * std::exp(-0.5 * std::pow(d0 / 0.16, 2)));
                    const double alpha1 = base_color.a *
                        (0.12 + 0.88 * std::exp(-0.5 * std::pow(d1 / 0.16, 2)));
                    addSegment(
                        first + s0 * (last - first),
                        first + s1 * (last - first),
                        color(base_color.r, base_color.g, base_color.b, alpha0),
                        color(base_color.r, base_color.g, base_color.b, alpha1));
                }
                const Eigen::Vector3d normal = (first - position) /
                    std::max((first - position).norm(), 1.0e-8);
                Eigen::Vector3d side = normal.cross(tangent);
                const double side_norm = side.norm();
                if (side_norm > 1.0e-8) {
                    side /= side_norm;
                    const double head = std::min(0.35 * shaft, 0.055 * field_radius_);
                    addSegment(
                        last,
                        last - 0.866 * head * tangent + 0.5 * head * side,
                        base_color, base_color);
                    addSegment(
                        last,
                        last - 0.866 * head * tangent - 0.5 * head * side,
                        base_color, base_color);
                }
            }
        }
        output.markers.push_back(std::move(quiver));

        const std::array<Eigen::Vector2d, 4> corners = {
            Eigen::Vector2d(0.0, 0.0),
            Eigen::Vector2d(camera.width() - 1.0, 0.0),
            Eigen::Vector2d(camera.width() - 1.0, camera.height() - 1.0),
            Eigen::Vector2d(0.0, camera.height() - 1.0)};
        std::array<Eigen::Vector3d, 4> boundary;
        for (std::size_t index = 0; index < corners.size(); ++index) {
            boundary[index] = project(corners[index]);
        }
        auto frustum = marker(
            stamp, "camera_fov", 0,
            visualization_msgs::msg::Marker::LINE_LIST);
        frustum.scale.x = 0.025;
        frustum.color = color(0.15, 0.65, 1.0, 0.85);
        for (const auto& corner : boundary) {
            frustum.points.push_back(point(position));
            frustum.points.push_back(point(corner));
        }
        for (std::size_t index = 0; index < boundary.size(); ++index) {
            frustum.points.push_back(point(boundary[index]));
            frustum.points.push_back(point(boundary[(index + 1) % boundary.size()]));
        }
        output.markers.push_back(std::move(frustum));

        const std::array<std::string, 3> ray_names = {
            "reference_ray", "goal_ray", "command_ray"};
        const std::array<Eigen::Vector2d, 3> ray_pixels = {
            solution.reference_pixel,
            solution.goal_pixel,
            solution.command_pixel};
        const std::array<std_msgs::msg::ColorRGBA, 3> ray_colors = {
            color(0.0, 1.0, 1.0),
            color(0.1, 1.0, 0.15),
            color(1.0, 1.0, 1.0)};
        for (std::size_t index = 0; index < ray_names.size(); ++index) {
            auto ray = marker(
                stamp, ray_names[index], static_cast<int>(index),
                visualization_msgs::msg::Marker::LINE_LIST);
            ray.scale.x = 0.04;
            ray.color = ray_colors[index];
            ray.points.push_back(point(position));
            ray.points.push_back(point(project(ray_pixels[index])));
            output.markers.push_back(std::move(ray));
        }

        auto hits = marker(
            stamp, "depth_hits", 0,
            visualization_msgs::msg::Marker::POINTS);
        hits.scale.x = 0.055;
        hits.scale.y = 0.055;
        hits.color = color(0.1, 0.9, 0.7, 0.9);
        for (const auto& camera_point : backprojectObstaclePoints(
                 solution.depth, camera, config_.depth_point_stride)) {
            hits.points.push_back(point(
                position + rotation_world_from_camera * camera_point));
        }
        output.markers.push_back(std::move(hits));
        field_publisher_->publish(output);
    }

    bool observationPose(const rclcpp::Time& stamp,
        Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (odometry_history_.empty()) return false;
        auto before = odometry_history_.front();
        auto after = odometry_history_.back();
        for (const auto& sample : odometry_history_) {
            const auto t = messageTime(sample->header.stamp);
            if (t <= stamp) before = sample;
            if (t >= stamp) { after = sample; break; }
        }
        const double ta = messageTime(before->header.stamp).seconds();
        const double tb = messageTime(after->header.stamp).seconds();
        if (std::min(std::abs(ta - stamp.seconds()), std::abs(tb - stamp.seconds())) > 0.04)
            return false;
        double weight = tb > ta ? std::max(0.0, std::min(1.0, (stamp.seconds()-ta)/(tb-ta))) : 0.0;
        const auto& a = before->pose.pose;
        const auto& b = after->pose.pose;
        Eigen::Quaterniond qa(a.orientation.w,a.orientation.x,a.orientation.y,a.orientation.z);
        Eigen::Quaterniond qb(b.orientation.w,b.orientation.x,b.orientation.y,b.orientation.z);
        if (!qa.coeffs().allFinite() || !qb.coeffs().allFinite() || qa.norm()<1e-6 || qb.norm()<1e-6)
            return false;
        *rotation = qa.normalized().slerp(weight,qb.normalized()).toRotationMatrix();
        *position = (1-weight)*Eigen::Vector3d(a.position.x,a.position.y,a.position.z) +
            weight*Eigen::Vector3d(b.position.x,b.position.y,b.position.z);
        return position->allFinite();
    }

    void paperControl(const rclcpp::Time& stamp, const rclcpp::Time& depth_time,
        const Camera& camera,const std::vector<double>& proposal_depth, std::uint64_t version,
        const std::vector<std::shared_ptr<const DepthObservation>>& current_views,
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& intent, double dt)
    {
        Eigen::Vector3d captured_position;
        Eigen::Matrix3d captured_rotation;
        if (!observationPose(depth_time, &captured_position, &captured_rotation)) {
            stop(stamp, position, "WAITING_DEPTH_POSE"); return;
        }
        // The direction chart can rotate independently of physical sensors.
        // Its depth is unknown: all geometric evidence remains in the actual
        // camera frusta below. This avoids gaps between four chart boundaries.
        const Eigen::Matrix3d chart=intentChartRotation(velocity,intent);
        const Camera virtual_camera=intentChartCamera(Camera(chart_width_,chart_height_,90.,68.,camera.maxDepth()));
        DepthObservation observation(virtual_camera,std::vector<double>(virtual_camera.rays().size(),0),position,
            chart,depth_time.seconds(),version,0);
        observation.supporting_views=current_views;
        auto start = std::chrono::steady_clock::now();
        Eigen::Vector3d proposal=Eigen::Vector3d::Zero();
        std::shared_ptr<const DepthObservation> proposal_observation;
        AngularSolution proposal_solution;bool proposal_goal_valid=false;
        if(paper_depth_proposal_&&intent.norm()>1e-5) {
            const Eigen::Vector3d direction=intent.normalized();
            if(!has_reference_direction_||(!use_fixed_goal_&&direction.dot(reference_direction_)<std::cos(.05))) {
                resetPlanningHistory();reference_direction_=direction;has_reference_direction_=true;
            }
            for(const auto& view:current_views)
                if(view->view==std::max(0,selected_view_index_))proposal_observation=view;
            if(!proposal_observation&&!current_views.empty())proposal_observation=current_views.front();
            if(proposal_observation) {
                const auto& view=*proposal_observation;
                RCLCPP_INFO_ONCE(get_logger(),"DEPTH_PROPOSAL conservative_grid=%dx%d measured_state=1 direction_only=1",
                    view.camera.width(),view.camera.height());
                SimConfig proposal_config=config_;
                proposal_config.reference_speed=std::min(intent.norm(),max_reference_speed_);
                // Plan from measured motion; preview both the vector shaper and plant
                // response instead of advancing an unexecuted virtual heading.
                proposal_config.control_dt=2.*config_.velocity_tau+dt;proposal_config.horizontal_only=false;
                proposal_config.clearance_reward*=64.0/view.camera.width();
                proposal_config.goal_projection_clearance*=view.camera.width()/64.0;
                proposal_config.goal_clearance_cap*=view.camera.width()/64.0;
                proposal_config.subpixel_goal=paper_subpixel_proposal_;
                proposal_config.angular_goal_cost=paper_angular_goal_cost_;proposal_config.recapture_reference=true;proposal_config.finite_goal=use_fixed_goal_;
                proposal_config.body_radius=paper_->envelopeRadius();proposal_config.safety_margin=0.;
                const auto camera_from_world=view.rotation.transpose().eval();
                const Eigen::Vector2d previous=directionPixel(view.camera,camera_from_world,
                    velocity.norm()>.03?velocity.normalized():direction);
                Eigen::Vector2d previous_goal;
                const Eigen::Vector2d* goal_hint=nullptr;
                if(has_previous_goal_direction_world_) {
                    previous_goal=directionPixel(view.camera,camera_from_world,previous_goal_direction_world_);goal_hint=&previous_goal;
                }
                const bool advance=view.version!=last_hysteresis_depth_version_;
                last_hysteresis_depth_version_=view.version;
                const bool retain=has_active_goal_time_&&(stamp-active_goal_time_).seconds()<goal_hold_time_;
                const Eigen::Vector3d goal=use_fixed_goal_?fixed_goal_:position+forward_lookahead_*direction;
                if(use_fixed_goal_) {
                    SimConfig approach=proposal_config;
                    approach.brake_accel=std::min(config_.brake_accel,config_.max_accel);
                    // Account for command shaping plus vehicle response before
                    // braking into the fixed target. Manual directional input
                    // has no artificial nearby endpoint and is unaffected.
                    approach.delay=.20+2.*config_.velocity_tau;
                    proposal_config.reference_speed=brakingSpeed((goal-position).norm(),
                        proposal_config.reference_speed,approach);
                }
                const auto& planning_depth=proposal_depth.size()==view.depth.size()?proposal_depth:view.depth;
                double corridor=std::max(.25,velocity.norm()*.2+
                    velocity.squaredNorm()/(2.*std::min(config_.brake_accel,config_.max_accel))+.25);
                if(use_fixed_goal_)corridor=std::min(corridor,(goal-position).norm());
                const auto admissible=[&](const Eigen::Vector2d& pixel) {
                    return paper_->proposalCorridor(observation,position,
                        view.rotation*view.camera.rayFromPixel(pixel),corridor,stamp.seconds());
                };
                // Native samples affect only the direction proposal. Complete
                // conservative depth and every native hit remain in the safety
                // evidence; an optimistic candidate can never authorize motion.
                std::vector<Eigen::Vector3d> proposal_points;
                if(native_proposal_stride_>0&&view.obstacle_points)
                    for(std::size_t i=0;i<view.obstacle_points->size();i+=native_proposal_stride_)
                        proposal_points.push_back((*view.obstacle_points)[i]);
                const auto proposed=computeGuidance(planning_depth,view.camera,proposal_config,position,velocity,goal,
                    previous,goal_hint,view.rotation,nullptr,nullptr,&planning_mask_state_,&planning_clear_counts_,
                    obstacle_clear_frames_,advance,retain,previous_potential_.size()==view.depth.size()?&previous_potential_:nullptr,
                    native_proposal_stride_>0?&proposal_points:nullptr,true,admissible,true,
                    paper_vehicle_origin_proposal_?&view.origin:nullptr);
                // The free-space-certified target supplies a direction only.
                // The whole-vector shaper and motion certificate determine its
                // reachable execution; no unexecuted angular integration state
                // is allowed to pull the vehicle away from the current goal.
                proposal=proposed.goal_valid?
                    (proposal_config.reference_speed*view.rotation*view.camera.rayFromPixel(proposed.returned_goal_pixel)).eval():
                    Eigen::Vector3d::Zero();
                if(paper_local_path_proposal_) {
                    ++local_path_evaluations_;
                    if(local_path_time_<0||stamp.seconds()<local_path_time_||stamp.seconds()-local_path_time_>=.10||
                        local_path_intent_.dot(direction)<std::cos(.05)) {
                        auto path_views=current_views;
                        for(const auto& old:paper_->retainedDepthObservations())
                            if(std::none_of(current_views.begin(),current_views.end(),[&](const auto& fresh){
                                return old->view==fresh->view&&old->stamp==fresh->stamp&&old->version==fresh->version;}))path_views.push_back(old);
                        cached_local_path_=localDepthPath(path_views,position,velocity,goal,paper_->envelopeRadius(),8.,2.*config_.velocity_tau);
                        local_path_time_=stamp.seconds();local_path_intent_=direction;++local_path_plans_;
                    }
                    const Eigen::Vector3d path_direction=cached_local_path_.valid?
                        localPathDirection(cached_local_path_.points,position,3.):Eigen::Vector3d::Zero();
                    if(path_direction.dot(direction)>1e-6&&paper_->proposalCorridor(observation,position,path_direction,corridor,stamp.seconds())) {
                        proposal=proposal_config.reference_speed*path_direction;++local_path_selected_;
                    }
                    if(local_path_evaluations_%500==0)RCLCPP_INFO(get_logger(),"LOCAL_PATH cycles=%zu replans=%zu selected=%zu",local_path_evaluations_,local_path_plans_,local_path_selected_);
                }
                proposal_solution=proposed.solution;proposal_goal_valid=proposed.goal_valid;
                previous_direction_world_=view.rotation*view.camera.rayFromPixel(proposed.solution.command_pixel);
                has_previous_direction_world_=true;
                previous_goal_direction_world_=view.rotation*view.camera.rayFromPixel(proposed.returned_goal_pixel);
                has_previous_goal_direction_world_=true;
                if(proposed.goal_reanchored||!has_active_goal_time_){active_goal_time_=stamp;has_active_goal_time_=true;}
                previous_potential_=proposed.solution.field_valid?proposed.solution.potential:std::vector<double>();
                // Prefer the current intent whenever the observed volume,
                // including other cameras and history, proves its reaction
                // and braking corridor. The single-view planning grid is only
                // a proposal; it must not veto a fully certified return to q.
                double intent_length=corridor+std::max(velocity.norm(),proposal_config.reference_speed)*
                    (2.*config_.velocity_tau+.5);
                if(use_fixed_goal_)intent_length=std::min(intent_length,(goal-position).norm());
                if(paper_intent_corridor_&&paper_->proposalCorridor(observation,position,direction,intent_length,stamp.seconds())) {
                    proposal=proposal_config.reference_speed*direction;
                    previous_goal_direction_world_=direction;has_previous_goal_direction_world_=true;
                    active_goal_time_=stamp;has_active_goal_time_=true;previous_potential_.clear();
                    proposal_solution.goal_pixel=directionPixel(view.camera,camera_from_world,direction);
                    proposal_solution.command_pixel=proposal_solution.goal_pixel;
                    proposal_solution.field_valid=false;
                    proposal_goal_valid=true;
                }
            }
        }
        std::ostringstream replay_data(std::ios::out|std::ios::binary);
        bool capture=!replay_directory_.empty() && consecutive_rejections_>=3 && stamp.seconds()-last_replay_time_>=5.;
        if(capture) {
            try {paper_->saveReplay(replay_data,observation,position,velocity,intent,stamp.seconds(),dt,paper_depth_proposal_?&proposal:nullptr);}
            catch(const std::exception& e){capture=false;RCLCPP_WARN(get_logger(),"Replay capture failed: %s",e.what());}
        }
        PaperResult result = paper_depth_proposal_?
            paper_->stepWithProposal(observation,position,velocity,intent,proposal,stamp.seconds(),dt):
            paper_->step(observation,position,velocity,intent,stamp.seconds(),dt);
        const PaperResult replay_result=result;
        auto finish = std::chrono::steady_clock::now();
        if(std::chrono::duration<double>(finish-current_control_wall_start_).count()>.08) {
            result.command.setZero();result.accepted=false;result.status="COMPUTE_DEADLINE";
        }
        paper_->confirmPublishedCommand(result.command);
        consecutive_rejections_=result.accepted?0:std::min(100,consecutive_rejections_+1);
        avoidance_compute_ms_.add(std::chrono::duration<double,std::milli>(finish-start).count());
        depth_to_command_ms_.add(std::max(0.0, (stamp-depth_time).seconds()*1000.0));
        ++guidance_frame_count_;
        if (!has_guidance_wall_span_) { first_guidance_wall_=start; has_guidance_wall_span_=true; }
        last_guidance_wall_=finish;
        publishCommand(stamp,position,result.command);
        if(capture) {
            last_replay_time_=stamp.seconds();
            const auto base=replay_directory_+"/frame_"+std::to_string(replay_sequence_++%16);
            std::ofstream binary(base+".bin",std::ios::binary|std::ios::trunc);
            binary<<replay_data.str();binary.close();
            std::ofstream expected(base+".json");const auto& r=replay_result;
            expected<<std::setprecision(17)<<"{\"time\":"<<stamp.seconds()<<",\"status\":\""<<r.status
                <<"\",\"reason\":\""<<r.build_reason<<"\",\"accepted\":"<<(r.accepted?"true":"false")
                <<",\"command\":["<<r.command.x()<<','<<r.command.y()<<','<<r.command.z()
                <<"],\"best_prefix\":"<<r.best_prefix<<",\"required_prefix\":"<<r.required_prefix
                <<",\"free_directions\":"<<r.free_directions<<",\"refined\":"<<(r.grid_refined?"true":"false")
            <<",\"command_changed\":"<<(r.command_changed?"true":"false")
            <<",\"intent_change_angle\":"<<r.intent_change_angle
            <<",\"reset_reason\":\""<<r.reset_reason<<"\",\"continued\":"<<(r.continued?"true":"false")<<"}\n";
            if(!binary||!expected)RCLCPP_WARN(get_logger(),"Replay write failed: %s",base.c_str());
        }
        last_guidance_time_=stamp; has_last_guidance_time_=true;
        setStatus(result.status);
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "PAPER_BUILD reason=%s required=%.4f best=%.4f free=%d refined=%d tubes=%zu revoked_tubes=%zu proofs=%zu",
            result.build_reason.c_str(),result.required_prefix,result.best_prefix,result.free_directions,
            result.grid_refined,paper_->recordedTubes(),paper_->revokedTubes(),paper_->reachedProofs());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "PAPER_EVIDENCE retained=%zu balls=%zu revoked_balls=%zu expired_views=%zu evicted_views=%zu radius=%.3f clearance=%.3f best_prefix=%.3f",paper_->retainedObservations(),paper_->recordedBalls(),paper_->revokedBalls(),paper_->expiredViews(),paper_->evictedViews(),paper_->envelopeRadius(),result.clearance,paper_->maximumClearance());
        Eigen::Vector3d target=Eigen::Vector3d::Zero();
        const auto* retained=paper_->observation();
        if(retained&&paper_->field().valid)
            target=retained->rotation*retained->camera.rayFromPixel(paper_->field().goal);
        if(!retained&&proposal.norm()>1e-8)
            target=proposal.normalized();
        const auto& reference_intent=paper_->referenceIntent();
        std::ostringstream diagnostic;
        diagnostic << std::setprecision(12) << "{\"status\":\"" << result.status
            << "\",\"chi\":" << result.chi << ",\"ju\":" << result.ju
            << ",\"jump\":" << result.jump << ",\"refreshed\":" << result.refreshed
            << ",\"command_changed\":" << result.command_changed
            << ",\"intent_change_angle\":" << result.intent_change_angle
            << ",\"continued\":" << result.continued << ",\"tracking\":" << result.tracking
            << ",\"accepted\":" << result.accepted << ",\"residual\":" << result.field_residual
            << ",\"coupling_error\":" << result.coupling_error
            << ",\"jacobian_norm\":" << result.jacobian_norm
            << ",\"execution_mismatch\":" << result.execution_mismatch
            << ",\"mismatch_valid\":" << result.mismatch_valid
            << ",\"jump_valid\":" << result.jump_valid
            << ",\"jump_bound\":" << result.jump_bound
            << ",\"envelope_radius\":" << paper_->envelopeRadius()
            << ",\"best_prefix\":" << paper_->maximumClearance()
            << ",\"build_reason\":\"" << result.build_reason << "\""
            << ",\"required_prefix\":" << result.required_prefix
            << ",\"free_directions\":" << result.free_directions
            << ",\"grid_refined\":" << result.grid_refined
            << ",\"recorded_tubes\":" << paper_->recordedTubes()
            << ",\"recorded_balls\":" << paper_->recordedBalls()
            << ",\"revoked_balls\":" << paper_->revokedBalls()
            << ",\"expired_views\":" << paper_->expiredViews()
            << ",\"evicted_views\":" << paper_->evictedViews()
            << ",\"diagnostic_stamp\":" << stamp.seconds()
            << ",\"input_x\":" << intent.x() << ",\"input_y\":" << intent.y() << ",\"input_z\":" << intent.z()
            << ",\"target_x\":" << target.x() << ",\"target_y\":" << target.y() << ",\"target_z\":" << target.z()
            << ",\"reference_intent_x\":" << reference_intent.x()
            << ",\"reference_intent_y\":" << reference_intent.y()
            << ",\"reference_intent_z\":" << reference_intent.z()
            << ",\"command_x\":" << result.command.x() << ",\"command_y\":" << result.command.y() << ",\"command_z\":" << result.command.z()
            << ",\"reset_reason\":\"" << result.reset_reason << "\"}";
        std_msgs::msg::String message; message.data=diagnostic.str(); paper_diagnostics_->publish(message);
        if(!retained&&proposal_observation&&proposal_goal_valid&&field_publisher_->get_subscription_count()>0&&
           (!has_last_field_publish_time_||(stamp-last_field_publish_time_).seconds()>=1./field_publish_rate_)) {
            publishField(stamp,position,proposal_observation->rotation,proposal_observation->camera,proposal_solution);
            last_field_publish_time_=stamp;has_last_field_publish_time_=true;
        }
        if (retained && paper_->field().valid && result.accepted) {
            const auto& field=paper_->field();
            AngularSolution solution;
            solution.depth=retained->depth;
            solution.potential.assign(retained->camera.width()*retained->camera.height(),std::numeric_limits<double>::quiet_NaN());
            solution.planning_mask.assign(solution.potential.size(),1);
            solution.free_distance.assign(solution.potential.size(),0.0);
            for (int y=0;y<field.height;++y) for (int x=0;x<field.width;++x) {
                int u=static_cast<int>(field.offset.x()+x*field.spacing);
                int v=static_cast<int>(field.offset.y()+y*field.spacing);
                if (u<0||v<0||u>=retained->camera.width()||v>=retained->camera.height()) continue;
                int dest=v*retained->camera.width()+u, source=y*field.width+x;
                solution.potential[dest]=field.potential[source];
                solution.planning_mask[dest]=field.blocked[source];
            }
            solution.reference_pixel=result.state;solution.source_pixel=field.source;
            solution.goal_pixel=field.goal;solution.command_pixel=result.command_pixel;solution.field_valid=true;
            publishField(stamp,retained->origin,retained->rotation,retained->camera,solution);
        }
    }

    void control()
    {
        current_control_wall_start_ = std::chrono::steady_clock::now();
        ++control_callback_count_;
        const rclcpp::Time stamp = get_clock()->now();
        nav_msgs::msg::Odometry::SharedPtr odometry;
        std::array<std::shared_ptr<Camera>, 4> cameras;
        std::array<std::vector<double>, 4> depths,proposal_depths;
        std::array<std::shared_ptr<const std::vector<Eigen::Vector3d>>,4> hits;
        std::array<rclcpp::Time, 4> depth_times;
        std::array<bool, 4> has_depth_times;
        std::array<std::uint64_t, 4> depth_versions;
        Eigen::Vector3d intent = Eigen::Vector3d::Zero();
        bool has_intent;
        rclcpp::Time intent_time(0, 0, get_clock()->get_clock_type());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odometry = odometry_;
            cameras = cameras_;
            depths = depths_; proposal_depths=proposal_depths_; hits=obstacle_points_;
            depth_times = depth_times_;
            has_depth_times = has_depth_times_;
            depth_versions = depth_versions_;
            intent = intent_;
            has_intent = has_intent_;
            intent_time = intent_time_;
        }
        if (!odometry) {
            setStatus("WAITING_ODOMETRY");
            return;
        }
        double control_dt = 0.02;
        if (has_last_guidance_time_) {
            const double measured = (stamp - last_guidance_time_).seconds();
            if (std::isfinite(measured) && measured > 0.0) {
                control_dt = std::max(0.005, std::min(0.10, measured));
            }
        }
        const auto& pose = odometry->pose.pose;
        const auto& twist = odometry->twist.twist;
        Eigen::Vector3d position(
            pose.position.x, pose.position.y, pose.position.z);
        const Eigen::Vector3d velocity(
            twist.linear.x, twist.linear.y, twist.linear.z);
        if (!vectorFinite(position) || !vectorFinite(velocity)) {
            position = Eigen::Vector3d(
                finiteOrZero(position.x()),
                finiteOrZero(position.y()),
                finiteOrZero(position.z()));
            stop(stamp, position, "INVALID_ODOMETRY");
            return;
        }
        std::vector<std::shared_ptr<const DepthObservation>> current_paper_views;
        auto evidence_start=std::chrono::steady_clock::now();
        if(paper_enabled_) {
            if(!seed_attempted_&&!seed_scene_.empty()) {
                // OmniGraph can emit an uninitialized zero pose on its first tick.
                if(position.norm()<1e-6) {stop(stamp,position,"WAITING_VERIFIED_START_POSE");return;}
                seed_attempted_=true;std::string evidence;
                std::vector<Eigen::Vector3d> verified_centers;
                if(certifyDefaultCloudStart(seed_scene_,seed_occupancy_,position,seed_radius_,&evidence)) {
                    verified_centers.push_back(position);

                }
                if(paper_->setVerifiedNeighborhood(verified_centers,seed_radius_))
                    RCLCPP_INFO(get_logger(),"VERIFIED_FREE_SEED center=(%.3f,%.3f,%.3f) radius=%.3f balls=%zu: %s",
                        position.x(),position.y(),position.z(),seed_radius_,verified_centers.size(),evidence.c_str());
                else RCLCPP_WARN(get_logger(),"FREE_SEED_REJECTED at (%.3f,%.3f,%.3f): %s",position.x(),position.y(),position.z(),evidence.c_str());
            }
            for(std::size_t view=0;view<cameras.size();++view) {
                if(!cameras[view]||!has_depth_times[view]||
                    age(stamp,depth_times[view],true)>max_depth_age_||depths[view].size()!=cameras[view]->rays().size())continue;
                Eigen::Vector3d captured;Eigen::Matrix3d rotation;
                if(!observationPose(depth_times[view],&captured,&rotation))continue;
                const Eigen::Matrix3d yaw=Eigen::AngleAxisd(camera_yaw_offsets_[view],Eigen::Vector3d::UnitZ()).toRotationMatrix();
                DepthObservation observed(*cameras[view],depths[view],
                    captured+rotation*yaw*camera_offset_,rotation*yaw*fixedCameraRotation(),
                    depth_times[view].seconds(),depth_versions[view],view);
                observed.obstacle_points=hits[view];
                paper_->ingestObservation(observed);
                for(int phase=0;phase<4;++phase)evidence_phase_ms_[phase].add(paper_->ingestionCosts()[phase]);
                // Current views retain native hits for direction proposals.
                // Historical copies and replay evidence already discard them.
                current_paper_views.push_back(std::make_shared<DepthObservation>(observed));
            }
        }
        evidence_ingest_ms_.add(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-evidence_start).count());
        evidence_start=std::chrono::steady_clock::now();
        // Preserve the actually reached footprint also on ZERO_INTENT frames:
        // the plant is still decelerating after operator release.
        if(paper_enabled_&&!current_paper_views.empty()) {
            DepthObservation evidence=*current_paper_views.front();
            evidence.supporting_views=current_paper_views;
            paper_->rememberVerifiedPosition(evidence,position,stamp.seconds());
        }
        evidence_record_ms_.add(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-evidence_start).count());
        if (use_fixed_goal_ && stop_at_goal_ &&
            (goal_reached_ || (position - fixed_goal_).norm() <= goal_tolerance_)) {
            goal_reached_ = true;
            stop(stamp, position, "GOAL_REACHED");
            return;
        }

        Eigen::Vector3d reference;
        if (fixed_forward_intent_) {
            reference = Eigen::Vector3d(config_.reference_speed, 0.0, 0.0);
        } else if (!has_intent || age(stamp, intent_time, has_intent) > intent_timeout_) {
            stop(stamp, position, "STALE_INTENT");
            return;
        } else {
            reference = intent;
        }
        if (!reference.allFinite()) {
            stop(stamp, position, "INVALID_INTENT");
            return;
        }
        const double requested_speed = reference.norm();
        const double speed = std::min(requested_speed, max_reference_speed_);
        if (speed <= 1.0e-5) {
            stop(stamp, position, "ZERO_INTENT");
            return;
        }
        const Eigen::Vector3d direction = reference / requested_speed;
        Eigen::Matrix3d body_rotation;
        if (!quaternionMatrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w, &body_rotation)) {
            stop(stamp, position, "INVALID_GUIDANCE");
            return;
        }
        std::size_t view_index = 0;
        if (horizontal_360_enabled_) {
            // Preserve a chart containing the measured state. Operator intent
            // remains the Eq. (6) target, including when it lies outside a view.
            const Eigen::Vector3d chart_direction = paper_enabled_&&velocity.norm()>.03
                ? velocity.normalized() : direction;
            const Eigen::Vector3d body_direction = body_rotation.transpose()*chart_direction;
            view_index = selectClosestYaw(
                std::atan2(body_direction.y(), body_direction.x()),
                camera_yaw_offsets_);
            if (selected_view_index_ != static_cast<int>(view_index)) {
                resetPlanningHistory();
                selected_view_index_ = static_cast<int>(view_index);
                RCLCPP_INFO(
                    get_logger(), "3D guidance switched to view %zu",
                    view_index);
            }
        }
        const std::shared_ptr<Camera>& camera = cameras[view_index];
        const std::vector<double>& depth = depths[view_index];
        const rclcpp::Time& depth_time = depth_times[view_index];
        const bool has_depth_time = has_depth_times[view_index];
        const std::uint64_t depth_version = depth_versions[view_index];
        if (!camera) {
            stop(stamp, position, "WAITING_DEPTH");
            return;
        }
        if (age(stamp, depth_time, has_depth_time) > max_depth_age_) {
            stop(stamp, position, "STALE_DEPTH");
            return;
        }
        if (paper_enabled_) {
            paperControl(stamp,depth_time,*camera,proposal_depths[view_index],depth_version,current_paper_views,
                position,velocity,reference,control_dt);
            return;
        }
        const Eigen::Matrix3d camera_yaw_rotation = Eigen::AngleAxisd(
            camera_yaw_offsets_[view_index], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d rotation_world_from_camera =
            body_rotation * camera_yaw_rotation * fixedCameraRotation();
        const Eigen::Matrix3d rotation_camera_from_world =
            rotation_world_from_camera.transpose();
        Eigen::Vector2d requested_pixel;
        if (!camera->pixelFromDirection(
                rotation_camera_from_world * direction, &requested_pixel)) {
            resetPlanningHistory();
            stop(stamp, position, "DIRECTION_OUTSIDE_DEPTH_FOV");
            return;
        }

        if (!has_reference_direction_ ||
            direction.dot(reference_direction_) < reanchor_cosine_) {
            reference_origin_ = position;
            reference_direction_ = direction;
            has_reference_direction_ = true;
            has_previous_goal_pixel_ = false;
            has_previous_goal_direction_world_ = false;
            has_active_goal_time_ = false;
            previous_potential_.clear();
        }
        config_.reference_speed = speed;
        Eigen::Vector3d goal = use_fixed_goal_
            ? fixed_goal_ : position + forward_lookahead_ * direction;
        try {
            Eigen::Vector2d previous_pixel = directionPixel(
                *camera, rotation_camera_from_world, direction);
            if (has_previous_direction_world_) {
                previous_pixel = directionPixel(
                    *camera, rotation_camera_from_world,
                    previous_direction_world_);
            } else if (has_previous_pixel_) {
                previous_pixel = previous_pixel_;
            }
            Eigen::Vector2d previous_goal_pixel;
            const Eigen::Vector2d* previous_goal = nullptr;
            if (has_previous_goal_direction_world_) {
                previous_goal_pixel = directionPixel(
                    *camera, rotation_camera_from_world,
                    previous_goal_direction_world_);
                previous_goal = &previous_goal_pixel;
            }
            const Eigen::Vector3d* reference_origin =
                use_reference_line_ ? &reference_origin_ : nullptr;
            const Eigen::Vector3d* reference_direction =
                use_reference_line_ ? &reference_direction_ : nullptr;
            const bool advance_obstacle_frame =
                depth_version != last_hysteresis_depth_version_;
            last_hysteresis_depth_version_ = depth_version;
            SimConfig runtime_config = config_;
            runtime_config.control_dt = control_dt;
            runtime_config.horizontal_only = false;
            const bool retain_previous_goal = has_previous_goal_direction_world_ &&
                has_active_goal_time_ &&
                (stamp - active_goal_time_).seconds() < goal_hold_time_;
            const std::vector<double>* initial_potential =
                previous_potential_.size() == depth.size()
                    ? &previous_potential_ : nullptr;
            const auto avoidance_start = std::chrono::steady_clock::now();
            const GuidanceResult result = computeGuidance(
                depth, *camera, runtime_config, position, velocity, goal,
                previous_pixel, previous_goal, rotation_world_from_camera,
                reference_origin, reference_direction,
                &planning_mask_state_, &planning_clear_counts_,
                obstacle_clear_frames_, advance_obstacle_frame,
                retain_previous_goal, initial_potential);
            const auto avoidance_end = std::chrono::steady_clock::now();
            avoidance_compute_ms_.add(std::chrono::duration<double, std::milli>(
                avoidance_end - avoidance_start).count());
            depth_to_command_ms_.add(std::max(
                0.0, (stamp - depth_time).seconds() * 1000.0));
            ++guidance_frame_count_;
            if (!has_guidance_wall_span_) {
                first_guidance_wall_ = avoidance_end;
                has_guidance_wall_span_ = true;
            }
            last_guidance_wall_ = avoidance_end;
            if (result.goal_reanchored || !has_active_goal_time_) {
                active_goal_time_ = stamp;
                has_active_goal_time_ = true;
            }
            previous_goal_pixel_ = result.returned_goal_pixel;
            has_previous_goal_pixel_ = true;
            previous_goal_direction_world_ = normalize(
                rotation_world_from_camera *
                    camera->rayFromPixel(result.returned_goal_pixel));
            has_previous_goal_direction_world_ =
                previous_goal_direction_world_.squaredNorm() > kEpsilon;
            previous_pixel_ = result.solution.command_pixel;
            has_previous_pixel_ = true;
            previous_direction_world_ = normalize(
                rotation_world_from_camera *
                    camera->rayFromPixel(result.solution.command_pixel));
            has_previous_direction_world_ =
                previous_direction_world_.squaredNorm() > kEpsilon;
            if (result.solution.field_valid) {
                previous_potential_ = result.solution.potential;
            } else {
                previous_potential_.clear();
            }
            if (depth_version != last_field_version_) {
                const bool field_due = !has_last_field_publish_time_ ||
                    stamp < last_field_publish_time_ ||
                    (stamp - last_field_publish_time_).seconds() >=
                        1.0 / field_publish_rate_;
                if (field_due) {
                    publishField(
                        stamp, position, rotation_world_from_camera,
                        *camera, result.solution);
                    last_field_publish_time_ = stamp;
                    has_last_field_publish_time_ = true;
                }
                last_field_version_ = depth_version;
            }
            if (!result.command_world.allFinite()) {
                stop(stamp, position, "INVALID_GUIDANCE");
                return;
            }
            // Execute the single planned 3-D ray. Limits scale its magnitude
            // uniformly; no direct Z injection or previous-velocity blending.
            const Eigen::Vector3d desired = result.command_world;
            double scale = 1.0;
            if (desired.norm() > max_reference_speed_) {
                scale = max_reference_speed_ / desired.norm();
            }
            if (std::abs(desired.z()) > max_vertical_speed_) {
                scale = std::min(scale, max_vertical_speed_ / std::abs(desired.z()));
            }
            const Eigen::Vector3d published = scale * desired;
            last_guidance_time_ = stamp;
            has_last_guidance_time_ = true;
            publishCommand(stamp, position, published);
            const std::string next_status =
                result.solution.field_valid ? "NAVIGATING" : "DEGRADED";
            if (next_status != last_status_) {
                RCLCPP_INFO(
                    get_logger(),
                    "3D diagnostic view=%zu requested=%.3f "
                    "free=%.3f brake=%.3f rollout=%.3f desired=%.3f "
                    "published=%.3f field=%s",
                    view_index, speed, result.selected_free_distance,
                    result.braking_limited_speed, result.rollout_limited_speed,
                    desired.norm(), published.norm(),
                    result.solution.field_valid ? "valid" : "degraded");
            }
            setStatus(next_status);
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "%s", error.what());
            stop(stamp, position, "INVALID_GUIDANCE");
        }
    }

    bool paper_enabled_ = false;
    int chart_width_=24,chart_height_=18;
    bool paper_local_path_proposal_=false;
    std::size_t local_path_evaluations_=0,local_path_selected_=0,local_path_plans_=0;
    LocalDepthPath cached_local_path_;
    double local_path_time_=-1.;
    Eigen::Vector3d local_path_intent_=Eigen::Vector3d::Zero();
    bool paper_vehicle_origin_proposal_=false,paper_subpixel_proposal_=false;
    bool paper_depth_proposal_=false,sampled_proposal_=false,paper_intent_corridor_=false,paper_angular_goal_cost_=false;
    int native_proposal_stride_=0;
    std::unique_ptr<PaperGuidance> paper_;
    std::string seed_scene_,seed_occupancy_;
    double seed_radius_=2.5;
    bool seed_attempted_=false;
    Eigen::Vector3d camera_offset_ = Eigen::Vector3d::Zero();
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odometry_history_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr paper_diagnostics_;
    std::mutex mutex_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;
    std::array<sensor_msgs::msg::CameraInfo::SharedPtr, 4> camera_infos_;
    std::array<std::shared_ptr<Camera>, 4> cameras_;
    std::array<std::vector<double>, 4> depths_,proposal_depths_;
    std::array<std::shared_ptr<const std::vector<Eigen::Vector3d>>,4> obstacle_points_;
    std::array<bool, 4> has_depth_times_{{false, false, false, false}};
    std::array<rclcpp::Time, 4> depth_times_;
    std::array<std::uint64_t, 4> depth_versions_{{0, 0, 0, 0}};
    Eigen::Vector3d intent_ = Eigen::Vector3d::Zero();
    bool has_intent_ = false;
    rclcpp::Time intent_time_;

    Eigen::Vector2d previous_pixel_ = Eigen::Vector2d::Zero();
    bool has_previous_pixel_ = false;
    Eigen::Vector2d previous_goal_pixel_ = Eigen::Vector2d::Zero();
    bool has_previous_goal_pixel_ = false;
    Eigen::Vector3d previous_direction_world_ = Eigen::Vector3d::Zero();
    bool has_previous_direction_world_ = false;
    Eigen::Vector3d previous_goal_direction_world_ = Eigen::Vector3d::Zero();
    bool has_previous_goal_direction_world_ = false;
    Eigen::Vector3d reference_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_direction_ = Eigen::Vector3d::Zero();
    bool has_reference_direction_ = false;
    Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
    bool has_active_goal_time_ = false;
    rclcpp::Time active_goal_time_;
    bool has_last_guidance_time_ = false;
    rclcpp::Time last_guidance_time_;
    std::vector<double> previous_potential_;
    std::string last_status_;
    bool goal_reached_ = false;
    std::uint64_t last_field_version_ =
        std::numeric_limits<std::uint64_t>::max();
    bool has_last_field_publish_time_ = false;
    rclcpp::Time last_field_publish_time_{0, 0, RCL_ROS_TIME};
    std::uint64_t last_hysteresis_depth_version_ =
        std::numeric_limits<std::uint64_t>::max();
    BinaryMask planning_mask_state_;
    std::vector<std::uint8_t> planning_clear_counts_;
    int selected_view_index_ = -1;

    std::string frame_id_;
    std::string odom_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::array<std::string, 4> depth_topics_;
    std::array<std::string, 4> camera_info_topics_;
    std::vector<double> camera_yaw_offsets_;
    std::string intent_topic_;
    std::string command_topic_;
    bool fixed_forward_intent_;
    bool horizontal_360_enabled_;
    double intent_timeout_;
    double max_depth_age_;
    int target_width_;
    int target_height_;
    int obstacle_clear_frames_;
    std::string replay_directory_;
    int consecutive_rejections_=0;
    std::size_t replay_sequence_=0;
    double last_replay_time_=-1e10;
    bool isaac_depth_contract_=false;
    double max_depth_;
    double forward_lookahead_;
    bool use_fixed_goal_;
    Eigen::Vector3d fixed_goal_;
    bool use_reference_line_;
    bool stop_at_goal_;
    double goal_tolerance_;
    double command_accel_limit_;
    double goal_hold_time_;
    double max_vertical_speed_;
    double max_reference_speed_;
    double command_yaw_;
    double field_radius_;
    double field_publish_rate_;
    double reanchor_cosine_;
    double performance_report_interval_;
    std::string performance_log_path_;
    std::string performance_run_id_;
    SimConfig config_;

    std::chrono::steady_clock::time_point performance_start_wall_;
    std::chrono::steady_clock::time_point last_performance_report_wall_;
    std::chrono::steady_clock::time_point current_control_wall_start_;
    std::chrono::steady_clock::time_point first_command_publish_wall_;
    std::chrono::steady_clock::time_point last_command_publish_wall_;
    std::chrono::steady_clock::time_point first_guidance_wall_;
    std::chrono::steady_clock::time_point last_guidance_wall_;
    bool has_last_command_publish_wall_ = false;
    bool has_guidance_wall_span_ = false;
    rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
    std::uint64_t control_callback_count_ = 0;
    std::uint64_t command_frame_count_ = 0;
    std::uint64_t guidance_frame_count_ = 0;
    TimingSeries avoidance_compute_ms_;
    TimingSeries command_interval_ms_;
    TimingSeries command_ros_interval_ms_;
    TimingSeries depth_to_command_ms_;
    TimingSeries callback_to_publish_ms_;
    TimingSeries evidence_ingest_ms_, evidence_record_ms_;
    std::array<TimingSeries,4> evidence_phase_ms_;
    TimingSeries publish_call_ms_;

    rclcpp::Publisher<pc_gvf_msgs::msg::PositionCommand>::SharedPtr
        command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        field_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr, 4>
        camera_info_subscriptions_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 4>
        depth_subscriptions_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        intent_subscription_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace depth_angular
}  // namespace pc_gvf

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<pc_gvf::depth_angular::DepthAngularControllerNode>();
    rclcpp::executors::SingleThreadedExecutor controller_executor,sensor_executor;
    controller_executor.add_node(node);
    sensor_executor.add_callback_group(node->sensorCallbackGroup(),node->get_node_base_interface());
    std::exception_ptr sensor_error,controller_error;
    std::thread sensor_thread([&] {
        try {sensor_executor.spin();}
        catch(...) {sensor_error=std::current_exception();rclcpp::shutdown();}
    });
    try {controller_executor.spin();}catch(...) {controller_error=std::current_exception();}
    rclcpp::shutdown();sensor_executor.cancel();sensor_thread.join();
    if(controller_error)std::rethrow_exception(controller_error);
    if(sensor_error)std::rethrow_exception(sensor_error);
    return 0;
}
