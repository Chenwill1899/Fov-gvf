#include "pc_gvf/depth_angular_core.hpp"

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
        command_jerk_limit_ = std::max(
            0.1, declare_parameter<double>("command_jerk_limit", 8.0));
        command_response_time_ = std::max(
            0.02, declare_parameter<double>("command_response_time", 0.22));
        speed_recovery_accel_ = std::max(
            0.05, declare_parameter<double>("speed_recovery_accel", 1.0));
        speed_brake_accel_ = std::max(
            speed_recovery_accel_,
            declare_parameter<double>("speed_brake_accel", 2.5));
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
        config_.planning_horizon = declare_parameter<double>(
            "planning_horizon", 3.0);
        config_.horizontal_only = horizontal_360_enabled_;

        command_publisher_ = create_publisher<pc_gvf_msgs::msg::PositionCommand>(
            command_topic_, 10);
        rclcpp::QoS latched_qos(1);
        latched_qos.transient_local();
        status_publisher_ = create_publisher<std_msgs::msg::String>(
            "~/status", latched_qos);
        field_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pc_gvf/angular_field", latched_qos);

        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr message) {
                std::lock_guard<std::mutex> lock(mutex_);
                odometry_ = std::move(message);
            });
        const std::size_t view_count = horizontal_360_enabled_ ? 4 : 1;
        for (std::size_t index = 0; index < view_count; ++index) {
            depth_times_[index] = rclcpp::Time(
                0, 0, get_clock()->get_clock_type());
            camera_info_subscriptions_[index] =
                create_subscription<sensor_msgs::msg::CameraInfo>(
                    camera_info_topics_[index], rclcpp::SensorDataQoS(),
                    [this, index](sensor_msgs::msg::CameraInfo::SharedPtr message) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        camera_infos_[index] = std::move(message);
                    });
            depth_subscriptions_[index] =
                create_subscription<sensor_msgs::msg::Image>(
                    depth_topics_[index], rclcpp::SensorDataQoS(),
                    [this, index](sensor_msgs::msg::Image::SharedPtr message) {
                        depthCallback(std::move(message), index);
                    });
        }
        intent_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            intent_topic_, 10,
            std::bind(&DepthAngularControllerNode::intentCallback, this,
                      std::placeholders::_1));
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
        const sensor_msgs::msg::Image& message) const
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
                (information->k[2] + 0.5) * scale_x - 0.5,
                (information->k[5] + 0.5) * scale_y - 0.5);
            const rclcpp::Time stamp = messageTime(message->header.stamp);
            std::lock_guard<std::mutex> lock(mutex_);
            depths_[view_index] = std::move(depth);
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

    void publishVerticalOnly(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& position,
        double vertical_speed,
        const std::string& status)
    {
        command_.setZero();
        command_acceleration_.setZero();
        filtered_target_speed_ = 0.0;
        has_filtered_target_speed_ = false;
        has_last_guidance_time_ = false;
        publishCommand(
            stamp, position, Eigen::Vector3d(0.0, 0.0, vertical_speed));
        setStatus(status);
    }

    Eigen::Vector3d limited(
        const Eigen::Vector3d& desired,
        double dt,
        bool safety_limited,
        bool emergency_stop)
    {
        dt = std::max(0.001, std::min(0.10, dt));
        const double target_speed = desired.norm();
        if (!has_filtered_target_speed_) {
            filtered_target_speed_ = command_.norm();
            has_filtered_target_speed_ = true;
        }
        if (target_speed < filtered_target_speed_) {
            filtered_target_speed_ = std::max(
                target_speed,
                filtered_target_speed_ - speed_brake_accel_ * dt);
        } else {
            filtered_target_speed_ = std::min(
                target_speed,
                filtered_target_speed_ + speed_recovery_accel_ * dt);
        }

        Eigen::Vector3d target_direction = Eigen::Vector3d::Zero();
        if (target_speed > kEpsilon) {
            target_direction = desired / target_speed;
        } else if (command_.norm() > kEpsilon) {
            target_direction = command_.normalized();
        }
        const Eigen::Vector3d filtered_desired =
            filtered_target_speed_ * target_direction;
        const Eigen::Vector3d error = filtered_desired - command_;
        const double active_accel_limit =
            safety_limited && target_speed < command_.norm()
                ? speed_brake_accel_ : command_accel_limit_;
        Eigen::Vector3d target_acceleration = clampNorm(
            error / command_response_time_, active_accel_limit);
        const Eigen::Vector3d acceleration_delta = clampNorm(
            target_acceleration - command_acceleration_,
            command_jerk_limit_ * dt);
        command_acceleration_ = clampNorm(
            command_acceleration_ + acceleration_delta,
            std::max(command_accel_limit_, speed_brake_accel_));
        Eigen::Vector3d candidate = command_ + command_acceleration_ * dt;

        // Only an immediate contact-distance emergency bypasses smoothing.
        // Predictive rollout braking keeps the asymmetric deceleration and
        // jerk limits, avoiding one-frame stop/restart pulses.
        if (emergency_stop && candidate.norm() > target_speed + 1.0e-6) {
            if (candidate.norm() > kEpsilon) {
                candidate *= target_speed / candidate.norm();
            } else {
                candidate.setZero();
            }
            filtered_target_speed_ = target_speed;
            command_acceleration_.setZero();
        }
        command_ = candidate;
        return command_;
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
        command_acceleration_.setZero();
        filtered_target_speed_ = 0.0;
        has_filtered_target_speed_ = false;
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

    void control()
    {
        current_control_wall_start_ = std::chrono::steady_clock::now();
        ++control_callback_count_;
        const rclcpp::Time stamp = get_clock()->now();
        nav_msgs::msg::Odometry::SharedPtr odometry;
        std::array<std::shared_ptr<Camera>, 4> cameras;
        std::array<std::vector<double>, 4> depths;
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
            depths = depths_;
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
        const double vertical_speed = horizontal_360_enabled_
            ? std::max(-max_vertical_speed_,
                std::min(max_vertical_speed_, reference.z()))
            : 0.0;
        Eigen::Vector3d planning_reference = reference;
        if (horizontal_360_enabled_) {
            planning_reference.z() = 0.0;
        }
        const double requested_speed = planning_reference.norm();
        const double speed = std::min(requested_speed, max_reference_speed_);
        if (speed <= 1.0e-5) {
            if (horizontal_360_enabled_ && std::abs(vertical_speed) > 1.0e-5) {
                resetPlanningHistory();
                publishVerticalOnly(
                    stamp, position, vertical_speed, "VERTICAL_DIRECT");
            } else {
                stop(stamp, position, "ZERO_INTENT");
            }
            return;
        }

        const Eigen::Vector3d direction = planning_reference / requested_speed;
        Eigen::Matrix3d body_rotation;
        if (!quaternionMatrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w, &body_rotation)) {
            stop(stamp, position, "INVALID_GUIDANCE");
            return;
        }
        std::size_t view_index = 0;
        if (horizontal_360_enabled_) {
            const Eigen::Vector3d body_direction =
                body_rotation.transpose() * direction;
            view_index = selectClosestYaw(
                std::atan2(body_direction.y(), body_direction.x()),
                camera_yaw_offsets_);
            if (selected_view_index_ != static_cast<int>(view_index)) {
                resetPlanningHistory();
                selected_view_index_ = static_cast<int>(view_index);
                RCLCPP_INFO(
                    get_logger(), "horizontal guidance switched to view %zu",
                    view_index);
            }
        }
        const std::shared_ptr<Camera>& camera = cameras[view_index];
        const std::vector<double>& depth = depths[view_index];
        const rclcpp::Time& depth_time = depth_times[view_index];
        const bool has_depth_time = has_depth_times[view_index];
        const std::uint64_t depth_version = depth_versions[view_index];
        if (!camera) {
            if (horizontal_360_enabled_) {
                publishVerticalOnly(
                    stamp, position, vertical_speed, "WAITING_DEPTH_Z_DIRECT");
            } else {
                stop(stamp, position, "WAITING_DEPTH");
            }
            return;
        }
        if (age(stamp, depth_time, has_depth_time) > max_depth_age_) {
            if (horizontal_360_enabled_) {
                publishVerticalOnly(
                    stamp, position, vertical_speed, "STALE_DEPTH_Z_DIRECT");
            } else {
                stop(stamp, position, "STALE_DEPTH");
            }
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
        if (horizontal_360_enabled_) {
            goal.z() = position.z();
        }
        const Eigen::Matrix3d camera_yaw_rotation = Eigen::AngleAxisd(
            camera_yaw_offsets_[view_index], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d rotation_world_from_camera =
            body_rotation * camera_yaw_rotation * fixedCameraRotation();
        const Eigen::Matrix3d rotation_camera_from_world =
            rotation_world_from_camera.transpose();

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
            runtime_config.horizontal_only = horizontal_360_enabled_;
            const bool retain_previous_goal = has_previous_goal_direction_world_ &&
                has_active_goal_time_ &&
                (stamp - active_goal_time_).seconds() < goal_hold_time_;
            const std::vector<double>* initial_potential =
                previous_potential_.size() == depth.size()
                    ? &previous_potential_ : nullptr;
            const auto avoidance_start = std::chrono::steady_clock::now();
            Eigen::Vector3d planning_velocity = velocity;
            if (horizontal_360_enabled_) {
                planning_velocity.z() = 0.0;
            }
            const GuidanceResult result = computeGuidance(
                depth, *camera, runtime_config, position, planning_velocity, goal,
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
                if (horizontal_360_enabled_) {
                    publishVerticalOnly(
                        stamp, position, vertical_speed,
                        "INVALID_HORIZONTAL_GUIDANCE_Z_DIRECT");
                } else {
                    stop(stamp, position, "INVALID_GUIDANCE");
                }
                return;
            }
            Eigen::Vector3d desired = result.command_world;
            if (horizontal_360_enabled_) {
                desired.z() = 0.0;
            } else {
                desired.z() = std::max(
                    -max_vertical_speed_, std::min(max_vertical_speed_, desired.z()));
            }
            Eigen::Vector3d published = limited(
                desired, control_dt, result.safety_limited,
                result.emergency_stop);
            if (horizontal_360_enabled_) {
                published = composeHorizontalVerticalCommand(
                    published, vertical_speed, max_vertical_speed_);
            }
            last_guidance_time_ = stamp;
            has_last_guidance_time_ = true;
            publishCommand(stamp, position, published);
            const std::string next_status =
                result.solution.field_valid ? "NAVIGATING" : "DEGRADED";
            if (next_status != last_status_) {
                RCLCPP_INFO(
                    get_logger(),
                    "horizontal diagnostic view=%zu requested=%.3f "
                    "free=%.3f brake=%.3f rollout=%.3f desired=%.3f "
                    "published=%.3f field=%s",
                    view_index, speed, result.selected_free_distance,
                    result.braking_limited_speed, result.rollout_limited_speed,
                    desired.head<2>().norm(), published.head<2>().norm(),
                    result.solution.field_valid ? "valid" : "degraded");
            }
            setStatus(next_status);
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "%s", error.what());
            if (horizontal_360_enabled_) {
                publishVerticalOnly(
                    stamp, position, vertical_speed,
                    "INVALID_HORIZONTAL_GUIDANCE_Z_DIRECT");
            } else {
                stop(stamp, position, "INVALID_GUIDANCE");
            }
        }
    }

    std::mutex mutex_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;
    std::array<sensor_msgs::msg::CameraInfo::SharedPtr, 4> camera_infos_;
    std::array<std::shared_ptr<Camera>, 4> cameras_;
    std::array<std::vector<double>, 4> depths_;
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
    Eigen::Vector3d command_acceleration_ = Eigen::Vector3d::Zero();
    double filtered_target_speed_ = 0.0;
    bool has_filtered_target_speed_ = false;
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
    double max_depth_;
    double forward_lookahead_;
    bool use_fixed_goal_;
    Eigen::Vector3d fixed_goal_;
    bool use_reference_line_;
    bool stop_at_goal_;
    double goal_tolerance_;
    double command_accel_limit_;
    double command_jerk_limit_;
    double command_response_time_;
    double speed_recovery_accel_;
    double speed_brake_accel_;
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
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace depth_angular
}  // namespace pc_gvf

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<pc_gvf::depth_angular::DepthAngularControllerNode>());
    rclcpp::shutdown();
    return 0;
}
