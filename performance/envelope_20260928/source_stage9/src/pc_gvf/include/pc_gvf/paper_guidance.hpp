#ifndef PC_GVF_PAPER_GUIDANCE_HPP
#define PC_GVF_PAPER_GUIDANCE_HPP
#include "pc_gvf/depth_angular_core.hpp"
#include <memory>
#include <array>
#include <map>
#include <tuple>
#include <limits>
#include <string>

namespace pc_gvf { namespace depth_angular {
// Numeric realization of 2026RAL_CWL.pdf, equations (3)-(31).
std::vector<double> conservativeDepthResize(const std::vector<double>& input,
    int width, int height, int out_width, int out_height, double max_depth,
    bool positive_infinity_is_no_return = false);
struct DepthObservation {
    Camera camera;
    std::vector<double> depth;
    Eigen::Vector3d origin;
    Eigen::Matrix3d rotation;
    double stamp = 0.0;
    std::uint64_t version = 0;
    int view = 0;
    mutable bool revoked = false;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> obstacle_points;
    std::vector<std::shared_ptr<const DepthObservation>> supporting_views;
    DepthObservation(const Camera& c, const std::vector<double>& d,
        const Eigen::Vector3d& p, const Eigen::Matrix3d& r, double t,
        std::uint64_t v = 0, int index = 0)
        : camera(c), depth(d), origin(p), rotation(r), stamp(t), version(v), view(index) {}
};
// Sphere containment in the union of valid depth-pixel frusta, tested against
// the exact truncated unknown cones and the camera visibility planes.
bool observedEnvelope(const DepthObservation& observation,
    const Eigen::Vector3d& center_world, double radius);
struct PaperConfig {
    SimConfig motion;
    double field_interval = 0.05;
    double observation_timeout = 0.5;
    double history_duration = 0.0;
    bool retain_verified_travel = false;
    double history_sample_interval = .5;
    double history_uncertainty_rate = .10;
    double depth_uncertainty = 0.03;
    double uncertainty_rate = 0.10;
    double max_speed = 2.0;
    double max_vertical_speed = 1.0;
    double transverse_gain = 1.0;
    double chart_speed = 32.0;
    double minimum_lookahead = .50;
    double minimum_gradient = 1e-5;
    double section_radius = 8.0;
    double max_return_arc = 50.0;
    double command_change_angle = 0.05;
    int coarse_factor = 2;
    int fine_width = 32;
    int fine_height = 24;
    bool feedback = true;
    bool continuation = true;
    bool coarse_fine = true;
};
struct AngularField {
    int width = 0, height = 0;
    Eigen::Vector2d offset = Eigen::Vector2d::Zero();
    double spacing = 1.0;
    BinaryMask blocked;
    std::vector<double> potential;
    std::vector<double> boundary_extension;
    Eigen::Vector2d source = Eigen::Vector2d::Zero(), goal = Eigen::Vector2d::Zero();
    bool valid = false;
    double residual = 0.0;
    bool contains(const Eigen::Vector2d& y) const;
    bool segmentFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
    double value(const Eigen::Vector2d& y) const;
    Eigen::Vector2d flow(const Eigen::Vector2d& y) const;
};
struct FlowBox {
    Eigen::Vector2d anchor = Eigen::Vector2d::Zero();
    Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
    double section_value = 0.0;
    bool valid = false;
    bool initialize(const AngularField& field, const Eigen::Vector2d& a, const PaperConfig& cfg);
    bool coordinate(const AngularField& field, const Eigen::Vector2d& y,
        const PaperConfig& cfg, double* chi) const;
    bool evaluate(const AngularField& field, const Eigen::Vector2d& y,
        const PaperConfig& cfg, double* chi, Eigen::Vector2d* jacobian) const;
};
struct PaperResult {
    Eigen::Vector3d command = Eigen::Vector3d::Zero();
    Eigen::Vector2d state = Eigen::Vector2d::Zero(), command_pixel = Eigen::Vector2d::Zero();
    double chi = 0.0, ju = 0.0, jump = 0.0, clearance = 0.0;
    double field_residual = 0.0, coupling_error = 0.0;
    double jacobian_norm = 0.0, execution_mismatch = 0.0, jump_bound = 0.0;
    bool mismatch_valid = false, jump_valid = false;
    bool refreshed = false, continued = false, tracking = false, accepted = false;
    std::string status = "WAITING_OBSERVATION";
    std::string reset_reason;
};
class PaperGuidance {
public:
    explicit PaperGuidance(const PaperConfig& config);
    PaperResult step(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& intent, double now, double dt);
    void reset();
    void ingestObservation(const DepthObservation& observation);
    bool setVerifiedSeed(const Eigen::Vector3d& center,double radius);
    bool setVerifiedNeighborhood(const std::vector<Eigen::Vector3d>& centers,double radius);
    // One radius for state admission, motion certificates and reached evidence.
    double envelopeRadius() const;
    bool rememberVerifiedPosition(const DepthObservation& observation,
        const Eigen::Vector3d& position, double now);
    double maximumClearance() const { return maximum_clearance_; }
    std::size_t recordedBalls() const { return verified_travel_.size(); }
    std::size_t revokedBalls() const { return revoked_balls_; }
    std::size_t expiredViews() const { return expired_views_; }
    std::size_t evictedViews() const { return evicted_views_; }
    bool envelopeKnown(const DepthObservation& observation,const Eigen::Vector3d& center,
        double radius,double now) const;
    const AngularField& field() const { return field_; }
    const FlowBox& box() const { return box_; }
    std::size_t retainedObservations() const { return history_.size(); }
    const DepthObservation* observation() const { return observation_.get(); }
    // Certified prefix in the envelope-eroded observed volume.
    double directionalClearance(const DepthObservation& observation,
        const Eigen::Vector3d& position, const Eigen::Vector3d& direction, double now,
        double maximum = std::numeric_limits<double>::infinity()) const;
    bool motionSafe(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& command, double now) const;
private:
    bool certifyMotion(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& command, double now,
        std::vector<std::pair<Eigen::Vector3d,double>>* prefix) const;
    bool build(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction, const Eigen::Vector2d& state, double speed, double now,
        const Eigen::Vector2d* source_hint, const Eigen::Vector2d* goal_hint, double measured_speed);
    bool continueAnchor(const DepthObservation& next, const Eigen::Vector2d& next_state,
        Eigen::Vector2d* anchor) const;
    bool segmentKnown(const DepthObservation& observation,const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,double radius,double now) const;
    std::vector<std::shared_ptr<const DepthObservation>> history_;
    std::map<int,std::pair<double,std::uint64_t>> checked_observations_;
    using EnvelopeCache = std::map<std::tuple<int,int,int>,std::pair<Eigen::Vector3d,double>>;
    mutable std::array<EnvelopeCache,4> envelope_caches_;
    bool cache_enabled_ = false;
    Eigen::Vector2d diagnostic_state_ = Eigen::Vector2d::Zero(), diagnostic_rate_ = Eigen::Vector2d::Zero();
    double diagnostic_time_ = 0.0;
    bool diagnostic_valid_ = false;
    std::vector<std::pair<Eigen::Vector3d,double>> seeds_;
    bool seed_initialized_ = false;
    std::vector<std::pair<Eigen::Vector3d,double>> verified_travel_;
    // Certified sweep including the full braking tail of the last accepted motion. Not general free space;
    // it can only certify the actually measured current footprint.
    std::vector<std::pair<Eigen::Vector3d,double>> pending_motion_;
    std::size_t revoked_balls_=0, expired_views_=0, evicted_views_=0;
    double maximum_clearance_=0.0;
    bool target_holding_ = false;
    PaperConfig cfg_;
    AngularField field_;
    FlowBox box_;
    std::unique_ptr<DepthObservation> observation_;
    std::vector<Eigen::Vector3d> reference_world_;
    Eigen::Vector3d previous_command_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_intent_ = Eigen::Vector3d::Zero();
    double last_update_ = -1.0, coupling_error_ = 0.0, field_lookahead_ = 0.0;
};
} }
#endif
