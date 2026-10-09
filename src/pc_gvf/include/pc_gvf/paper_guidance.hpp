#ifndef PC_GVF_PAPER_GUIDANCE_HPP
#define PC_GVF_PAPER_GUIDANCE_HPP
#include "pc_gvf/depth_angular_core.hpp"
#include "pc_gvf/dynamic_obstacles.hpp"
#include "pc_gvf/spherical_memory.hpp"
#include "pc_gvf/shared_obstacles.hpp"
#include <memory>
#include <array>
#include <map>
#include <tuple>
#include <limits>
#include <string>
#include <iosfwd>

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
    double memory_uncertainty_rate = 0.;
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
// Virtual chart follows measured motion; at rest only operator intent selects it.
Camera intentChartCamera(const Camera& physical);
Eigen::Matrix3d intentChartRotation(const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent);
struct PaperConfig {
    SimConfig motion;
    double field_interval = 0.05;
    double observation_timeout = 0.5;
    double history_duration = 0.0;
    bool retain_verified_travel = false;
    // Static-scene volume already proved by an accepted full-motion check.
    // This stores geometry, never a claim that the vehicle followed a forecast.
    bool retain_certified_volume = false;
    // Keep consistent static depth columns when a local native hit conflicts.
    bool local_history_repair = false;
    bool dynamic_obstacles = false;
    bool incremental_field = false;
    bool spherical_memory = false;
    bool shared_obstacles = false;
    double history_sample_interval = .5;
    int history_max_observations = 32;
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
    bool continuous_certificates = true;
    bool adaptive_lookahead = false;
    bool adaptive_grid = false;
    double certificate_resolution = .005;
    // Extra observed room at the predicted stop, retained on measured arrival.
    double restart_clearance = 0.0;
    // Teleoperation policies; ROS enables them, pure-paper baselines do not.
    bool intent_guard = false;
    bool smooth_speed = false;
    bool model_velocity_shaping = false;
    bool proposal_feedforward = false;
    double proposal_jerk_limit = 8.;
    // Velocity-command ramp, not the physical acceleration used by certification.
    // Zero inherits the physical limit for legacy behavior.
    double proposal_command_accel = 0.;
    // Reference response is independent of the measured physical plant lag.
    double proposal_response_time = .22;
    // Compensate first-order velocity tracking, still certify the final command.
    bool response_preview = false;
    bool certified_direct = false;
};
// A capsule proven to lie in observed free volume along a measured segment.
struct CertifiedTube {
    Eigen::Vector3d a, b;
    double radius = 0.0;
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
    int iterations = 0;
    bool incremental = false;
    bool contains(const Eigen::Vector2d& y) const;
    bool segmentFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
    double value(const Eigen::Vector2d& y) const;
    Eigen::Vector2d flow(const Eigen::Vector2d& y) const;
};
AngularField solveHarmonicField(AngularField field,const std::vector<double>& boundary,
    const std::vector<double>& guess = {});
std::vector<double> reprojectPotential(const AngularField& previous,const DepthObservation& previous_pose,
    const AngularField& next,const DepthObservation& next_pose,double range);
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
    bool command_changed = false;
    double intent_change_angle = 0.0;
    std::string status = "WAITING_OBSERVATION";
    std::string reset_reason;
    std::string build_reason;
    double required_prefix = 0.0, best_prefix = 0.0;
    int free_directions = 0;
    bool grid_refined = false;
};
class PaperGuidance {
public:
    explicit PaperGuidance(const PaperConfig& config);
    PaperResult step(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& intent, double now, double dt);
    Eigen::Vector3d incrementalGoalProposal(const Eigen::Vector3d& position,
        const Eigen::Vector3d& intent,double now) const;
    PaperResult stepWithProposal(const DepthObservation& observation,const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,const Eigen::Vector3d& proposal,double now,double dt);
    void reset();
    // Synchronize post-planning watchdog overrides with the next planning step.
    void confirmPublishedCommand(const Eigen::Vector3d& command);
    void ingestObservation(const DepthObservation& observation);
    bool setVerifiedSeed(const Eigen::Vector3d& center,double radius);
    bool setVerifiedNeighborhood(const std::vector<Eigen::Vector3d>& centers,double radius);
    // One radius for state admission, motion certificates and reached evidence.
    double envelopeRadius() const;
    bool rememberVerifiedPosition(const DepthObservation& observation,
        const Eigen::Vector3d& position, double now);
    double maximumClearance() const { return maximum_clearance_; }
    std::size_t recordedBalls() const { return verified_travel_.size(); }
    std::size_t reachedProofs() const { return reached_proofs_.size(); }
    std::size_t recordedTubes() const { return verified_tubes_.size(); }
    std::size_t revokedTubes() const { return revoked_tubes_; }
    // Same-build, versioned offline replay. No scene geometry enters this file.
    void saveReplay(std::ostream& out, const DepthObservation& observation,
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& intent, double now, double dt,
        const Eigen::Vector3d* proposal = nullptr) const;
    static PaperResult replay(std::istream& in,std::ostream* audit = nullptr,
        bool enable_incremental_for_ablation = false);
    std::size_t revokedBalls() const { return revoked_balls_; }
    std::size_t expiredViews() const { return expired_views_; }
    std::size_t evictedViews() const { return evicted_views_; }
    bool envelopeKnown(const DepthObservation& observation,const Eigen::Vector3d& center,
        double radius,double now) const;
    bool ingestShared(const SharedObstaclePacket& packet,double now,const std::string& frame) {return shared_.merge(packet,now,frame);}
    const DynamicObstacles& obstacleTracks() const {return dynamic_;}
    Eigen::Vector3d memoryProposal(const DepthObservation& observation,const Eigen::Vector3d& position,
        const Eigen::Vector3d& intent,double now) const;
    const AngularField& field() const { return field_; }
    const FlowBox& box() const { return box_; }
    const Eigen::Vector3d& referenceIntent() const { return reference_intent_; }
    std::size_t retainedObservations() const { return history_.size(); }
    const std::array<double,4>& ingestionCosts() const {return ingestion_cost_ms_;}
    const std::vector<std::shared_ptr<const DepthObservation>>& retainedDepthObservations() const { return history_; }
    const DepthObservation* observation() const { return observation_.get(); }
    // Certified prefix in the envelope-eroded observed volume.
    double directionalClearance(const DepthObservation& observation,
        const Eigen::Vector3d& position, const Eigen::Vector3d& direction, double now,
        double maximum = std::numeric_limits<double>::infinity()) const;
    // A proposal goal must also have an observed corridor beyond stopping;
    // the actual curved execution still needs its independent motion proof.
    bool proposalCorridor(const DepthObservation& observation,const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction,double length,double now);
    // Geometric proposal proof only; actual execution still needs motionSafe.
    bool proposalPath(const DepthObservation& observation,
        const std::vector<Eigen::Vector3d>& points,double now) const;
    bool motionSafe(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& command, double now) const;
private:
    friend struct PaperReplayCodec;
    PaperResult stepImpl(const DepthObservation& observation,const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,double now,double dt);
    bool certifyMotion(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& command, double now,
        std::vector<std::pair<Eigen::Vector3d,double>>* prefix) const;
    bool build(const DepthObservation& observation, const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction, const Eigen::Vector2d& state, double speed, double now,
        const Eigen::Vector2d* source_hint, const Eigen::Vector2d* goal_hint, double measured_speed);
    bool continueAnchor(const DepthObservation& next, const Eigen::Vector2d& next_state,
        Eigen::Vector2d* anchor) const;
    bool segmentKnown(const DepthObservation& observation,const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,double radius,double now,bool pending = false) const;
    bool envelopeKnownImpl(const DepthObservation& observation,const Eigen::Vector3d& center,
        double radius,double now,bool pending) const;
    void rememberTube(const DepthObservation& observation,const Eigen::Vector3d& position,double now);
    std::vector<std::shared_ptr<const DepthObservation>> history_;
    std::array<double,4> ingestion_cost_ms_{{0,0,0,0}};
    std::map<int,std::pair<double,std::uint64_t>> checked_observations_;
    // Ingress sampling cadence is independent of coverage eviction/revocation.
    std::map<int,double> last_retained_stamp_;
    using EnvelopeCache = std::map<std::tuple<int,int,int>,std::pair<Eigen::Vector3d,double>>;
    mutable std::array<EnvelopeCache,4> envelope_caches_;
    bool cache_enabled_ = false;
    Eigen::Vector2d diagnostic_state_ = Eigen::Vector2d::Zero(), diagnostic_rate_ = Eigen::Vector2d::Zero();
    double diagnostic_time_ = 0.0;
    bool diagnostic_valid_ = false;
    Eigen::Vector3d field_position_ = Eigen::Vector3d::Zero();
    std::vector<std::pair<Eigen::Vector3d,double>> seeds_;
    bool seed_initialized_ = false;
    std::vector<std::pair<Eigen::Vector3d,double>> verified_travel_;
    // Independently proved balls covering the accepted motion and braking tail.
    // Default: used only after arrival. Explicit static-volume mode also reuses
    // these exact geometries before arrival; fresh native hits revoke conflicts.
    std::vector<std::pair<Eigen::Vector3d,double>> pending_motion_;
    std::vector<CertifiedTube> verified_tubes_;
    std::vector<std::pair<Eigen::Vector3d,double>> reached_proofs_;
    Eigen::Vector3d last_reached_ = Eigen::Vector3d::Zero();
    double last_reached_time_ = -1;
    std::size_t revoked_tubes_=0;
    std::size_t revoked_balls_=0, expired_views_=0, evicted_views_=0;
    std::string build_reason_;
    double required_prefix_=0;
    int free_directions_=0;
    bool grid_refined_=false;
    double maximum_clearance_=0.0;
    bool target_holding_ = false;
    int recovery_cursor_ = 0;
    SharedObstacleMap shared_;
    SphericalMemory spherical_;
    DynamicObstacles dynamic_;
    PaperConfig cfg_;
    AngularField field_;
    FlowBox box_;
    std::unique_ptr<DepthObservation> observation_;
    std::vector<Eigen::Vector3d> reference_world_;
    Eigen::Vector3d previous_command_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d proposal_acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d proposal_reference_ = Eigen::Vector3d::Zero();
    bool proposal_reference_valid_ = false;
    // Latest field target and active-reference intent have different lifetimes.
    // Depth refreshes must not erase accumulated operator direction changes.
    Eigen::Vector3d previous_intent_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_intent_ = Eigen::Vector3d::Zero();
    double last_update_ = -1.0, coupling_error_ = 0.0, field_lookahead_ = 0.0;
};
} }
#endif
