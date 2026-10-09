#include "pc_gvf/depth_angular_core.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <set>
#include <stdexcept>

namespace pc_gvf {
namespace depth_angular {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool finite(double value)
{
    return std::isfinite(value);
}

void validateGridSize(std::size_t size, int width, int height)
{
    if (width <= 0 || height <= 0 ||
        size != static_cast<std::size_t>(width) * height) {
        throw std::invalid_argument("grid size does not match its dimensions");
    }
}

std::size_t gridIndex(int u, int v, int width)
{
    return static_cast<std::size_t>(v) * width + u;
}

int roundedClampedPixel(double value, int limit)
{
    const double clipped = std::max(0.0, std::min(value, limit - 1.0));
    return static_cast<int>(std::nearbyint(clipped));
}

double dotProduct(const std::vector<double>& left, const std::vector<double>& right)
{
    double result = 0.0;
    for (std::size_t index = 0; index < left.size(); ++index) {
        result += left[index] * right[index];
    }
    return result;
}

}  // namespace

std::vector<double> euclideanDistanceToBlocked(
    const BinaryMask& blocked_mask, int width, int height)
{
    validateGridSize(blocked_mask.size(), width, height);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distance(blocked_mask.size(), infinity);
    const auto blocked_count = std::count_if(blocked_mask.begin(), blocked_mask.end(),
        [](std::uint8_t value) { return value != 0; });
    if (blocked_count == static_cast<std::ptrdiff_t>(blocked_mask.size()))
        return std::vector<double>(blocked_mask.size(), 0.0);
    if (blocked_count == 0) {
        for (int v = 0; v < height; ++v)
            for (int u = 0; u < width; ++u)
                distance[gridIndex(u, v, width)] =
                    std::hypot(static_cast<double>(u), static_cast<double>(v) + 1.0);
        return distance;
    }

    // The row pass gives exact horizontal squared distances. Empty rows stay
    // infinite and are excluded from the column envelope below.
    for (int v = 0; v < height; ++v) {
        int nearest = -1;
        for (int u = 0; u < width; ++u) {
            const auto index = gridIndex(u, v, width);
            if (blocked_mask[index] != 0) {
                nearest = u;
            }
            if (nearest >= 0) {
                const double delta = static_cast<double>(u) - nearest;
                distance[index] = delta * delta;
            }
        }
        nearest = -1;
        for (int u = width - 1; u >= 0; --u) {
            const auto index = gridIndex(u, v, width);
            if (blocked_mask[index] != 0) nearest = u;
            if (nearest >= 0) {
                const double delta = static_cast<double>(nearest) - u;
                distance[index] = std::min(distance[index], delta * delta);
            }
        }
    }
    // For each column, minimize f(q) + (v-q)^2 over rows q using its lower
    // parabola envelope. Each row is pushed/popped at most once: O(width*height).
    // This is exact Euclidean pixel clearance, not a chamfer approximation.
    std::vector<double> column(height), boundaries(static_cast<std::size_t>(height) + 1);
    std::vector<int> sites(height);
    for (int u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v)
            column[v] = distance[gridIndex(u, v, width)];
        int last = -1;
        for (int q = 0; q < height; ++q) {
            if (!std::isfinite(column[q])) continue;
            double crossing = -infinity;
            while (last >= 0) {
                const int previous = sites[last];
                crossing = ((column[q] + static_cast<double>(q) * q) -
                    (column[previous] + static_cast<double>(previous) * previous)) /
                    (2.0 * (q - previous));
                if (crossing > boundaries[last]) break;
                --last;
            }
            ++last;
            sites[last] = q;
            boundaries[last] = last == 0 ? -infinity : crossing;
            boundaries[last + 1] = infinity;
        }
        int active = 0;
        for (int v = 0; v < height; ++v) {
            while (active < last && boundaries[active + 1] < v) ++active;
            const int q = sites[active];
            const double delta = static_cast<double>(v) - q;
            distance[gridIndex(u, v, width)] = std::sqrt(column[q] + delta * delta);
        }
    }
    return distance;
}

void applyObstacleReleaseHysteresis(
    const BinaryMask& observed_mask,
    int clear_frames,
    bool advance_frame,
    BinaryMask* stable_mask,
    std::vector<std::uint8_t>* clear_counts)
{
    if (stable_mask == nullptr || clear_counts == nullptr) {
        throw std::invalid_argument("hysteresis state pointers must not be null");
    }
    if (clear_frames < 1 || clear_frames > 255) {
        throw std::invalid_argument("clear_frames must be between 1 and 255");
    }
    if (stable_mask->size() != observed_mask.size() ||
        clear_counts->size() != observed_mask.size()) {
        *stable_mask = observed_mask;
        clear_counts->assign(observed_mask.size(), 0);
        return;
    }
    if (!advance_frame) {
        return;
    }
    for (std::size_t index = 0; index < observed_mask.size(); ++index) {
        if (observed_mask[index] != 0) {
            (*stable_mask)[index] = 1;
            (*clear_counts)[index] = 0;
        } else if ((*stable_mask)[index] != 0) {
            const int next_count = std::min(
                clear_frames, static_cast<int>((*clear_counts)[index]) + 1);
            (*clear_counts)[index] = static_cast<std::uint8_t>(next_count);
            if (next_count >= clear_frames) {
                (*stable_mask)[index] = 0;
                (*clear_counts)[index] = 0;
            }
        } else {
            (*clear_counts)[index] = 0;
        }
    }
}

Eigen::Vector3d normalize(
    const Eigen::Vector3d& value, const Eigen::Vector3d* fallback)
{
    const double norm = value.norm();
    if (norm > kEpsilon && value.allFinite()) {
        return value / norm;
    }
    return fallback == nullptr ? Eigen::Vector3d::Zero() : *fallback;
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double limit)
{
    const double norm = value.norm();
    if (norm <= limit || norm <= kEpsilon) {
        return value;
    }
    return value * (limit / norm);
}

std::size_t selectClosestYaw(
    double desired_yaw,
    const std::vector<double>& camera_yaws)
{
    if (!std::isfinite(desired_yaw) || camera_yaws.empty()) {
        throw std::invalid_argument("camera yaw selection requires finite input");
    }
    std::size_t best = 0;
    double best_error = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < camera_yaws.size(); ++index) {
        if (!std::isfinite(camera_yaws[index])) {
            throw std::invalid_argument("camera yaw offsets must be finite");
        }
        const double delta = std::atan2(
            std::sin(desired_yaw - camera_yaws[index]),
            std::cos(desired_yaw - camera_yaws[index]));
        const double error = std::abs(delta);
        if (error < best_error) {
            best = index;
            best_error = error;
        }
    }
    return best;
}

Eigen::Matrix3d fixedCameraRotation()
{
    Eigen::Matrix3d rotation;
    rotation <<
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0;
    return rotation;
}

bool quaternionMatrix(
    double x, double y, double z, double w, Eigen::Matrix3d* rotation)
{
    if (rotation == nullptr) {
        return false;
    }
    const double norm = x * x + y * y + z * z + w * w;
    if (!finite(norm) || norm < 1.0e-12) {
        return false;
    }
    const double scale = 2.0 / norm;
    *rotation <<
        1.0 - scale * (y * y + z * z),
        scale * (x * y - z * w),
        scale * (x * z + y * w),
        scale * (x * y + z * w),
        1.0 - scale * (x * x + z * z),
        scale * (y * z - x * w),
        scale * (x * z - y * w),
        scale * (y * z + x * w),
        1.0 - scale * (x * x + y * y);
    return rotation->allFinite();
}

Camera::Camera(
    int width,
    int height,
    double horizontal_fov_deg,
    double vertical_fov_deg,
    double max_depth)
: width_(width),
  height_(height),
  horizontal_fov_deg_(horizontal_fov_deg),
  vertical_fov_deg_(vertical_fov_deg),
  max_depth_(max_depth),
  fx_(0.0),
  fy_(0.0),
  cx_(0.5 * (width - 1)),
  cy_(0.5 * (height - 1))
{
    if (width <= 0 || height <= 0 ||
        !finite(horizontal_fov_deg) || !finite(vertical_fov_deg) ||
        horizontal_fov_deg <= 0.0 || horizontal_fov_deg >= 180.0 ||
        vertical_fov_deg <= 0.0 || vertical_fov_deg >= 180.0 ||
        !finite(max_depth) || max_depth <= 0.0) {
        throw std::invalid_argument("invalid camera dimensions, FOV, or range");
    }
    fx_ = 0.5 * width_ /
        std::tan(horizontal_fov_deg_ * kPi / 360.0);
    fy_ = 0.5 * height_ /
        std::tan(vertical_fov_deg_ * kPi / 360.0);
    rebuildRays();
}

void Camera::setIntrinsics(double fx, double fy, double cx, double cy)
{
    if (!finite(fx) || !finite(fy) || !finite(cx) || !finite(cy) ||
        fx <= 0.0 || fy <= 0.0) {
        throw std::invalid_argument("invalid pinhole camera intrinsics");
    }
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    rebuildRays();
}

Eigen::Vector3d Camera::rayFromPixel(const Eigen::Vector2d& pixel) const
{
    return normalize(Eigen::Vector3d(
        (pixel.x() - cx_) / fx_,
        (pixel.y() - cy_) / fy_,
        1.0));
}

bool Camera::pixelFromDirection(
    const Eigen::Vector3d& direction_camera, Eigen::Vector2d* pixel) const
{
    if (pixel == nullptr || !direction_camera.allFinite() ||
        direction_camera.z() <= 1.0e-5) {
        return false;
    }
    const Eigen::Vector2d projected(
        fx_ * direction_camera.x() / direction_camera.z() + cx_,
        fy_ * direction_camera.y() / direction_camera.z() + cy_);
    if (!inside(projected)) {
        return false;
    }
    *pixel = projected;
    return true;
}

bool Camera::inside(const Eigen::Vector2d& pixel, double margin) const
{
    return pixel.allFinite() && finite(margin) &&
        margin <= pixel.x() && pixel.x() <= width_ - 1 - margin &&
        margin <= pixel.y() && pixel.y() <= height_ - 1 - margin;
}

const Eigen::Vector3d& Camera::ray(int u, int v) const
{
    if (u < 0 || u >= width_ || v < 0 || v >= height_) {
        throw std::out_of_range("camera ray index outside image");
    }
    return rays_camera_[static_cast<std::size_t>(v) * width_ + u];
}

void Camera::rebuildRays()
{
    rays_camera_.resize(static_cast<std::size_t>(width_) * height_);
    for (int v = 0; v < height_; ++v) {
        for (int u = 0; u < width_; ++u) {
            rays_camera_[static_cast<std::size_t>(v) * width_ + u] =
                rayFromPixel(Eigen::Vector2d(u, v));
        }
    }
}

std::vector<Eigen::Vector3d> backprojectObstaclePoints(
    const std::vector<double>& depth, const Camera& camera, int stride)
{
    const std::size_t pixel_count =
        static_cast<std::size_t>(camera.width()) * camera.height();
    if (depth.size() != pixel_count) {
        throw std::invalid_argument("depth size does not match camera dimensions");
    }
    if (stride <= 0) {
        throw std::invalid_argument("depth point stride must be positive");
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(pixel_count / static_cast<std::size_t>(stride * stride));
    const double hit_limit = camera.maxDepth() - 1.0e-6;
    for (int v = 0; v < camera.height(); ++v) {
        if (stride > 1 && v % stride != 0) {
            continue;
        }
        for (int u = 0; u < camera.width(); ++u) {
            if (stride > 1 && u % stride != 0) {
                continue;
            }
            const double z = depth[static_cast<std::size_t>(v) * camera.width() + u];
            if (!finite(z) || z <= 0.0 || z >= hit_limit) {
                continue;
            }
            points.emplace_back(
                z * (static_cast<double>(u) - camera.cx()) / camera.fx(),
                z * (static_cast<double>(v) - camera.cy()) / camera.fy(),
                z);
        }
    }
    return points;
}

std::vector<double> collisionConeFreeDistance(
    const std::vector<Eigen::Vector3d>& points_camera,
    const Camera& camera,
    double effective_radius,
    int chunk_size)
{
    if (!finite(effective_radius) || effective_radius < 0.0) {
        throw std::invalid_argument("effective radius must be finite and nonnegative");
    }
    if (chunk_size <= 0) {
        throw std::invalid_argument("collision-cone chunk size must be positive");
    }
    for (const Eigen::Vector3d& point : points_camera) {
        if (!point.allFinite()) {
            throw std::invalid_argument("obstacle points must be finite");
        }
    }

    const double sensor_limited_free =
        std::max(0.0, camera.maxDepth() - effective_radius);
    std::vector<double> free(camera.rays().size(), sensor_limited_free);
    if (points_camera.empty()) {
        return free;
    }

    const double radius_squared = effective_radius * effective_radius;
    // Project each sphere's exact tangent bounds before testing rays. Rays
    // outside this rectangle cannot meet the sphere; the original quadratic
    // intersection remains the narrow phase. This avoids a full N*M scan.
    for (const Eigen::Vector3d& point : points_camera) {
        int x0=0, x1=camera.width()-1, y0=0, y1=camera.height()-1;
        if (point.z() > effective_radius + 1e-9) {
            const double denominator=point.z()*point.z()-radius_squared;
            auto bounds=[&](double value,double focal,double center,int count,int* lo,int* hi) {
                const double root=effective_radius*std::sqrt(std::max(0.,value*value+denominator));
                const double a=focal*(value*point.z()-root)/denominator+center;
                const double b=focal*(value*point.z()+root)/denominator+center;
                // Extra pixel padding also covers roundoff at grazing rays.
                *lo=std::max(0,static_cast<int>(std::floor(std::max(-2.,std::min(double(count+1),a))))-1);
                *hi=std::min(count-1,static_cast<int>(std::ceil(std::max(-2.,std::min(double(count+1),b))))+1);
            };
            bounds(point.x(),camera.fx(),camera.cx(),camera.width(),&x0,&x1);
            bounds(point.y(),camera.fy(),camera.cy(),camera.height(),&y0,&y1);
        }
        const double norm_squared=point.squaredNorm();
        for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
            const std::size_t index=gridIndex(x,y,camera.width());
            const double longitudinal=camera.rays()[index].dot(point);
            const double lateral_squared=std::max(0.,norm_squared-longitudinal*longitudinal);
            if(longitudinal>0. && lateral_squared<radius_squared) {
                const double root=std::sqrt(std::max(0.,radius_squared-lateral_squared));
                free[index]=std::min(free[index],longitudinal-root);
            }
        }
    }
    return free;
}

bool nearestFreePixel(
    const Eigen::Vector2d& query,
    const BinaryMask& free_mask,
    int width,
    int height,
    const Eigen::Vector2d* history,
    double left_bias,
    Eigen::Vector2d* result)
{
    validateGridSize(free_mask.size(), width, height);
    if (result == nullptr || !query.allFinite() ||
        (history != nullptr && !history->allFinite()) || !finite(left_bias)) {
        throw std::invalid_argument("invalid nearest-free-pixel input");
    }

    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best = Eigen::Vector2d::Zero();
    const double width_scale = std::max(1.0, static_cast<double>(width));
    const double height_scale = std::max(1.0, static_cast<double>(height));
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            if (free_mask[gridIndex(u, v, width)] == 0) {
                continue;
            }
            const double du = (u - query.x()) / width_scale;
            const double dv = (v - query.y()) / height_scale;
            double cost = du * du + dv * dv;
            if (history != nullptr) {
                const double history_du = (u - history->x()) / width;
                const double history_dv = (v - history->y()) / height;
                cost += 0.15 * (
                    history_du * history_du + history_dv * history_dv);
            }
            cost += left_bias * (u - query.x()) / width_scale;
            if (!found || cost < best_cost) {
                found = true;
                best_cost = cost;
                best = Eigen::Vector2d(static_cast<double>(u),
                                       static_cast<double>(v));
            }
        }
    }
    if (found) {
        *result = best;
    }
    return found;
}

std::vector<int> labelFreeComponents(
    const BinaryMask& free_mask,
    int width,
    int height,
    int* component_count)
{
    validateGridSize(free_mask.size(), width, height);
    std::vector<int> labels(free_mask.size(), 0);
    int next_label = 0;
    for (int seed_v = 0; seed_v < height; ++seed_v) {
        for (int seed_u = 0; seed_u < width; ++seed_u) {
            const std::size_t seed_index = gridIndex(seed_u, seed_v, width);
            if (free_mask[seed_index] == 0 || labels[seed_index] != 0) {
                continue;
            }
            ++next_label;
            labels[seed_index] = next_label;
            std::deque<Eigen::Vector2i> queue;
            queue.emplace_back(seed_u, seed_v);
            while (!queue.empty()) {
                const Eigen::Vector2i current = queue.front();
                queue.pop_front();
                for (int dv = -1; dv <= 1; ++dv) {
                    for (int du = -1; du <= 1; ++du) {
                        if (du == 0 && dv == 0) {
                            continue;
                        }
                        const int u = current.x() + du;
                        const int v = current.y() + dv;
                        if (u < 0 || u >= width || v < 0 || v >= height) {
                            continue;
                        }
                        const std::size_t index = gridIndex(u, v, width);
                        if (free_mask[index] == 0 || labels[index] != 0) {
                            continue;
                        }
                        labels[index] = next_label;
                        queue.emplace_back(u, v);
                    }
                }
            }
        }
    }
    if (component_count != nullptr) {
        *component_count = next_label;
    }
    return labels;
}

GoalSelection chooseSafeGoal(
    const Eigen::Vector2d& reference,
    const Eigen::Vector2d& source,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d* previous_goal,
    const SimConfig& config,
    const Camera* angular_camera)
{
    validateGridSize(blocked_mask.size(), width, height);
    BinaryMask free_mask(blocked_mask.size(), 0);
    for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
        free_mask[index] = blocked_mask[index] == 0 ? 1 : 0;
    }

    GoalSelection selection;
    selection.labels = labelFreeComponents(free_mask, width, height);
    Eigen::Vector2d safe_source;
    if (!nearestFreePixel(
            source, free_mask, width, height, previous_goal,
            config.deterministic_left_bias, &safe_source)) {
        return selection;
    }
    const int source_u = static_cast<int>(std::nearbyint(safe_source.x()));
    const int source_v = static_cast<int>(std::nearbyint(safe_source.y()));
    const int component = selection.labels[gridIndex(source_u, source_v, width)];
    if (component <= 0) {
        return selection;
    }

    const int reference_u = roundedClampedPixel(reference.x(), width);
    const int reference_v = roundedClampedPixel(reference.y(), height);
    if (selection.labels[gridIndex(reference_u, reference_v, width)] == component) {
        selection.valid = true;
        selection.source = safe_source;
        selection.goal = Eigen::Vector2d(reference_u, reference_v);
        return selection;
    }

    const std::vector<double> clearance =
        euclideanDistanceToBlocked(blocked_mask, width, height);
    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_goal = Eigen::Vector2d::Zero();
    const auto candidate_cost=[&](int u,int v) {
        const auto index=gridIndex(u,v,width);
        const double du = (u - reference.x()) / width;
        const double dv = (v - reference.y()) / height;
        double cost = du * du + dv * dv;
        // A pixel fraction is not a physical angle when horizontal and
        // vertical FoV differ. Use the true 3-D ray separation for the
        // paper-style proposal, retaining the legacy cost by default.
        const Eigen::Vector2d candidate(u,v);
        if(angular_camera) {
            const double angle=angularDistance(*angular_camera,candidate,reference);
            cost=.25*angle*angle;
        }
        const double rewarded_clearance=config.goal_clearance_cap>0.?
            std::min(clearance[index],config.goal_clearance_cap):clearance[index];
        cost -= config.clearance_reward * rewarded_clearance;
        if (previous_goal != nullptr) {
            const double previous_du = (u - previous_goal->x()) / width;
            const double previous_dv = (v - previous_goal->y()) / height;
            const double separation=angular_camera?
                .25*std::pow(angularDistance(*angular_camera,candidate,*previous_goal),2):
                previous_du*previous_du+previous_dv*previous_dv;
            cost += config.hysteresis_weight*separation;
        }
        cost += config.deterministic_left_bias *
            (u - reference.x()) / width;
        return cost;
    };
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const std::size_t index = gridIndex(u, v, width);
            if (selection.labels[index] != component) {
                continue;
            }
            const double cost=candidate_cost(u,v);
            if (!found || cost < best_cost) {
                found = true;
                best_cost = cost;
                best_goal = Eigen::Vector2d(u, v);
            }
        }
    }
    if (!found) {
        return selection;
    }
    selection.valid = true;
    selection.source = safe_source;
    selection.goal = best_goal;
    if(config.subpixel_goal) {
        const int u=static_cast<int>(best_goal.x()),v=static_cast<int>(best_goal.y());
        // Refine only inside a wholly free 3x3 neighborhood. Each fitted
        // coordinate stays in the selected pixel, preserving its component.
        bool interior=u>0&&v>0&&u+1<width&&v+1<height;
        for(int y=v-1;interior&&y<=v+1;++y)for(int x=u-1;interior&&x<=u+1;++x)
            interior=selection.labels[gridIndex(x,y,width)]==component;
        if(interior)for(int axis=0;axis<2;++axis) {
            const double low=candidate_cost(u-(axis==0),v-(axis==1));
            const double high=candidate_cost(u+(axis==0),v+(axis==1));
            const double curvature=low+high-2.*best_cost;
            if(std::isfinite(curvature)&&curvature>1e-10)
                selection.goal[axis]+=std::max(-.49,std::min(.49,.5*(low-high)/curvature));
        }
    }
    return selection;
}

BinaryMask diskMask(
    int width,
    int height,
    const Eigen::Vector2d& center,
    int radius)
{
    if (width <= 0 || height <= 0 || !center.allFinite()) {
        throw std::invalid_argument("invalid disk-mask input");
    }
    BinaryMask result(static_cast<std::size_t>(width) * height, 0);
    const double radius_squared =
        static_cast<double>(radius) * static_cast<double>(radius);
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const double du = u - center.x();
            const double dv = v - center.y();
            if (du * du + dv * dv <= radius_squared) {
                result[gridIndex(u, v, width)] = 1;
            }
        }
    }
    return result;
}

HarmonicSolution solveAngularHarmonic(
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source_point,
    const Eigen::Vector2d& goal_point,
    const SimConfig& config,
    const std::vector<double>* initial_potential)
{
    validateGridSize(blocked_mask.size(), width, height);
    if (config.field_max_iterations < 0 ||
        !finite(config.field_tolerance) || config.field_tolerance < 0.0) {
        throw std::invalid_argument("invalid harmonic solver configuration");
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    HarmonicSolution solution;
    solution.potential.assign(blocked_mask.size(), nan);
    BinaryMask source = diskMask(
        width, height, source_point, config.source_radius_cells);
    BinaryMask goal = diskMask(
        width, height, goal_point, config.goal_radius_cells);
    bool any_source = false;
    bool any_goal = false;
    bool overlap = false;
    for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
        source[index] = source[index] != 0 && blocked_mask[index] == 0 ? 1 : 0;
        goal[index] = goal[index] != 0 && blocked_mask[index] == 0 ? 1 : 0;
        any_source = any_source || source[index] != 0;
        any_goal = any_goal || goal[index] != 0;
        overlap = overlap || (source[index] != 0 && goal[index] != 0);
    }
    if (!any_source || !any_goal) {
        return solution;
    }
    if (overlap) {
        for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
            if (blocked_mask[index] == 0) {
                solution.potential[index] = 0.0;
            }
            if (source[index] != 0) {
                solution.potential[index] = 1.0;
            }
        }
        solution.valid = true;
        return solution;
    }

    std::vector<int> unknown_index(blocked_mask.size(), -1);
    int unknown_count = 0;
    for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
        if (blocked_mask[index] == 0 && source[index] == 0 && goal[index] == 0) {
            unknown_index[index] = unknown_count++;
        }
    }
    if (unknown_count == 0) {
        for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
            if (source[index] != 0) {
                solution.potential[index] = 1.0;
            }
            if (goal[index] != 0) {
                solution.potential[index] = 0.0;
            }
        }
        solution.valid = true;
        return solution;
    }

    std::vector<double> diagonal(static_cast<std::size_t>(unknown_count), 1.0);
    std::vector<double> rhs(static_cast<std::size_t>(unknown_count), 0.0);
    std::vector<std::vector<int>> unknown_neighbors(
        static_cast<std::size_t>(unknown_count));
    const int neighbor_dv[4] = {-1, 1, 0, 0};
    const int neighbor_du[4] = {0, 0, -1, 1};
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const std::size_t index = gridIndex(u, v, width);
            const int row = unknown_index[index];
            if (row < 0) {
                continue;
            }
            double degree = 0.0;
            for (int neighbor = 0; neighbor < 4; ++neighbor) {
                const int next_v = v + neighbor_dv[neighbor];
                const int next_u = u + neighbor_du[neighbor];
                if (next_v < 0 || next_v >= height ||
                    next_u < 0 || next_u >= width) {
                    continue;
                }
                const std::size_t next_index =
                    gridIndex(next_u, next_v, width);
                if (blocked_mask[next_index] != 0) {
                    continue;
                }
                degree += 1.0;
                if (unknown_index[next_index] >= 0) {
                    unknown_neighbors[static_cast<std::size_t>(row)].push_back(
                        unknown_index[next_index]);
                } else if (source[next_index] != 0) {
                    rhs[static_cast<std::size_t>(row)] += 1.0;
                }
            }
            diagonal[static_cast<std::size_t>(row)] =
                degree <= 0.0 ? 1.0 : degree;
        }
    }

    const auto apply_matrix = [&diagonal, &unknown_neighbors](
        const std::vector<double>& input) {
        std::vector<double> output(input.size(), 0.0);
        for (std::size_t row = 0; row < input.size(); ++row) {
            output[row] = diagonal[row] * input[row];
            for (const int column : unknown_neighbors[row]) {
                output[row] -= input[static_cast<std::size_t>(column)];
            }
        }
        return output;
    };

    std::vector<double> x(static_cast<std::size_t>(unknown_count), 0.0);
    if (initial_potential != nullptr &&
        initial_potential->size() == blocked_mask.size()) {
        for (std::size_t index = 0; index < unknown_index.size(); ++index) {
            const int row = unknown_index[index];
            const double value = (*initial_potential)[index];
            if (row >= 0 && finite(value)) {
                x[static_cast<std::size_t>(row)] =
                    std::max(0.0, std::min(1.0, value));
            }
        }
    }
    const std::vector<double> matrix_x = apply_matrix(x);
    std::vector<double> residual(rhs.size(), 0.0);
    for (std::size_t index = 0; index < residual.size(); ++index) {
        residual[index] = rhs[index] - matrix_x[index];
    }
    std::vector<double> direction = residual;
    double residual_squared = dotProduct(residual, residual);
    const double target = config.field_tolerance * std::sqrt(dotProduct(rhs, rhs));
    bool converged = std::sqrt(residual_squared) <= target;
    for (int iteration = 0;
         iteration < config.field_max_iterations && !converged; ++iteration) {
        const std::vector<double> matrix_direction = apply_matrix(direction);
        const double denominator = dotProduct(direction, matrix_direction);
        if (!finite(denominator) || denominator == 0.0) {
            break;
        }
        const double alpha = residual_squared / denominator;
        for (std::size_t index = 0; index < x.size(); ++index) {
            x[index] += alpha * direction[index];
            residual[index] -= alpha * matrix_direction[index];
        }
        const double next_residual_squared = dotProduct(residual, residual);
        converged = std::sqrt(next_residual_squared) <= target;
        if (converged) {
            residual_squared = next_residual_squared;
            break;
        }
        if (!finite(next_residual_squared) || residual_squared == 0.0) {
            break;
        }
        const double beta = next_residual_squared / residual_squared;
        for (std::size_t index = 0; index < direction.size(); ++index) {
            direction[index] = residual[index] + beta * direction[index];
        }
        residual_squared = next_residual_squared;
    }

    for (const double value : x) {
        if (!finite(value)) {
            converged = false;
            break;
        }
    }
    for (std::size_t index = 0; index < blocked_mask.size(); ++index) {
        if (source[index] != 0) {
            solution.potential[index] = 1.0;
        } else if (goal[index] != 0) {
            solution.potential[index] = 0.0;
        }
    }
    if (!converged) {
        return solution;
    }
    for (std::size_t index = 0; index < unknown_index.size(); ++index) {
        if (unknown_index[index] >= 0) {
            solution.potential[index] = std::max(
                0.0,
                std::min(1.0, x[static_cast<std::size_t>(unknown_index[index])]));
        }
    }
    solution.valid = true;
    return solution;
}

double bilinearSample(
    const std::vector<double>& image,
    int width,
    int height,
    const Eigen::Vector2d& pixel,
    double default_value)
{
    validateGridSize(image.size(), width, height);
    const double u = pixel.x();
    const double v = pixel.y();
    if (!finite(u) || !finite(v)) {
        throw std::invalid_argument("bilinear sample pixel must be finite");
    }
    if (u < 0.0 || v < 0.0 || u > width - 1 || v > height - 1) {
        return default_value;
    }
    const int u0 = static_cast<int>(std::floor(u));
    const int v0 = static_cast<int>(std::floor(v));
    const int u1 = std::min(u0 + 1, width - 1);
    const int v1 = std::min(v0 + 1, height - 1);
    const double tu = u - u0;
    const double tv = v - v0;
    const double value00 = image[gridIndex(u0, v0, width)];
    const double value10 = image[gridIndex(u1, v0, width)];
    const double value01 = image[gridIndex(u0, v1, width)];
    const double value11 = image[gridIndex(u1, v1, width)];
    if (!finite(value00) || !finite(value10) ||
        !finite(value01) || !finite(value11)) {
        return default_value;
    }
    return (1.0 - tv) * ((1.0 - tu) * value00 + tu * value10) +
        tv * ((1.0 - tu) * value01 + tu * value11);
}

std::vector<Eigen::Vector2d> discreteHarmonicPath(
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source,
    const Eigen::Vector2d& goal_point,
    int max_steps)
{
    validateGridSize(potential.size(), width, height);
    validateGridSize(blocked_mask.size(), width, height);
    Eigen::Vector2i current(
        roundedClampedPixel(source.x(), width),
        roundedClampedPixel(source.y(), height));
    const Eigen::Vector2i goal(
        static_cast<int>(std::nearbyint(goal_point.x())),
        static_cast<int>(std::nearbyint(goal_point.y())));
    std::vector<Eigen::Vector2d> path;
    path.push_back(current.cast<double>());
    std::set<std::pair<int, int>> visited;
    visited.emplace(current.x(), current.y());
    for (int step = 0; step < max_steps; ++step) {
        if ((current - goal).cast<double>().norm() <= 1.5) {
            path.push_back(goal.cast<double>());
            break;
        }
        bool found = false;
        double best_cost = std::numeric_limits<double>::infinity();
        int best_u = 0;
        int best_v = 0;
        for (int dv = -1; dv <= 1; ++dv) {
            for (int du = -1; du <= 1; ++du) {
                if (du == 0 && dv == 0) {
                    continue;
                }
                const int u = current.x() + du;
                const int v = current.y() + dv;
                if (u < 0 || u >= width || v < 0 || v >= height) {
                    continue;
                }
                const std::size_t index = gridIndex(u, v, width);
                if (blocked_mask[index] != 0 || !finite(potential[index])) {
                    continue;
                }
                const double repeat_penalty =
                    visited.count(std::make_pair(u, v)) == 0 ? 0.0 : 0.2;
                const double goal_tie = 1.0e-4 * std::hypot(
                    static_cast<double>(u - goal.x()),
                    static_cast<double>(v - goal.y()));
                const double cost = potential[index] + repeat_penalty + goal_tie;
                if (!found || cost < best_cost) {
                    found = true;
                    best_cost = cost;
                    best_u = u;
                    best_v = v;
                }
            }
        }
        if (!found) {
            break;
        }
        const Eigen::Vector2i next(best_u, best_v);
        if (next == current) {
            break;
        }
        current = next;
        path.push_back(current.cast<double>());
        visited.emplace(current.x(), current.y());
    }
    return path;
}

Eigen::Vector2d continuousHarmonicTarget(
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    int width,
    int height,
    const Eigen::Vector2d& source,
    const Eigen::Vector2d& goal,
    int gradient_radius_cells,
    double lookahead_cells)
{
    validateGridSize(potential.size(), width, height);
    validateGridSize(blocked_mask.size(), width, height);
    if (!source.allFinite() || !goal.allFinite() ||
        gradient_radius_cells < 1 || !finite(lookahead_cells) ||
        lookahead_cells <= 0.0) {
        throw std::invalid_argument("invalid continuous harmonic target input");
    }

    // Fit a local plane to the potential instead of selecting one of eight
    // neighboring pixels.  The fit reaches beyond the source Dirichlet disk,
    // where a one-pixel finite difference would otherwise be exactly zero.
    double suu = 0.0;
    double suv = 0.0;
    double svv = 0.0;
    double sup = 0.0;
    double svp = 0.0;
    const double sigma = std::max(1.0, 0.55 * gradient_radius_cells);
    const int center_u = roundedClampedPixel(source.x(), width);
    const int center_v = roundedClampedPixel(source.y(), height);
    const double center_value = bilinearSample(
        potential, width, height, source,
        potential[gridIndex(center_u, center_v, width)]);
    for (int v = std::max(0, center_v - gradient_radius_cells);
         v <= std::min(height - 1, center_v + gradient_radius_cells); ++v) {
        for (int u = std::max(0, center_u - gradient_radius_cells);
             u <= std::min(width - 1, center_u + gradient_radius_cells); ++u) {
            const std::size_t index = gridIndex(u, v, width);
            if (blocked_mask[index] != 0 || !finite(potential[index])) {
                continue;
            }
            const double du = u - source.x();
            const double dv = v - source.y();
            const double radius_squared = du * du + dv * dv;
            if (radius_squared <= kEpsilon ||
                radius_squared > gradient_radius_cells * gradient_radius_cells) {
                continue;
            }
            const double weight = std::exp(
                -0.5 * radius_squared / (sigma * sigma));
            const double dp = potential[index] - center_value;
            suu += weight * du * du;
            suv += weight * du * dv;
            svv += weight * dv * dv;
            sup += weight * du * dp;
            svp += weight * dv * dp;
        }
    }
    const double determinant = suu * svv - suv * suv;
    Eigen::Vector2d descent = goal - source;
    if (std::abs(determinant) > 1.0e-10) {
        const double gradient_u = (svv * sup - suv * svp) / determinant;
        const double gradient_v = (suu * svp - suv * sup) / determinant;
        const Eigen::Vector2d fitted_descent(-gradient_u, -gradient_v);
        if (fitted_descent.allFinite() && fitted_descent.norm() > 1.0e-7) {
            descent = fitted_descent;
        }
    }
    if (descent.norm() <= kEpsilon) {
        return source;
    }
    Eigen::Vector2d target = source + lookahead_cells * descent.normalized();
    target.x() = std::max(0.0, std::min(target.x(), width - 1.0));
    target.y() = std::max(0.0, std::min(target.y(), height - 1.0));

    // Keep the look-ahead endpoint in free space; shortening preserves the
    // continuous direction while avoiding a target inside an inflated cone.
    for (int attempt = 0; attempt < 6; ++attempt) {
        const int u = roundedClampedPixel(target.x(), width);
        const int v = roundedClampedPixel(target.y(), height);
        if (blocked_mask[gridIndex(u, v, width)] == 0) {
            return target;
        }
        target = 0.5 * (source + target);
    }
    return source;
}

double angularDistance(
    const Camera& camera,
    const Eigen::Vector2d& first,
    const Eigen::Vector2d& second)
{
    const Eigen::Vector3d first_ray = camera.rayFromPixel(first);
    const Eigen::Vector3d second_ray = camera.rayFromPixel(second);
    const double cosine = std::max(-1.0, std::min(1.0, first_ray.dot(second_ray)));
    return std::acos(cosine);
}

Eigen::Vector2d angularRateLimit(
    const Camera& camera,
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to,
    double max_angle)
{
    const double angle = angularDistance(camera, from, to);
    if (angle <= max_angle || angle <= kEpsilon) {
        return to;
    }
    return from + (max_angle / angle) * (to - from);
}

Eigen::Vector3d referenceConvergedDirection(
    const Eigen::Vector3d& field_direction_world,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& reference_origin_world,
    const Eigen::Vector3d& reference_direction_world,
    double observed_clearance,
    const SimConfig& config)
{
    Eigen::Vector2d reference_xy = reference_direction_world.head<2>();
    const double reference_norm = reference_xy.norm();
    if (reference_norm > kEpsilon && reference_xy.allFinite()) {
        reference_xy /= reference_norm;
    } else {
        reference_xy.setZero();
    }
    const double horizontal_speed = field_direction_world.head<2>().norm();
    if (horizontal_speed <= kEpsilon || reference_xy.norm() <= kEpsilon) {
        return field_direction_world;
    }
    const Eigen::Vector2d normal_xy(-reference_xy.y(), reference_xy.x());
    const double transverse =
        (position_world.head<2>() - reference_origin_world.head<2>()).dot(normal_xy);
    const double field_angle = std::atan2(
        field_direction_world.y(), field_direction_world.x());
    const double target_angle = std::atan2(reference_xy.y(), reference_xy.x()) -
        std::atan(transverse / std::max(kEpsilon, config.convergence_length));
    double delta = std::atan2(
        std::sin(target_angle - field_angle),
        std::cos(target_angle - field_angle));
    const double xi = std::max(0.0, std::min(
        1.0,
        (observed_clearance - config.convergence_distance) /
            std::max(kEpsilon, config.convergence_margin)));
    const double gate = xi * xi * (3.0 - 2.0 * xi);
    delta = std::max(
        -config.convergence_max_angle,
        std::min(config.convergence_max_angle, delta)) * gate;
    const double corrected_angle = field_angle + delta;
    Eigen::Vector3d corrected = field_direction_world;
    corrected.x() = horizontal_speed * std::cos(corrected_angle);
    corrected.y() = horizontal_speed * std::sin(corrected_angle);
    return normalize(corrected, &field_direction_world);
}

Eigen::Vector2d selectCommandDirection(
    const Camera& camera,
    const Eigen::Vector2d& previous,
    const Eigen::Vector2d& safe_source,
    const Eigen::Vector2d& goal,
    const std::vector<double>& potential,
    const BinaryMask& blocked_mask,
    const SimConfig& config)
{
    const double max_angle = config.max_direction_rate * config.control_dt;
    Eigen::Vector2d target;
    if (angularDistance(camera, previous, safe_source) > 0.5 * max_angle) {
        target = safe_source;
    } else if (config.continuous_harmonic_guidance) {
        target = continuousHarmonicTarget(
            potential, blocked_mask, camera.width(), camera.height(),
            safe_source, goal, config.harmonic_gradient_radius_cells,
            config.harmonic_gradient_lookahead_cells);
    } else {
        const std::vector<Eigen::Vector2d> path = discreteHarmonicPath(
            potential, blocked_mask, camera.width(), camera.height(),
            safe_source, goal);
        target = path.empty() ? goal : path[std::min<std::size_t>(2, path.size() - 1)];
    }
    Eigen::Vector2d command = angularRateLimit(camera, previous, target, max_angle);
    command.x() = std::max(0.0, std::min(command.x(), camera.width() - 1.0));
    command.y() = std::max(0.0, std::min(command.y(), camera.height() - 1.0));
    return command;
}

double brakingSpeed(
    double distance,
    double reference_speed,
    const SimConfig& config)
{
    const double usable = std::max(0.0, distance - 0.05);
    const double brake_acceleration = config.brake_accel;
    const double safe = -brake_acceleration * config.delay + std::sqrt(
        std::pow(brake_acceleration * config.delay, 2.0) +
        2.0 * brake_acceleration * usable);
    return std::max(0.0, std::min(reference_speed, safe));
}

bool rolloutIsSafe(
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const Eigen::Vector3d& command_world,
    const std::vector<Eigen::Vector3d>& points_world,
    const Eigen::Vector3d& camera_position_world,
    const Eigen::Matrix3d& rotation_camera_from_world,
    const Camera& camera,
    const SimConfig& config)
{
    if (points_world.empty()) {
        return true;
    }
    if (config.rollout_dt <= 0.0 || config.velocity_tau == 0.0) {
        throw std::invalid_argument("invalid rollout time configuration");
    }
    Eigen::Vector3d position = position_world;
    Eigen::Vector3d velocity = velocity_world;
    const double margin_scale = std::min(
        1.0,
        velocity_world.norm() / std::max(kEpsilon, config.reference_speed));
    const double effective_radius = config.body_radius + config.safety_margin +
        margin_scale * config.rollout_margin;
    const int steps = static_cast<int>(
        std::ceil(config.rollout_horizon / config.rollout_dt));
    for (int step = 0; step < steps; ++step) {
        const Eigen::Vector3d acceleration = clampNorm(
            (command_world - velocity) / config.velocity_tau,
            config.max_accel);
        const Eigen::Vector3d next_velocity =
            velocity + config.rollout_dt * acceleration;
        const Eigen::Vector3d next_position =
            position + config.rollout_dt * next_velocity;
        const Eigen::Vector3d segment = next_position - position;
        const double denominator = segment.squaredNorm();
        double nearest_squared = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector3d& point : points_world) {
            Eigen::Vector3d delta;
            if (denominator <= kEpsilon) {
                delta = point - position;
            } else {
                const double projection = std::max(
                    0.0,
                    std::min(1.0, (point - position).dot(segment) / denominator));
                const Eigen::Vector3d closest = position + projection * segment;
                delta = point - closest;
            }
            nearest_squared = std::min(nearest_squared, delta.squaredNorm());
        }
        if (nearest_squared <= effective_radius * effective_radius) {
            return false;
        }
        Eigen::Vector2d pixel;
        const Eigen::Vector3d relative_camera =
            rotation_camera_from_world * (next_position - camera_position_world);
        if (!camera.pixelFromDirection(relative_camera, &pixel)) {
            return false;
        }
        position = next_position;
        velocity = next_velocity;
    }
    return true;
}

double depthRolloutLimit(
    double desired_speed,
    const Eigen::Vector3d& ray_world,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const std::vector<Eigen::Vector3d>& points_world,
    const Eigen::Matrix3d& rotation_camera_from_world,
    const Camera& camera,
    const SimConfig& config)
{
    if (desired_speed <= 1.0e-5) {
        return 0.0;
    }
    Eigen::Vector3d command = desired_speed * ray_world;
    if (rolloutIsSafe(
            position_world, velocity_world, command, points_world,
            position_world, rotation_camera_from_world, camera, config)) {
        return desired_speed;
    }
    double low = 0.0;
    double high = desired_speed;
    for (int iteration = 0; iteration < 10; ++iteration) {
        const double middle = 0.5 * (low + high);
        command = middle * ray_world;
        if (rolloutIsSafe(
                position_world, velocity_world, command, points_world,
                position_world, rotation_camera_from_world, camera, config)) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return low;
}

GuidanceResult computeGuidance(
    const std::vector<double>& depth,
    const Camera& camera,
    const SimConfig& config,
    const Eigen::Vector3d& position_world,
    const Eigen::Vector3d& velocity_world,
    const Eigen::Vector3d& goal_world,
    const Eigen::Vector2d& previous_pixel,
    const Eigen::Vector2d* previous_goal_pixel,
    const Eigen::Matrix3d& rotation_world_from_camera,
    const Eigen::Vector3d* reference_origin_world,
    const Eigen::Vector3d* reference_direction_world,
    BinaryMask* planning_mask_state,
    std::vector<std::uint8_t>* planning_clear_counts,
    int obstacle_clear_frames,
    bool advance_obstacle_frame,
    bool retain_previous_goal,
    const std::vector<double>* initial_potential,
    const std::vector<Eigen::Vector3d>* native_points_camera,
    bool direction_proposal_only,
    const std::function<bool(const Eigen::Vector2d&)>& goal_admissible,
    bool target_proposal_only,
    const Eigen::Vector3d* camera_origin_world)
{
    GuidanceResult result;
    result.solution.depth = depth;
    const Eigen::Matrix3d rotation_camera_from_world =
        rotation_world_from_camera.transpose();
    const std::vector<Eigen::Vector3d> resized_points = native_points_camera?
        std::vector<Eigen::Vector3d>():backprojectObstaclePoints(depth,camera,config.depth_point_stride);
    const auto& sensor_points=native_points_camera?*native_points_camera:resized_points;
    // Collision cones and the goal ray must share the current vehicle center.
    // Keep optical axes, but translate hits from the captured sensor origin;
    // this also accounts for translation since image acquisition. Physical
    // camera frusta remain unchanged in the external motion certificate.
    std::vector<Eigen::Vector3d> centered_points;
    if(camera_origin_world) {
        if(!camera_origin_world->allFinite())
            throw std::invalid_argument("camera origin must be finite");
        const Eigen::Vector3d offset=rotation_camera_from_world*(*camera_origin_world-position_world);
        centered_points.reserve(sensor_points.size());
        for(const auto& point:sensor_points)centered_points.push_back(point+offset);
    }
    const auto& points_camera=camera_origin_world?centered_points:sensor_points;
    const double effective_radius = config.body_radius + config.safety_margin;
    result.solution.free_distance = collisionConeFreeDistance(
        points_camera, camera, effective_radius, config.cone_chunk_size);

    const Eigen::Vector3d goal_delta = goal_world - position_world;
    const double reference_speed = std::min(
        config.reference_speed,
        std::sqrt(std::max(
            0.0, 2.0 * config.brake_accel * goal_delta.norm())));
    const Eigen::Vector3d forward_fallback = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d reference_direction_world_normalized =
        normalize(goal_delta, &forward_fallback);
    const Eigen::Vector3d reference_direction_camera =
        rotation_camera_from_world * reference_direction_world_normalized;
    Eigen::Vector2d reference_pixel;
    if (!camera.pixelFromDirection(reference_direction_camera, &reference_pixel)) {
        if (reference_direction_camera.z() > kEpsilon) {
            reference_pixel = Eigen::Vector2d(
                camera.fx() * reference_direction_camera.x() /
                    reference_direction_camera.z() + camera.cx(),
                camera.fy() * reference_direction_camera.y() /
                    reference_direction_camera.z() + camera.cy());
        } else {
            reference_pixel = Eigen::Vector2d(
                reference_direction_camera.x() >= 0.0
                    ? camera.width() - 2.0
                    : 1.0,
                camera.cy());
        }
        reference_pixel.x() = std::max(
            1.0, std::min(reference_pixel.x(), camera.width() - 2.0));
        reference_pixel.y() = std::max(
            1.0, std::min(reference_pixel.y(), camera.height() - 2.0));
    }
    if (config.horizontal_only) {
        reference_pixel.y() = camera.cy();
    }
    result.solution.reference_pixel = reference_pixel;

    const double stop_distance = reference_speed * config.delay +
        reference_speed * reference_speed / (2.0 * config.brake_accel);
    double planning_distance = std::min(
        camera.maxDepth() - effective_radius,
        reference_speed * config.planning_horizon + stop_distance + 0.35);
    if(config.finite_goal)planning_distance=std::min(planning_distance,goal_delta.norm());
    result.solution.planning_mask.resize(result.solution.free_distance.size(), 0);
    for (std::size_t index = 0;
         index < result.solution.free_distance.size(); ++index) {
        result.solution.planning_mask[index] =
            result.solution.free_distance[index] <= planning_distance ? 1 : 0;
    }
    for (int u = 0; u < camera.width(); ++u) {
        result.solution.planning_mask[gridIndex(u, 0, camera.width())] = 1;
        result.solution.planning_mask[
            gridIndex(u, camera.height() - 1, camera.width())] = 1;
    }
    for (int v = 0; v < camera.height(); ++v) {
        result.solution.planning_mask[gridIndex(0, v, camera.width())] = 1;
        result.solution.planning_mask[
            gridIndex(camera.width() - 1, v, camera.width())] = 1;
    }
    if (planning_mask_state != nullptr || planning_clear_counts != nullptr) {
        if (planning_mask_state == nullptr || planning_clear_counts == nullptr) {
            throw std::invalid_argument(
                "both planning-mask hysteresis states must be provided");
        }
        applyObstacleReleaseHysteresis(
            result.solution.planning_mask, obstacle_clear_frames,
            advance_obstacle_frame, planning_mask_state, planning_clear_counts);
        result.solution.planning_mask = *planning_mask_state;
    }

    Eigen::Vector2d planning_previous_pixel = previous_pixel;
    if (config.horizontal_only) {
        const int horizontal_row = roundedClampedPixel(
            camera.cy(), camera.height());
        planning_previous_pixel.y() = camera.cy();
        for (int v = 0; v < camera.height(); ++v) {
            if (v == horizontal_row) {
                continue;
            }
            for (int u = 0; u < camera.width(); ++u) {
                result.solution.planning_mask[
                    gridIndex(u, v, camera.width())] = 1;
            }
        }
    }

    GoalSelection selection = chooseSafeGoal(
        reference_pixel, planning_previous_pixel, result.solution.planning_mask,
        camera.width(), camera.height(), previous_goal_pixel, config,config.angular_goal_cost?&camera:nullptr);
    if (selection.valid && retain_previous_goal && previous_goal_pixel != nullptr) {
        const int previous_u = roundedClampedPixel(
            previous_goal_pixel->x(), camera.width());
        const int previous_v = roundedClampedPixel(
            previous_goal_pixel->y(), camera.height());
        const int source_u = roundedClampedPixel(
            selection.source.x(), camera.width());
        const int source_v = roundedClampedPixel(
            selection.source.y(), camera.height());
        const std::size_t previous_index = gridIndex(
            previous_u, previous_v, camera.width());
        const std::size_t source_index = gridIndex(
            source_u, source_v, camera.width());
        if (result.solution.planning_mask[previous_index] == 0 &&
            selection.labels[previous_index] > 0 &&
            selection.labels[previous_index] == selection.labels[source_index]) {
            selection.goal = config.subpixel_goal?*previous_goal_pixel:Eigen::Vector2d(previous_u, previous_v);
        }
    }
    // Reclaim the continuous operator direction once its complete angular
    // segment is clear. Goal hysteresis must not prolong an obsolete detour.
    // This only generates a proposal; the paper path still certifies the
    // actual 3-D motion and full braking trajectory after output shaping.
    bool direct_reference = selection.valid && config.recapture_reference;
    if (direct_reference) {
        const Eigen::Vector2d delta = reference_pixel - planning_previous_pixel;
        const int steps = std::max(1, static_cast<int>(std::ceil(delta.norm()*4.)));
        for (int i=0; i<=steps && direct_reference; ++i) {
            const Eigen::Vector2d p = planning_previous_pixel + (double(i)/steps)*delta;
            const int x0=static_cast<int>(std::floor(p.x())), y0=static_cast<int>(std::floor(p.y()));
            for(int y=y0;y<=y0+1;++y)for(int x=x0;x<=x0+1;++x)
                if(x<0||y<0||x>=camera.width()||y>=camera.height()||
                   result.solution.planning_mask[gridIndex(x,y,camera.width())])direct_reference=false;
        }
        if(direct_reference)selection.goal=reference_pixel;
    }
    if(goal_admissible) {
        for(int attempt=0;selection.valid && attempt<12;++attempt) {
            if(goal_admissible(selection.goal))break;
            direct_reference=false;
            const int u=roundedClampedPixel(selection.goal.x(),camera.width());
            const int v=roundedClampedPixel(selection.goal.y(),camera.height());
            for(int y=std::max(0,v-1);y<=std::min(camera.height()-1,v+1);++y)
                for(int x=std::max(0,u-1);x<=std::min(camera.width()-1,u+1);++x)
                    result.solution.planning_mask[gridIndex(x,y,camera.width())]=1;
            selection=chooseSafeGoal(reference_pixel,planning_previous_pixel,result.solution.planning_mask,
                camera.width(),camera.height(),nullptr,config,config.angular_goal_cost?&camera:nullptr);
            if(attempt==11)selection.valid=false;
        }
    }
    // Keep the selected detour side, but remove its unnecessary clearance
    // bias toward the middle of a wide opening. Project toward q only inside
    // the same free component, with a pixel margin and a fresh volume proof.
    if(selection.valid&&config.goal_projection_clearance>0.&&goal_admissible) {
        const auto clearance=euclideanDistanceToBlocked(result.solution.planning_mask,camera.width(),camera.height());
        const Eigen::Vector2d delta=selection.goal-reference_pixel;
        const int steps=std::max(1,static_cast<int>(std::ceil(delta.norm()*4.)));
        const int goal_u=roundedClampedPixel(selection.goal.x(),camera.width());
        const int goal_v=roundedClampedPixel(selection.goal.y(),camera.height());
        const int component=selection.labels[gridIndex(goal_u,goal_v,camera.width())];
        int queries=0;
        for(int i=0;i<steps&&queries<4;++i) {
            const Eigen::Vector2d point=reference_pixel+(double(i)/steps)*delta;
            const int u=roundedClampedPixel(point.x(),camera.width()),v=roundedClampedPixel(point.y(),camera.height());
            bool room=true;
            const int x0=static_cast<int>(std::floor(point.x())),y0=static_cast<int>(std::floor(point.y()));
            for(int y=y0;room&&y<=y0+1;++y)for(int x=x0;room&&x<=x0+1;++x)
                room=x>=0&&y>=0&&x<camera.width()&&y<camera.height()&&
                    !result.solution.planning_mask[gridIndex(x,y,camera.width())]&&
                    clearance[gridIndex(x,y,camera.width())]>=config.goal_projection_clearance;
            if(room&&selection.labels[gridIndex(u,v,camera.width())]==component) {
                ++queries;if(goal_admissible(point)){selection.goal=point;break;}
            }
        }
    }
    result.goal_reanchored = selection.valid &&
        (previous_goal_pixel == nullptr ||
         angularDistance(camera, selection.goal, *previous_goal_pixel) >
            0.5 * kPi / 180.0);
    result.goal_valid=selection.valid;
    if(target_proposal_only) {
        result.solution.source_pixel=selection.valid?selection.source:planning_previous_pixel;
        result.solution.goal_pixel=selection.valid?selection.goal:planning_previous_pixel;
        result.solution.command_pixel=result.solution.goal_pixel;
        result.solution.potential.assign(result.solution.planning_mask.size(),std::numeric_limits<double>::quiet_NaN());
        result.returned_goal_pixel=result.solution.goal_pixel;
        if(selection.valid)result.command_world=reference_speed*rotation_world_from_camera*
            camera.rayFromPixel(selection.goal);
        // No harmonic field was computed: goal validity and field validity are
        // deliberately separate. The returned vector is not motion-authorized.
        return result;
    }
    Eigen::Vector2d command_pixel;
    if (!selection.valid) {
        result.solution.source_pixel = planning_previous_pixel;
        result.solution.goal_pixel = planning_previous_pixel;
        result.solution.potential.assign(
            result.solution.planning_mask.size(),
            std::numeric_limits<double>::quiet_NaN());
        result.solution.field_valid = false;
        command_pixel = planning_previous_pixel;
    } else {
        result.solution.source_pixel = selection.source;
        result.solution.goal_pixel = selection.goal;
        if (direct_reference || angularDistance(camera, selection.source, selection.goal) <
            0.4 * kPi / 180.0) {
            result.solution.potential.assign(
                result.solution.planning_mask.size(),
                std::numeric_limits<double>::quiet_NaN());
            for (std::size_t index = 0;
                 index < result.solution.planning_mask.size(); ++index) {
                if (result.solution.planning_mask[index] == 0) {
                    result.solution.potential[index] = 0.0;
                }
            }
            result.solution.field_valid = true;
            command_pixel = angularRateLimit(
                camera, planning_previous_pixel, selection.goal,
                config.max_direction_rate * config.control_dt);
        } else {
            const HarmonicSolution harmonic = solveAngularHarmonic(
                result.solution.planning_mask, camera.width(), camera.height(),
                selection.source, selection.goal, config, initial_potential);
            result.solution.potential = harmonic.potential;
            result.solution.field_valid = harmonic.valid;
            if (harmonic.valid) {
                command_pixel = selectCommandDirection(
                    camera, planning_previous_pixel, selection.source, selection.goal,
                    harmonic.potential, result.solution.planning_mask, config);
            } else {
                command_pixel = angularRateLimit(
                    camera, planning_previous_pixel, selection.goal,
                    config.max_direction_rate * config.control_dt);
            }
        }
    }

    if (config.horizontal_only) {
        command_pixel.y() = camera.cy();
    }

    Eigen::Vector3d ray_camera = camera.rayFromPixel(command_pixel);
    Eigen::Vector3d ray_world = rotation_world_from_camera * ray_camera;
    if (reference_origin_world != nullptr &&
        reference_direction_world != nullptr) {
        double observed_clearance = camera.maxDepth();
        for (const Eigen::Vector3d& point : points_camera) {
            observed_clearance = std::min(observed_clearance, point.norm());
        }
        ray_world = referenceConvergedDirection(
            ray_world, position_world, *reference_origin_world,
            *reference_direction_world, observed_clearance, config);
        ray_camera = rotation_camera_from_world * ray_world;
        Eigen::Vector2d corrected_pixel;
        if (camera.pixelFromDirection(ray_camera, &corrected_pixel)) {
            command_pixel = corrected_pixel;
        } else if (ray_camera.z() > kEpsilon) {
            command_pixel = Eigen::Vector2d(
                camera.fx() * ray_camera.x() / ray_camera.z() + camera.cx(),
                camera.fy() * ray_camera.y() / ray_camera.z() + camera.cy());
            command_pixel.x() = std::max(
                1.0, std::min(command_pixel.x(), camera.width() - 2.0));
            command_pixel.y() = std::max(
                1.0, std::min(command_pixel.y(), camera.height() - 2.0));
        }
        ray_camera = camera.rayFromPixel(command_pixel);
        ray_world = rotation_world_from_camera * ray_camera;
    }
    if (config.horizontal_only) {
        command_pixel.y() = camera.cy();
        ray_camera = camera.rayFromPixel(command_pixel);
        ray_world = rotation_world_from_camera * ray_camera;
        ray_world.z() = 0.0;
        ray_world = normalize(ray_world);
    }
    result.solution.command_pixel = command_pixel;

    if(direction_proposal_only) {
        result.command_world=reference_speed*ray_world;
        result.returned_goal_pixel=result.solution.goal_pixel;
        return result;
    }
    const double free_distance = bilinearSample(
        result.solution.free_distance, camera.width(), camera.height(),
        command_pixel, 0.0);
    const double braking_limited_speed = brakingSpeed(
        free_distance, reference_speed, config);
    double speed = braking_limited_speed;
    std::vector<Eigen::Vector3d> points_world;
    points_world.reserve(points_camera.size());
    for (const Eigen::Vector3d& point_camera : points_camera) {
        points_world.push_back(
            position_world + rotation_world_from_camera * point_camera);
    }
    speed = depthRolloutLimit(
        speed, ray_world, position_world, velocity_world, points_world,
        rotation_camera_from_world, camera, config);
    result.selected_free_distance = free_distance;
    result.braking_limited_speed = braking_limited_speed;
    result.rollout_limited_speed = speed;
    result.command_world = speed * ray_world;
    result.safety_limited = speed + 1.0e-4 < reference_speed;
    result.emergency_stop =
        braking_limited_speed <= 1.0e-5 && free_distance <= 0.051;
    result.returned_goal_pixel = result.solution.goal_pixel;
    return result;
}

}  // namespace depth_angular
}  // namespace pc_gvf
