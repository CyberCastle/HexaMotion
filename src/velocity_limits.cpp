/**
 * @file velocity_limits.cpp
 * @brief Migrated velocity limits using WorkspaceValidator
 *
 * This is the migrated version that delegates all workspace-related calculations
 * to WorkspaceValidator for consistency and reduced code duplication.
 */

#include "velocity_limits.h"
#include "gait_config.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "workspace_analyzer.h"
#include <algorithm>
#include <cmath>
#include <memory>

class VelocityLimits::Impl {
  public:
    std::unique_ptr<WorkspaceAnalyzer> workspace_analyzer_;
    const RobotModel &model_;
    LimitMap limit_map_;
    std::array<double, 360> angular_accel_map_{}; // Separate angular acceleration per bearing
    WorkspaceConfig workspace_config_;
    GaitConfig current_gait_config_;
    double angular_velocity_scaling_;

    explicit Impl(const RobotModel &model)
        : model_(model), angular_velocity_scaling_(DEFAULT_ANGULAR_SCALING) {
        // Initialize WorkspaceAnalyzer for all workspace calculations
        ValidationConfig config;
        config.enable_collision_checking = true;
        config.enable_joint_limit_checking = true;
        config.safety_margin = 30.0;

        workspace_analyzer_ = std::make_unique<WorkspaceAnalyzer>(model, ComputeConfig::medium(), config);

        // Initialize workspace config with physical robot reference height
        // When all servo angles are 0°, robot body is positioned at getDefaultHeightOffset()
        workspace_config_.reference_height = model.getDefaultHeightOffset();

        // Initialize with default gait configuration
        current_gait_config_ = GaitConfig();
    }
};

VelocityLimits::VelocityLimits(const RobotModel &model)
    : pimpl_(std::make_unique<Impl>(model)) {
    // Initialize limits with default gait configuration
    generateLimits(pimpl_->current_gait_config_);
}

VelocityLimits::~VelocityLimits() = default;

void VelocityLimits::generateLimits(const GaitConfig &gait_config) {
    pimpl_->current_gait_config_ = gait_config;

    // Use WorkspaceValidator instead of custom workspace calculation
    calculateWorkspace(gait_config);

    // Calculate overshoot compensation using workspace data
    calculateOvershoot(gait_config);

    // For each direction (bearing), calculate velocity and acceleration limits using validator constraints
    for (int bearing = 0; bearing < 360; ++bearing) {
        pimpl_->limit_map_.limits[bearing] = calculateLimitsForBearing(static_cast<double>(bearing), gait_config);
        pimpl_->angular_accel_map_[bearing] = pimpl_->limit_map_.limits[bearing].angular_accel;
    }
}

VelocityLimits::LimitValues VelocityLimits::getLimit(double bearing_degrees) const {
    // Normalize bearing to 0-359 range
    double normalized_bearing = normalizeBearing(bearing_degrees);

    // Use interpolation for smooth transitions between discrete bearing values
    return interpolateLimits(normalized_bearing);
}

// Unified configuration interface overloads
void VelocityLimits::generateLimits(const GaitConfiguration &gait_config) {
    // Convert unified configuration to internal format
    GaitConfig internal_config;
    internal_config.frequency = gait_config.getStepFrequency();
    internal_config.stance_ratio = gait_config.getStanceRatio();
    internal_config.swing_ratio = gait_config.getSwingRatio();
    internal_config.time_to_max_stride = gait_config.time_to_max_stride;
    internal_config.step_length = gait_config.step_length;
    internal_config.max_velocity = gait_config.max_velocity;

    // Use existing implementation
    generateLimits(internal_config);
}

void VelocityLimits::calculateWorkspace(const GaitConfig &gait_config) {
    // Replace complex workspace calculation with WorkspaceValidator

    // Get workspace bounds for all legs
    double min_walkspace_radius = 1000.0; // Start with large value
    double min_stance_radius = 1000.0;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        auto bounds = pimpl_->workspace_analyzer_->getWorkspaceBounds(leg);

        // Use the most restrictive values across all legs
        min_walkspace_radius = std::min(min_walkspace_radius, bounds.max_reach);
        min_stance_radius = std::min(min_stance_radius, bounds.max_reach * 0.8);
    }

    // Apply safety scaling
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();

    pimpl_->workspace_config_.walkspace_radius = min_walkspace_radius * scaling_factors.workspace_scale;
    pimpl_->workspace_config_.stance_radius = min_stance_radius * scaling_factors.workspace_scale;
    pimpl_->workspace_config_.safety_margin = scaling_factors.safety_margin;

    // Initialize scaled radius equal to base radius (will deduct overshoot later if compat mode)
    pimpl_->workspace_config_.scaled_walkspace_radius = pimpl_->workspace_config_.walkspace_radius;

    // Ensure minimum reasonable values using constants
    pimpl_->workspace_config_.walkspace_radius =
        std::max(pimpl_->workspace_config_.walkspace_radius, 0.05);
    pimpl_->workspace_config_.stance_radius =
        std::max(pimpl_->workspace_config_.stance_radius, 0.03);
}

void VelocityLimits::calculateWorkspace(const GaitConfiguration &gait_config) {
    // Convert unified configuration to internal format
    GaitConfig internal_config;
    internal_config.frequency = gait_config.getStepFrequency();
    internal_config.stance_ratio = gait_config.getStanceRatio();
    internal_config.swing_ratio = gait_config.getSwingRatio();
    internal_config.time_to_max_stride = gait_config.time_to_max_stride;

    // Use existing implementation
    calculateWorkspace(internal_config);
}

VelocityLimits::LimitValues VelocityLimits::scaleVelocityLimits(
    const LimitValues &input_velocities, double angular_velocity_percentage) const {

    // - If there's no angular demand (percentage <= 0) return original limits (no artificial reduction)
    // - Preserve angular_z limit (do NOT scale it down to zero when no demand)
    // - Apply only kinematic coupling: v_planar <= omega_demand * stance_radius
    // - Maintain direction of linear components while scaling magnitude if needed

    if (angular_velocity_percentage <= 0.0) {
        return input_velocities; // No coupling needed
    }

    LimitValues scaled = input_velocities; // Start from base (keep angular_z unchanged)

    double stance_radius = std::max(1.0, pimpl_->workspace_config_.stance_radius);
    // Requested angular velocity (rad/s)
    double requested_w = angular_velocity_percentage * input_velocities.angular_z;
    double kinematic_cap = std::abs(requested_w) * stance_radius; // mm/s

    // Current planar magnitude of allowable linear limits
    double linear_mag = std::hypot(scaled.linear_x, scaled.linear_y);
    if (linear_mag > 1e-9 && kinematic_cap > 0.0 && linear_mag > kinematic_cap) {
        double reduction = kinematic_cap / linear_mag;
        // Keep at least a modest fraction (avoid total stall unless kinematic_cap==0)
        reduction = math_utils::clamp<double>(reduction, 0.0, 1.0);
        scaled.linear_x *= reduction;
        scaled.linear_y *= reduction;
    }

    return scaled;
}

bool VelocityLimits::validateVelocityInputs(double vx, double vy, double omega) const {
    // Global bypass (debug / external safety control)
    if (!pimpl_->model_.getParams().enable_velocity_limits) {
        return true;
    }
    double bearing = calculateBearing(vx, vy);
    LimitValues limits = getLimit(bearing);

    // Check if velocities are within calculated limits
    return (std::abs(vx) <= limits.linear_x &&
            std::abs(vy) <= limits.linear_y &&
            std::abs(omega) <= limits.angular_z);
}

VelocityLimits::LimitValues VelocityLimits::interpolateLimits(double bearing_degrees) const {
    int index1 = getBearingIndex(bearing_degrees);
    int index2 = (index1 + 1) % 360;

    double t = bearing_degrees - static_cast<double>(index1);

    const LimitValues &limits1 = pimpl_->limit_map_.limits[index1];
    const LimitValues &limits2 = pimpl_->limit_map_.limits[index2];

    LimitValues interpolated;
    interpolated.linear_x = interpolateValue(limits1.linear_x, limits2.linear_x, t);
    interpolated.linear_y = interpolateValue(limits1.linear_y, limits2.linear_y, t);
    interpolated.angular_z = interpolateValue(limits1.angular_z, limits2.angular_z, t);
    interpolated.acceleration = interpolateValue(limits1.acceleration, limits2.acceleration, t);
    interpolated.angular_accel = interpolateValue(limits1.angular_accel, limits2.angular_accel, t);

    return interpolated;
}

VelocityLimits::LimitValues VelocityLimits::applyAccelerationLimits(
    const LimitValues &target_velocities,
    const LimitValues &current_velocities,
    double dt) const {

    LimitValues limited_velocities = target_velocities;

    // Calculate required accelerations
    double accel_x = (target_velocities.linear_x - current_velocities.linear_x) / dt;
    double accel_y = (target_velocities.linear_y - current_velocities.linear_y) / dt;
    double accel_z = (target_velocities.angular_z - current_velocities.angular_z) / dt;

    // Apply acceleration limits using constraints
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    double max_accel = target_velocities.acceleration * scaling_factors.acceleration_scale;

    if (std::abs(accel_x) > max_accel) {
        limited_velocities.linear_x = current_velocities.linear_x +
                                      (accel_x > 0 ? max_accel : -max_accel) * dt;
    }

    if (std::abs(accel_y) > max_accel) {
        limited_velocities.linear_y = current_velocities.linear_y +
                                      (accel_y > 0 ? max_accel : -max_accel) * dt;
    }

    if (std::abs(accel_z) > max_accel) {
        limited_velocities.angular_z = current_velocities.angular_z +
                                       (accel_z > 0 ? max_accel : -max_accel) * dt;
    }

    return limited_velocities;
}

void VelocityLimits::calculateOvershoot(const GaitConfig &gait_config) {

    // Physical basis: if accelerating from rest to v_max with constant a_max, distance = 0.5 * (v_max^2 / a_max)
    // Additionally, if ramp time (t_ramp = time_to_max_stride) is specified,
    // theoretical distance under constant accel is 0.5 * a * t_ramp^2.
    // We'll compute both and take the minimum (more conservative), then cap to a fraction of walkspace radius.

    auto constraints = pimpl_->workspace_analyzer_->calculateVelocityConstraints(0, 0.0); // forward
    double v_max = constraints.max_linear_velocity;
    double a_max = std::max(1e-6, constraints.max_acceleration); // avoid div by zero
    double t_ramp = std::max(0.0, gait_config.time_to_max_stride);

    double dist_v2_over_a = 0.5 * (v_max * v_max) / a_max;  // mm
    double dist_time_based = 0.5 * a_max * t_ramp * t_ramp; // mm
    double raw_overshoot = std::min(dist_v2_over_a, dist_time_based);

    // Cap overshoot to a fraction of available walkspace radius (prevents unrealistic large values)
    double walk_r = std::max(1.0, pimpl_->workspace_config_.walkspace_radius);
    double max_allowable = 0.25 * walk_r; // at most 25% of effective radius
    raw_overshoot = std::min(raw_overshoot, max_allowable);

    // Apply global scaling & safety margin (kept moderate)
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    double safety = scaling_factors.safety_margin; // typically <=1
    raw_overshoot *= safety;

    pimpl_->workspace_config_.overshoot_x = raw_overshoot;
    pimpl_->workspace_config_.overshoot_y = raw_overshoot;

    // Scaled walkspace radius now mirrors base; prior compatibility mode (diameter traversal) removed.
    pimpl_->workspace_config_.scaled_walkspace_radius = pimpl_->workspace_config_.walkspace_radius;
}

void VelocityLimits::updateGaitParameters(const GaitConfig &gait_config) {
    generateLimits(gait_config);
}

void VelocityLimits::updateGaitParameters(const GaitConfiguration &gait_config) {
    // Convert unified configuration to internal format
    GaitConfig internal_config;
    internal_config.frequency = gait_config.getStepFrequency();
    internal_config.stance_ratio = gait_config.getStanceRatio();
    internal_config.swing_ratio = gait_config.getSwingRatio();
    internal_config.time_to_max_stride = gait_config.time_to_max_stride;

    // Use existing implementation
    generateLimits(internal_config);
}

const VelocityLimits::WorkspaceConfig &VelocityLimits::getWorkspaceConfig() const {
    return pimpl_->workspace_config_;
}

VelocityLimits::LimitValues VelocityLimits::getMaxLimits() const {
    LimitValues max_limits;

    for (const auto &limits : pimpl_->limit_map_.limits) {
        max_limits.linear_x = std::max(max_limits.linear_x, limits.linear_x);
        max_limits.linear_y = std::max(max_limits.linear_y, limits.linear_y);
        max_limits.angular_z = std::max(max_limits.angular_z, limits.angular_z);
        max_limits.acceleration = std::max(max_limits.acceleration, limits.acceleration);
    }

    return max_limits;
}

VelocityLimits::LimitValues VelocityLimits::getMinLimits() const {
    LimitValues min_limits(1e6f, 1e6f, 1e6f, 1e6f); // Initialize to large values

    for (const auto &limits : pimpl_->limit_map_.limits) {
        min_limits.linear_x = std::min(min_limits.linear_x, limits.linear_x);
        min_limits.linear_y = std::min(min_limits.linear_y, limits.linear_y);
        min_limits.angular_z = std::min(min_limits.angular_z, limits.angular_z);
        min_limits.acceleration = std::min(min_limits.acceleration, limits.acceleration);
    }

    return min_limits;
}

std::vector<VelocityLimits::LimitValues> VelocityLimits::getAllLimits() const {
    std::vector<LimitValues> all_limits;
    all_limits.reserve(360);

    for (const auto &limits : pimpl_->limit_map_.limits) {
        all_limits.push_back(limits);
    }

    return all_limits;
}

double VelocityLimits::normalizeBearing(double bearing_degrees) {
    // Normalize bearing to 0-359.999 range
    bearing_degrees = std::fmod(bearing_degrees, 360.0);
    if (bearing_degrees < 0.0) {
        bearing_degrees += 360.0;
    }
    return bearing_degrees;
}

double VelocityLimits::calculateBearing(double vx, double vy) {
    // Calculate bearing from velocity components
    if (std::abs(vx) < 1e-10f && std::abs(vy) < 1e-10f) {
        return 0.0; // Default bearing for zero velocity
    }

    double bearing_rad = std::atan2(vy, vx);
    double bearing_deg = math_utils::radiansToDegrees(bearing_rad);
    return normalizeBearing(bearing_deg);
}

double VelocityLimits::calculateMaxLinearSpeed(double walkspace_radius, const GaitConfig &gait_config) const {
    // Updated stride-based formula (v2):
    //  1. Use gait_config.step_length directly. This value is already derived from the
    //     standing horizontal reach via GAIT_*_LENGTH_FACTOR in gait_config_factory.cpp.
    //  2. Deduct overshoot (2 * avg_overshoot) to ensure the executed stride comfortably fits
    //     inside the available walkspace radius across accelerate/decelerate phases.
    //  3. Convert to speed: v = effective_stride_length * step_frequency.
    //  4. Apply global velocity scaling (workspace analyzer).
    //  5. Cap by BOTH global model params.max_velocity and gait_config.max_velocity (if >0) and a
    //     hard engineering ceiling to prevent runaway values in misconfiguration.
    //  6. Guard: if frequency <= 0 or step_length <= 0 return 0.
    (void)walkspace_radius; // Retained in signature for future adaptive scaling (currently unused directly).
    if (gait_config.frequency <= 0.0 || gait_config.step_length <= 0.0)
        return 0.0;

    double stride_length = gait_config.step_length;
    // Overshoot deduction (2x average overshoot) maintains conservative effective stride.
    double avg_overshoot = (pimpl_->workspace_config_.overshoot_x + pimpl_->workspace_config_.overshoot_y) * 0.5;
    stride_length = std::max(0.0, stride_length - 2.0 * avg_overshoot);
    double max_speed = stride_length * gait_config.frequency;
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    max_speed *= scaling_factors.velocity_scale;

    const auto &params = pimpl_->model_.getParams();
    double global_cap = params.max_velocity > 0.0 ? params.max_velocity : DEFAULT_MAX_LINEAR_VELOCITY;
    double gait_cap = (gait_config.max_velocity > 0.0) ? gait_config.max_velocity : global_cap;
    double configured_cap = std::min(global_cap, gait_cap);

    return std::min({max_speed, configured_cap, 5000.0});
}

double VelocityLimits::calculateMaxAngularSpeed(double max_linear_speed, double stance_radius) const {
    // Use WorkspaceValidator angular scaling
    if (stance_radius <= 0.0 || max_linear_speed <= 0.0) {
        return 0.0;
    }

    double max_angular = max_linear_speed / stance_radius; // rad/s (linear mm/s divided by mm)

    // Apply angular scaling
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    max_angular *= scaling_factors.angular_scale;

    // Apply reasonable limits to prevent extreme values
    const auto &params = pimpl_->model_.getParams();

    // params.max_angular_velocity is assumed in degrees/s per constants; convert if >0
    double configured_cap_rad = (params.max_angular_velocity > 0.0)
                                    ? params.max_angular_velocity * DEGREES_TO_RADIANS_FACTOR
                                    : (DEFAULT_MAX_ANGULAR_VELOCITY * DEGREES_TO_RADIANS_FACTOR);
    return std::min({max_angular, configured_cap_rad, 10.0});
}

double VelocityLimits::calculateMaxAcceleration(double max_speed, double time_to_max) const {
    // Use WorkspaceValidator acceleration constraints
    if (time_to_max <= 0.0 || max_speed <= 0.0) {
        return 0.0;
    }

    double max_accel = max_speed / time_to_max;

    // Apply acceleration scaling
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    max_accel *= scaling_factors.acceleration_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_accel, 10000.0); // Cap retains mm/s²
}

VelocityLimits::LimitValues VelocityLimits::calculateLimitsForBearing(
    double bearing_degrees, const GaitConfig &gait_config) const {

    // Use WorkspaceValidator instead of complex leg analysis

    // Find the most constraining leg using validation
    double min_effective_radius = pimpl_->workspace_config_.walkspace_radius;
    VelocityConstraints most_restrictive;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        auto constraints = pimpl_->workspace_analyzer_->calculateVelocityConstraints(leg, bearing_degrees);

        // Use the most restrictive constraints
        if (leg == 0 || constraints.workspace_radius < min_effective_radius) {
            min_effective_radius = constraints.workspace_radius;
            most_restrictive = constraints;
        }
    }

    // Calculate limits based on the most constraining constraints
    double max_linear_speed = calculateMaxLinearSpeed(min_effective_radius, gait_config);

    double max_angular_speed = calculateMaxAngularSpeed(max_linear_speed,
                                                        pimpl_->workspace_config_.stance_radius);

    double max_acceleration = calculateMaxAcceleration(max_linear_speed,
                                                       gait_config.time_to_max_stride);

    // Create limit values using constraints
    LimitValues limits;
    limits.linear_x = std::min(max_linear_speed, most_restrictive.max_linear_velocity);
    limits.linear_y = std::min(max_linear_speed, most_restrictive.max_linear_velocity);
    limits.angular_z = std::min(max_angular_speed, most_restrictive.max_angular_velocity);
    limits.acceleration = std::min(max_acceleration, most_restrictive.max_acceleration);

    // Estimate angular acceleration: reach max_angular_speed in same time_to_max_stride
    if (gait_config.time_to_max_stride > 0.0) {
        limits.angular_accel = limits.angular_z / gait_config.time_to_max_stride;
    } else {
        limits.angular_accel = 0.0;
    }

    // Ensure stance radius never exceeds walkspace (sanity) and is morphologically plausible
    if (pimpl_->workspace_config_.stance_radius > pimpl_->workspace_config_.walkspace_radius) {
        pimpl_->workspace_config_.stance_radius = pimpl_->workspace_config_.walkspace_radius;
    }

    // Final guard: linear limit should not exceed circumference constraint for instantaneous rotation
    if (limits.angular_z > 0.0) {
        double kinematic_cap = limits.angular_z * pimpl_->workspace_config_.stance_radius;
        limits.linear_x = std::min(limits.linear_x, kinematic_cap);
        limits.linear_y = std::min(limits.linear_y, kinematic_cap);
    }

    return limits;
}

double VelocityLimits::interpolateValue(double value1, double value2, double factor) const {
    return value1 + (value2 - value1) * factor;
}

int VelocityLimits::getBearingIndex(double bearing_degrees) const {
    return static_cast<int>(std::floor(bearing_degrees)) % 360;
}

void VelocityLimits::setSafetyMargin(double margin) {
    pimpl_->workspace_config_.safety_margin = margin;

    // Update validator safety margin
    pimpl_->workspace_analyzer_->updateSafetyMargin(margin);
}

void VelocityLimits::setAngularVelocityScaling(double scaling) {
    pimpl_->angular_velocity_scaling_ = math_utils::clamp<double>(scaling, 0.1, 2.0);
}

double VelocityLimits::getOvershootX() const {
    return pimpl_->workspace_config_.overshoot_x;
}

double VelocityLimits::getOvershootY() const {
    return pimpl_->workspace_config_.overshoot_y;
}

double VelocityLimits::getPhysicalReferenceHeight() const {
    return pimpl_->workspace_config_.reference_height;
}

double VelocityLimits::getAngularAcceleration(double bearing_degrees) const {
    double b = normalizeBearing(bearing_degrees);
    int i1 = getBearingIndex(b);
    int i2 = (i1 + 1) % 360;
    double t = b - static_cast<double>(i1);
    double a1 = pimpl_->angular_accel_map_[i1];
    double a2 = pimpl_->angular_accel_map_[i2];
    return interpolateValue(a1, a2, t);
}

std::array<double, 360> VelocityLimits::getAngularAccelerationMap() const {
    return pimpl_->angular_accel_map_;
}
