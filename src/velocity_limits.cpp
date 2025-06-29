/**
 * @file velocity_limits.cpp
 * @brief Migrated velocity limits using WorkspaceValidator
 *
 * This is the migrated version that delegates all workspace-related calculations
 * to WorkspaceValidator for consistency and reduced code duplication.
 */

#include "velocity_limits.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "workspace_validator.h"
#include <algorithm>
#include <cmath>
#include <memory>

class VelocityLimits::Impl {
  public:
    std::unique_ptr<WorkspaceValidator> workspace_validator_;
    const RobotModel &model_;
    LimitMap limit_map_;
    WorkspaceConfig workspace_config_;
    GaitConfig current_gait_config_;
    float angular_velocity_scaling_;

    explicit Impl(const RobotModel &model)
        : model_(model), angular_velocity_scaling_(DEFAULT_ANGULAR_SCALING) {
        // Initialize WorkspaceValidator for all workspace calculations
        ValidationConfig config;
        config.enable_collision_checking = true;
        config.enable_joint_limit_checking = true;
        config.safety_margin = 30.0f;

        workspace_validator_ = std::make_unique<WorkspaceValidator>(model, config);

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

    // Generate limits for all bearings (0-359 degrees) using validation
    for (int bearing = 0; bearing < 360; ++bearing) {
        pimpl_->limit_map_.limits[bearing] = calculateLimitsForBearing(
            static_cast<float>(bearing), gait_config);
    }
}

VelocityLimits::LimitValues VelocityLimits::getLimit(float bearing_degrees) const {
    // Normalize bearing to 0-359 range
    float normalized_bearing = normalizeBearing(bearing_degrees);

    // Use interpolation for smooth transitions between discrete bearing values
    return interpolateLimits(normalized_bearing);
}

void VelocityLimits::calculateWorkspace(const GaitConfig &gait_config) {
    // Replace complex workspace calculation with WorkspaceValidator

    // Get workspace bounds for all legs
    float min_walkspace_radius = 1000.0f; // Start with large value
    float min_stance_radius = 1000.0f;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        auto bounds = pimpl_->workspace_validator_->getWorkspaceBounds(leg);

        // Use the most restrictive values across all legs
        min_walkspace_radius = std::min(min_walkspace_radius, bounds.max_radius);
        min_stance_radius = std::min(min_stance_radius, bounds.max_radius * 0.8f);
    }

    // Apply safety scaling
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();

    pimpl_->workspace_config_.walkspace_radius = min_walkspace_radius * scaling_factors.workspace_scale;
    pimpl_->workspace_config_.stance_radius = min_stance_radius * scaling_factors.workspace_scale;
    pimpl_->workspace_config_.safety_margin = scaling_factors.safety_margin;

    // Ensure minimum reasonable values using constants
    pimpl_->workspace_config_.walkspace_radius = std::max(pimpl_->workspace_config_.walkspace_radius, 0.05f);
    pimpl_->workspace_config_.stance_radius = std::max(pimpl_->workspace_config_.stance_radius, 0.03f);
}

VelocityLimits::LimitValues VelocityLimits::scaleVelocityLimits(
    const LimitValues &input_velocities, float angular_velocity_percentage) const {

    LimitValues scaled_limits = input_velocities;

    // Apply angular velocity scaling using scaling factors
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    float angular_scale = angular_velocity_percentage * scaling_factors.angular_scale;
    scaled_limits.angular_z *= angular_scale;

    // Scale linear velocities based on angular velocity demand
    // High angular velocities reduce available linear velocity
    float linear_scale = 1.0f - (std::abs(angular_scale) * 0.3f); // 30% coupling factor
    linear_scale = std::max(0.1f, linear_scale);                  // Minimum 10% linear velocity

    scaled_limits.linear_x *= linear_scale;
    scaled_limits.linear_y *= linear_scale;

    return scaled_limits;
}

bool VelocityLimits::validateVelocityInputs(float vx, float vy, float omega) const {
    float bearing = calculateBearing(vx, vy);
    LimitValues limits = getLimit(bearing);

    // Check if velocities are within calculated limits
    return (std::abs(vx) <= limits.linear_x &&
            std::abs(vy) <= limits.linear_y &&
            std::abs(omega) <= limits.angular_z);
}

VelocityLimits::LimitValues VelocityLimits::interpolateLimits(float bearing_degrees) const {
    int index1 = getBearingIndex(bearing_degrees);
    int index2 = (index1 + 1) % 360;

    float t = bearing_degrees - static_cast<float>(index1);

    const LimitValues &limits1 = pimpl_->limit_map_.limits[index1];
    const LimitValues &limits2 = pimpl_->limit_map_.limits[index2];

    LimitValues interpolated;
    interpolated.linear_x = interpolateValue(limits1.linear_x, limits2.linear_x, t);
    interpolated.linear_y = interpolateValue(limits1.linear_y, limits2.linear_y, t);
    interpolated.angular_z = interpolateValue(limits1.angular_z, limits2.angular_z, t);
    interpolated.acceleration = interpolateValue(limits1.acceleration, limits2.acceleration, t);

    return interpolated;
}

VelocityLimits::LimitValues VelocityLimits::applyAccelerationLimits(
    const LimitValues &target_velocities,
    const LimitValues &current_velocities,
    float dt) const {

    LimitValues limited_velocities = target_velocities;

    // Calculate required accelerations
    float accel_x = (target_velocities.linear_x - current_velocities.linear_x) / dt;
    float accel_y = (target_velocities.linear_y - current_velocities.linear_y) / dt;
    float accel_z = (target_velocities.angular_z - current_velocities.angular_z) / dt;

    // Apply acceleration limits using constraints
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    float max_accel = target_velocities.acceleration * scaling_factors.acceleration_scale;

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
    // Use velocity constraints instead of custom calculation

    // Get velocity constraints from validator for forward direction (0 degrees)
    auto constraints = pimpl_->workspace_validator_->calculateVelocityConstraints(0, 0.0f);

    float max_speed = constraints.max_linear_velocity;
    float max_acceleration = constraints.max_acceleration;

    // Overshoot distance during acceleration phase
    float accel_time = gait_config.time_to_max_stride;
    pimpl_->workspace_config_.overshoot_x = 0.5f * max_acceleration * accel_time * accel_time;
    pimpl_->workspace_config_.overshoot_y = pimpl_->workspace_config_.overshoot_x; // Symmetric for now

    // Apply safety margin
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    pimpl_->workspace_config_.overshoot_x *= scaling_factors.safety_margin;
    pimpl_->workspace_config_.overshoot_y *= scaling_factors.safety_margin;
}

void VelocityLimits::updateGaitParameters(const GaitConfig &gait_config) {
    generateLimits(gait_config);
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

float VelocityLimits::normalizeBearing(float bearing_degrees) {
    // Normalize bearing to 0-359.999 range
    bearing_degrees = std::fmod(bearing_degrees, 360.0f);
    if (bearing_degrees < 0.0f) {
        bearing_degrees += 360.0f;
    }
    return bearing_degrees;
}

float VelocityLimits::calculateBearing(float vx, float vy) {
    // Calculate bearing from velocity components
    if (std::abs(vx) < 1e-10f && std::abs(vy) < 1e-10f) {
        return 0.0f; // Default bearing for zero velocity
    }

    float bearing_rad = std::atan2(vy, vx);
    float bearing_deg = math_utils::radiansToDegrees(bearing_rad);
    return normalizeBearing(bearing_deg);
}

float VelocityLimits::calculateMaxLinearSpeed(float walkspace_radius,
                                              float on_ground_ratio, float frequency) const {
    // Use simplified OpenSHC-equivalent calculation with constraints
    if (on_ground_ratio <= 0.0f || frequency <= 0.0f || walkspace_radius <= 0.0f) {
        return 0.0f;
    }

    // Ensure reasonable bounds to prevent numerical issues
    float cycle_time = on_ground_ratio / frequency;
    if (cycle_time <= 0.0f) {
        return 0.0f;
    }

    float max_speed = (walkspace_radius * 2.0f) / cycle_time;

    // Apply safety limits
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    max_speed *= scaling_factors.velocity_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_speed, 5.0f); // Cap at 5 m/s for safety
}

float VelocityLimits::calculateMaxAngularSpeed(float max_linear_speed, float stance_radius) const {
    // Use WorkspaceValidator angular scaling
    if (stance_radius <= 0.0f || max_linear_speed <= 0.0f) {
        return 0.0f;
    }

    float max_angular = max_linear_speed / stance_radius;

    // Apply angular scaling
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    max_angular *= scaling_factors.angular_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_angular, 10.0f); // Cap at 10 rad/s for safety
}

float VelocityLimits::calculateMaxAcceleration(float max_speed, float time_to_max) const {
    // Use WorkspaceValidator acceleration constraints
    if (time_to_max <= 0.0f || max_speed <= 0.0f) {
        return 0.0f;
    }

    float max_accel = max_speed / time_to_max;

    // Apply acceleration scaling
    auto scaling_factors = pimpl_->workspace_validator_->getScalingFactors();
    max_accel *= scaling_factors.acceleration_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_accel, 10.0f); // Cap at 10 m/s² for safety
}

VelocityLimits::LimitValues VelocityLimits::calculateLimitsForBearing(
    float bearing_degrees, const GaitConfig &gait_config) const {

    // Use WorkspaceValidator instead of complex leg analysis

    // Find the most constraining leg using validation
    float min_effective_radius = pimpl_->workspace_config_.walkspace_radius;
    VelocityConstraints most_restrictive;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        auto constraints = pimpl_->workspace_validator_->calculateVelocityConstraints(leg, bearing_degrees);

        // Use the most restrictive constraints
        if (leg == 0 || constraints.workspace_radius < min_effective_radius) {
            min_effective_radius = constraints.workspace_radius;
            most_restrictive = constraints;
        }
    }

    // Calculate limits based on the most constraining constraints
    float max_linear_speed = calculateMaxLinearSpeed(min_effective_radius,
                                                     gait_config.stance_ratio,
                                                     gait_config.frequency);

    float max_angular_speed = calculateMaxAngularSpeed(max_linear_speed,
                                                       pimpl_->workspace_config_.stance_radius);

    float max_acceleration = calculateMaxAcceleration(max_linear_speed,
                                                      gait_config.time_to_max_stride);

    // Create limit values using constraints
    LimitValues limits;
    limits.linear_x = std::min(max_linear_speed, most_restrictive.max_linear_velocity);
    limits.linear_y = std::min(max_linear_speed, most_restrictive.max_linear_velocity);
    limits.angular_z = std::min(max_angular_speed, most_restrictive.max_angular_velocity);
    limits.acceleration = std::min(max_acceleration, most_restrictive.max_acceleration);

    return limits;
}

float VelocityLimits::interpolateValue(float value1, float value2, float factor) const {
    return value1 + (value2 - value1) * factor;
}

int VelocityLimits::getBearingIndex(float bearing_degrees) const {
    return static_cast<int>(std::floor(bearing_degrees)) % 360;
}

void VelocityLimits::setSafetyMargin(float margin) {
    pimpl_->workspace_config_.safety_margin = margin;

    // Update validator safety margin
    pimpl_->workspace_validator_->updateSafetyMargin(margin);
}

void VelocityLimits::setAngularVelocityScaling(float scaling) {
    pimpl_->angular_velocity_scaling_ = scaling;

    // Update validator angular scaling
    pimpl_->workspace_validator_->updateAngularScaling(scaling);
}

float VelocityLimits::getOvershootX() const {
    return pimpl_->workspace_config_.overshoot_x;
}

float VelocityLimits::getOvershootY() const {
    return pimpl_->workspace_config_.overshoot_y;
}
