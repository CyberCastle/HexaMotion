#include "velocity_limits.h"

/**
 * @file velocity_limits.cpp
 * @brief Helper functions for validating commanded velocities.
 */
#include "math_utils.h"
#include <algorithm>
#include <cmath>

VelocityLimits::VelocityLimits(const RobotModel &model)
    : model_(model), angular_velocity_scaling_(1.0f) {
    // Initialize with default gait configuration
    current_gait_config_ = GaitConfig();
    generateLimits(current_gait_config_);
}

void VelocityLimits::generateLimits(const GaitConfig &gait_config) {
    current_gait_config_ = gait_config;

    // Calculate workspace parameters
    calculateWorkspace(gait_config);

    // Calculate overshoot compensation
    calculateOvershoot(gait_config);

    // Generate limits for all bearings (0-359 degrees)
    for (int bearing = 0; bearing < 360; ++bearing) {
        limit_map_.limits[bearing] = calculateLimitsForBearing(
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
    const Parameters &params = model_.getParams();

    // Calculate effective leg reach (similar to OpenSHC's workspace calculation)
    float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;

    // Apply safety margin to prevent workspace boundary violations
    float safe_reach = leg_reach * workspace_config_.safety_margin;

    // Enhanced workspace calculation using individual leg analysis
    calculateLegWorkspaces();

    // Calculate walkspace radius based on hexagon geometry
    // This is the effective radius where feet can safely step
    workspace_config_.walkspace_radius = std::max(workspace_config_.walkspace_radius,
                                                  params.hexagon_radius + safe_reach * 0.7f);

    // Stance radius for angular velocity calculations
    // This represents the effective turning radius of the robot
    workspace_config_.stance_radius = params.hexagon_radius + safe_reach * 0.5f;

    // Ensure minimum reasonable values
    workspace_config_.walkspace_radius = std::max(workspace_config_.walkspace_radius, 0.05f);
    workspace_config_.stance_radius = std::max(workspace_config_.stance_radius, 0.03f);
}

VelocityLimits::LimitValues VelocityLimits::scaleVelocityLimits(
    const LimitValues &input_velocities, float angular_velocity_percentage) const {

    LimitValues scaled_limits = input_velocities;

    // Apply angular velocity scaling (equivalent to OpenSHC's velocity scaling)
    float angular_scale = angular_velocity_percentage * angular_velocity_scaling_;
    scaled_limits.angular_z *= angular_scale;

    // Scale linear velocities based on angular velocity demand
    // High angular velocities reduce available linear velocity
    float linear_scale = 1.0f - (std::abs(angular_scale) * 0.3f);
    linear_scale = std::max(0.1f, linear_scale); // Minimum 10% linear velocity

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

    float factor = bearing_degrees - static_cast<float>(index1);

    const LimitValues &limits1 = limit_map_.limits[index1];
    const LimitValues &limits2 = limit_map_.limits[index2];

    LimitValues interpolated;
    interpolated.linear_x = interpolateValue(limits1.linear_x, limits2.linear_x, factor);
    interpolated.linear_y = interpolateValue(limits1.linear_y, limits2.linear_y, factor);
    interpolated.angular_z = interpolateValue(limits1.angular_z, limits2.angular_z, factor);
    interpolated.acceleration = interpolateValue(limits1.acceleration, limits2.acceleration, factor);

    return interpolated;
}

VelocityLimits::LimitValues VelocityLimits::applyAccelerationLimits(
    const LimitValues &target_velocities,
    const LimitValues &current_velocities,
    float dt) const {

    LimitValues limited_velocities = target_velocities;

    // Calculate maximum velocity change per timestep
    float max_delta_v = target_velocities.acceleration * dt;

    // Limit linear X velocity change
    float delta_vx = target_velocities.linear_x - current_velocities.linear_x;
    if (std::abs(delta_vx) > max_delta_v) {
        limited_velocities.linear_x = current_velocities.linear_x +
                                      (delta_vx > 0 ? max_delta_v : -max_delta_v);
    }

    // Limit linear Y velocity change
    float delta_vy = target_velocities.linear_y - current_velocities.linear_y;
    if (std::abs(delta_vy) > max_delta_v) {
        limited_velocities.linear_y = current_velocities.linear_y +
                                      (delta_vy > 0 ? max_delta_v : -max_delta_v);
    }

    // Limit angular velocity change (with different scaling for rotational acceleration)
    float max_delta_omega = target_velocities.acceleration * dt * 2.0f; // Angular acceleration is typically higher
    float delta_omega = target_velocities.angular_z - current_velocities.angular_z;
    if (std::abs(delta_omega) > max_delta_omega) {
        limited_velocities.angular_z = current_velocities.angular_z +
                                       (delta_omega > 0 ? max_delta_omega : -max_delta_omega);
    }

    return limited_velocities;
}

void VelocityLimits::calculateOvershoot(const GaitConfig &gait_config) {
    // Calculate overshoot compensation based on acceleration and gait timing
    // This prevents legs from exceeding workspace boundaries during acceleration

    float max_speed = calculateMaxLinearSpeed(workspace_config_.walkspace_radius,
                                              gait_config.stance_ratio, gait_config.frequency);

    float max_acceleration = calculateMaxAcceleration(max_speed, gait_config.time_to_max_stride);

    // Overshoot distance during acceleration phase
    float accel_time = gait_config.time_to_max_stride;
    workspace_config_.overshoot_x = 0.5f * max_acceleration * accel_time * accel_time;
    workspace_config_.overshoot_y = workspace_config_.overshoot_x; // Symmetric for now

    // Add safety margin
    workspace_config_.overshoot_x *= 1.2f;
    workspace_config_.overshoot_y *= 1.2f;
}

void VelocityLimits::updateGaitParameters(const GaitConfig &gait_config) {
    generateLimits(gait_config);
}

VelocityLimits::LimitValues VelocityLimits::getMaxLimits() const {
    LimitValues max_limits;

    for (const auto &limits : limit_map_.limits) {
        max_limits.linear_x = std::max(max_limits.linear_x, limits.linear_x);
        max_limits.linear_y = std::max(max_limits.linear_y, limits.linear_y);
        max_limits.angular_z = std::max(max_limits.angular_z, limits.angular_z);
        max_limits.acceleration = std::max(max_limits.acceleration, limits.acceleration);
    }

    return max_limits;
}

VelocityLimits::LimitValues VelocityLimits::getMinLimits() const {
    LimitValues min_limits(1e6f, 1e6f, 1e6f, 1e6f); // Initialize to large values

    for (const auto &limits : limit_map_.limits) {
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

    for (const auto &limits : limit_map_.limits) {
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
    // Equivalent to OpenSHC's speed calculation:
    // max_speed = (walkspace_radius * 2.0) / (on_ground_ratio / frequency)
    if (on_ground_ratio <= 0.0f || frequency <= 0.0f || walkspace_radius <= 0.0f) {
        return 0.0f;
    }

    // Ensure reasonable bounds to prevent numerical issues
    float cycle_time = on_ground_ratio / frequency;
    if (cycle_time <= 0.0f) {
        return 0.0f;
    }

    float max_speed = (walkspace_radius * 2.0f) / cycle_time;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_speed, 5.0f); // Cap at 5 m/s for safety
}

float VelocityLimits::calculateMaxAngularSpeed(float max_linear_speed, float stance_radius) const {
    // Equivalent to OpenSHC's angular speed calculation:
    // max_angular_speed = max_linear_speed / stance_radius
    if (stance_radius <= 0.0f || max_linear_speed <= 0.0f) {
        return 0.0f;
    }

    float max_angular = max_linear_speed / stance_radius;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_angular, 10.0f); // Cap at 10 rad/s for safety
}

float VelocityLimits::calculateMaxAcceleration(float max_speed, float time_to_max) const {
    // Equivalent to OpenSHC's acceleration calculation:
    // max_acceleration = max_speed / time_to_max_stride
    if (time_to_max <= 0.0f || max_speed <= 0.0f) {
        return 0.0f;
    }

    float max_accel = max_speed / time_to_max;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_accel, 10.0f); // Cap at 10 m/s² for safety
}

void VelocityLimits::calculateLegWorkspaces() {
    // Enhanced workspace calculation equivalent to OpenSHC's generateWorkspace
    const Parameters &params = model_.getParams();
    float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float safe_reach = leg_reach * workspace_config_.safety_margin;

    // Initialize workspace arrays for each leg
    float leg_workspace_radius[NUM_LEGS][360]; // Radius for each bearing (0-359°)

    // Calculate individual leg workspaces using kinematic constraints
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D leg_base = getLegBasePosition(leg);

        for (int bearing = 0; bearing < 360; ++bearing) {
            float bearing_rad = math_utils::degreesToRadians(bearing);

            // Calculate maximum reachable point at this bearing
            float max_radius = 0.0f;

            // Test different radii to find kinematic limit
            for (float test_radius = 0.0f; test_radius < leg_reach; test_radius += 5.0f) {
                Point3D test_point;
                test_point.x = leg_base.x + test_radius * cos(bearing_rad);
                test_point.y = leg_base.y + test_radius * sin(bearing_rad);
                test_point.z = leg_base.z - params.robot_height; // Ground level

                // Check if point is kinematically reachable
                // This would normally use inverse kinematics validation
                // For now, use simplified geometric constraint
                float distance_from_base = sqrt(
                    (test_point.x - leg_base.x) * (test_point.x - leg_base.x) +
                    (test_point.y - leg_base.y) * (test_point.y - leg_base.y) +
                    (test_point.z - leg_base.z) * (test_point.z - leg_base.z));

                if (distance_from_base <= leg_reach * 0.8f) { // 80% of theoretical reach
                    max_radius = test_radius;
                } else {
                    break; // Found limit
                }
            }

            leg_workspace_radius[leg][bearing] = max_radius;
        }
    }

    // Calculate combined workspace limits
    float min_walkspace_radius = leg_reach;
    float min_stance_radius = leg_reach;

    for (int bearing = 0; bearing < 360; ++bearing) {
        float min_leg_radius = leg_reach;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            min_leg_radius = std::min(min_leg_radius, leg_workspace_radius[leg][bearing]);
        }

        min_walkspace_radius = std::min(min_walkspace_radius, min_leg_radius);
        min_stance_radius = std::min(min_stance_radius, min_leg_radius * 0.7f);
    }

    // Store calculated values with safety margins
    workspace_config_.walkspace_radius = min_walkspace_radius * workspace_config_.safety_margin;
    workspace_config_.stance_radius = min_stance_radius * workspace_config_.safety_margin;

    // Calculate overshoot compensation based on gait dynamics
    workspace_config_.overshoot_x = workspace_config_.walkspace_radius * 0.1f;
    workspace_config_.overshoot_y = workspace_config_.walkspace_radius * 0.1f;
}

float VelocityLimits::calculateEffectiveRadius(int leg_index, float bearing_degrees) const {
    // Calculate effective workspace radius in a specific direction
    // This accounts for leg positioning and workspace geometry

    Point3D leg_base = getLegBasePosition(leg_index);
    float leg_angle = std::atan2(leg_base.y, leg_base.x);
    float leg_angle_deg = math_utils::radiansToDegrees(leg_angle);

    // Calculate angle difference between desired bearing and leg direction
    float angle_diff = std::abs(normalizeBearing(bearing_degrees - leg_angle_deg));
    if (angle_diff > 180.0f) {
        angle_diff = 360.0f - angle_diff;
    }

    // Effective radius decreases as we move away from leg's natural direction
    float efficiency = std::cos(math_utils::degreesToRadians(angle_diff));
    efficiency = std::max(0.3f, efficiency); // Minimum 30% efficiency

    return workspace_config_.walkspace_radius * efficiency;
}

Point3D VelocityLimits::getLegBasePosition(int leg_index) const {
    const Parameters &params = model_.getParams();
    float base_angle = leg_index * 60.0f; // Hexagon geometry

    Point3D base_pos;
    base_pos.x = params.hexagon_radius * std::cos(math_utils::degreesToRadians(base_angle));
    base_pos.y = params.hexagon_radius * std::sin(math_utils::degreesToRadians(base_angle));
    base_pos.z = 0.0f;

    return base_pos;
}

VelocityLimits::LimitValues VelocityLimits::calculateLimitsForBearing(
    float bearing_degrees, const GaitConfig &gait_config) const {

    // Find the most constraining leg for this bearing direction
    float min_effective_radius = workspace_config_.walkspace_radius;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        float effective_radius = calculateEffectiveRadius(leg, bearing_degrees);
        min_effective_radius = std::min(min_effective_radius, effective_radius);
    }

    // Calculate limits based on the most constraining workspace
    float max_linear_speed = calculateMaxLinearSpeed(min_effective_radius,
                                                     gait_config.stance_ratio,
                                                     gait_config.frequency);

    float max_angular_speed = calculateMaxAngularSpeed(max_linear_speed,
                                                       workspace_config_.stance_radius);

    float max_acceleration = calculateMaxAcceleration(max_linear_speed,
                                                      gait_config.time_to_max_stride);

    // Create limit values for this bearing
    LimitValues limits;
    limits.linear_x = max_linear_speed;
    limits.linear_y = max_linear_speed;
    limits.angular_z = max_angular_speed;
    limits.acceleration = max_acceleration;

    return limits;
}

float VelocityLimits::interpolateValue(float value1, float value2, float factor) const {
    return value1 + (value2 - value1) * factor;
}

int VelocityLimits::getBearingIndex(float bearing_degrees) const {
    return static_cast<int>(std::floor(bearing_degrees)) % 360;
}
