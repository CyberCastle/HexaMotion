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
        // When all servo angles are 0°, robot body is positioned at z = -tibia_length
        workspace_config_.reference_height = -model.getParams().tibia_length;

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

    // Para cada dirección (bearing), calcular límites de velocidad y aceleración
    for (int bearing = 0; bearing < 360; ++bearing) {
        double walkspace_radius = pimpl_->workspace_config_.walkspace_radius;
        double step_frequency = gait_config.frequency;

        // Maximum linear speed based on step length and frequency
        double max_step_length = walkspace_radius * 2.0;                    // Maximum step length
        double max_linear_speed = (max_step_length * step_frequency) / 2.0; // Average speed

        // Maximum angular speed based on stance radius
        double stance_radius = walkspace_radius * 0.8; // Effective stance radius
        double max_angular_speed = max_linear_speed / stance_radius;

        // Acceleration limits based on step timing
        double time_to_max_stride = gait_config.time_to_max_stride;
        double max_linear_acceleration = max_linear_speed / time_to_max_stride;

        // Crear y asignar los límites
        LimitValues limits;
        limits.linear_x = max_linear_speed;
        limits.linear_y = max_linear_speed;
        limits.angular_z = max_angular_speed;
        limits.acceleration = max_linear_acceleration;
        pimpl_->limit_map_.limits[bearing] = limits;
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

    LimitValues scaled_limits = input_velocities;

    // Apply angular velocity scaling using scaling factors
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    double angular_scale = angular_velocity_percentage * scaling_factors.angular_scale;
    scaled_limits.angular_z *= angular_scale;

    // Scale linear velocities based on angular velocity demand
    // High angular velocities reduce available linear velocity
    double linear_scale = 1.0 - (std::abs(angular_scale) * 0.3); // 30% coupling factor
    linear_scale = std::max(0.1, linear_scale);                  // Minimum 10% linear velocity

    scaled_limits.linear_x *= linear_scale;
    scaled_limits.linear_y *= linear_scale;

    return scaled_limits;
}

bool VelocityLimits::validateVelocityInputs(double vx, double vy, double omega) const {
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
    // Use velocity constraints instead of custom calculation

    // Get velocity constraints from validator for forward direction (0 degrees)
    auto constraints = pimpl_->workspace_analyzer_->calculateVelocityConstraints(0, 0.0);

    double max_acceleration = constraints.max_acceleration;

    // Overshoot distance during acceleration phase
    double accel_time = gait_config.time_to_max_stride;
    pimpl_->workspace_config_.overshoot_x = WORKSPACE_SCALING_FACTOR * max_acceleration * accel_time * accel_time;
    pimpl_->workspace_config_.overshoot_y = pimpl_->workspace_config_.overshoot_x; // Symmetric for now

    // Apply safety margin
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    pimpl_->workspace_config_.overshoot_x *= scaling_factors.safety_margin;
    pimpl_->workspace_config_.overshoot_y *= scaling_factors.safety_margin;
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

double VelocityLimits::calculateMaxLinearSpeed(double walkspace_radius,
                                               double on_ground_ratio, double frequency) const {
    // Use simplified OpenSHC-equivalent calculation with constraints
    if (on_ground_ratio <= 0.0 || frequency <= 0.0 || walkspace_radius <= 0.0) {
        return 0.0;
    }

    // Ensure reasonable bounds to prevent numerical issues
    double cycle_time = on_ground_ratio / frequency;
    if (cycle_time <= 0.0) {
        return 0.0;
    }

    double max_speed = (walkspace_radius * 2.0) / cycle_time;

    // Apply safety limits
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    max_speed *= scaling_factors.velocity_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_speed, 5000.0); // Cap at 5000 mm/s for safety
}

double VelocityLimits::calculateMaxAngularSpeed(double max_linear_speed, double stance_radius) const {
    // Use WorkspaceValidator angular scaling
    if (stance_radius <= 0.0 || max_linear_speed <= 0.0) {
        return 0.0;
    }

    double max_angular = max_linear_speed / stance_radius;

    // Apply angular scaling
    auto scaling_factors = pimpl_->workspace_analyzer_->getScalingFactors();
    max_angular *= scaling_factors.angular_scale;

    // Apply reasonable limits to prevent extreme values
    return std::min(max_angular, 10.0); // Cap at 10 rad/s for safety
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
    return std::min(max_accel, 10000.0); // Cap at 10000 mm/s² for safety
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
    double max_linear_speed = calculateMaxLinearSpeed(min_effective_radius,
                                                      gait_config.stance_ratio,
                                                      gait_config.frequency);

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

Point3D VelocityLimits::calculateStrideVector(double linear_velocity_x, double linear_velocity_y,
                                              double angular_velocity, const Point3D &current_tip_position,
                                              double stance_ratio, double step_frequency) {
    // OpenSHC equivalent stride vector calculation

    // 1. Linear stride vector (OpenSHC: stride_vector_linear)
    Point3D stride_vector_linear(linear_velocity_x, linear_velocity_y, 0.0);

    // 2. Angular stride vector (OpenSHC: angular_velocity.cross(radius))
    // Get radius by projecting current tip position to XY plane (OpenSHC: getRejection)
    Point3D radius = current_tip_position;
    radius.z = 0.0; // Project to XY plane (equivalent to getRejection)

    // Calculate angular stride vector using cross product
    // For 2D case: cross(angular_velocity * k, radius) = (-angular_velocity * radius.y, angular_velocity * radius.x, 0)
    Point3D stride_vector_angular(-angular_velocity * radius.y, angular_velocity * radius.x, 0.0);

    // 3. Combination (OpenSHC: stride_vector_linear + stride_vector_angular)
    Point3D stride_vector = stride_vector_linear + stride_vector_angular;

    // 4. Scaling by stance ratio and frequency (OpenSHC: stride_vector_ *= (on_ground_ratio / step.frequency_))
    double scaling_factor = stance_ratio / step_frequency;
    stride_vector = stride_vector * scaling_factor;

    return stride_vector;
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
