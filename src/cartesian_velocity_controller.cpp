#include "cartesian_velocity_controller.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

CartesianVelocityController::CartesianVelocityController(const RobotModel &model)
    : model_(model), velocity_control_enabled_(true) {

    // Initialize with default configurations
    resetToDefaults();
}

bool CartesianVelocityController::updateServoSpeeds(float linear_velocity_x, float linear_velocity_y,
                                                    float angular_velocity, GaitType current_gait) {
    if (!velocity_control_enabled_) {
        return true; // Use default speeds when disabled
    }

    // Store current velocity commands
    current_linear_vx_ = linear_velocity_x;
    current_linear_vy_ = linear_velocity_y;
    current_angular_velocity_ = angular_velocity;
    current_gait_ = current_gait;

    // Calculate velocity-based scaling factors
    float linear_magnitude = std::sqrt(linear_velocity_x * linear_velocity_x + linear_velocity_y * linear_velocity_y);
    float linear_scale = calculateLinearVelocityScale(linear_magnitude);
    float angular_scale = calculateAngularVelocityScale(std::abs(angular_velocity));
    float gait_adjustment = calculateGaitSpeedAdjustment(current_gait);

    // Update servo speeds for each leg
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Calculate leg-specific compensation
        float leg_compensation = calculateLegSpeedCompensation(leg, linear_velocity_x,
                                                               linear_velocity_y, angular_velocity);

        // Base speed from robot parameters
        const Parameters &params = model_.getParams();
        float base_speed = params.default_servo_speed;

        // Calculate combined scaling factors
        float combined_scale = linear_scale * angular_scale * gait_adjustment * leg_compensation;

        // Apply to each joint in the leg
        leg_servo_speeds_[leg].coxa.base_speed = base_speed;
        leg_servo_speeds_[leg].coxa.velocity_scaling = combined_scale;
        leg_servo_speeds_[leg].coxa.angular_compensation = angular_scale;
        leg_servo_speeds_[leg].coxa.gait_adjustment = gait_adjustment;

        leg_servo_speeds_[leg].femur.base_speed = base_speed;
        leg_servo_speeds_[leg].femur.velocity_scaling = combined_scale * 1.1f; // Femur moves more
        leg_servo_speeds_[leg].femur.angular_compensation = angular_scale;
        leg_servo_speeds_[leg].femur.gait_adjustment = gait_adjustment;

        leg_servo_speeds_[leg].tibia.base_speed = base_speed;
        leg_servo_speeds_[leg].tibia.velocity_scaling = combined_scale * 1.2f; // Tibia moves most
        leg_servo_speeds_[leg].tibia.angular_compensation = angular_scale;
        leg_servo_speeds_[leg].tibia.gait_adjustment = gait_adjustment;

        // Apply workspace constraints
        for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
            ServoSpeedConfig *joint_config = nullptr;
            switch (joint) {
            case 0:
                joint_config = &leg_servo_speeds_[leg].coxa;
                break;
            case 1:
                joint_config = &leg_servo_speeds_[leg].femur;
                break;
            case 2:
                joint_config = &leg_servo_speeds_[leg].tibia;
                break;
            }

            if (joint_config) {
                float workspace_constrained_speed = applyWorkspaceConstraints(leg, joint,
                                                                              joint_config->getEffectiveSpeed());
                // Update the velocity scaling to match workspace constraints
                if (joint_config->base_speed > 0.0f) {
                    joint_config->velocity_scaling = workspace_constrained_speed / joint_config->base_speed;
                }
            }
        }
    }

    return true;
}

float CartesianVelocityController::getServoSpeed(int leg_index, int joint_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
        return model_.getParams().default_servo_speed; // Return default for invalid indices
    }

    const LegServoSpeeds &leg_speeds = leg_servo_speeds_[leg_index];

    switch (joint_index) {
    case 0:
        return leg_speeds.coxa.getEffectiveSpeed();
    case 1:
        return leg_speeds.femur.getEffectiveSpeed();
    case 2:
        return leg_speeds.tibia.getEffectiveSpeed();
    default:
        return model_.getParams().default_servo_speed;
    }
}

const CartesianVelocityController::LegServoSpeeds &CartesianVelocityController::getLegServoSpeeds(int leg_index) const {
    static LegServoSpeeds default_speeds; // Fallback for invalid indices

    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return default_speeds;
    }

    return leg_servo_speeds_[leg_index];
}

void CartesianVelocityController::setVelocityScaling(const VelocityScaling &scaling) {
    velocity_scaling_ = scaling;

    // Validate and clamp scaling parameters
    velocity_scaling_.linear_velocity_scale = std::max(0.1f, std::min(5.0f, velocity_scaling_.linear_velocity_scale));
    velocity_scaling_.angular_velocity_scale = std::max(0.1f, std::min(5.0f, velocity_scaling_.angular_velocity_scale));
    velocity_scaling_.minimum_speed_ratio = std::max(0.05f, std::min(0.95f, velocity_scaling_.minimum_speed_ratio));
    velocity_scaling_.maximum_speed_ratio = std::max(1.05f, std::min(3.0f, velocity_scaling_.maximum_speed_ratio));
}

void CartesianVelocityController::setGaitSpeedModifiers(const GaitSpeedModifiers &modifiers) {
    gait_modifiers_ = modifiers;

    // Validate and clamp gait modifiers
    gait_modifiers_.tripod_speed_factor = std::max(0.1f, std::min(2.0f, gait_modifiers_.tripod_speed_factor));
    gait_modifiers_.wave_speed_factor = std::max(0.1f, std::min(2.0f, gait_modifiers_.wave_speed_factor));
    gait_modifiers_.ripple_speed_factor = std::max(0.1f, std::min(2.0f, gait_modifiers_.ripple_speed_factor));
    gait_modifiers_.metachronal_speed_factor = std::max(0.1f, std::min(2.0f, gait_modifiers_.metachronal_speed_factor));
    gait_modifiers_.adaptive_speed_factor = std::max(0.1f, std::min(2.0f, gait_modifiers_.adaptive_speed_factor));
}

void CartesianVelocityController::setVelocityControlEnabled(bool enable) {
    velocity_control_enabled_ = enable;

    if (!enable) {
        resetToDefaults();
    }
}

float CartesianVelocityController::getCurrentVelocityMagnitude() const {
    float linear_magnitude = std::sqrt(current_linear_vx_ * current_linear_vx_ +
                                       current_linear_vy_ * current_linear_vy_);
    float angular_magnitude = std::abs(current_angular_velocity_);

    // Combine linear and angular velocities with appropriate weighting
    // Angular velocity is weighted by typical robot radius for dimensional consistency
    const Parameters &params = model_.getParams();
    float angular_linear_equiv = angular_magnitude * (params.hexagon_radius / 1000.0f); // Convert mm to m

    return linear_magnitude + angular_linear_equiv * 0.5f; // Angular contributes 50% to total
}

void CartesianVelocityController::resetToDefaults() {
    const Parameters &params = model_.getParams();
    float default_speed = params.default_servo_speed;

    // Reset all leg servo speeds to defaults
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        leg_servo_speeds_[leg].coxa = ServoSpeedConfig{default_speed, 1.0f, 1.0f, 1.0f};
        leg_servo_speeds_[leg].femur = ServoSpeedConfig{default_speed, 1.0f, 1.0f, 1.0f};
        leg_servo_speeds_[leg].tibia = ServoSpeedConfig{default_speed, 1.0f, 1.0f, 1.0f};
    }

    // Reset velocity state
    current_linear_vx_ = 0.0f;
    current_linear_vy_ = 0.0f;
    current_angular_velocity_ = 0.0f;
}

float CartesianVelocityController::calculateLinearVelocityScale(float velocity_magnitude) const {
    if (velocity_magnitude < 0.001f) { // Very small velocity, minimal scaling
        return velocity_scaling_.minimum_speed_ratio;
    }

    // Map velocity magnitude to speed scaling using the robot's maximum velocity as reference
    const Parameters &params = model_.getParams();
    float max_velocity_ms = params.max_velocity / 1000.0f; // Convert mm/s to m/s

    if (max_velocity_ms < 0.001f) {
        return 1.0f; // Default scaling if max velocity not properly set
    }

    // Calculate linear scaling: higher velocity = higher servo speed
    float velocity_ratio = velocity_magnitude / max_velocity_ms;
    velocity_ratio = std::min(1.0f, velocity_ratio); // Clamp to max

    // Apply scaling with configured parameters
    float scale = velocity_scaling_.minimum_speed_ratio +
                  (velocity_scaling_.maximum_speed_ratio - velocity_scaling_.minimum_speed_ratio) * velocity_ratio;

    return std::max(velocity_scaling_.minimum_speed_ratio,
                    std::min(velocity_scaling_.maximum_speed_ratio, scale));
}

float CartesianVelocityController::calculateAngularVelocityScale(float angular_velocity) const {
    if (angular_velocity < 0.001f) { // Very small angular velocity
        return 1.0f;
    }

    // Map angular velocity to speed scaling using the robot's maximum angular velocity as reference
    const Parameters &params = model_.getParams();
    float max_angular_velocity_rads = math_utils::degreesToRadians(params.max_angular_velocity);

    if (max_angular_velocity_rads < 0.001f) {
        return 1.0f; // Default scaling if max angular velocity not properly set
    }

    // Calculate angular scaling: higher angular velocity = higher servo speed for outer legs
    float angular_ratio = angular_velocity / max_angular_velocity_rads;
    angular_ratio = std::min(1.0f, angular_ratio); // Clamp to max

    // Angular motion typically requires faster servo speeds
    float scale = 1.0f + angular_ratio * velocity_scaling_.angular_velocity_scale;

    return std::max(velocity_scaling_.minimum_speed_ratio,
                    std::min(velocity_scaling_.maximum_speed_ratio, scale));
}

float CartesianVelocityController::calculateGaitSpeedAdjustment(GaitType gait) const {
    switch (gait) {
    case TRIPOD_GAIT:
        return gait_modifiers_.tripod_speed_factor;
    case WAVE_GAIT:
        return gait_modifiers_.wave_speed_factor;
    case RIPPLE_GAIT:
        return gait_modifiers_.ripple_speed_factor;
    case METACHRONAL_GAIT:
        return gait_modifiers_.metachronal_speed_factor;
    case ADAPTIVE_GAIT:
        return gait_modifiers_.adaptive_speed_factor;
    default:
        return 1.0f;
    }
}

float CartesianVelocityController::calculateLegSpeedCompensation(int leg_index, float linear_vx,
                                                                 float linear_vy, float angular_vel) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return 1.0f;
    }

    // Get leg position in robot frame
    const Parameters &params = model_.getParams();
    float leg_angle_deg = leg_index * 60.0f; // Hexapod legs spaced 60° apart
    float leg_angle_rad = math_utils::degreesToRadians(leg_angle_deg);

    // Leg position relative to robot center
    float leg_x = params.hexagon_radius * std::cos(leg_angle_rad) / 1000.0f; // Convert to meters
    float leg_y = params.hexagon_radius * std::sin(leg_angle_rad) / 1000.0f;

    // Calculate the velocity of this leg due to body motion
    // Linear motion: all legs move at same speed
    float linear_velocity_magnitude = std::sqrt(linear_vx * linear_vx + linear_vy * linear_vy);

    // Angular motion: outer legs move faster than inner legs
    // Tangential velocity = angular_velocity * radius_from_center
    float radius_from_center = std::sqrt(leg_x * leg_x + leg_y * leg_y);
    float tangential_velocity = std::abs(angular_vel) * radius_from_center;

    // Combined velocity demand for this leg
    float total_leg_velocity = linear_velocity_magnitude + tangential_velocity;

    // Scale servo speed based on leg velocity demand
    // Higher demand = higher servo speed
    float max_expected_velocity = 0.5f; // m/s - typical maximum leg velocity
    float velocity_ratio = std::min(1.0f, total_leg_velocity / max_expected_velocity);

    // Apply compensation: faster legs need higher servo speeds
    float compensation = 1.0f + velocity_ratio * 0.5f; // Up to 50% speed increase

    return std::max(0.5f, std::min(2.0f, compensation)); // Clamp to reasonable range
}

float CartesianVelocityController::applyWorkspaceConstraints(int leg_index, int joint_index, float base_speed) const {
    // Apply basic constraints based on joint type
    // Different joints may have different speed characteristics

    float constrained_speed = base_speed;

    // Joint-specific constraints
    switch (joint_index) {
    case 0: // Coxa joint
        // Coxa typically has lower speed requirements
        constrained_speed *= 0.9f;
        break;
    case 1: // Femur joint
        // Femur carries most of the leg motion load
        constrained_speed *= 1.0f;
        break;
    case 2: // Tibia joint
        // Tibia provides fine positioning and may need higher speeds
        constrained_speed *= 1.1f;
        break;
    }

    // Apply velocity scaling constraints
    constrained_speed = std::max(velocity_scaling_.minimum_speed_ratio * model_.getParams().default_servo_speed,
                                 std::min(velocity_scaling_.maximum_speed_ratio * model_.getParams().default_servo_speed,
                                          constrained_speed));

    // Final servo speed limits (typical servo constraints)
    return std::max(0.1f, std::min(3.0f, constrained_speed));
}
