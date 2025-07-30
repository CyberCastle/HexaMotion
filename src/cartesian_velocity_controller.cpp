#include "cartesian_velocity_controller.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "workspace_validator.h" // Use validator for workspace constraints
#include <algorithm>
#include <cmath>

CartesianVelocityController::CartesianVelocityController(const RobotModel &model)
    : model_(model), velocity_control_enabled_(true) {

    // Initialize workspace validator for velocity constraints
    ValidationConfig config;
    config.enable_collision_checking = false;  // Disable for performance in velocity control
    config.enable_joint_limit_checking = true; // Enable for accurate servo speed calculation
    workspace_validator_ = std::make_unique<WorkspaceValidator>(
        const_cast<RobotModel &>(model), config);

    // Initialize with default configurations
    resetToDefaults();
}

CartesianVelocityController::~CartesianVelocityController() = default;

bool CartesianVelocityController::updateServoSpeeds(double linear_velocity_x, double linear_velocity_y,
                                                    double angular_velocity, GaitType current_gait) {
    if (!velocity_control_enabled_) {
        return true; // Use default speeds when disabled
    }

    // Store current velocity commands
    current_linear_vx_ = linear_velocity_x;
    current_linear_vy_ = linear_velocity_y;
    current_angular_velocity_ = angular_velocity;
    current_gait_ = current_gait;

    // Calculate velocity-based scaling factors
    double linear_magnitude = std::sqrt(linear_velocity_x * linear_velocity_x + linear_velocity_y * linear_velocity_y);
    double linear_scale = calculateLinearVelocityScale(linear_magnitude);
    double angular_scale = calculateAngularVelocityScale(std::abs(angular_velocity));
    double gait_adjustment = calculateGaitSpeedAdjustment(current_gait);

    // Update servo speeds for each leg
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Calculate leg-specific compensation
        double leg_compensation = calculateLegSpeedCompensation(leg, linear_velocity_x,
                                                                linear_velocity_y, angular_velocity);

        // Base speed from robot parameters
        const Parameters &params = model_.getParams();
        double base_speed = params.default_servo_speed;

        // Calculate combined scaling factors
        double combined_scale = linear_scale * angular_scale * gait_adjustment * leg_compensation;

        // Apply to each joint in the leg
        leg_servo_speeds_[leg].coxa.base_speed = base_speed;
        leg_servo_speeds_[leg].coxa.velocity_scaling = combined_scale;
        leg_servo_speeds_[leg].coxa.angular_compensation = angular_scale;
        leg_servo_speeds_[leg].coxa.gait_adjustment = gait_adjustment;

        leg_servo_speeds_[leg].femur.base_speed = base_speed;
        leg_servo_speeds_[leg].femur.velocity_scaling = combined_scale * FEMUR_VELOCITY_MULTIPLIER; // Femur moves more
        leg_servo_speeds_[leg].femur.angular_compensation = angular_scale;
        leg_servo_speeds_[leg].femur.gait_adjustment = gait_adjustment;

        leg_servo_speeds_[leg].tibia.base_speed = base_speed;
        leg_servo_speeds_[leg].tibia.velocity_scaling = combined_scale * TIBIA_VELOCITY_MULTIPLIER; // Tibia moves most
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
                double workspace_constrained_speed = applyWorkspaceConstraints(leg, joint,
                                                                               joint_config->getEffectiveSpeed());
                // Update the velocity scaling to match workspace constraints
                if (joint_config->base_speed > 0.0) {
                    joint_config->velocity_scaling = workspace_constrained_speed / joint_config->base_speed;
                }
            }
        }
    }

    return true;
}

double CartesianVelocityController::getServoSpeed(int leg_index, int joint_index) const {
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
    velocity_scaling_.linear_velocity_scale =
        math_utils::clamp<double>(velocity_scaling_.linear_velocity_scale, VELOCITY_SCALE_MIN, VELOCITY_SCALE_MAX);
    velocity_scaling_.angular_velocity_scale =
        math_utils::clamp<double>(velocity_scaling_.angular_velocity_scale, VELOCITY_SCALE_MIN, VELOCITY_SCALE_MAX);
    velocity_scaling_.minimum_speed_ratio =
        math_utils::clamp<double>(velocity_scaling_.minimum_speed_ratio, SPEED_RATIO_MIN, SPEED_RATIO_MAX_VALIDATION);
    velocity_scaling_.maximum_speed_ratio =
        math_utils::clamp<double>(velocity_scaling_.maximum_speed_ratio, SPEED_RATIO_MIN_VALIDATION, SPEED_RATIO_MAX);
}

void CartesianVelocityController::setGaitSpeedModifiers(const GaitSpeedModifiers &modifiers) {
    gait_modifiers_ = modifiers;

    // Validate and clamp gait modifiers
    gait_modifiers_.tripod_speed_factor =
        math_utils::clamp<double>(gait_modifiers_.tripod_speed_factor, GAIT_MODIFIER_MIN, GAIT_MODIFIER_MAX);
    gait_modifiers_.wave_speed_factor =
        math_utils::clamp<double>(gait_modifiers_.wave_speed_factor, GAIT_MODIFIER_MIN, GAIT_MODIFIER_MAX);
    gait_modifiers_.ripple_speed_factor =
        math_utils::clamp<double>(gait_modifiers_.ripple_speed_factor, GAIT_MODIFIER_MIN, GAIT_MODIFIER_MAX);
    gait_modifiers_.metachronal_speed_factor =
        math_utils::clamp<double>(gait_modifiers_.metachronal_speed_factor, GAIT_MODIFIER_MIN, GAIT_MODIFIER_MAX);
    gait_modifiers_.adaptive_speed_factor =
        math_utils::clamp<double>(gait_modifiers_.adaptive_speed_factor, GAIT_MODIFIER_MIN, GAIT_MODIFIER_MAX);
}

void CartesianVelocityController::setVelocityControlEnabled(bool enable) {
    velocity_control_enabled_ = enable;

    if (!enable) {
        resetToDefaults();
    }
}

double CartesianVelocityController::getCurrentVelocityMagnitude() const {
    double linear_magnitude = std::sqrt(current_linear_vx_ * current_linear_vx_ +
                                        current_linear_vy_ * current_linear_vy_);
    double angular_magnitude = std::abs(current_angular_velocity_);

    // Combine linear and angular velocities with appropriate weighting
    // Angular velocity is weighted by typical robot radius for dimensional consistency
    const Parameters &params = model_.getParams();
    double angular_linear_equiv = angular_magnitude * params.hexagon_radius; // Keep in mm

    return linear_magnitude + angular_linear_equiv * 0.5; // Angular contributes 50% to total
}

void CartesianVelocityController::resetToDefaults() {
    const Parameters &params = model_.getParams();
    double default_speed = params.default_servo_speed;

    // Reset all leg servo speeds to defaults
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        leg_servo_speeds_[leg].coxa = ServoSpeedConfig{default_speed, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT};
        leg_servo_speeds_[leg].femur = ServoSpeedConfig{default_speed, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT};
        leg_servo_speeds_[leg].tibia = ServoSpeedConfig{default_speed, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT};
    }

    // Reset velocity state
    current_linear_vx_ = 0.0;
    current_linear_vy_ = 0.0;
    current_angular_velocity_ = 0.0;
}

double CartesianVelocityController::calculateLinearVelocityScale(double velocity_magnitude) const {
    if (velocity_magnitude < VELOCITY_THRESHOLD) { // Very small velocity, minimal scaling
        return velocity_scaling_.minimum_speed_ratio;
    }

    // Map velocity magnitude to speed scaling using the robot's maximum velocity as reference
    const Parameters &params = model_.getParams();
    double max_velocity_mmps = params.max_velocity; // Keep in mm/s

    if (max_velocity_mmps < VELOCITY_THRESHOLD) {
        return SERVO_SPEED_DEFAULT; // Default scaling if max velocity not properly set
    }

    // Calculate linear scaling: higher velocity = higher servo speed
    double velocity_ratio = velocity_magnitude / max_velocity_mmps;
    velocity_ratio = math_utils::clamp<double>(velocity_ratio, 0.0, SERVO_SPEED_DEFAULT);

    // Apply scaling with configured parameters
    double scale = velocity_scaling_.minimum_speed_ratio +
                   (velocity_scaling_.maximum_speed_ratio - velocity_scaling_.minimum_speed_ratio) * velocity_ratio;

    return math_utils::clamp<double>(scale, velocity_scaling_.minimum_speed_ratio,
                              velocity_scaling_.maximum_speed_ratio);
}

double CartesianVelocityController::calculateAngularVelocityScale(double angular_velocity) const {
    if (angular_velocity < VELOCITY_THRESHOLD) { // Very small angular velocity
        return SERVO_SPEED_DEFAULT;
    }

    // Map angular velocity to speed scaling using the robot's maximum angular velocity as reference
    const Parameters &params = model_.getParams();
    double max_angular_velocity_rads = params.max_angular_velocity; // Already in rad/s

    if (max_angular_velocity_rads < VELOCITY_THRESHOLD) {
        return SERVO_SPEED_DEFAULT; // Default scaling if max angular velocity not properly set
    }

    // Calculate angular scaling: higher angular velocity = higher servo speed for outer legs
    double angular_ratio = angular_velocity / max_angular_velocity_rads;
    angular_ratio = math_utils::clamp<double>(angular_ratio, 0.0, SERVO_SPEED_DEFAULT);

    // Angular motion typically requires faster servo speeds
    double scale = SERVO_SPEED_DEFAULT + angular_ratio * velocity_scaling_.angular_velocity_scale;

    return math_utils::clamp<double>(scale, velocity_scaling_.minimum_speed_ratio,
                              velocity_scaling_.maximum_speed_ratio);
}

double CartesianVelocityController::calculateGaitSpeedAdjustment(GaitType gait) const {
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
        return SERVO_SPEED_DEFAULT;
    }
}

double CartesianVelocityController::calculateLegSpeedCompensation(int leg_index, double linear_vx,
                                                                  double linear_vy, double angular_vel) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return SERVO_SPEED_DEFAULT;
    }

    // Get leg position in robot frame
    Point3D base_pos = model_.getLegBasePosition(leg_index);
    double leg_x = base_pos.x; // Keep in mm
    double leg_y = base_pos.y; // Keep in mm

    // Calculate the velocity of this leg due to body motion
    // Linear motion: all legs move at same speed
    double linear_velocity_magnitude = std::sqrt(linear_vx * linear_vx + linear_vy * linear_vy);

    // Angular motion: outer legs move faster than inner legs
    // Tangential velocity = angular_velocity * radius_from_center
    double radius_from_center = std::sqrt(leg_x * leg_x + leg_y * leg_y);
    double tangential_velocity = std::abs(angular_vel) * radius_from_center;

    // Combined velocity demand for this leg
    double total_leg_velocity = linear_velocity_magnitude + tangential_velocity;

    // Scale servo speed based on leg velocity demand
    // Higher demand = higher servo speed
    double max_expected_velocity = 500.0; // mm/s - typical maximum leg velocity
    double velocity_ratio =
        math_utils::clamp<double>(total_leg_velocity / max_expected_velocity, 0.0,
                           SERVO_SPEED_DEFAULT);

    // Apply compensation: faster legs need higher servo speeds
    double compensation = SERVO_SPEED_DEFAULT + velocity_ratio * 0.5; // Up to 50% speed increase

    return math_utils::clamp<double>(compensation, LEG_COMPENSATION_MIN, LEG_COMPENSATION_MAX);
}

double CartesianVelocityController::applyWorkspaceConstraints(int leg_index, int joint_index, double base_speed) const {
    // Use WorkspaceValidator for workspace constraints instead of hardcoded factors

    if (!workspace_validator_) {
        return base_speed; // Safety fallback
    }

    double constrained_speed = base_speed;

    // Get scaling factors instead of hardcoded constants
    auto scaling_factors = workspace_validator_->getScalingFactors();

    // Apply joint-specific constraints using scaling
    switch (joint_index) {
    case 0: // Coxa joint
        // Coxa typically has lower speed requirements
        constrained_speed *= scaling_factors.workspace_scale; // Use workspace scaling
        break;
    case 1: // Femur joint
        // Femur carries most of the leg motion load - use velocity scaling
        constrained_speed *= scaling_factors.velocity_scale;
        break;
    case 2: // Tibia joint
        // Tibia provides fine positioning - use full scaling
        constrained_speed *= scaling_factors.velocity_scale * 1.1; // 10% boost for precision
        break;
    }

    // Apply velocity scaling constraints
    const Parameters &params = model_.getParams();
    double min_speed = velocity_scaling_.minimum_speed_ratio * params.default_servo_speed;
    double max_speed = velocity_scaling_.maximum_speed_ratio * params.default_servo_speed;

    // Apply safety margin
    min_speed *= scaling_factors.safety_margin;
    max_speed *= scaling_factors.safety_margin;

    constrained_speed = math_utils::clamp<double>(constrained_speed, min_speed, max_speed);

    // Final servo speed limits (typical servo constraints)
    return math_utils::clamp<double>(constrained_speed, SERVO_SPEED_MIN, SERVO_SPEED_MAX);
}
