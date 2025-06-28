#include "walk_controller.h"
#include "hexamotion_constants.h"
#include "workspace_validator.h" // Use unified validator instead

WalkController::WalkController(RobotModel &m)
    : model(m), current_gait(TRIPOD_GAIT), gait_phase(0.0f),
      terrain_adaptation_(m), velocity_limits_(m) {
    // Initialize terrain adaptation system
    terrain_adaptation_.initialize();

    // Initialize velocity limits with default gait parameters
    VelocityLimits::GaitConfig default_gait;
    default_gait.frequency = DEFAULT_ANGULAR_SCALING;
    default_gait.stance_ratio = 0.6f;
    default_gait.swing_ratio = 0.4f;
    default_gait.time_to_max_stride = 2.0f;
    velocity_limits_.generateLimits(default_gait);

    // Initialize current velocities to zero
    current_velocities_ = VelocityLimits::LimitValues();

    // Initialize current leg positions array for collision tracking
    for (int i = 0; i < NUM_LEGS; i++) {
        current_leg_positions_[i] = Point3D(0, 0, 0);
    }

    // Initialize unified workspace validator with optimized settings
    ValidationConfig config;
    config.safety_margin_factor = 0.65f;        // Same as original 65% safety margin
    config.collision_safety_margin = 30.0f;     // 30mm safety between legs
    config.enable_collision_checking = true;    // Enable collision avoidance
    config.enable_joint_limit_checking = false; // Disable for performance (IK already checks)
    workspace_validator_ = std::make_unique<WorkspaceValidator>(m, config);
}

bool WalkController::setGaitType(GaitType gait) {
    current_gait = gait;
    gait_phase = 0.0f;
    return true;
}

bool WalkController::planGaitSequence(float vx, float vy, float omega) {
    // Store current velocity commands for gait pattern decisions
    current_velocities_.linear_x = vx;
    current_velocities_.linear_y = vy;
    current_velocities_.angular_z = omega;

    // Apply velocity limits and validation
    if (!validateVelocityCommand(vx, vy, omega)) {
        // If commanded velocities exceed limits, apply constraints
        VelocityLimits::LimitValues limited = applyVelocityLimits(vx, vy, omega);
        current_velocities_ = limited;
    }

    return true;
}

void WalkController::updateGaitPhase(float dt) {
    gait_phase += dt;
    if (gait_phase >= 1.0f)
        gait_phase -= 1.0f;
}

Point3D WalkController::footTrajectory(int leg_index, float phase, float step_height, float step_length,
                                       float stance_duration, float swing_duration, float robot_height,
                                       const float leg_phase_offsets[NUM_LEGS], LegState leg_states[NUM_LEGS],
                                       IFSRInterface *fsr, IIMUInterface *imu) {
    // Update terrain adaptation system
    terrain_adaptation_.update(fsr, imu);

    float leg_phase = phase + leg_phase_offsets[leg_index];
    if (leg_phase >= 1.0f)
        leg_phase -= 1.0f;
    Point3D trajectory;
    float base_angle = leg_index * LEG_ANGLE_SPACING;
    const Parameters &p = model.getParams();

    // Calculate leg base position (hexagon corner)
    float base_x = p.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
    float base_y = p.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));

    // Calculate default foot position within reachable workspace
    float leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
    float min_reach = std::abs(p.femur_length - p.tibia_length);

    // Keep a safety margin so the target remains achievable, 65% of leg reach.
    // The previous implementation summed the hexagon radius and the safe reach
    // which could place the target well outside or inside the workspace.  Here
    // we compute a radial distance that also respects the minimum reachable
    // distance considering the current robot height.
    float safe_reach = leg_reach * 0.65f;
    float min_xy = sqrtf(std::max(0.0f, min_reach * min_reach - robot_height * robot_height));

    float desired_radius = std::max(safe_reach,
                                    p.hexagon_radius + min_xy + step_length * WORKSPACE_SCALING_FACTOR);

    float default_foot_x = desired_radius * cos(math_utils::degreesToRadians(base_angle));
    float default_foot_y = desired_radius * sin(math_utils::degreesToRadians(base_angle));

    if (leg_phase < stance_duration) {
        leg_states[leg_index] = STANCE_PHASE;
        float support_progress = leg_phase / stance_duration;
        float stance_radius = desired_radius +
                              step_length * WORKSPACE_SCALING_FACTOR * (1.0f - 2.0f * support_progress);
        trajectory.x = stance_radius * cos(math_utils::degreesToRadians(base_angle));
        trajectory.y = stance_radius * sin(math_utils::degreesToRadians(base_angle));
        trajectory.z = -robot_height;
    } else {
        leg_states[leg_index] = SWING_PHASE;
        float swing_progress = (leg_phase - stance_duration) / swing_duration;

        Eigen::Vector3f ctrl[5];
        float start_radius = std::max(desired_radius - step_length * WORKSPACE_SCALING_FACTOR,
                                      p.hexagon_radius + min_xy);
        float end_radius = desired_radius + step_length * WORKSPACE_SCALING_FACTOR;
        float start_x = start_radius * cos(math_utils::degreesToRadians(base_angle));
        float end_x = end_radius * cos(math_utils::degreesToRadians(base_angle));
        float start_y = start_radius * sin(math_utils::degreesToRadians(base_angle));
        float end_y = end_radius * sin(math_utils::degreesToRadians(base_angle));
        float z_base = -robot_height;
        ctrl[0] = Eigen::Vector3f(start_x, start_y, z_base);
        ctrl[1] = Eigen::Vector3f(start_x, start_y, z_base + step_height * WORKSPACE_SCALING_FACTOR);
        ctrl[2] = Eigen::Vector3f((start_x + end_x) / ANGULAR_ACCELERATION_FACTOR,
                                  (start_y + end_y) / ANGULAR_ACCELERATION_FACTOR,
                                  z_base + step_height);
        ctrl[3] = Eigen::Vector3f(end_x, end_y, z_base + step_height * WORKSPACE_SCALING_FACTOR);
        ctrl[4] = Eigen::Vector3f(end_x, end_y, z_base);
        Eigen::Vector3f pos = math_utils::quarticBezier(ctrl, swing_progress);
        trajectory.x = pos[0];
        trajectory.y = pos[1];
        trajectory.z = pos[2];

        // Apply terrain adaptation for swing phase
        trajectory = terrain_adaptation_.adaptTrajectoryForTerrain(leg_index, trajectory,
                                                                   leg_states[leg_index], swing_progress);
    }

    // SIMPLIFIED & UNIFIED: Use WorkspaceValidator for all validation
    // This replaces the old scattered validation code with a single, optimized system
    auto validation_result = workspace_validator_->validateTarget(
        leg_index, trajectory, current_leg_positions_, true);

    // Use the validated and constrained position
    trajectory = validation_result.constrained_position;

    // Update current leg position for future collision checks
    current_leg_positions_[leg_index] = trajectory;

    return trajectory;
}

// Terrain adaptation methods
void WalkController::enableRoughTerrainMode(bool enabled, bool force_normal_touchdown,
                                            bool proactive_adaptation) {
    terrain_adaptation_.setRoughTerrainMode(enabled);
    terrain_adaptation_.setForceNormalTouchdown(force_normal_touchdown);

    if (proactive_adaptation) {
        // Enable proactive terrain adaptation features
        terrain_adaptation_.setGravityAlignedTips(true);
    }
}

void WalkController::enableForceNormalTouchdown(bool enabled) {
    terrain_adaptation_.setForceNormalTouchdown(enabled);
}

void WalkController::enableGravityAlignedTips(bool enabled) {
    terrain_adaptation_.setGravityAlignedTips(enabled);
}

void WalkController::setExternalTarget(int leg_index, const TerrainAdaptation::ExternalTarget &target) {
    terrain_adaptation_.setExternalTarget(leg_index, target);
}

void WalkController::setExternalDefault(int leg_index, const TerrainAdaptation::ExternalTarget &default_pos) {
    terrain_adaptation_.setExternalDefault(leg_index, default_pos);
}

// Terrain state accessors
const TerrainAdaptation::WalkPlane &WalkController::getWalkPlane() const {
    return terrain_adaptation_.getWalkPlane();
}

const TerrainAdaptation::ExternalTarget &WalkController::getExternalTarget(int leg_index) const {
    return terrain_adaptation_.getExternalTarget(leg_index);
}

const TerrainAdaptation::StepPlane &WalkController::getStepPlane(int leg_index) const {
    return terrain_adaptation_.getStepPlane(leg_index);
}

bool WalkController::hasTouchdownDetection(int leg_index) const {
    return terrain_adaptation_.hasTouchdownDetection(leg_index);
}

Eigen::Vector3f WalkController::estimateGravity() const {
    return terrain_adaptation_.estimateGravity();
}

// Velocity limiting methods
VelocityLimits::LimitValues WalkController::getVelocityLimits(float bearing_degrees) const {
    return velocity_limits_.getLimit(bearing_degrees);
}

VelocityLimits::LimitValues WalkController::applyVelocityLimits(float vx, float vy, float omega) const {
    // Calculate bearing from velocity components
    float bearing = VelocityLimits::calculateBearing(vx, vy);

    // Get limits for this bearing
    VelocityLimits::LimitValues limits = velocity_limits_.getLimit(bearing);

    // Apply limits to input velocities
    VelocityLimits::LimitValues limited_velocities;
    limited_velocities.linear_x = std::max(-limits.linear_x, std::min(limits.linear_x, vx));
    limited_velocities.linear_y = std::max(-limits.linear_y, std::min(limits.linear_y, vy));
    limited_velocities.angular_z = std::max(-limits.angular_z, std::min(limits.angular_z, omega));
    limited_velocities.acceleration = limits.acceleration;

    return limited_velocities;
}

bool WalkController::validateVelocityCommand(float vx, float vy, float omega) const {
    return velocity_limits_.validateVelocityInputs(vx, vy, omega);
}

void WalkController::updateVelocityLimits(float frequency, float stance_ratio, float time_to_max_stride) {
    VelocityLimits::GaitConfig gait_config;
    gait_config.frequency = frequency;
    gait_config.stance_ratio = stance_ratio;
    gait_config.swing_ratio = DEFAULT_ANGULAR_SCALING - stance_ratio;
    gait_config.time_to_max_stride = time_to_max_stride;

    velocity_limits_.updateGaitParameters(gait_config);
}

void WalkController::setVelocitySafetyMargin(float margin) {
    velocity_limits_.setSafetyMargin(margin);
}

void WalkController::setAngularVelocityScaling(float scaling) {
    velocity_limits_.setAngularVelocityScaling(scaling);
}

const VelocityLimits::LimitValues &WalkController::getCurrentVelocities() const {
    return current_velocities_;
}

VelocityLimits::WorkspaceConfig WalkController::getWorkspaceConfig() const {
    return velocity_limits_.getWorkspaceConfig();
}
