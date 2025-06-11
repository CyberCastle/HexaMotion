#include "manual_pose_controller.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

ManualPoseController::ManualPoseController(RobotModel &model)
    : model_(model), current_mode_(POSE_TRANSLATION), interpolation_speed_(0.1f),
      smooth_transitions_(true) {
}

void ManualPoseController::initialize() {
    initializePoseLimits();
    initializeDefaultPresets();
    resetPose();
}

void ManualPoseController::setPoseMode(PoseMode mode) {
    current_mode_ = mode;
}

void ManualPoseController::processInput(float x, float y, float z) {
    switch (current_mode_) {
    case POSE_TRANSLATION:
        handleTranslationInput(x, y, z);
        break;
    case POSE_ROTATION:
        handleRotationInput(x, y, z);
        break;
    case POSE_LEG_INDIVIDUAL:
        // Use x as leg index, y and z as position adjustments
        handleIndividualLegInput(static_cast<int>(x), 0.0f, y, z);
        break;
    case POSE_BODY_HEIGHT:
        handleHeightInput(z);
        break;
    case POSE_COMBINED:
        handleCombinedInput(x, y, z);
        break;
    case POSE_MANUAL_BODY:
        // Manual body pose combines translation and rotation
        handleTranslationInput(x * 0.7f, y * 0.7f, 0);
        handleRotationInput(x * 0.3f, y * 0.3f, z);
        break;
    case POSE_CUSTOM:
        // Custom pose mode - implementation depends on specific requirements
        break;
    }

    // Apply constraints
    constrainTranslation(current_pose_.body_position);
    constrainRotation(current_pose_.body_rotation);
    constrainHeight(current_pose_.body_height);
}

void ManualPoseController::processInputExtended(float x, float y, float z, float aux) {
    switch (current_mode_) {
    case POSE_LEG_INDIVIDUAL:
        handleIndividualLegInput(static_cast<int>(aux), x, y, z);
        break;
    case POSE_COMBINED:
        // Use aux as blend factor between translation and rotation
        handleTranslationInput(x * aux, y * aux, z * aux);
        handleRotationInput(x * (1.0f - aux), y * (1.0f - aux), z * (1.0f - aux));
        break;
    default:
        processInput(x, y, z);
        break;
    }
}

void ManualPoseController::setTargetPose(const PoseState &target) {
    target_pose_ = target;

    // Constrain target pose
    constrainTranslation(target_pose_.body_position);
    constrainRotation(target_pose_.body_rotation);
    constrainHeight(target_pose_.body_height);

    for (int i = 0; i < NUM_LEGS; i++) {
        constrainLegPosition(i, target_pose_.leg_positions[i]);
    }
}

void ManualPoseController::updatePoseInterpolation(float dt) {
    if (!smooth_transitions_) {
        current_pose_ = target_pose_;
        return;
    }

    float alpha = std::min(1.0f, interpolation_speed_ * dt * 60.0f); // Normalize to 60 FPS

    // Interpolate body position
    current_pose_.body_position = interpolatePoint3D(current_pose_.body_position,
                                                     target_pose_.body_position, alpha);

    // Interpolate body rotation
    current_pose_.body_rotation = interpolatePoint3D(current_pose_.body_rotation,
                                                     target_pose_.body_rotation, alpha);

    // Interpolate body height
    current_pose_.body_height = interpolateFloat(current_pose_.body_height,
                                                 target_pose_.body_height, alpha);

    // Interpolate leg positions
    for (int i = 0; i < NUM_LEGS; i++) {
        current_pose_.leg_positions[i] = interpolatePoint3D(current_pose_.leg_positions[i],
                                                            target_pose_.leg_positions[i], alpha);
    }

    // Interpolate blend factor
    current_pose_.pose_blend_factor = interpolateFloat(current_pose_.pose_blend_factor,
                                                       target_pose_.pose_blend_factor, alpha);
}

void ManualPoseController::resetPose() {
    current_pose_ = PoseState();
    target_pose_ = PoseState();

    // Calculate default leg positions
    calculateDefaultLegPositions();
}

void ManualPoseController::setSmoothTransitions(bool enable, float speed) {
    smooth_transitions_ = enable;
    interpolation_speed_ = std::max(0.01f, std::min(1.0f, speed));
}

void ManualPoseController::savePosePreset(const std::string &name) {
    pose_presets_[name] = current_pose_;
}

bool ManualPoseController::loadPosePreset(const std::string &name) {
    auto it = pose_presets_.find(name);
    if (it != pose_presets_.end()) {
        setTargetPose(it->second);
        return true;
    }
    return false;
}

std::vector<std::string> ManualPoseController::getPosePresetNames() const {
    std::vector<std::string> names;
    for (const auto &preset : pose_presets_) {
        names.push_back(preset.first);
    }
    return names;
}

bool ManualPoseController::applyPose(const PoseState &pose, Point3D leg_positions[NUM_LEGS],
                                     JointAngles joint_angles[NUM_LEGS]) {
    bool success = true;

    // Calculate leg positions with body pose applied
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D default_pos = calculateDefaultLegPosition(i, pose.body_height);

        // Apply body position offset
        Point3D offset_pos = default_pos + pose.body_position;

        // Apply body rotation (simplified - would need proper transformation matrix)
        // For now, just apply as small rotational offsets
        float cos_yaw = cos(pose.body_rotation.z);
        float sin_yaw = sin(pose.body_rotation.z);

        Point3D rotated_pos;
        rotated_pos.x = offset_pos.x * cos_yaw - offset_pos.y * sin_yaw;
        rotated_pos.y = offset_pos.x * sin_yaw + offset_pos.y * cos_yaw;
        rotated_pos.z = offset_pos.z;

        // Apply individual leg adjustments
        leg_positions[i] = rotated_pos + pose.leg_positions[i];

        // Calculate joint angles
        joint_angles[i] = model_.inverseKinematics(i, leg_positions[i]);

        // Check if solution is valid
        if (!model_.checkJointLimits(i, joint_angles[i])) {
            success = false;
        }
    }

    return success;
}

void ManualPoseController::initializePoseLimits() {
    const Parameters &params = model_.getParams();

    // Set reasonable pose limits based on robot geometry
    pose_limits_.translation_limits = Point3D(50.0f, 50.0f, 30.0f); // ±50mm X,Y, ±30mm Z
    pose_limits_.rotation_limits = Point3D(0.524f, 0.524f, M_PI);   // ±30° roll/pitch, ±180° yaw
    pose_limits_.height_min = 50.0f;
    pose_limits_.height_max = 150.0f;
    pose_limits_.leg_reach_limit = params.femur_length + params.tibia_length;
}

void ManualPoseController::initializeDefaultPresets() {
    // Create default pose presets
    PoseState neutral_pose;
    neutral_pose.body_height = 90.0f;
    pose_presets_["neutral"] = neutral_pose;

    PoseState high_pose;
    high_pose.body_height = 120.0f;
    pose_presets_["high"] = high_pose;

    PoseState low_pose;
    low_pose.body_height = 60.0f;
    pose_presets_["low"] = low_pose;

    PoseState forward_lean;
    forward_lean.body_rotation.y = 0.174f; // 10 degrees
    forward_lean.body_height = 90.0f;
    pose_presets_["forward_lean"] = forward_lean;
}

void ManualPoseController::handleTranslationInput(float x, float y, float z) {
    current_pose_.body_position.x += x * input_scaling_.translation_scale;
    current_pose_.body_position.y += y * input_scaling_.translation_scale;
    current_pose_.body_position.z += z * input_scaling_.translation_scale;
}

void ManualPoseController::handleRotationInput(float roll, float pitch, float yaw) {
    current_pose_.body_rotation.x += roll * input_scaling_.rotation_scale;
    current_pose_.body_rotation.y += pitch * input_scaling_.rotation_scale;
    current_pose_.body_rotation.z += yaw * input_scaling_.rotation_scale;
}

void ManualPoseController::handleIndividualLegInput(int leg_index, float x, float y, float z) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        current_pose_.leg_positions[leg_index].x += x * input_scaling_.leg_scale;
        current_pose_.leg_positions[leg_index].y += y * input_scaling_.leg_scale;
        current_pose_.leg_positions[leg_index].z += z * input_scaling_.leg_scale;

        constrainLegPosition(leg_index, current_pose_.leg_positions[leg_index]);
    }
}

void ManualPoseController::handleHeightInput(float delta_height) {
    current_pose_.body_height += delta_height * input_scaling_.height_scale;
}

void ManualPoseController::handleCombinedInput(float x, float y, float z) {
    // Split input between translation and rotation
    handleTranslationInput(x * 0.6f, y * 0.6f, 0);
    handleRotationInput(x * 0.4f, y * 0.4f, z);
}

void ManualPoseController::constrainTranslation(Point3D &translation) {
    translation.x = std::max(-pose_limits_.translation_limits.x,
                             std::min(pose_limits_.translation_limits.x, translation.x));
    translation.y = std::max(-pose_limits_.translation_limits.y,
                             std::min(pose_limits_.translation_limits.y, translation.y));
    translation.z = std::max(-pose_limits_.translation_limits.z,
                             std::min(pose_limits_.translation_limits.z, translation.z));
}

void ManualPoseController::constrainRotation(Point3D &rotation) {
    rotation.x = std::max(-pose_limits_.rotation_limits.x,
                          std::min(pose_limits_.rotation_limits.x, rotation.x));
    rotation.y = std::max(-pose_limits_.rotation_limits.y,
                          std::min(pose_limits_.rotation_limits.y, rotation.y));
    rotation.z = std::max(-pose_limits_.rotation_limits.z,
                          std::min(pose_limits_.rotation_limits.z, rotation.z));
}

void ManualPoseController::constrainHeight(float &height) {
    height = std::max(pose_limits_.height_min, std::min(pose_limits_.height_max, height));
}

void ManualPoseController::constrainLegPosition(int leg_index, Point3D &position) {
    Point3D leg_origin = model_.getLegOrigin(leg_index);
    Point3D relative_pos = position - leg_origin;
    float distance = math_utils::magnitude(relative_pos);

    if (distance > pose_limits_.leg_reach_limit) {
        float scale = pose_limits_.leg_reach_limit / distance;
        relative_pos.x *= scale;
        relative_pos.y *= scale;
        relative_pos.z *= scale;
        position = leg_origin + relative_pos;
    }
}

Point3D ManualPoseController::interpolatePoint3D(const Point3D &from, const Point3D &to, float t) {
    return Point3D(
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t,
        from.z + (to.z - from.z) * t);
}

float ManualPoseController::interpolateFloat(float from, float to, float t) {
    return from + (to - from) * t;
}

void ManualPoseController::calculateDefaultLegPositions() {
    for (int i = 0; i < NUM_LEGS; i++) {
        current_pose_.leg_positions[i] = calculateDefaultLegPosition(i, current_pose_.body_height);
        target_pose_.leg_positions[i] = current_pose_.leg_positions[i];
    }
}

Point3D ManualPoseController::calculateDefaultLegPosition(int leg_index, float height) {
    const Parameters &params = model_.getParams();

    // Calculate default stance position for this leg
    float angle = leg_index * M_PI / 3.0f;       // 60 degrees between legs
    float radius = params.hexagon_radius * 0.8f; // Default stance radius

    return Point3D(
        radius * cos(angle),
        radius * sin(angle),
        -height);
}
