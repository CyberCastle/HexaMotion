#include "manual_body_pose_controller.h"
#include "hexamotion_constants.h"

/**
 * @file manual_body_pose_controller.cpp
 * @brief Implements manual pose control utilities.
 */
#include "math_utils.h"
#include <algorithm>
#include <cmath>

ManualPoseController::ManualPoseController(RobotModel &model)
    : model_(model), current_mode_(POSE_TRANSLATION),
      pose_limits_{Point3D(200.0f, 200.0f, 100.0f), Point3D(0.5f, 0.5f, 0.5f), 50.0f, 200.0f, 300.0f},
      interpolation_speed_(MIN_SERVO_VELOCITY), smooth_transitions_(true) {
}

void ManualPoseController::initialize() {
    initializePoseLimits();
    initializeDefaultPresets();
    resetPose();
}

void ManualPoseController::setPoseMode(PoseMode mode) {
    current_mode_ = mode;
}

void ManualPoseController::processInput(double x, double y, double z) {
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

void ManualPoseController::processInputExtended(double x, double y, double z, double aux) {
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

void ManualPoseController::updatePoseInterpolation(double dt) {
    if (!smooth_transitions_) {
        current_pose_ = target_pose_;
        return;
    }

    double alpha = std::min(DEFAULT_ANGULAR_SCALING, interpolation_speed_ * dt * 60.0); // Normalize to 60 FPS

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

void ManualPoseController::setSmoothTransitions(bool enable, double speed) {
    smooth_transitions_ = enable;
    interpolation_speed_ = std::max(0.01, std::min(DEFAULT_ANGULAR_SCALING, speed));
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

        // Apply complete body rotation using proper transformation matrix
        Point3D rotated_pos;
        if (pose.use_quaternion) {
            // Use quaternion rotation for more accurate transformation
            Eigen::Vector3d pos_vec(offset_pos.x, offset_pos.y, offset_pos.z);

            // Convert quaternion to rotation matrix and apply
            Eigen::Vector4d q = pose.body_quaternion;
            double w = q[0], x = q[1], y = q[2], z = q[3];

            Eigen::Matrix3d rot_matrix;
            rot_matrix << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);

            Eigen::Vector3d rotated_vec = rot_matrix * pos_vec;
            rotated_pos = Point3D(rotated_vec[0], rotated_vec[1], rotated_vec[2]);
        } else {
            // Use Euler angle rotation with complete 3D transformation
            Eigen::Vector3d rotation_rad(pose.body_rotation.x, pose.body_rotation.y, pose.body_rotation.z);
            rotated_pos = math_utils::rotatePoint(offset_pos, rotation_rad);
        }

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
    neutral_pose.body_height = DEFAULT_MAX_ANGULAR_VELOCITY;
    pose_presets_["neutral"] = neutral_pose;

    PoseState high_pose;
    high_pose.body_height = 120.0f;
    pose_presets_["high"] = high_pose;

    PoseState low_pose;
    low_pose.body_height = 0.0;
    pose_presets_["low"] = low_pose;

    PoseState forward_lean;
    forward_lean.body_rotation.y = 0.174f; // 10 degrees
    forward_lean.body_height = DEFAULT_MAX_ANGULAR_VELOCITY;
    pose_presets_["forward_lean"] = forward_lean;
}

void ManualPoseController::handleTranslationInput(double x, double y, double z) {
    current_pose_.body_position.x += x * input_scaling_.translation_scale;
    current_pose_.body_position.y += y * input_scaling_.translation_scale;
    current_pose_.body_position.z += z * input_scaling_.translation_scale;
}

void ManualPoseController::handleRotationInput(double roll, double pitch, double yaw) {
    current_pose_.body_rotation.x += roll * input_scaling_.rotation_scale;
    current_pose_.body_rotation.y += pitch * input_scaling_.rotation_scale;
    current_pose_.body_rotation.z += yaw * input_scaling_.rotation_scale;
}

void ManualPoseController::handleIndividualLegInput(int leg_index, double x, double y, double z) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        current_pose_.leg_positions[leg_index].x += x * input_scaling_.leg_scale;
        current_pose_.leg_positions[leg_index].y += y * input_scaling_.leg_scale;
        current_pose_.leg_positions[leg_index].z += z * input_scaling_.leg_scale;

        constrainLegPosition(leg_index, current_pose_.leg_positions[leg_index]);
    }
}

void ManualPoseController::handleHeightInput(double delta_height) {
    current_pose_.body_height += delta_height * input_scaling_.height_scale;
}

void ManualPoseController::handleCombinedInput(double x, double y, double z) {
    // Split input between translation and rotation
    handleTranslationInput(x * 0.6f, y * 0.6f, 0);
    handleRotationInput(x * 0.4f, y * 0.4f, z);
}

void ManualPoseController::constrainTranslation(Point3D &translation) const {
    translation.x = std::max(-pose_limits_.translation_limits.x,
                             std::min(pose_limits_.translation_limits.x, translation.x));
    translation.y = std::max(-pose_limits_.translation_limits.y,
                             std::min(pose_limits_.translation_limits.y, translation.y));
    translation.z = std::max(-pose_limits_.translation_limits.z,
                             std::min(pose_limits_.translation_limits.z, translation.z));
}

void ManualPoseController::constrainRotation(Point3D &rotation) const {
    rotation.x = std::max(-pose_limits_.rotation_limits.x,
                          std::min(pose_limits_.rotation_limits.x, rotation.x));
    rotation.y = std::max(-pose_limits_.rotation_limits.y,
                          std::min(pose_limits_.rotation_limits.y, rotation.y));
    rotation.z = std::max(-pose_limits_.rotation_limits.z,
                          std::min(pose_limits_.rotation_limits.z, rotation.z));
}

void ManualPoseController::constrainHeight(double &height) const {
    height = std::max(pose_limits_.height_min, std::min(pose_limits_.height_max, height));
}

void ManualPoseController::constrainLegPosition(int leg_index, Point3D &position) {
    // Get leg origin using frame transformation with zero joint angles
    JointAngles zero_angles(0, 0, 0);
    Pose leg_origin_pose = model_.getPoseRobotFrame(leg_index, zero_angles, Pose::Identity());
    Point3D leg_origin = leg_origin_pose.position;

    Point3D relative_pos = position - leg_origin;
    double distance = math_utils::magnitude(relative_pos);

    if (distance > pose_limits_.leg_reach_limit) {
        double scale = pose_limits_.leg_reach_limit / distance;
        relative_pos.x *= scale;
        relative_pos.y *= scale;
        relative_pos.z *= scale;
        position = leg_origin + relative_pos;
    }
}

Point3D ManualPoseController::interpolatePoint3D(const Point3D &from, const Point3D &to, double t) {
    return Point3D(
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t,
        from.z + (to.z - from.z) * t);
}

double ManualPoseController::interpolateFloat(double from, double to, double t) {
    return from + (to - from) * t;
}

void ManualPoseController::calculateDefaultLegPositions() {
    for (int i = 0; i < NUM_LEGS; i++) {
        current_pose_.leg_positions[i] = calculateDefaultLegPosition(i, current_pose_.body_height);
        target_pose_.leg_positions[i] = current_pose_.leg_positions[i];
    }
}

Point3D ManualPoseController::calculateDefaultLegPosition(int leg_index, double height) {
    const Parameters &params = model_.getParams();

    // Calculate default stance position for this leg
    double angle = leg_index * M_PI / SERVO_SPEED_MAX; // 60 degrees between legs
    double radius = params.hexagon_radius * 0.8f;      // Default stance radius

    return Point3D(
        radius * cos(angle),
        radius * sin(angle),
        -height);
}

void ManualPoseController::setPoseQuaternion(const Point3D &position, const Eigen::Vector4d &quaternion, double blend_factor) {
    target_pose_.body_position = position;
    target_pose_.body_quaternion = quaternion;
    target_pose_.pose_blend_factor = blend_factor;
    target_pose_.use_quaternion = true;
    target_pose_.pose_active = true;

    // Update Euler representation for compatibility
    target_pose_.body_rotation = math_utils::quaternionToEulerPoint3D(quaternion);

    // Also update current pose immediately for consistency
    current_pose_.body_position = position;
    current_pose_.body_quaternion = quaternion;
    current_pose_.pose_blend_factor = blend_factor;
    current_pose_.use_quaternion = true;
    current_pose_.pose_active = true;
    current_pose_.body_rotation = math_utils::quaternionToEulerPoint3D(quaternion);
}

void ManualPoseController::interpolateToQuaternionPose(const Point3D &target_pos, const Eigen::Vector4d &target_quat, double speed) {
    if (!smooth_transitions_ || speed >= DEFAULT_ANGULAR_SCALING) {
        setPoseQuaternion(target_pos, target_quat, DEFAULT_ANGULAR_SCALING);
        current_pose_ = target_pose_;
        return;
    }

    // Clamp speed
    speed = std::max(0.0, std::min(DEFAULT_ANGULAR_SCALING, speed));

    // Linear interpolation for position
    current_pose_.body_position.x += speed * (target_pos.x - current_pose_.body_position.x);
    current_pose_.body_position.y += speed * (target_pos.y - current_pose_.body_position.y);
    current_pose_.body_position.z += speed * (target_pos.z - current_pose_.body_position.z);

    // Spherical linear interpolation for quaternion
    Eigen::Vector4d current_quat = getCurrentQuaternion();

    // Simple SLERP implementation
    double dot = current_quat[0] * target_quat[0] + current_quat[1] * target_quat[1] +
                current_quat[2] * target_quat[2] + current_quat[3] * target_quat[3];

    // Take shorter path
    Eigen::Vector4d target_adjusted = target_quat;
    if (dot < 0.0f) {
        target_adjusted = -target_quat;
        dot = -dot;
    }

    // Linear interpolation for very close quaternions
    if (dot > 0.9995f) {
        current_pose_.body_quaternion = current_quat + speed * (target_adjusted - current_quat);
        // Normalize
        double norm = sqrt(current_pose_.body_quaternion[0] * current_pose_.body_quaternion[0] +
                          current_pose_.body_quaternion[1] * current_pose_.body_quaternion[1] +
                          current_pose_.body_quaternion[2] * current_pose_.body_quaternion[2] +
                          current_pose_.body_quaternion[3] * current_pose_.body_quaternion[3]);
        if (norm > 0.0f) {
            current_pose_.body_quaternion = current_pose_.body_quaternion / norm;
        }
    } else {
        // Spherical interpolation
        double theta_0 = acos(std::abs(dot));
        double sin_theta_0 = sin(theta_0);
        double theta = theta_0 * speed;
        double sin_theta = sin(theta);

        double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;

        current_pose_.body_quaternion = s0 * current_quat + s1 * target_adjusted;
    }

    current_pose_.use_quaternion = true;
    current_pose_.pose_active = true;

    // Update Euler representation for compatibility
    current_pose_.body_rotation = math_utils::quaternionToEulerPoint3D(current_pose_.body_quaternion);
}

Eigen::Vector4d ManualPoseController::getCurrentQuaternion() const {
    if (current_pose_.use_quaternion) {
        return current_pose_.body_quaternion;
    } else {
        // Convert from Euler angles
        return math_utils::eulerPoint3DToQuaternion(current_pose_.body_rotation);
    }
}

void ManualPoseController::setUseQuaternion(bool use_quat) {
    if (use_quat && !current_pose_.use_quaternion) {
        // Convert current Euler to quaternion
        current_pose_.body_quaternion = math_utils::eulerPoint3DToQuaternion(current_pose_.body_rotation);
        target_pose_.body_quaternion = math_utils::eulerPoint3DToQuaternion(target_pose_.body_rotation);
    } else if (!use_quat && current_pose_.use_quaternion) {
        // Convert current quaternion to Euler
        current_pose_.body_rotation = math_utils::quaternionToEulerPoint3D(current_pose_.body_quaternion);
        target_pose_.body_rotation = math_utils::quaternionToEulerPoint3D(target_pose_.body_quaternion);
    }

    current_pose_.use_quaternion = use_quat;
    target_pose_.use_quaternion = use_quat;
}
