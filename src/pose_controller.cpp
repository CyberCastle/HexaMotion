#include "pose_controller.h"
#include "hexamotion_constants.h"
#include <cmath>
/**
 * @file pose_controller.cpp
 * @brief Implementation of the kinematic pose controller.
 */

PoseController::PoseController(RobotModel &m, const PoseConfiguration &config)
    : model(m), pose_config(config), trajectory_in_progress(false), trajectory_progress(0.0f), trajectory_step_count(0) {
    // Initialize trajectory arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = Point3D(0, 0, 0);
        trajectory_start_angles[i] = JointAngles(0, 0, 0);
        trajectory_target_positions[i] = Point3D(0, 0, 0);
        trajectory_target_angles[i] = JointAngles(0, 0, 0);
    }
}

bool PoseController::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                 Leg legs[NUM_LEGS]) {
    // OpenSHC-style: Check pose limits before applying
    if (!checkPoseLimits(position, orientation)) {
        return false; // Pose exceeds configured limits
    }

    // Use smooth trajectory as default behavior if enabled
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation) {
        return setBodyPoseSmooth(position, orientation, legs);
    }

    // Original implementation for compatibility when smooth trajectory is disabled
    return setBodyPoseImmediate(position, orientation, legs);
}

bool PoseController::setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]) {
    JointAngles angles = model.inverseKinematics(leg_index, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    legs[leg_index].setJointAngles(angles);
    legs[leg_index].setTipPosition(model.forwardKinematics(leg_index, angles));

    // Note: Servo commands are now handled by the calling LocomotionSystem through setLegJointAngles
    // This ensures consistency and proper state management

    return true;
}

// OpenSHC-style pose calculation using dynamic configuration
// Calculates leg positions based on pose configuration and desired body height
bool PoseController::calculatePoseFromConfig(double height_offset, Leg legs[NUM_LEGS]) {
    // Calculate Z position based on body clearance and height offset
    double target_z = -(pose_config.body_clearance * 1000.0f + height_offset); // Convert to mm and apply offset

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration (convert from meters to mm)
        double stance_x_mm = pose_config.leg_stance_positions[i].x * 1000.0f;
        double stance_y_mm = pose_config.leg_stance_positions[i].y * 1000.0f;

        Point3D target_pos;
        target_pos.x = stance_x_mm;
        target_pos.y = stance_y_mm;
        target_pos.z = target_z;

        // Calculate joint angles using inverse kinematics
        JointAngles angles = model.inverseKinematics(i, target_pos);

        // Check joint limits
        if (!model.checkJointLimits(i, angles)) {
            return false;
        }

        legs[i].setTipPosition(target_pos);
        legs[i].setJointAngles(angles);
    }

    return true;
}

void PoseController::initializeDefaultPose(Leg legs[NUM_LEGS]) {
    // OpenSHC-style: Use configuration-based calculation instead of direct geometry
    calculatePoseFromConfig(0.0f, legs); // No height offset for default pose
}

bool PoseController::setStandingPose(Leg legs[NUM_LEGS]) {
    // OpenSHC-style: Standing pose uses pre-configured joint angles, not calculated positions
    // This ensures coxa ≈ 0° and femur/tibia equal for all legs (symmetric posture)

    for (int i = 0; i < NUM_LEGS; i++) {
        // Use configured standing pose joint angles
        JointAngles angles;
        angles.coxa = pose_config.standing_pose_joints[i].coxa;
        angles.femur = pose_config.standing_pose_joints[i].femur;
        angles.tibia = pose_config.standing_pose_joints[i].tibia;

        // Check joint limits
        angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                           model.getParams().coxa_angle_limits[1]);
        angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                            model.getParams().femur_angle_limits[1]);
        angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                            model.getParams().tibia_angle_limits[1]);

        // Calculate resulting position using forward kinematics
        legs[i].setTipPosition(model.forwardKinematics(i, angles));
        legs[i].setJointAngles(angles);

        // Note: Servo commands are now handled by the calling LocomotionSystem through setLegJointAngles
        // This ensures consistency and proper state management
    }

    return true;
}

bool PoseController::setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                           Leg legs[NUM_LEGS]) {
    // Use smooth trajectory with quaternion if enabled
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation &&
        model.getParams().smooth_trajectory.use_quaternion_slerp) {
        return setBodyPoseSmoothQuaternion(position, quaternion, legs);
    }

    // Original implementation for compatibility
    Point3D euler_rad = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3d orientation(
        math_utils::radiansToDegrees(euler_rad.x),
        math_utils::radiansToDegrees(euler_rad.y),
        math_utils::radiansToDegrees(euler_rad.z));
    return setBodyPose(position, orientation, legs);
}

bool PoseController::interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
                                     const Eigen::Vector3d &end_pos, const Eigen::Vector4d &end_quat,
                                     double t, Leg legs[NUM_LEGS]) {
    // Clamp interpolation parameter
    t = std::max(0.0, std::min(DEFAULT_ANGULAR_SCALING, t));

    // Linear interpolation for position
    Eigen::Vector3d interp_pos = start_pos + t * (end_pos - start_pos);

    // Spherical linear interpolation (SLERP) for quaternion
    Eigen::Vector4d interp_quat = quaternionSlerp(start_quat, end_quat, t);

    return setBodyPoseQuaternion(interp_pos, interp_quat, legs);
}

// Quaternion spherical linear interpolation (SLERP)
Eigen::Vector4d PoseController::quaternionSlerp(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, double t) {
    // Compute dot product
    double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // If dot product is negative, take the shorter path by negating one quaternion
    Eigen::Vector4d q2_adj = q2;
    if (dot < 0.0f) {
        q2_adj = -q2;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation to avoid numerical issues
    if (dot > 0.9995f) {
        Eigen::Vector4d result = q1 + t * (q2_adj - q1);
        double norm = sqrt(result[0] * result[0] + result[1] * result[1] +
                          result[2] * result[2] + result[3] * result[3]);
        if (norm > 0.0f) {
            result = result / norm;
        }
        return result;
    }

    // Calculate angle and perform SLERP
    double theta_0 = acos(std::abs(dot));
    double sin_theta_0 = sin(theta_0);

    double theta = theta_0 * t;
    double sin_theta = sin(theta);

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    return s0 * q1 + s1 * q2_adj;
}

bool PoseController::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                       Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Check if smooth trajectory is enabled
    if (!model.getParams().smooth_trajectory.use_current_servo_positions) {
        // Fall back to original non-smooth method
        return setBodyPoseImmediate(position, orientation, legs);
    }

    // If not already in progress, initialize trajectory from current servo positions
    if (!trajectory_in_progress) {
        return initializeTrajectoryFromCurrent(position, orientation, legs, servos);
    } else {
        // Continue existing trajectory
        return updateTrajectoryStep(legs);
    }
}

bool PoseController::setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                                 Leg legs[NUM_LEGS]) {
    // Convert quaternion to Euler angles and use smooth method
    Point3D euler_rad = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3d orientation(
        math_utils::radiansToDegrees(euler_rad.x),
        math_utils::radiansToDegrees(euler_rad.y),
        math_utils::radiansToDegrees(euler_rad.z));
    return setBodyPoseSmooth(position, orientation, legs);
}

bool PoseController::getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]) {
    if (!servos) {
        return false;
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        // Read current servo angles
        JointAngles angles;
        angles.coxa = servos->getJointAngle(i, 0);
        angles.femur = servos->getJointAngle(i, 1);
        angles.tibia = servos->getJointAngle(i, 2);

        // Calculate corresponding leg positions using forward kinematics
        legs[i].setJointAngles(angles);
        legs[i].setTipPosition(model.forwardKinematics(i, angles));
    }

    return true;
}

bool PoseController::initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                                     const Eigen::Vector3d &target_orientation,
                                                     Leg legs[NUM_LEGS], IServoInterface *servos) {
    // OpenSHC-style: Check pose limits before starting trajectory
    if (!checkPoseLimits(target_position, target_orientation)) {
        return false; // Target pose exceeds configured limits
    }

    // Get current servo positions as starting point if servo interface is provided
    if (servos && !getCurrentServoPositions(servos, legs)) {
        // If can't read current positions, fall back to passed positions
        for (int i = 0; i < NUM_LEGS; i++) {
            trajectory_start_positions[i] = legs[i].getTipPosition();
            trajectory_start_angles[i] = legs[i].getJointAngles();
        }
    } else if (!servos) {
        // Use passed positions as starting point when no servo interface is provided
        for (int i = 0; i < NUM_LEGS; i++) {
            trajectory_start_positions[i] = legs[i].getTipPosition();
            trajectory_start_angles[i] = legs[i].getJointAngles();
        }
    }

    // Calculate target leg positions using OpenSHC-style configuration-based calculation
    // Create temporary leg objects for target calculation
    Leg temp_legs[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_legs[i].setTipPosition(trajectory_start_positions[i]);
        temp_legs[i].setJointAngles(trajectory_start_angles[i]);
    }

    // Calculate target pose using OpenSHC-style method (bypassing smooth trajectory to avoid recursion)
    if (!setBodyPoseImmediate(target_position, target_orientation, temp_legs)) {
        return false;
    }

    // Store target positions
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_target_positions[i] = temp_legs[i].getTipPosition();
        trajectory_target_angles[i] = temp_legs[i].getJointAngles();
    }

    // Initialize trajectory state
    trajectory_in_progress = true;
    trajectory_progress = 0.0f;
    trajectory_step_count = 0;

    return updateTrajectoryStep(legs);
}

bool PoseController::updateTrajectoryStep(Leg legs[NUM_LEGS]) {
    const auto &config = model.getParams().smooth_trajectory;

    // Calculate interpolation progress
    trajectory_step_count++;
    trajectory_progress = static_cast<double>(trajectory_step_count) /
                         static_cast<double>(config.max_interpolation_steps);
    trajectory_progress =
        std::min(DEFAULT_ANGULAR_SCALING,
                 trajectory_progress * config.interpolation_speed * 10.0); // Scale for responsiveness

    // Interpolate each leg position and angles
    for (int i = 0; i < NUM_LEGS; i++) {
        // Linear interpolation for positions
        Point3D new_position;
        new_position.x = trajectory_start_positions[i].x +
                         trajectory_progress * (trajectory_target_positions[i].x - trajectory_start_positions[i].x);
        new_position.y = trajectory_start_positions[i].y +
                         trajectory_progress * (trajectory_target_positions[i].y - trajectory_start_positions[i].y);
        new_position.z = trajectory_start_positions[i].z +
                         trajectory_progress * (trajectory_target_positions[i].z - trajectory_start_positions[i].z);

        // Linear interpolation for joint angles
        JointAngles new_angles;
        new_angles.coxa = trajectory_start_angles[i].coxa +
                          trajectory_progress * (trajectory_target_angles[i].coxa - trajectory_start_angles[i].coxa);
        new_angles.femur = trajectory_start_angles[i].femur +
                           trajectory_progress * (trajectory_target_angles[i].femur - trajectory_start_angles[i].femur);
        new_angles.tibia = trajectory_start_angles[i].tibia +
                           trajectory_progress * (trajectory_target_angles[i].tibia - trajectory_start_angles[i].tibia);

        // Apply joint limits
        new_angles.coxa = model.constrainAngle(new_angles.coxa,
                                               model.getParams().coxa_angle_limits[0],
                                               model.getParams().coxa_angle_limits[1]);
        new_angles.femur = model.constrainAngle(new_angles.femur,
                                                model.getParams().femur_angle_limits[0],
                                                model.getParams().femur_angle_limits[1]);
        new_angles.tibia = model.constrainAngle(new_angles.tibia,
                                                model.getParams().tibia_angle_limits[0],
                                                model.getParams().tibia_angle_limits[1]);

        // Update leg object
        legs[i].setTipPosition(new_position);
        legs[i].setJointAngles(new_angles);

        // Note: Servo commands are now handled by the calling LocomotionSystem through setLegJointAngles
        // This ensures consistency and proper state management during smooth trajectory interpolation
    }

    // Check if trajectory is complete
    if (isTrajectoryComplete()) {
        trajectory_in_progress = false;
        trajectory_progress = 0.0f;
        trajectory_step_count = 0;
    }

    return true;
}

bool PoseController::isTrajectoryComplete() const {
    const auto &config = model.getParams().smooth_trajectory;

    // Check if we've reached maximum steps or full progress
    if (trajectory_step_count >= config.max_interpolation_steps || trajectory_progress >= DEFAULT_ANGULAR_SCALING) {
        return true;
    }

    // Note: Position tolerance checking removed since we no longer have access to servo interface
    // The calling code should handle position tolerance checking if needed

    return true;
}

bool PoseController::setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                          Leg legs[NUM_LEGS]) {
    // OpenSHC-style: Transform stance positions based on body pose
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Start with configured stance position (convert from meters to mm)
        Point3D stance_pos;
        stance_pos.x = pose_config.leg_stance_positions[i].x * 1000.0f;
        stance_pos.y = pose_config.leg_stance_positions[i].y * 1000.0f;
        stance_pos.z = -(pose_config.body_clearance * 1000.0f); // Body clearance

        // Apply body transformation: translate relative to body position, then rotate
        Point3D leg_body_relative(stance_pos.x - position[0], stance_pos.y - position[1], stance_pos.z - position[2]);
        Eigen::Vector3d orientation_rad(math_utils::degreesToRadians(orientation[0]),
                                       math_utils::degreesToRadians(orientation[1]),
                                       math_utils::degreesToRadians(orientation[2]));
        Point3D leg_rotated = math_utils::rotatePoint(leg_body_relative, orientation_rad);
        Point3D leg_world(position[0] + leg_rotated.x,
                          position[1] + leg_rotated.y,
                          position[2] + leg_rotated.z);

        JointAngles angles = model.inverseKinematics(i, leg_world);
        if (!model.checkJointLimits(i, angles))
            return false;
        legs[i].setJointAngles(angles);
        legs[i].setTipPosition(leg_world);

        // Note: Servo commands are now handled by the calling LocomotionSystem through setLegJointAngles
        // This ensures consistency and proper state management
    }
    return true;
}

// OpenSHC-style pose limit validation
bool PoseController::checkPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    // Convert position to meters for comparison with config limits
    double pos_x_m = position[0] / 1000.0f;
    double pos_y_m = position[1] / 1000.0f;
    double pos_z_m = position[2] / 1000.0f;

    // Check translation limits
    if (std::abs(pos_x_m) > pose_config.max_translation.x ||
        std::abs(pos_y_m) > pose_config.max_translation.y ||
        std::abs(pos_z_m) > pose_config.max_translation.z) {
        return false;
    }

    // Convert orientation from degrees to radians for comparison
    double roll_rad = math_utils::degreesToRadians(orientation[0]);
    double pitch_rad = math_utils::degreesToRadians(orientation[1]);
    double yaw_rad = math_utils::degreesToRadians(orientation[2]);

    // Check rotation limits
    if (std::abs(roll_rad) > pose_config.max_rotation.roll ||
        std::abs(pitch_rad) > pose_config.max_rotation.pitch ||
        std::abs(yaw_rad) > pose_config.max_rotation.yaw) {
        return false;
    }

    return true;
}

void PoseController::configureSmoothTrajectory(bool use_current_positions, double interpolation_speed, uint8_t max_steps) {
    // Configure smooth trajectory parameters (OpenSHC-equivalent global parameter modification)
    auto &config = const_cast<Parameters &>(model.getParams()).smooth_trajectory;
    config.use_current_servo_positions = use_current_positions;
    config.interpolation_speed =
        std::max(0.01, std::min(DEFAULT_ANGULAR_SCALING, interpolation_speed));
    config.max_interpolation_steps = std::max(static_cast<uint8_t>(1), std::min(static_cast<uint8_t>(100), max_steps));
}
