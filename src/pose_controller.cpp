#include "pose_controller.h"
#include "hexamotion_constants.h"
#include <cmath>
/**
 * @file pose_controller.cpp
 * @brief Implementation of the kinematic pose controller.
 */

PoseController::PoseController(RobotModel &m, IServoInterface *s, const PoseConfiguration &config)
    : model(m), servos(s), pose_config(config), trajectory_in_progress(false), trajectory_progress(0.0f), trajectory_step_count(0) {
    // Initialize trajectory arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = Point3D(0, 0, 0);
        trajectory_start_angles[i] = JointAngles(0, 0, 0);
        trajectory_target_positions[i] = Point3D(0, 0, 0);
        trajectory_target_angles[i] = JointAngles(0, 0, 0);
    }
}

bool PoseController::setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                                 Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    // OpenSHC-style: Check pose limits before applying
    if (!checkPoseLimits(position, orientation)) {
        return false; // Pose exceeds configured limits
    }

    // Use smooth trajectory as default behavior if enabled
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation) {
        return setBodyPoseSmooth(position, orientation, leg_pos, joint_q);
    }

    // Original implementation for compatibility when smooth trajectory is disabled
    return setBodyPoseImmediate(position, orientation, leg_pos, joint_q);
}

bool PoseController::setLegPosition(int leg_index, const Point3D &position,
                                    Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    JointAngles angles = model.inverseKinematics(leg_index, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    leg_pos[leg_index] = model.forwardKinematics(leg_index, angles);
    joint_q[leg_index] = angles;

    if (servos) {
        float speed = model.getParams().default_servo_speed;
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            // Check servo status flags before movement
            if (servos->hasBlockingStatusFlags(leg_index, j)) {
                return false; // Servo blocked by status flags
            }
            float angle = (j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));
            servos->setJointAngleAndSpeed(leg_index, j, angle, speed);
        }
    }

    return true;
}

// OpenSHC-style pose calculation using dynamic configuration
// Calculates leg positions based on pose configuration and desired body height
bool PoseController::calculatePoseFromConfig(float height_offset, Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    // Calculate Z position based on body clearance and height offset
    float target_z = -(pose_config.body_clearance * 1000.0f + height_offset); // Convert to mm and apply offset

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration (convert from meters to mm)
        float stance_x_mm = pose_config.leg_stance_positions[i].x * 1000.0f;
        float stance_y_mm = pose_config.leg_stance_positions[i].y * 1000.0f;

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

        leg_pos[i] = target_pos;
        joint_q[i] = angles;
    }

    return true;
}

void PoseController::initializeDefaultPose(Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS],
                                           float hex_radius, float robot_height) {
    // OpenSHC-style: Use configuration-based calculation instead of direct geometry
    // The hex_radius and robot_height parameters are kept for compatibility but
    // the actual calculation uses the dynamic pose configuration
    calculatePoseFromConfig(0.0f, leg_pos, joint_q); // No height offset for default pose
}

bool PoseController::setStandingPose(Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS], float robot_height) {
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
        leg_pos[i] = model.forwardKinematics(i, angles);
        joint_q[i] = angles;

        // Apply to servos if available
        if (servos) {
            float speed = model.getParams().default_servo_speed;
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                if (servos->hasBlockingStatusFlags(i, j)) {
                    return false; // Servo blocked by status flags
                }
                float angle = (j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));
                servos->setJointAngleAndSpeed(i, j, angle, speed);
            }
        }
    }

    return true;
}

bool PoseController::setBodyPoseQuaternion(const Eigen::Vector3f &position, const Eigen::Vector4f &quaternion,
                                           Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Use smooth trajectory with quaternion if enabled
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation &&
        model.getParams().smooth_trajectory.use_quaternion_slerp) {
        return setBodyPoseSmoothQuaternion(position, quaternion, leg_positions, joint_angles);
    }

    // Original implementation for compatibility
    Point3D euler_degrees = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3f orientation = math_utils::point3DToVector3f(euler_degrees);
    return setBodyPose(position, orientation, leg_positions, joint_angles);
}

bool PoseController::interpolatePose(const Eigen::Vector3f &start_pos, const Eigen::Vector4f &start_quat,
                                     const Eigen::Vector3f &end_pos, const Eigen::Vector4f &end_quat,
                                     float t, Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Clamp interpolation parameter
    t = std::max(0.0f, std::min(DEFAULT_ANGULAR_SCALING, t));

    // Linear interpolation for position
    Eigen::Vector3f interp_pos = start_pos + t * (end_pos - start_pos);

    // Spherical linear interpolation (SLERP) for quaternion
    Eigen::Vector4f interp_quat = quaternionSlerp(start_quat, end_quat, t);

    return setBodyPoseQuaternion(interp_pos, interp_quat, leg_positions, joint_angles);
}

// Quaternion spherical linear interpolation (SLERP)
Eigen::Vector4f PoseController::quaternionSlerp(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2, float t) {
    // Compute dot product
    float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // If dot product is negative, take the shorter path by negating one quaternion
    Eigen::Vector4f q2_adj = q2;
    if (dot < 0.0f) {
        q2_adj = -q2;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation to avoid numerical issues
    if (dot > 0.9995f) {
        Eigen::Vector4f result = q1 + t * (q2_adj - q1);
        float norm = sqrt(result[0] * result[0] + result[1] * result[1] +
                          result[2] * result[2] + result[3] * result[3]);
        if (norm > 0.0f) {
            result = result / norm;
        }
        return result;
    }

    // Calculate angle and perform SLERP
    float theta_0 = acos(std::abs(dot));
    float sin_theta_0 = sin(theta_0);

    float theta = theta_0 * t;
    float sin_theta = sin(theta);

    float s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    return s0 * q1 + s1 * q2_adj;
}

bool PoseController::setBodyPoseSmooth(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                                       Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Check if smooth trajectory is enabled
    if (!model.getParams().smooth_trajectory.use_current_servo_positions) {
        // Fall back to original non-smooth method
        return setBodyPoseImmediate(position, orientation, leg_positions, joint_angles);
    }

    // If not already in progress, initialize trajectory from current servo positions
    if (!trajectory_in_progress) {
        return initializeTrajectoryFromCurrent(position, orientation, leg_positions, joint_angles);
    } else {
        // Continue existing trajectory
        return updateTrajectoryStep(leg_positions, joint_angles);
    }
}

bool PoseController::setBodyPoseSmoothQuaternion(const Eigen::Vector3f &position, const Eigen::Vector4f &quaternion,
                                                 Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Convert quaternion to Euler angles and use smooth method
    Point3D euler_degrees = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3f orientation = math_utils::point3DToVector3f(euler_degrees);
    return setBodyPoseSmooth(position, orientation, leg_positions, joint_angles);
}

bool PoseController::getCurrentServoPositions(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    if (!servos) {
        return false;
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        // Read current servo angles
        joint_angles[i].coxa = servos->getJointAngle(i, 0);
        joint_angles[i].femur = servos->getJointAngle(i, 1);
        joint_angles[i].tibia = servos->getJointAngle(i, 2);

        // Calculate corresponding leg positions using forward kinematics
        leg_positions[i] = model.forwardKinematics(i, joint_angles[i]);
    }

    return true;
}

bool PoseController::initializeTrajectoryFromCurrent(const Eigen::Vector3f &target_position,
                                                     const Eigen::Vector3f &target_orientation,
                                                     Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // OpenSHC-style: Check pose limits before starting trajectory
    if (!checkPoseLimits(target_position, target_orientation)) {
        return false; // Target pose exceeds configured limits
    }

    // Get current servo positions as starting point (OpenSHC-style)
    if (!getCurrentServoPositions(trajectory_start_positions, trajectory_start_angles)) {
        // If can't read current positions, fall back to passed positions
        for (int i = 0; i < NUM_LEGS; i++) {
            trajectory_start_positions[i] = leg_positions[i];
            trajectory_start_angles[i] = joint_angles[i];
        }
    }

    // Calculate target leg positions using OpenSHC-style configuration-based calculation
    Point3D temp_leg_positions[NUM_LEGS];
    JointAngles temp_joint_angles[NUM_LEGS];

    // Copy current positions to temp arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_leg_positions[i] = trajectory_start_positions[i];
        temp_joint_angles[i] = trajectory_start_angles[i];
    }

    // Calculate target pose using OpenSHC-style method (bypassing smooth trajectory to avoid recursion)
    if (!setBodyPoseImmediate(target_position, target_orientation, temp_leg_positions, temp_joint_angles)) {
        return false;
    }

    // Store target positions
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_target_positions[i] = temp_leg_positions[i];
        trajectory_target_angles[i] = temp_joint_angles[i];
    }

    // Initialize trajectory state
    trajectory_in_progress = true;
    trajectory_progress = 0.0f;
    trajectory_step_count = 0;

    return updateTrajectoryStep(leg_positions, joint_angles);
}

bool PoseController::updateTrajectoryStep(Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    const auto &config = model.getParams().smooth_trajectory;

    // Calculate interpolation progress
    trajectory_step_count++;
    trajectory_progress = static_cast<float>(trajectory_step_count) / static_cast<float>(config.max_interpolation_steps);
    trajectory_progress = std::min(DEFAULT_ANGULAR_SCALING, trajectory_progress * config.interpolation_speed * 10.0f); // Scale for responsiveness

    // Interpolate each leg position and angles
    for (int i = 0; i < NUM_LEGS; i++) {
        // Linear interpolation for positions
        leg_positions[i].x = trajectory_start_positions[i].x +
                             trajectory_progress * (trajectory_target_positions[i].x - trajectory_start_positions[i].x);
        leg_positions[i].y = trajectory_start_positions[i].y +
                             trajectory_progress * (trajectory_target_positions[i].y - trajectory_start_positions[i].y);
        leg_positions[i].z = trajectory_start_positions[i].z +
                             trajectory_progress * (trajectory_target_positions[i].z - trajectory_start_positions[i].z);

        // Linear interpolation for joint angles
        joint_angles[i].coxa = trajectory_start_angles[i].coxa +
                               trajectory_progress * (trajectory_target_angles[i].coxa - trajectory_start_angles[i].coxa);
        joint_angles[i].femur = trajectory_start_angles[i].femur +
                                trajectory_progress * (trajectory_target_angles[i].femur - trajectory_start_angles[i].femur);
        joint_angles[i].tibia = trajectory_start_angles[i].tibia +
                                trajectory_progress * (trajectory_target_angles[i].tibia - trajectory_start_angles[i].tibia);

        // Apply joint limits
        joint_angles[i].coxa = model.constrainAngle(joint_angles[i].coxa,
                                                    model.getParams().coxa_angle_limits[0],
                                                    model.getParams().coxa_angle_limits[1]);
        joint_angles[i].femur = model.constrainAngle(joint_angles[i].femur,
                                                     model.getParams().femur_angle_limits[0],
                                                     model.getParams().femur_angle_limits[1]);
        joint_angles[i].tibia = model.constrainAngle(joint_angles[i].tibia,
                                                     model.getParams().tibia_angle_limits[0],
                                                     model.getParams().tibia_angle_limits[1]);

        // Send servo commands with smooth speed
        if (servos) {
            float speed = model.getParams().default_servo_speed * config.interpolation_speed;
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                // Check servo status flags before movement
                if (servos->hasBlockingStatusFlags(i, j)) {
                    return false; // Servo blocked by status flags
                }
                float angle = (j == 0 ? joint_angles[i].coxa : (j == 1 ? joint_angles[i].femur : joint_angles[i].tibia));
                servos->setJointAngleAndSpeed(i, j, angle, speed);
            }
        }
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

    // Check if all joints are within tolerance of target positions
    if (servos) {
        for (int i = 0; i < NUM_LEGS; i++) {
            // Check position tolerance (approximate)
            float pos_diff_x = std::abs(trajectory_target_positions[i].x - trajectory_start_positions[i].x) * (DEFAULT_ANGULAR_SCALING - trajectory_progress);
            float pos_diff_y = std::abs(trajectory_target_positions[i].y - trajectory_start_positions[i].y) * (DEFAULT_ANGULAR_SCALING - trajectory_progress);
            float pos_diff_z = std::abs(trajectory_target_positions[i].z - trajectory_start_positions[i].z) * (DEFAULT_ANGULAR_SCALING - trajectory_progress);

            float total_diff = sqrt(pos_diff_x * pos_diff_x + pos_diff_y * pos_diff_y + pos_diff_z * pos_diff_z);
            if (total_diff > config.position_tolerance_mm) {
                return false;
            }
        }
    }

    return true;
}

bool PoseController::setBodyPoseImmediate(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                                          Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    // OpenSHC-style: Transform stance positions based on body pose
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Start with configured stance position (convert from meters to mm)
        Point3D stance_pos;
        stance_pos.x = pose_config.leg_stance_positions[i].x * 1000.0f;
        stance_pos.y = pose_config.leg_stance_positions[i].y * 1000.0f;
        stance_pos.z = -(pose_config.body_clearance * 1000.0f); // Body clearance

        // Apply body transformation: translate relative to body position, then rotate
        Point3D leg_body_relative(stance_pos.x - position[0], stance_pos.y - position[1], stance_pos.z - position[2]);
        Point3D leg_rotated = math_utils::rotatePoint(leg_body_relative, orientation);
        Point3D leg_world(position[0] + leg_rotated.x,
                          position[1] + leg_rotated.y,
                          position[2] + leg_rotated.z);

        JointAngles angles = model.inverseKinematics(i, leg_world);
        if (!model.checkJointLimits(i, angles))
            return false;
        joint_q[i] = angles;
        leg_pos[i] = leg_world;

        if (servos) {
            float speed = model.getParams().default_servo_speed;
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                // Check servo status flags before movement
                if (servos->hasBlockingStatusFlags(i, j)) {
                    return false; // Servo blocked by status flags
                }
                float angle = (j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));
                servos->setJointAngleAndSpeed(i, j, angle, speed);
            }
        }
    }
    return true;
}

// OpenSHC-style pose limit validation
bool PoseController::checkPoseLimits(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation) {
    // Convert position to meters for comparison with config limits
    float pos_x_m = position[0] / 1000.0f;
    float pos_y_m = position[1] / 1000.0f;
    float pos_z_m = position[2] / 1000.0f;

    // Check translation limits
    if (std::abs(pos_x_m) > pose_config.max_translation.x ||
        std::abs(pos_y_m) > pose_config.max_translation.y ||
        std::abs(pos_z_m) > pose_config.max_translation.z) {
        return false;
    }

    // Convert orientation from degrees to radians for comparison
    float roll_rad = math_utils::degreesToRadians(orientation[0]);
    float pitch_rad = math_utils::degreesToRadians(orientation[1]);
    float yaw_rad = math_utils::degreesToRadians(orientation[2]);

    // Check rotation limits
    if (std::abs(roll_rad) > pose_config.max_rotation.roll ||
        std::abs(pitch_rad) > pose_config.max_rotation.pitch ||
        std::abs(yaw_rad) > pose_config.max_rotation.yaw) {
        return false;
    }

    return true;
}

void PoseController::configureSmoothTrajectory(bool use_current_positions, float interpolation_speed, uint8_t max_steps) {
    // Configure smooth trajectory parameters (OpenSHC-equivalent global parameter modification)
    auto &config = const_cast<Parameters &>(model.getParams()).smooth_trajectory;
    config.use_current_servo_positions = use_current_positions;
    config.interpolation_speed = std::max(0.01f, std::min(DEFAULT_ANGULAR_SCALING, interpolation_speed));
    config.max_interpolation_steps = std::max(static_cast<uint8_t>(1), std::min(static_cast<uint8_t>(100), max_steps));
}
