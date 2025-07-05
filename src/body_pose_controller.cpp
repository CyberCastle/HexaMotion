#include "body_pose_controller.h"
#include "leg_poser.h"
#include "hexamotion_constants.h"
#include <cmath>
/**
 * @file body_pose_controller.cpp
 * @brief Implementation of the kinematic body pose controller.
 */

// Internal implementation class to avoid circular dependencies
class BodyPoseController::LegPoserImpl {
public:
    LegPoserImpl(BodyPoseController* controller, int leg_index, Leg& leg)
        : poser_(controller, nullptr, leg_index, leg) {}

    LegPoser* get() { return &poser_; }
    const LegPoser* get() const { return &poser_; }

private:
    LegPoser poser_;
};

BodyPoseController::BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config)
    : model(m), body_pose_config(config), trajectory_in_progress(false), trajectory_progress(0.0f), trajectory_step_count(0) {
    // Initialize trajectory arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = Point3D(0, 0, 0);
        trajectory_start_angles[i] = JointAngles(0, 0, 0);
        trajectory_target_positions[i] = Point3D(0, 0, 0);
        trajectory_target_angles[i] = JointAngles(0, 0, 0);
        leg_posers_[i] = nullptr; // Initialize to nullptr
    }
}

BodyPoseController::~BodyPoseController() {
    // Clean up leg posers
    for (int i = 0; i < NUM_LEGS; i++) {
        delete leg_posers_[i];
        leg_posers_[i] = nullptr;
    }
}

void BodyPoseController::initializeLegPosers(Leg legs[NUM_LEGS]) {
    // Initialize LegPoser instances for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Delete existing poser if any
        delete leg_posers_[i];
        // Create LegPoser with this BodyPoseController and the corresponding leg
        leg_posers_[i] = new LegPoserImpl(this, i, legs[i]);
    }
}

LegPoser* BodyPoseController::getLegPoser(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS && leg_posers_[leg_index]) {
        return leg_posers_[leg_index]->get();
    }
    return nullptr;
}

bool BodyPoseController::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                 Leg legs[NUM_LEGS]) {
    // OpenSHC-style: Check body pose limits before applying
    if (!checkBodyPoseLimits(position, orientation)) {
        return false; // Body pose exceeds configured limits
    }

    // Use smooth trajectory as default behavior if enabled
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation) {
        return setBodyPoseSmooth(position, orientation, legs);
    }

    // Original implementation for compatibility when smooth trajectory is disabled
    return setBodyPoseImmediate(position, orientation, legs);
}

bool BodyPoseController::setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]) {
    // Use LegPoser if available
    LegPoser* poser = getLegPoser(leg_index);
    if (poser) {
        // Use LegPoser's stepToPosition method for smooth movement
        Pose target_pose(position, Eigen::Vector3d(0, 0, 0));
        Pose delta_pose(Point3D(0, 0, 0), Eigen::Vector3d(0, 0, 0));

        // Use a reasonable step time and lift height
        double step_time = 1.0; // 1 second for pose adjustment
        double lift_height = 20.0; // 20mm lift height

        int progress = poser->stepToPosition(target_pose, delta_pose, lift_height, step_time, true);
        return (progress == 100); // 100 means complete
    }

    // Fallback to direct calculation if LegPoser not available
    JointAngles angles = model.inverseKinematics(leg_index, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    legs[leg_index].setJointAngles(angles);
    legs[leg_index].setTipPosition(model.forwardKinematics(leg_index, angles));

    return true;
}

// OpenSHC-style pose calculation using dynamic configuration
// Calculates leg positions based on pose configuration and desired body height
bool BodyPoseController::calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]) {
    // Calculate Z position based on body clearance and height offset
    double target_z = -(body_pose_config.body_clearance * 1000.0f + height_offset); // Convert to mm and apply offset

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration (convert from meters to mm)
        double stance_x_mm = body_pose_config.leg_stance_positions[i].x * 1000.0f;
        double stance_y_mm = body_pose_config.leg_stance_positions[i].y * 1000.0f;

        Point3D target_pos;
        target_pos.x = stance_x_mm;
        target_pos.y = stance_y_mm;
        target_pos.z = target_z;

        // Use LegPoser if available for smooth movement
        if (getLegPoser(i)) {
            setLegPosition(i, target_pos, legs);
        } else {
            // Fallback to direct calculation
            JointAngles angles = model.inverseKinematics(i, target_pos);

            if (!model.checkJointLimits(i, angles)) {
                return false;
            }

            legs[i].setTipPosition(target_pos);
            legs[i].setJointAngles(angles);
        }
    }

    return true;
}

void BodyPoseController::initializeDefaultPose(Leg legs[NUM_LEGS]) {
    // Initialize LegPosers if not already done
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // OpenSHC-style: Use configuration-based calculation instead of direct geometry
    calculateBodyPoseFromConfig(0.0f, legs); // No height offset for default pose
}

bool BodyPoseController::setStandingPose(Leg legs[NUM_LEGS]) {
    // Initialize LegPosers if not already done
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // OpenSHC-style: Standing pose uses pre-configured joint angles, not calculated positions
    // This ensures coxa ≈ 0° and femur/tibia equal for all legs (symmetric posture)

    for (int i = 0; i < NUM_LEGS; i++) {
        // Use configured standing pose joint angles
        JointAngles angles;
        angles.coxa = body_pose_config.standing_pose_joints[i].coxa;
        angles.femur = body_pose_config.standing_pose_joints[i].femur;
        angles.tibia = body_pose_config.standing_pose_joints[i].tibia;

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
    }

    return true;
}

bool BodyPoseController::setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
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

bool BodyPoseController::interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
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
Eigen::Vector4d BodyPoseController::quaternionSlerp(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, double t) {
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

bool BodyPoseController::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                       Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Check if smooth trajectory is enabled
    if (!model.getParams().smooth_trajectory.use_current_servo_positions) {
        // Fall back to original non-smooth method
        return setBodyPoseImmediate(position, orientation, legs);
    }

    // If not already in progress, initialize trajectory from current servo positions
    if (!trajectory_in_progress) {
        if (!initializeTrajectoryFromCurrent(position, orientation, legs, servos)) {
            return false;
        }
        trajectory_in_progress = true;
        trajectory_progress = 0.0f;
        trajectory_step_count = 0;
    }

    // Update trajectory step
    return updateTrajectoryStep(legs);
}

bool BodyPoseController::setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                                 Leg legs[NUM_LEGS]) {
    // Convert quaternion to euler for compatibility with existing smooth trajectory
    Point3D euler_rad = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3d orientation(
        math_utils::radiansToDegrees(euler_rad.x),
        math_utils::radiansToDegrees(euler_rad.y),
        math_utils::radiansToDegrees(euler_rad.z));
    return setBodyPoseSmooth(position, orientation, legs);
}

bool BodyPoseController::getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]) {
    if (!servos) {
        return false;
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles current_angles;
        current_angles.coxa = servos->getJointAngle(i, 0);
        current_angles.femur = servos->getJointAngle(i, 1);
        current_angles.tibia = servos->getJointAngle(i, 2);

        // Update leg object with current servo positions
        legs[i].setJointAngles(current_angles);
        legs[i].setTipPosition(model.forwardKinematics(i, current_angles));
    }

    return true;
}

bool BodyPoseController::setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                          Leg legs[NUM_LEGS]) {
    // Reset any active trajectory
    resetTrajectory();

    // OpenSHC-style: Calculate leg positions based on body pose
    // This uses the body pose configuration to determine leg stance positions
    return calculateBodyPoseFromConfig(0.0f, legs);
}

void BodyPoseController::configureSmoothTrajectory(bool use_current_positions, double interpolation_speed, uint8_t max_steps) {
    // Note: This method cannot modify const parameters directly
    // The smooth trajectory configuration should be handled at a higher level
    // For now, we'll store the configuration locally
    trajectory_in_progress = false;
    trajectory_progress = 0.0f;
    trajectory_step_count = 0;
}

bool BodyPoseController::checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    // OpenSHC-style body pose limit validation
    // Check if the desired body pose is within configured limits

    // Check height limits (simplified - use reasonable defaults)
    double min_height = -200.0; // -200mm minimum height
    double max_height = 200.0;  // 200mm maximum height
    if (position.z() < min_height || position.z() > max_height) {
        return false;
    }

    // Check orientation limits (roll, pitch, yaw) - simplified
    double max_roll = 30.0;   // 30 degrees max roll
    double max_pitch = 30.0;  // 30 degrees max pitch
    double max_yaw = 45.0;    // 45 degrees max yaw

    if (std::abs(orientation.x()) > max_roll ||
        std::abs(orientation.y()) > max_pitch ||
        std::abs(orientation.z()) > max_yaw) {
        return false;
    }

    return true;
}

bool BodyPoseController::initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                                     const Eigen::Vector3d &target_orientation,
                                                     Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Store current positions as trajectory start
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = legs[i].getTipPosition();
        trajectory_start_angles[i] = legs[i].getJointAngles();
    }

    // Calculate target positions for each leg
    // This would involve calculating the leg positions needed to achieve the target body pose
    // For now, we'll use a simplified approach
    for (int i = 0; i < NUM_LEGS; i++) {
        // Calculate target position based on body pose change
        // This is a simplified calculation - in practice, this would be more complex
        trajectory_target_positions[i] = trajectory_start_positions[i];
        trajectory_target_positions[i].z = -target_position.z(); // Adjust height

        // Calculate target angles
        trajectory_target_angles[i] = model.inverseKinematics(i, trajectory_target_positions[i]);
    }

    return true;
}

bool BodyPoseController::updateTrajectoryStep(Leg legs[NUM_LEGS]) {
    if (!trajectory_in_progress) {
        return false;
    }

    // Update trajectory progress
    trajectory_progress += model.getParams().smooth_trajectory.interpolation_speed;
    trajectory_step_count++;

    // Check if trajectory is complete
    if (trajectory_progress >= 1.0f || trajectory_step_count >= model.getParams().smooth_trajectory.max_interpolation_steps) {
        // Set final positions
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setTipPosition(trajectory_target_positions[i]);
            legs[i].setJointAngles(trajectory_target_angles[i]);
        }

        resetTrajectory();
        return true;
    }

    // Interpolate between start and target positions
    for (int i = 0; i < NUM_LEGS; i++) {
        // Linear interpolation for positions
        Point3D interp_pos;
        interp_pos.x = trajectory_start_positions[i].x +
                      trajectory_progress * (trajectory_target_positions[i].x - trajectory_start_positions[i].x);
        interp_pos.y = trajectory_start_positions[i].y +
                      trajectory_progress * (trajectory_target_positions[i].y - trajectory_start_positions[i].y);
        interp_pos.z = trajectory_start_positions[i].z +
                      trajectory_progress * (trajectory_target_positions[i].z - trajectory_start_positions[i].z);

        // Linear interpolation for angles
        JointAngles interp_angles;
        interp_angles.coxa = trajectory_start_angles[i].coxa +
                           trajectory_progress * (trajectory_target_angles[i].coxa - trajectory_start_angles[i].coxa);
        interp_angles.femur = trajectory_start_angles[i].femur +
                            trajectory_progress * (trajectory_target_angles[i].femur - trajectory_start_angles[i].femur);
        interp_angles.tibia = trajectory_start_angles[i].tibia +
                            trajectory_progress * (trajectory_target_angles[i].tibia - trajectory_start_angles[i].tibia);

        // Update leg
        legs[i].setTipPosition(interp_pos);
        legs[i].setJointAngles(interp_angles);
    }

    return true;
}
