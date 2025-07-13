#include "body_pose_controller.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

/**
 * @file body_pose_controller.cpp
 * @brief Implementation of the body pose controller for HexaMotion
 *
 * This implementation is adapted from OpenSHC's PoseController but simplified for HexaMotion architecture.
 * No progress tracking - that's handled by LocomotionSystem.
 */

// Internal implementation class to avoid circular dependencies
class BodyPoseController::LegPoserImpl {
  public:
    LegPoserImpl(int leg_index, Leg &leg, RobotModel &model)
        : poser_(leg_index, leg, model) {}

    LegPoser *get() { return &poser_; }
    const LegPoser *get() const { return &poser_; }

  private:
    LegPoser poser_;
};

BodyPoseController::BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config)
    : model(m), body_pose_config(config), auto_pose_enabled(false), trajectory_in_progress(false),
      trajectory_progress(0.0), trajectory_step_count(0), current_gait_type_(TRIPOD_GAIT),
      step_to_new_stance_current_group(0), step_to_new_stance_sequence_generated(false),
      direct_startup_sequence_initialized(false), shutdown_sequence_initialized(false) {

    // Initialize leg posers
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_posers_[i] = nullptr;
    }

    // Initialize trajectory arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = Point3D(0, 0, 0);
        trajectory_start_angles[i] = JointAngles(0, 0, 0);
        trajectory_target_positions[i] = Point3D(0, 0, 0);
        trajectory_target_angles[i] = JointAngles(0, 0, 0);
    }

    // Initialize auto-pose configuration from factory
    auto_pose_config = createAutoPoseConfiguration(model.getParams());
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
        // Create LegPoser with the corresponding leg
        leg_posers_[i] = new LegPoserImpl(i, legs[i], model);
    }
}

LegPoser *BodyPoseController::getLegPoser(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS && leg_posers_[leg_index]) {
        return leg_posers_[leg_index]->get();
    }
    return nullptr;
}

bool BodyPoseController::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                     Leg legs[NUM_LEGS]) {
    // Check body pose limits before applying
    if (!checkBodyPoseLimits(position, orientation)) {
        return false;
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
    // Use current joint angles as starting point for IK (OpenSHC approach)
    JointAngles current_angles = legs[leg_index].getJointAngles();
    JointAngles angles = model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    // Update leg with new joint angles and calculated position
    legs[leg_index].setJointAngles(angles);
    Point3D calculated_position = model.forwardKinematicsGlobalCoordinates(leg_index, angles);
    legs[leg_index].setCurrentTipPositionGlobal(model, calculated_position);

    return true;
}

bool BodyPoseController::calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]) {
    // Calculate Z position based on body clearance and height offset
    double target_z = -(body_pose_config.body_clearance + height_offset);

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration
        double stance_x_mm = body_pose_config.leg_stance_positions[i].x;
        double stance_y_mm = body_pose_config.leg_stance_positions[i].y;

        Point3D target_pos;
        target_pos.x = stance_x_mm;
        target_pos.y = stance_y_mm;
        target_pos.z = target_z;

        // Use LegPoser if available for smooth movement
        if (getLegPoser(i)) {
            setLegPosition(i, target_pos, legs);
        } else {
            // Fallback to direct calculation
            JointAngles current_angles = legs[i].getJointAngles();
            JointAngles angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, target_pos);

            if (!model.checkJointLimits(i, angles)) {
                return false;
            }

            legs[i].setCurrentTipPositionGlobal(model, target_pos);
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

    // Use configuration-based calculation instead of direct geometry
    calculateBodyPoseFromConfig(0.0, legs);
}

bool BodyPoseController::setStandingPose(Leg legs[NUM_LEGS]) {
    // Initialize LegPosers if not already done
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // Standing pose uses pre-configured joint angles, not calculated positions
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
        legs[i].setJointAngles(angles);
        legs[i].updateTipPosition(model);
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
    t = std::max(0.0, std::min(ANGULAR_SCALING, t));

    // Linear interpolation for position
    Eigen::Vector3d interp_pos = start_pos + t * (end_pos - start_pos);

    // Spherical linear interpolation (SLERP) for quaternion
    Eigen::Vector4d interp_quat = math_utils::quaternionSlerp(start_quat, end_quat, t);

    return setBodyPoseQuaternion(interp_pos, interp_quat, legs);
}

bool BodyPoseController::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                           Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Check if smooth trajectory is enabled
    if (!model.getParams().smooth_trajectory.use_current_servo_positions) {
        return setBodyPoseImmediate(position, orientation, legs);
    }

    // If not already in progress, initialize trajectory from current servo positions
    if (!trajectory_in_progress) {
        if (!initializeTrajectoryFromCurrent(position, orientation, legs, servos)) {
            return false;
        }
        trajectory_in_progress = true;
        trajectory_progress = 0.0;
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
        legs[i].setCurrentTipPositionGlobal(model, model.forwardKinematicsGlobalCoordinates(i, current_angles));
    }

    return true;
}

bool BodyPoseController::setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                              Leg legs[NUM_LEGS]) {
    // Reset any active trajectory
    resetTrajectory();

    // Calculate leg positions based on body pose
    return calculateBodyPoseFromConfig(0.0, legs);
}

void BodyPoseController::configureSmoothTrajectory(bool use_current_positions, double interpolation_speed, uint8_t max_steps) {
    // Reset trajectory state
    resetTrajectory();
}

bool BodyPoseController::checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    // Check translation limits using configured max_translation values
    if (std::abs(position.x()) > body_pose_config.max_translation.x ||
        std::abs(position.y()) > body_pose_config.max_translation.y ||
        std::abs(position.z()) > body_pose_config.max_translation.z) {
        return false;
    }

    // Check rotation limits using configured max_rotation values
    double roll_rad = math_utils::degreesToRadians(orientation.x());
    double pitch_rad = math_utils::degreesToRadians(orientation.y());
    double yaw_rad = math_utils::degreesToRadians(orientation.z());

    if (std::abs(roll_rad) > body_pose_config.max_rotation.roll ||
        std::abs(pitch_rad) > body_pose_config.max_rotation.pitch ||
        std::abs(yaw_rad) > body_pose_config.max_rotation.yaw) {
        return false;
    }

    return true;
}

Eigen::Vector3d BodyPoseController::calculateBodyPosition(Leg legs[NUM_LEGS]) const {
    // Calculate body position based on average leg positions
    double total_z = 0.0;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D leg_pos = legs[i].getCurrentTipPositionGlobal();
        total_z += leg_pos.z;
    }
    double average_leg_z = total_z / NUM_LEGS;

    // Return body position with calculated Z height
    // X and Y remain at origin for standing pose
    return Eigen::Vector3d(0.0, 0.0, average_leg_z);
}

bool BodyPoseController::initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                                         const Eigen::Vector3d &target_orientation,
                                                         Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Store current positions as trajectory start
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = legs[i].getCurrentTipPositionGlobal();
        trajectory_start_angles[i] = legs[i].getJointAngles();
    }

    // Calculate target positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_target_positions[i] = trajectory_start_positions[i];
        trajectory_target_positions[i].z = -target_position.z();

        // Calculate target angles using current angles as starting point
        JointAngles current_angles = legs[i].getJointAngles();
        trajectory_target_angles[i] = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, trajectory_target_positions[i]);
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
    if (trajectory_progress >= 1.0 || trajectory_step_count >= model.getParams().smooth_trajectory.max_interpolation_steps) {
        // Set final positions
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setCurrentTipPositionGlobal(model, trajectory_target_positions[i]);
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
        legs[i].setCurrentTipPositionGlobal(model, interp_pos);
        legs[i].setJointAngles(interp_angles);
    }

    return true;
}

void BodyPoseController::resetTrajectory() {
    trajectory_in_progress = false;
    trajectory_progress = 0.0;
    trajectory_step_count = 0;
}

// Tripod leg coordination for stance transition (OpenSHC equivalent)
bool BodyPoseController::stepToNewStance(Leg legs[NUM_LEGS], double step_height, double step_time) {

    // Use class-level tripod leg groups configuration
    const int legs_in_group = 3;

    // Initialize sequence if not already done
    if (!step_to_new_stance_sequence_generated) {
        double robot_z = -model.getParams().standing_height;
        int initialized_posers = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            auto *poser = leg_posers_[i] ? leg_posers_[i]->get() : nullptr;
            if (!poser) {
                continue;
            }
            const auto &cfg = body_pose_config.leg_stance_positions[i];
            poser->setTargetPosition(Point3D{cfg.x, cfg.y, robot_z});
            poser->resetStepToPosition();
            initialized_posers++;
        }
        step_to_new_stance_current_group = 0;
        step_to_new_stance_sequence_generated = true;

        // If no posers are initialized, return false immediately
        if (initialized_posers == 0) {
            return false;
        }
    }

    // Move legs in current group and track completion
    int completed_legs = 0;
    int valid_legs = 0;
    for (int idx = 0; idx < legs_in_group; ++idx) {
        int i = tripod_leg_groups[step_to_new_stance_current_group][idx];
        auto *poser = leg_posers_[i] ? leg_posers_[i]->get() : nullptr;
        if (!poser) {
            continue;
        }
        valid_legs++;
        bool leg_complete = poser->stepToPosition(poser->getTargetPosition(), step_height, step_time);
        Point3D new_pos = poser->getCurrentPosition();
        JointAngles curr_angles = legs[i].getJointAngles();
        JointAngles new_angles = model.inverseKinematicsCurrentGlobalCoordinates(i, curr_angles, new_pos);
        legs[i].setCurrentTipPositionGlobal(model, new_pos);
        legs[i].setJointAngles(new_angles);
        if (leg_complete) {
            completed_legs++;
        }
    }

    // Advance group or finish when all valid legs complete
    if (completed_legs == valid_legs && valid_legs > 0) {
        if (step_to_new_stance_current_group == 0) {
            step_to_new_stance_current_group = 1;
            // Reset leg posers for Group B
            for (int idx = 0; idx < legs_in_group; ++idx) {
                int i = tripod_leg_groups[step_to_new_stance_current_group][idx];
                auto *poser = leg_posers_[i] ? leg_posers_[i]->get() : nullptr;
                if (poser) {
                    poser->resetStepToPosition();
                }
            }
        } else {
            step_to_new_stance_sequence_generated = false;
            step_to_new_stance_current_group = 0;
            return true;
        }
    }
    return false;
}

// Execute startup sequence (READY -> RUNNING transition)
bool BodyPoseController::executeStartupSequence(Leg legs[NUM_LEGS]) {

    // OpenSHC Architecture Compliance:
    // According to OpenSHC documentation and implementation:
    // - stepToNewStance() is exclusively for tripod gait coordination
    //   "The stepping motion is coordinated such that half of the legs execute the step at any one time
    //    (for a hexapod this results in a Tripod stepping coordination)"
    // - directStartup() is for all other gaits (wave, ripple, metachronal, etc.)
    //   "moves them in a linear trajectory directly from their current tip position to its default tip position...
    //    The joint states for each leg are saved for the default tip position and then the joint moved
    //    independently from initial position to saved default positions"

    double step_height = body_pose_config.swing_height;
    double step_time = 1.0 / DEFAULT_STEP_FREQUENCY; // Default 1Hz step frequency

    // Check if we're using tripod gait - stepToNewStance is only for tripod gait
    bool use_tripod_coordination = (current_gait_type_ == TRIPOD_GAIT);

    bool complete = false;
    if (use_tripod_coordination) {
        // Use stepToNewStance for tripod gait coordination (OpenSHC style)
        // This coordinates legs in two groups of 3, moving one group at a time
        complete = stepToNewStance(legs, step_height, step_time);
    } else {
        // Use executeDirectStartup for wave, ripple, metachronal, and other gaits (OpenSHC style)
        // This provides simultaneous leg coordination (all legs move together)
        complete = executeDirectStartup(legs);
    }

    return complete;
}

// Execute direct startup sequence (simultaneous leg coordination)
bool BodyPoseController::executeDirectStartup(Leg legs[NUM_LEGS]) {
    // Initialize sequence if not done
    if (!direct_startup_sequence_initialized) {
        direct_startup_sequence_initialized = true;
    }

    // Use time_to_start directly as total sequence time (OpenSHC style)
    double total_time = body_pose_config.time_to_start;
    double step_height = body_pose_config.swing_height;

    // Move all legs simultaneously to standing pose positions
    int completed_legs = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_posers_[i]) {
            // Get standing pose joint angles
            const auto &standing_joints = body_pose_config.standing_pose_joints[i];
            JointAngles target_angles;
            target_angles.coxa = standing_joints.coxa;
            target_angles.femur = standing_joints.femur;
            target_angles.tibia = standing_joints.tibia;

            // Calculate target position from joint angles
            Point3D target_position = model.forwardKinematicsGlobalCoordinates(i, target_angles);

            bool leg_complete = leg_posers_[i]->get()->stepToPosition(target_position, step_height, total_time);

            // Update leg with current position
            legs[i].setCurrentTipPositionGlobal(model, leg_posers_[i]->get()->getCurrentPosition());
            JointAngles current_angles = legs[i].getJointAngles();
            legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, legs[i].getCurrentTipPositionGlobal()));

            if (leg_complete) {
                completed_legs++;
            }
        }
    }

    if (completed_legs >= NUM_LEGS) {
        direct_startup_sequence_initialized = false;
        return true;
    }

    return false;
}

// Execute shutdown sequence (RUNNING -> READY transition)
bool BodyPoseController::executeShutdownSequence(Leg legs[NUM_LEGS]) {
    // Initialize sequence if not done
    if (!shutdown_sequence_initialized) {
        shutdown_sequence_initialized = true;
    }

    // Move legs back to standing pose positions using configured values
    double step_height = body_pose_config.swing_height;
    double step_time = 1.0 / DEFAULT_STEP_FREQUENCY; // Default 1Hz step frequency
    int completed_legs = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_posers_[i]) {
            // Get standing pose joint angles
            const auto &standing_joints = body_pose_config.standing_pose_joints[i];
            JointAngles target_angles;
            target_angles.coxa = standing_joints.coxa;
            target_angles.femur = standing_joints.femur;
            target_angles.tibia = standing_joints.tibia;

            // Calculate target position from joint angles
            Point3D target_position = model.forwardKinematicsGlobalCoordinates(i, target_angles);

            bool leg_complete = leg_posers_[i]->get()->stepToPosition(target_position, step_height, step_time);

            // Update leg with current position
            legs[i].setCurrentTipPositionGlobal(model, leg_posers_[i]->get()->getCurrentPosition());
            JointAngles current_angles = legs[i].getJointAngles();
            legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, legs[i].getCurrentTipPositionGlobal()));

            if (leg_complete) {
                completed_legs++;
            }
        }
    }

    if (completed_legs >= NUM_LEGS) {
        shutdown_sequence_initialized = false;
        return true;
    }

    return false;
}

// Update auto-pose during gait execution (OpenSHC equivalent)
bool BodyPoseController::updateAutoPose(double gait_phase, Leg legs[NUM_LEGS]) {
    if (!auto_pose_enabled || !auto_pose_config.enabled) {
        return true; // Auto-pose disabled, but not an error
    }

    // Enhanced OpenSHC-style auto-pose integration with LegPoser
    // Convert gait phase (0.0-1.0) to phase (0-100) for LegPoser
    int phase = static_cast<int>(gait_phase * AUTO_POSE_PHASE_CONVERSION_FACTOR);

    // Apply auto-pose compensation using LegPoser for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_posers_[i]) {
            // Use LegPoser's enhanced updateAutoPose method
            leg_posers_[i]->get()->updateAutoPose(phase);

            // Update the leg object with the new position from LegPoser
            Point3D new_position = leg_posers_[i]->get()->getCurrentPosition();
            legs[i].setCurrentTipPositionGlobal(model, new_position);

            // Update joint angles to match the new position
            JointAngles current_angles = legs[i].getJointAngles();
            JointAngles new_angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, new_position);
            legs[i].setJointAngles(new_angles);
        }
    }

    // Additional body-level compensation for tripod gait stability
    // This provides extra compensation beyond individual leg adjustments
    if (auto_pose_config.tripod_mode_enabled) {
        // Get tripod groups from configuration
        const auto &group_a_legs = auto_pose_config.tripod_group_a_legs;
        const auto &group_b_legs = auto_pose_config.tripod_group_b_legs;

        // Calculate which group is in stance phase
        bool group_a_stance = (gait_phase < AUTO_POSE_GAIT_PHASE_THRESHOLD);
        bool group_b_stance = (gait_phase >= AUTO_POSE_GAIT_PHASE_THRESHOLD);

        // Get amplitudes from configuration
        const auto &roll_amplitudes = auto_pose_config.roll_amplitudes;
        const auto &z_amplitudes = auto_pose_config.z_amplitudes;

        // Calculate additional body-level compensation
        double roll_compensation = 0.0;
        double z_compensation = 0.0;

        if (group_a_stance && roll_amplitudes.size() >= 2 && z_amplitudes.size() >= 2) {
            // Group A in stance (AR, CR, BL) - compensate for Group B in swing
            roll_compensation = roll_amplitudes[0] * AUTO_POSE_BODY_COMPENSATION_REDUCTION; // Reduced amplitude for body-level
            z_compensation = z_amplitudes[0] * AUTO_POSE_BODY_COMPENSATION_REDUCTION;
        } else if (group_b_stance && roll_amplitudes.size() >= 2 && z_amplitudes.size() >= 2) {
            // Group B in stance (BR, CL, AL) - compensate for Group A in swing
            roll_compensation = roll_amplitudes[1] * AUTO_POSE_BODY_COMPENSATION_REDUCTION; // Reduced amplitude for body-level
            z_compensation = z_amplitudes[1] * AUTO_POSE_BODY_COMPENSATION_REDUCTION;
        }

        // Apply body-level compensation to stance legs only
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg_posers_[i]) {
                bool is_group_a = std::find(group_a_legs.begin(), group_a_legs.end(), i) != group_a_legs.end();
                bool is_group_b = std::find(group_b_legs.begin(), group_b_legs.end(), i) != group_b_legs.end();

                // Apply compensation only to stance legs
                if ((is_group_a && group_a_stance) || (is_group_b && group_b_stance)) {
                    Point3D current_pos = legs[i].getCurrentTipPositionGlobal();

                    // Apply roll compensation based on leg position
                    double y_offset = current_pos.y;
                    double roll_z_offset = roll_compensation * y_offset;

                    // Apply Z compensation
                    Point3D body_compensated_pos = current_pos;
                    body_compensated_pos.z += z_compensation + roll_z_offset;

                    // Update leg position with body-level compensation
                    legs[i].setCurrentTipPositionGlobal(model, body_compensated_pos);

                    // Recalculate joint angles for body-compensated position
                    JointAngles current_angles = legs[i].getJointAngles();
                    JointAngles body_compensated_angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, body_compensated_pos);
                    legs[i].setJointAngles(body_compensated_angles);
                }
            }
        }
    }

    return true;
}
