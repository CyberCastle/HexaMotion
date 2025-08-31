#include "body_pose_controller.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "leg_poser.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <vector>

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
    : model(m), body_pose_config(config), auto_pose_enabled(false), current_gait_type_(TRIPOD_GAIT),
      trajectory_in_progress(false), trajectory_progress(0.0), trajectory_step_count(0),
      step_to_new_stance_current_group(0), step_to_new_stance_sequence_generated(false),
      shutdown_sequence_initialized(false),
      // OpenSHC state variables initialization
      executing_transition_(false), transition_step_(0), transition_step_count_(0),
      set_target_(false), proximity_alert_(false), horizontal_transition_complete_(false),
      vertical_transition_complete_(false), first_sequence_execution_(true),
      reset_transition_sequence_(false), legs_completed_step_(0), current_group_(0),
      pack_step_(0) {

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

    // Initialize walk plane pose system (OpenSHC equivalent with Bézier curves)
    walk_plane_pose_ = Pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
    walk_plane_pose_enabled = false;
    walk_plane_update_threshold = 1.0; // 1mm threshold

    // Initialize Bézier curve control system
    walk_plane_bezier_in_progress = false;
    walk_plane_bezier_time = 0.0;
    walk_plane_bezier_duration = 1.0; // 1 second transition time

    // Initialize control nodes arrays
    for (int i = 0; i < 5; i++) {
        walk_plane_position_nodes[i] = Point3D(0.0, 0.0, body_pose_config.body_clearance);
        walk_plane_rotation_nodes[i] = Eigen::Quaterniond::Identity();
    }

    // Initialize auto-pose configuration (default: tripod)
    std::string gait_name = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    auto_pose_config = createAutoPoseConfigurationForGait(model.getParams(), gait_name);
}

// -------------------------------------------------------------------------------------------------
// Partial OpenSHC PoseController::updateCurrentPose adaptation
// Only updates walk plane pose (clearance / plane estimation) and auto pose modulation.
// Excludes: IMU gravity estimation, manual pose input filtering, pose reset sequences,
// dynamic stiffness adjustments, external target transforms, and progress tracking.
// Rationale: HexaMotion centralises those concerns elsewhere (LocomotionSystem / IMU modules).
void BodyPoseController::updateCurrentPose(double gait_phase, Leg legs[NUM_LEGS]) {
    // Keep walk plane pose coherent with current stance distribution.
    updateWalkPlanePose(legs);

    // Apply auto-pose patterning (phase-synchronised) if enabled.
    // We ignore the return value (currently always true) since we perform no fallback action here.
    updateAutoPose(gait_phase, legs);
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
    legs[leg_index].setCurrentTipPositionGlobal(calculated_position);

    return true;
}

bool BodyPoseController::calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]) {
    // Update walk plane pose with current leg positions
    updateWalkPlanePose(legs);

    // Calculate target Z position using walk plane pose
    double target_z;
    if (walk_plane_pose_enabled) {
        // Use walk plane pose height (already includes body clearance)
        target_z = walk_plane_pose_.position.z + height_offset;
    } else {
        // Fallback to traditional body clearance calculation
        target_z = -(body_pose_config.body_clearance + height_offset);
    }

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration
        double stance_x_mm = body_pose_config.leg_stance_positions[i].x;
        double stance_y_mm = body_pose_config.leg_stance_positions[i].y;

        Point3D target_pos;
        target_pos.x = stance_x_mm;
        target_pos.y = stance_y_mm;
        target_pos.z = target_z;

        // Apply walk plane pose rotation if enabled
        if (walk_plane_pose_enabled) {
            // Transform leg position by walk plane pose rotation
            Eigen::Vector3d leg_vector(target_pos.x, target_pos.y, target_pos.z);
            Eigen::Vector3d rotated_vector = walk_plane_pose_.rotation * leg_vector;
            target_pos.x = rotated_vector.x();
            target_pos.y = rotated_vector.y();
            target_pos.z = rotated_vector.z();
        }

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

            legs[i].setCurrentTipPositionGlobal(target_pos);
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

    // Apply configured standing pose joint angles for each leg
    for (int i = 0; i < NUM_LEGS; ++i) {
        const auto &standing_joints = body_pose_config.standing_pose_joints[i];

        JointAngles angles;
        angles.coxa = standing_joints.coxa;
        angles.femur = standing_joints.femur;
        angles.tibia = standing_joints.tibia;

        // Update leg with standing joint angles and corresponding tip position
        legs[i].setJointAngles(angles);
        Point3D pos = model.forwardKinematicsGlobalCoordinates(i, angles);
        legs[i].setCurrentTipPositionGlobal(pos);
    }

    return true;
}

StandingPoseJoints BodyPoseController::getStandingPoseJoints(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return {0.0, 0.0, 0.0};
    }
    return body_pose_config.standing_pose_joints[leg_index];
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

        initializeTrajectoryFromCurrent(position, orientation, legs, servos);
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
        legs[i].setCurrentTipPositionGlobal(model.forwardKinematicsGlobalCoordinates(i, current_angles));
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
    // Legacy method - parameters are ignored in current implementation
    // Only resets trajectory state for compatibility
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

    // Use walk plane pose for body position calculation (OpenSHC equivalent)
    if (walk_plane_pose_enabled) {
        // Update walk plane pose for current leg positions
        const_cast<BodyPoseController *>(this)->updateWalkPlanePose(legs);

        return Eigen::Vector3d(
            walk_plane_pose_.position.x,
            walk_plane_pose_.position.y,
            walk_plane_pose_.position.z);
    }

    // Fallback to legacy calculation if walk plane pose disabled
    double total_z = 0.0;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D leg_pos = legs[i].getCurrentTipPositionGlobal();
        total_z += leg_pos.z;
    }
    double average_leg_z = total_z / NUM_LEGS;

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
            legs[i].setCurrentTipPositionGlobal(trajectory_target_positions[i]);
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
        legs[i].setCurrentTipPositionGlobal(interp_pos);
        legs[i].setJointAngles(interp_angles);
    }

    return true;
}

void BodyPoseController::resetTrajectory() {
    trajectory_in_progress = false;
    trajectory_progress = 0.0;
    trajectory_step_count = 0;
}

bool BodyPoseController::beginInitialStandingPoseTransition(Leg legs[NUM_LEGS]) {
    if (initial_standing_active_) {
        return true; // already active
    }
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }
    const Parameters &params = model.getParams();

    // Allocate / reset profiles
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            delete initial_standing_profiles_[l][j];
            initial_standing_profiles_[l][j] = nullptr;
        }
    }

    double vmax_deg = (params.max_angular_velocity > 0.0) ? params.max_angular_velocity : DEFAULT_MAX_ANGULAR_VELOCITY;
    double vmax = vmax_deg * DEGREES_TO_RADIANS_FACTOR;
    double amax = vmax * 4.0;
    double jmax = amax * 10.0;

    // Phase 1: only align coxa joints to target orientation (leave femur/tibia unchanged for now)
    initial_standing_total_time_ = 0.0;
    initial_standing_phase_ = InitialStandingPhase::ALIGN;
    initial_standing_lift_profiles_created_ = false;
    for (int l = 0; l < NUM_LEGS; ++l) {
        JointAngles ja = legs[l].getJointAngles();
        StandingPoseJoints sj = body_pose_config.standing_pose_joints[l];
        double current = ja.coxa;
        double target = sj.coxa;
        if (std::fabs(current - target) < 1e-6) {
            initial_standing_profiles_[l][0] = nullptr; // already aligned
            continue;
        }
        auto *p = new SCurveProfile();
        p->init(current, target, vmax, amax, jmax);
        initial_standing_profiles_[l][0] = p;
        initial_standing_total_time_ = std::max(initial_standing_total_time_, p->totalTime());
    }

    if (initial_standing_total_time_ <= 0.0) {
        setStandingPose(legs);
        initial_standing_active_ = false;
        initial_standing_time_ = 0.0;
        return true; // finished immediately
    }

    initial_standing_time_ = 0.0;
    initial_standing_active_ = true;
    return true;
}

bool BodyPoseController::isInitialStandingAligned(const Leg legs[NUM_LEGS]) const {
    if (!(initial_standing_active_ && initial_standing_phase_ == InitialStandingPhase::ALIGN))
        return false;
    for (int l = 0; l < NUM_LEGS; ++l) {
        StandingPoseJoints sj = body_pose_config.standing_pose_joints[l];
        double cur = legs[l].getJointAngles().coxa;
        if (std::fabs(cur - sj.coxa) > initial_standing_align_tolerance_)
            return false;
    }
    return true;
}

bool BodyPoseController::stepInitialStandingPoseTransition(Leg legs[NUM_LEGS], double dt,
                                                           double out_pos[NUM_LEGS][DOF_PER_LEG],
                                                           double out_vel[NUM_LEGS][DOF_PER_LEG],
                                                           double out_acc[NUM_LEGS][DOF_PER_LEG]) {
    if (!initial_standing_active_) {
        return false; // nothing to advance
    }
    if (dt <= 0.0) {
        dt = model.getTimeDelta();
        if (dt <= 0.0)
            dt = 0.02;
    }
    initial_standing_time_ = std::min(initial_standing_time_ + dt, initial_standing_total_time_);
    double t = initial_standing_time_;

    // Phase handling
    if (initial_standing_phase_ == InitialStandingPhase::ALIGN) {
        // Sample only coxa profiles
        for (int l = 0; l < NUM_LEGS; ++l) {
            JointAngles ja = legs[l].getJointAngles();
            auto *p = initial_standing_profiles_[l][0];
            if (p) {
                auto s = p->sample(t);
                ja.coxa = s.position;
                if (out_pos)
                    out_pos[l][0] = s.position;
                if (out_vel)
                    out_vel[l][0] = s.velocity;
                if (out_acc)
                    out_acc[l][0] = s.acceleration;
            }
            legs[l].setJointAngles(ja);
        }
        bool align_done = (initial_standing_time_ >= initial_standing_total_time_ - 1e-9);
        if (align_done) {
            // Free coxa profiles
            for (int l = 0; l < NUM_LEGS; ++l) {
                delete initial_standing_profiles_[l][0];
                initial_standing_profiles_[l][0] = nullptr;
            }
            // Prepare LIFT phase profiles (femur & tibia) lazily
            initial_standing_phase_ = InitialStandingPhase::LIFT;
            initial_standing_time_ = 0.0;
            initial_standing_total_time_ = 0.0;
            // Recompute dynamic limits (reuse same heuristic as begin)
            const Parameters &params = model.getParams();
            double vmax_deg = (params.max_angular_velocity > 0.0) ? params.max_angular_velocity : DEFAULT_MAX_ANGULAR_VELOCITY;
            double vmax = vmax_deg * DEGREES_TO_RADIANS_FACTOR;
            double amax = vmax * 4.0;
            double jmax = amax * 10.0;
            for (int l = 0; l < NUM_LEGS; ++l) {
                JointAngles ja = legs[l].getJointAngles();
                StandingPoseJoints sj = body_pose_config.standing_pose_joints[l];
                // Femur (index 1)
                {
                    double cur = ja.femur;
                    double tgt = sj.femur;
                    if (std::fabs(cur - tgt) > 1e-6) {
                        auto *p = new SCurveProfile();
                        p->init(cur, tgt, vmax, amax, jmax);
                        initial_standing_profiles_[l][1] = p;
                        initial_standing_total_time_ = std::max(initial_standing_total_time_, p->totalTime());
                    }
                }
                // Tibia (index 2)
                {
                    double cur = ja.tibia;
                    double tgt = sj.tibia;
                    if (std::fabs(cur - tgt) > 1e-6) {
                        auto *p = new SCurveProfile();
                        p->init(cur, tgt, vmax, amax, jmax);
                        initial_standing_profiles_[l][2] = p;
                        initial_standing_total_time_ = std::max(initial_standing_total_time_, p->totalTime());
                    }
                }
            }
            if (initial_standing_total_time_ <= 0.0) {
                // Already at target for femur/tibia -> finalize immediately
                for (int l = 0; l < NUM_LEGS; ++l) {
                    JointAngles ja = legs[l].getJointAngles();
                    Point3D pos = model.forwardKinematicsGlobalCoordinates(l, ja);
                    legs[l].setCurrentTipPositionGlobal(pos);
                    legs[l].setStepPhase(STANCE_PHASE);
                    if (leg_posers_[l]) {
                        leg_posers_[l]->get()->setTargetPosition(pos);
                        leg_posers_[l]->get()->resetStepToPosition();
                    }
                }
                initial_standing_active_ = false;
                return true;
            }
            return true; // continue with new phase
        }
        return false; // still aligning
    } else {          // LIFT phase
        for (int l = 0; l < NUM_LEGS; ++l) {
            JointAngles ja = legs[l].getJointAngles();
            for (int j = 1; j < DOF_PER_LEG; ++j) { // femur, tibia
                auto *p = initial_standing_profiles_[l][j];
                if (!p)
                    continue;
                auto s = p->sample(t);
                if (j == 1)
                    ja.femur = s.position;
                else
                    ja.tibia = s.position;
                if (out_pos)
                    out_pos[l][j] = s.position;
                if (out_vel)
                    out_vel[l][j] = s.velocity;
                if (out_acc)
                    out_acc[l][j] = s.acceleration;
            }
            legs[l].setJointAngles(ja);
        }
        bool lift_done = (initial_standing_time_ >= initial_standing_total_time_ - 1e-9);
        if (lift_done) {
            for (int l = 0; l < NUM_LEGS; ++l) {
                JointAngles ja = legs[l].getJointAngles();
                Point3D pos = model.forwardKinematicsGlobalCoordinates(l, ja);
                legs[l].setCurrentTipPositionGlobal(pos);
                legs[l].setStepPhase(STANCE_PHASE);
                if (leg_posers_[l]) {
                    leg_posers_[l]->get()->setTargetPosition(pos);
                    leg_posers_[l]->get()->resetStepToPosition();
                }
                for (int j = 1; j < DOF_PER_LEG; ++j) {
                    delete initial_standing_profiles_[l][j];
                    initial_standing_profiles_[l][j] = nullptr;
                }
            }
            initial_standing_active_ = false;
            return true;
        }
        return false; // still lifting
    }
}

// Tripod leg coordination for stance transition (OpenSHC equivalent)
bool BodyPoseController::stepToNewStance(Leg legs[NUM_LEGS], double step_height, double step_time) {
    // OpenSHC stepToNewStance() implementation - Tripod coordination only
    // "The stepping motion is coordinated such that half of the legs execute the step at any one time
    //  (for a hexapod this results in a Tripod stepping coordination)"

    const int leg_count = NUM_LEGS;
    const int legs_per_group = leg_count / 2; // 3 legs per group for hexapod
    bool all_current_group_complete = true;

    // Initialize sequence if not already done
    if (!step_to_new_stance_sequence_generated) {
        // Set target positions using LegStepper's default tip poses (OpenSHC approach)
        // This ensures continuity with gait execution instead of jumping to static standing poses
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg_posers_[i]) {
                // Get default tip pose from leg's stepper (OpenSHC equivalent)
                // This maintains continuity with the planned gait trajectory
                Point3D current_position = legs[i].getCurrentTipPositionGlobal();

                // Use current position as target to maintain stance (OpenSHC behavior)
                // The actual step target will be set by gait execution, not startup sequence
                leg_posers_[i]->get()->setTargetPosition(current_position);
            }
        }
        step_to_new_stance_current_group = 0;
        step_to_new_stance_sequence_generated = true;
    }

    // Process current group legs (OpenSHC approach)
    int completed_legs_in_group = 0;
    for (int idx = 0; idx < legs_per_group; idx++) {
        int leg_index = tripod_leg_groups[step_to_new_stance_current_group][idx];

        if (leg_posers_[leg_index]) {
            LegPoser *leg_poser = leg_posers_[leg_index]->get();

            // Set legs in current group to SWING phase during movement
            legs[leg_index].setStepPhase(SWING_PHASE);

            // Execute step to target position for this leg
            bool leg_complete = leg_poser->stepToPosition(leg_poser->getTargetPosition(), step_height, step_time);

            // Update leg state from poser (OpenSHC pattern)
            legs[leg_index].setCurrentTipPositionGlobal(leg_poser->getCurrentPosition());
            JointAngles current_angles = legs[leg_index].getJointAngles();
            legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, legs[leg_index].getCurrentTipPositionGlobal()));

            if (leg_complete) {
                // When leg completes movement, set back to STANCE phase
                legs[leg_index].setStepPhase(STANCE_PHASE);
                completed_legs_in_group++;
            } else {
                all_current_group_complete = false;
            }
        }
    }

    // Ensure legs not in current group are in STANCE phase
    for (int i = 0; i < NUM_LEGS; i++) {
        bool in_current_group = false;
        for (int idx = 0; idx < legs_per_group; idx++) {
            if (i == tripod_leg_groups[step_to_new_stance_current_group][idx]) {
                in_current_group = true;
                break;
            }
        }
        if (!in_current_group) {
            legs[i].setStepPhase(STANCE_PHASE);
        }
    }

    // Check if current group is complete and advance to next group or finish
    if (all_current_group_complete && completed_legs_in_group == legs_per_group) {
        if (step_to_new_stance_current_group == 0) {
            // Move to second group (Group B)
            step_to_new_stance_current_group = 1;
            // Reset posers for second group
            for (int idx = 0; idx < legs_per_group; idx++) {
                int leg_index = tripod_leg_groups[step_to_new_stance_current_group][idx];
                if (leg_posers_[leg_index]) {
                    leg_posers_[leg_index]->get()->resetStepToPosition();
                }
            }
        } else {
            // Both groups complete - sequence finished
            step_to_new_stance_sequence_generated = false;
            step_to_new_stance_current_group = 0;
            return true;
        }
    }

    return false;
}

// Execute startup sequence (READY -> RUNNING transition)
bool BodyPoseController::executeStartupSequence(Leg legs[NUM_LEGS]) {
    // OpenSHC logic replication: transition sequence with alternating horizontal and vertical
    // steps to reach standing pose. Intermediate targets are generated on first run and reused.

    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // First execution initialization
    if (first_sequence_execution_) {
        // Phase 0: store initial positions (implicit first target)
        // Phase 1 (horizontal): move XY towards standing target preserving current Z
        // Phase 2 (vertical): adjust Z down/up to standing

        // Build final standing targets
        for (int i = 0; i < NUM_LEGS; ++i) {
            const auto &standing_joints = body_pose_config.standing_pose_joints[i];
            JointAngles sj;
            sj.coxa = standing_joints.coxa;
            sj.femur = standing_joints.femur;
            sj.tibia = standing_joints.tibia;
            Point3D standing_pos = model.forwardKinematicsGlobalCoordinates(i, sj);
            startup_final_targets_[i] = standing_pos;

            // Horizontal: keep initial Z, take standing XY
            Point3D init_pos = legs[i].getCurrentTipPositionGlobal();
            Point3D horiz = standing_pos;
            horiz.z = init_pos.z; // mantener z
            startup_horizontal_targets_[i] = horiz;
        }
        transition_step_ = 0;       // 0 = horizontal, 1 = vertical
        transition_step_count_ = 2; // Two total phases
        set_target_ = true;
        proximity_alert_ = false;
        horizontal_transition_complete_ = false;
        vertical_transition_complete_ = false;
        executing_transition_ = true;
        first_sequence_execution_ = false; // After setup
    }

    // Timings similar to OpenSHC constants
    double step_frequency = DEFAULT_STEP_FREQUENCY;     // TODO map from params
    double horiz_time = 1.0 / step_frequency;           // Normalized horizontal transition time
    double vert_time = 3.0 / step_frequency;            // Normalized vertical transition time
    double lift_height = body_pose_config.swing_height; // Use swing height for horizontal phase

    bool sequence_complete = false;

    // Horizontal phase (XY toward target)
    if (transition_step_ == 0) {
        bool all_complete = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!leg_posers_[i])
                continue;
            LegPoser *lp = leg_posers_[i]->get();
            if (set_target_) {
                lp->setTargetPosition(startup_horizontal_targets_[i]);
                lp->resetStepToPosition();
            }
            bool done = lp->stepToPosition(lp->getTargetPosition(), lift_height, horiz_time);
            legs[i].setCurrentTipPositionGlobal(lp->getCurrentPosition());
            JointAngles ca = legs[i].getJointAngles();
            legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, ca, legs[i].getCurrentTipPositionGlobal()));
            all_complete = all_complete && done;
        }
        set_target_ = false;
        if (all_complete) {
            horizontal_transition_complete_ = true;
            set_target_ = true; // prepare vertical phase
            transition_step_ = 1;
        }
    }
    // Vertical phase (Z adjustment)
    else if (transition_step_ == 1) {
        bool all_complete = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!leg_posers_[i])
                continue;
            LegPoser *lp = leg_posers_[i]->get();
            if (set_target_) {
                lp->setTargetPosition(startup_final_targets_[i]);
                lp->resetStepToPosition();
            }
            // Vertical: adjust Z without additional lift
            bool done = lp->stepToPosition(lp->getTargetPosition(), 0.0, vert_time);
            legs[i].setCurrentTipPositionGlobal(lp->getCurrentPosition());
            JointAngles ca = legs[i].getJointAngles();
            legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, ca, legs[i].getCurrentTipPositionGlobal()));
            all_complete = all_complete && done;
        }
        set_target_ = false;
        if (all_complete) {
            vertical_transition_complete_ = true;
            sequence_complete = true;
        }
    }

    if (sequence_complete) {
        executing_transition_ = false;
        transition_step_ = transition_step_count_;
        // Ensure exact final standing pose
        for (int i = 0; i < NUM_LEGS; ++i) {
            const auto &standing_joints = body_pose_config.standing_pose_joints[i];
            JointAngles sj;
            sj.coxa = standing_joints.coxa;
            sj.femur = standing_joints.femur;
            sj.tibia = standing_joints.tibia;
            legs[i].setJointAngles(sj);
            Point3D pos = model.forwardKinematicsGlobalCoordinates(i, sj);
            legs[i].setCurrentTipPositionGlobal(pos);
        }
        return true;
    }

    return false; // still in progress
}

int BodyPoseController::getStartupProgressPercent() const {
    if (transition_step_count_ == 0)
        return 0;
    if (transition_step_ >= transition_step_count_)
        return 100;
    double phase_fraction = (transition_step_ == 0 ? 0.0 : 0.5);
    double leg_avg = 0.0;
    int counted = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (leg_posers_[i]) {
            // LegPoserImpl expected to expose underlying LegPoser via get()
            leg_avg += leg_posers_[i]->get()->getCurrentStepProgress();
            counted++;
        }
    }
    if (counted > 0)
        leg_avg /= counted;
    double phase_progress = leg_avg * 0.5; // each phase worth 50%
    int percent = static_cast<int>((phase_fraction + phase_progress) * 100.0 + 0.5);
    if (percent > 99 && transition_step_ < transition_step_count_)
        percent = 99;
    if (percent < 0)
        percent = 0;
    return percent;
}

// Execute shutdown sequence (RUNNING -> READY transition - OpenSHC equivalent)
bool BodyPoseController::executeShutdownSequence(Leg legs[NUM_LEGS]) {
    // OpenSHC shutdown is essentially the reverse of startup
    // It moves legs from their current walking positions back to ready/standing positions
    // Unlike startup, shutdown doesn't need different methods for different gaits
    // All legs move simultaneously back to standing configuration

    // Initialize sequence if not done
    if (!shutdown_sequence_initialized) {
        shutdown_sequence_initialized = true;

        // Initialize all leg posers for shutdown
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg_posers_[i]) {
                // Set target to standing pose position
                const auto &standing_joints = body_pose_config.standing_pose_joints[i];
                JointAngles target_angles;
                target_angles.coxa = standing_joints.coxa;
                target_angles.femur = standing_joints.femur;
                target_angles.tibia = standing_joints.tibia;

                Point3D target_position = model.forwardKinematicsGlobalCoordinates(i, target_angles);
                leg_posers_[i]->get()->setTargetPosition(target_position);
                leg_posers_[i]->get()->resetStepToPosition();
            }
        }
    }

    // Execute shutdown transition for all legs simultaneously
    double step_height = body_pose_config.swing_height;
    double step_time = 1.0 / DEFAULT_STEP_FREQUENCY;
    bool all_legs_complete = true;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_posers_[i]) {
            LegPoser *leg_poser = leg_posers_[i]->get();

            // Step to standing position (OpenSHC approach)
            bool leg_complete = leg_poser->stepToPosition(leg_poser->getTargetPosition(), step_height, step_time);

            // Update leg with current position from poser
            legs[i].setCurrentTipPositionGlobal(leg_poser->getCurrentPosition());
            JointAngles current_angles = legs[i].getJointAngles();
            legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, legs[i].getCurrentTipPositionGlobal()));

            if (!leg_complete) {
                all_legs_complete = false;
            }
        }
    }

    // Reset state when sequence completes
    if (all_legs_complete) {
        shutdown_sequence_initialized = false;
        return true;
    }

    return false;
}

// Update auto-pose during gait execution (OpenSHC equivalent)
bool BodyPoseController::updateAutoPose(double gait_phase, Leg legs[NUM_LEGS]) {
    // Mantener plano de caminata actualizado (despeje del cuerpo)
    updateWalkPlanePose(legs);
    if (!auto_pose_enabled || !auto_pose_config.enabled)
        return true; // nothing to do

    // 1. Determine pose cycle length.
    //    If pose_frequency == -1.0 we assume sync with gait step cycle and use configured pose_phase_length.
    //    Otherwise we still use pose_phase_length as discrete resolution for the pose cycle.
    int base_period = auto_pose_config.pose_phase_length;
    if (base_period <= 0) {
        // Derive fallback from the largest index found in starts/ends (robust for partial configurations)
        int max_idx = 0;
        for (int v : auto_pose_config.pose_phase_starts)
            if (v > max_idx)
                max_idx = v;
        for (int v : auto_pose_config.pose_phase_ends)
            if (v > max_idx)
                max_idx = v;
        base_period = std::max(4, max_idx + 1); // reasonable minimum
    }

    // 2. Convert gait_phase [0,1) to integer phase index in [0, base_period)
    double wrapped = gait_phase - std::floor(gait_phase);
    int current_phase_index = static_cast<int>(wrapped * base_period) % base_period;

    // 3. Cyclic window utilities and modular distance helpers
    auto inWindow = [base_period](int start, int end, int value) {
        if (start == end)
            return false; // empty window
        if (start < end)
            return value >= start && value < end;
        return value >= start || value < end; // wraps end of cycle
    };
    auto modDist = [base_period](int from, int to) {
        return (to - from + base_period) % base_period; // forward modular distance [0, base_period)
    };
    auto smoothstep = [](double x) {
        x = math_utils::clamp(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    };

    // 4. Active amplitude selection (average of all phases whose window contains the current index)
    auto computeAxis = [&](const std::vector<double> &amps) {
        if (amps.empty())
            return 0.0;
        double acc = 0.0;
        int count = 0;
        for (size_t i = 0; i < auto_pose_config.pose_phase_starts.size() && i < amps.size(); ++i) {
            if (inWindow(auto_pose_config.pose_phase_starts[i], auto_pose_config.pose_phase_ends[i], current_phase_index)) {
                acc += amps[i];
                ++count;
            }
        }
        return count > 0 ? acc / count : 0.0;
    };

    double base_roll = computeAxis(auto_pose_config.roll_amplitudes);
    double base_pitch = computeAxis(auto_pose_config.pitch_amplitudes);
    double base_yaw = computeAxis(auto_pose_config.yaw_amplitudes);
    double base_x = computeAxis(auto_pose_config.x_amplitudes);
    double base_y = computeAxis(auto_pose_config.y_amplitudes);
    double base_z = computeAxis(auto_pose_config.z_amplitudes);
    double gravity_factor = computeAxis(auto_pose_config.gravity_amplitudes); // scaling factor (>=0)

    // 5. Delegar por pierna al LegPoser (paridad OpenSHC). Cada LegPoser recalcula offsets y aplica umbral.
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        if (leg_posers_[leg_index]) {
            leg_posers_[leg_index]->get()->updateAutoPose(current_phase_index, auto_pose_config, body_pose_config);
        }
    }
    return true;
}

// Walk plane pose system implementation (OpenSHC equivalent)
void BodyPoseController::updateWalkPlanePose(Leg legs[NUM_LEGS]) {
    if (!walk_plane_pose_enabled) {
        return;
    }

    // Calculate walk plane normal and height from stance leg positions
    Point3D walk_plane_normal = calculateWalkPlaneNormal(legs);
    double walk_plane_height = calculateWalkPlaneHeight(legs);

    // Create target walk plane pose
    Pose target_walk_plane_pose;
    target_walk_plane_pose.position = Point3D(0.0, 0.0, walk_plane_height + body_pose_config.body_clearance);

    // Set orientation: align body with walk plane normal
    Eigen::Vector3d unit_z(0.0, 0.0, 1.0);
    Eigen::Vector3d walk_normal(walk_plane_normal.x, walk_plane_normal.y, walk_plane_normal.z);
    walk_normal.normalize();
    target_walk_plane_pose.rotation = Eigen::Quaterniond::FromTwoVectors(unit_z, walk_normal);

    // Check if change is significant enough to update
    double position_change = (target_walk_plane_pose.position - walk_plane_pose_.position).norm();
    double rotation_change = target_walk_plane_pose.rotation.angularDistance(walk_plane_pose_.rotation);

    bool need_update = (position_change > walk_plane_update_threshold || rotation_change > 0.01);

    if (!need_update) {
        // Only advance if a Bézier transition is currently active
        if (walk_plane_bezier_in_progress) {
            walk_plane_bezier_time += model.getTimeDelta();
            double t = std::min(1.0, walk_plane_bezier_time / walk_plane_bezier_duration);
            double smooth_t = math_utils::smoothStep(t);
            walk_plane_pose_.position = math_utils::quarticBezier(walk_plane_position_nodes, smooth_t);
            walk_plane_pose_.rotation = walk_plane_rotation_nodes[0].slerp(smooth_t, walk_plane_rotation_nodes[4]);
            if (t >= 1.0) {
                walk_plane_bezier_in_progress = false;
#ifdef TESTING_ENABLED
                std::cout << "[WalkPlane] Bezier transition completed." << std::endl;
#endif
            }
        }
        return;
    }

#ifdef TESTING_ENABLED
    std::cout << "[WalkPlane] target_height=" << target_walk_plane_pose.position.z
              << " current_height=" << walk_plane_pose_.position.z
              << " pos_change=" << position_change << " rot_change=" << rotation_change << std::endl;
#endif

    // Small changes -> direct assignment (skip Bézier smoothing for minor adjustments)
    if (position_change < 200.0 && rotation_change < 0.1) {
        walk_plane_pose_ = target_walk_plane_pose;
        walk_plane_bezier_in_progress = false;
#ifdef TESTING_ENABLED
        std::cout << "[WalkPlane] Direct assignment applied. New height=" << walk_plane_pose_.position.z << std::endl;
#endif
        return;
    }

    // Determine if the target changed significantly (decides whether to (re)build Bézier control nodes)
    bool target_changed = !walk_plane_bezier_in_progress ||
                          (target_walk_plane_pose.position - walk_plane_position_nodes[4]).norm() > 1e-3 ||
                          walk_plane_rotation_nodes[4].angularDistance(target_walk_plane_pose.rotation) > 1e-4;

    if (target_changed) {
        Point3D start_position = walk_plane_pose_.position;
        Point3D end_position = target_walk_plane_pose.position;
        Eigen::Quaterniond start_rotation = walk_plane_pose_.rotation;
        Eigen::Quaterniond end_rotation = target_walk_plane_pose.rotation;

        // Generate quartic Bézier control nodes for position (OpenSHC-inspired smoothing) only when target changes
        walk_plane_position_nodes[0] = start_position;
        walk_plane_position_nodes[1] = start_position + (end_position - start_position) * 0.2;
        walk_plane_position_nodes[2] = start_position + (end_position - start_position) * 0.5;
        walk_plane_position_nodes[3] = start_position + (end_position - start_position) * 0.8;
        walk_plane_position_nodes[4] = end_position;

        // Generate corresponding Slerp-based control nodes for orientation interpolation
        walk_plane_rotation_nodes[0] = start_rotation;
        walk_plane_rotation_nodes[1] = start_rotation.slerp(0.2, end_rotation);
        walk_plane_rotation_nodes[2] = start_rotation.slerp(0.5, end_rotation);
        walk_plane_rotation_nodes[3] = start_rotation.slerp(0.8, end_rotation);
        walk_plane_rotation_nodes[4] = end_rotation;

        walk_plane_bezier_in_progress = true;
        walk_plane_bezier_time = 0.0;

#ifdef TESTING_ENABLED
        std::cout << "[WalkPlane] Bezier transition (re)started." << std::endl;
#endif
    }

    // Advance current Bézier transition
    if (walk_plane_bezier_in_progress) {
        walk_plane_bezier_time += model.getTimeDelta();
        double t = std::min(1.0, walk_plane_bezier_time / walk_plane_bezier_duration);
        double smooth_t = math_utils::smoothStep(t);
        walk_plane_pose_.position = math_utils::quarticBezier(walk_plane_position_nodes, smooth_t);
        walk_plane_pose_.rotation = walk_plane_rotation_nodes[0].slerp(smooth_t, walk_plane_rotation_nodes[4]);
        if (t >= 1.0) {
            walk_plane_bezier_in_progress = false;

#ifdef TESTING_ENABLED
            std::cout << "[WalkPlane] Bezier transition completed." << std::endl;
#endif
        }
    }
}

Point3D BodyPoseController::calculateWalkPlaneNormal(Leg legs[NUM_LEGS]) const {
    // Collect stance leg positions for plane fitting
    std::vector<Point3D> stance_points;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            stance_points.push_back(legs[i].getCurrentTipPositionGlobal());
        }
    }

    // Need at least 3 points to define a plane
    if (stance_points.size() < 3) {
        return Point3D(0.0, 0.0, 1.0); // Default to horizontal plane
    }

    // Prepare data for least squares plane fitting
    std::vector<double> raw_A;
    std::vector<double> raw_B;

    for (const auto &point : stance_points) {
        raw_A.push_back(point.x);
        raw_A.push_back(point.y);
        raw_A.push_back(1.0);
        raw_B.push_back(point.z);
    }

    // Solve for plane equation: ax + by + c = z
    double a, b, c;
    if (math_utils::solveLeastSquaresPlane(raw_A.data(), raw_B.data(), stance_points.size(), a, b, c)) {
        // Convert plane coefficients to normal vector
        // Plane equation: ax + by - z + c = 0
        // Normal vector: (a, b, -1)
        double normal_magnitude = std::sqrt(a * a + b * b + 1.0);
        Point3D normal(-a / normal_magnitude, -b / normal_magnitude, 1.0 / normal_magnitude);

        // Ensure normal points upward (positive Z component)
        if (normal.z < 0) {
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        return normal;
    }

    // Fallback to horizontal plane if calculation fails
    return Point3D(0.0, 0.0, 1.0);
}

double BodyPoseController::calculateWalkPlaneHeight(Leg legs[NUM_LEGS]) const {
    // Calculate average Z position of stance legs
    double total_z = 0.0;
    int stance_count = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            total_z += legs[i].getCurrentTipPositionGlobal().z;
            stance_count++;
        }
    }

    if (stance_count > 0) {
        return total_z / stance_count;
    }

    // Fallback: use current walk plane height minus body clearance
    return walk_plane_pose_.position.z - body_pose_config.body_clearance;
}

Pose BodyPoseController::getWalkPlanePose() const {
    return walk_plane_pose_;
}

void BodyPoseController::setWalkPlanePose(const Pose &pose) {
    walk_plane_pose_ = pose;

    // Reset Bézier transition state
    walk_plane_bezier_in_progress = false;
    walk_plane_bezier_time = 0.0;

    // Initialize all control nodes to current pose
    for (int i = 0; i < 5; i++) {
        walk_plane_position_nodes[i] = pose.position;
        walk_plane_rotation_nodes[i] = pose.rotation;
    }
}

void BodyPoseController::setWalkPlanePoseEnabled(bool enabled) {
    walk_plane_pose_enabled = enabled;
    if (enabled) {
        // Reset to neutral position
        Pose neutral_pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
        walk_plane_pose_ = neutral_pose;

        // Reset Bézier transition state
        walk_plane_bezier_in_progress = false;
        walk_plane_bezier_time = 0.0;

        // Initialize all control nodes to neutral pose
        for (int i = 0; i < 5; i++) {
            walk_plane_position_nodes[i] = neutral_pose.position;
            walk_plane_rotation_nodes[i] = neutral_pose.rotation;
        }
    }
}

bool BodyPoseController::isWalkPlanePoseEnabled() const {
    return walk_plane_pose_enabled;
}

int BodyPoseController::packLegs(Leg legs[NUM_LEGS], double time_to_pack) {
    // OpenSHC packLegs implementation adapted for HexaMotion

    // Get pack height from configuration
    double pack_height = 150.0; // Default pack height in mm

    // Get body position as pack position (use default center position)
    Point3D pack_position;
    pack_position.x = 0.0;         // Center position
    pack_position.y = 0.0;         // Center position
    pack_position.z = pack_height; // Raise to pack height

    int completed_legs = 0;

    // Step all legs to pack position
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        if (leg_posers_[leg_index]) {
            bool success = leg_posers_[leg_index]->get()->stepToPosition(pack_position, 20.0, 1.0); // 20mm step height, 1s time
            if (success) {
                // Update leg state
                legs[leg_index].setCurrentTipPositionGlobal(pack_position);
                JointAngles current_angles = legs[leg_index].getJointAngles();
                legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, pack_position));
                completed_legs++;
            }
        }
    }

    // Update pack step tracking
    pack_step_++;
    legs_completed_step_ = completed_legs;

    // Return progress percentage
    int progress = (completed_legs * 100) / NUM_LEGS;
    return progress;
}

int BodyPoseController::unpackLegs(Leg legs[NUM_LEGS], double time_to_unpack) {
    // OpenSHC unpackLegs implementation adapted for HexaMotion

    // Get default stance height
    double stance_height = 50.0; // Default stance height in mm

    int completed_legs = 0;

    // Calculate unpack positions for each leg
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        // Calculate default standing position for each leg based on robot geometry
        Point3D default_position;

        // Basic hexapod leg positioning (simplified)
        double radius = 120.0;                   // Default radius from center in mm
        double angle = leg_index * (M_PI / 3.0); // 60 degrees between legs

        default_position.x = radius * cos(angle);
        default_position.y = radius * sin(angle);
        default_position.z = stance_height;

        if (leg_posers_[leg_index]) {
            bool success = leg_posers_[leg_index]->get()->stepToPosition(default_position, 20.0, 1.0); // 20mm step height, 1s time
            if (success) {
                // Update leg state
                legs[leg_index].setCurrentTipPositionGlobal(default_position);
                JointAngles current_angles = legs[leg_index].getJointAngles();
                legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, default_position));
                completed_legs++;
            }
        }
    }

    // Update tracking
    pack_step_--;
    legs_completed_step_ = completed_legs;

    // Return progress percentage
    int progress = (completed_legs * 100) / NUM_LEGS;
    return progress;
}

int BodyPoseController::poseForLegManipulation(Leg legs[NUM_LEGS]) {
    // OpenSHC poseForLegManipulation implementation adapted for HexaMotion
    // Generates poses for manual leg manipulation while maintaining stability

    // Create manipulation pose using HexaMotion types
    Eigen::Vector3d position(0.0, 0.0, -10.0);  // Lower by 10mm
    Eigen::Vector3d orientation(0.0, 5.0, 0.0); // 5 degree forward pitch in degrees

    // Optional: slight roll for asymmetric leg operations
    if (current_group_ == 0) {        // Right legs
        orientation[0] = -2.0;        // Slight right lean
    } else if (current_group_ == 1) { // Left legs
        orientation[0] = 2.0;         // Slight left lean
    }

    // Apply the manipulation pose to the robot
    bool success = setBodyPose(position, orientation, legs);

    // Return completion percentage
    return success ? 100 : 0;
}

int BodyPoseController::executeSequence(const std::string &sequence_type, Leg legs[NUM_LEGS]) {
    // OpenSHC executeSequence implementation adapted for HexaMotion
    // This is the main sequence execution method that handles complex startup/shutdown

    // Initialize sequence execution state
    executing_transition_ = true;
    transition_step_ = 0;

    int progress = 0;

    // Execute sequence based on type
    if (sequence_type == "startup") {
        bool complete = executeStartupSequence(legs);
        progress = complete ? 100 : 50; // Simple progress tracking
    } else if (sequence_type == "shutdown") {
        bool complete = executeShutdownSequence(legs);
        progress = complete ? 100 : 50; // Simple progress tracking
    } else {
        // Unknown sequence type
        executing_transition_ = false;
        return 0;
    }

    // Update sequence state
    if (progress == 100) {
        executing_transition_ = false;
        transition_step_ = transition_step_count_;
    }

    return progress;
}
