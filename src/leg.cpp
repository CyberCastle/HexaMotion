#include "leg.h"
#include "hexamotion_constants.h"

Leg::Leg(int leg_id, const RobotModel &model)
    : model_(model), leg_id_(leg_id), leg_name_(("Leg_" + std::to_string(leg_id)).c_str()), joint_angles_(0.0, 0.0, 0.0), tip_position_(0.0, 0.0, 0.0), base_position_(0.0, 0.0, 0.0), step_phase_(STANCE_PHASE), gait_phase_(0.0), in_contact_(false), contact_force_(0.0), fsr_history_index_(0), leg_phase_offset_(0.0), desired_tip_position_(0.0, 0.0, 0.0), default_angles_(0.0, 0.0, 0.0), default_tip_position_(0.0, 0.0, 0.0) {

    // Initialize FSR contact history
    for (int i = 0; i < 3; ++i) {
        fsr_contact_history_[i] = 0.0;
    }

    // Validate leg ID
    if (leg_id < 0 || leg_id >= NUM_LEGS) {
        // Invalid leg ID - this should not happen in normal operation
        return;
    }

    // Calculate base position
    base_position_ = model_.getLegBasePosition(leg_id_);
    base_position_.z = 0.0;
}

void Leg::setJointAngles(const JointAngles &angles) {
    joint_angles_ = angles;
    // Synchronize tip position using forward kinematics
    updateTipPosition();
}

double Leg::getJointAngle(int joint_index) const {
    switch (joint_index) {
    case 0:
        return joint_angles_.coxa;
    case 1:
        return joint_angles_.femur;
    case 2:
        return joint_angles_.tibia;
    default:
        return 0.0;
    }
}

void Leg::setJointAngle(int joint_index, double angle) {
    switch (joint_index) {
    case 0:
        joint_angles_.coxa = angle;
        break;
    case 1:
        joint_angles_.femur = angle;
        break;
    case 2:
        joint_angles_.tibia = angle;
        break;
    }
    // Synchronize tip position using forward kinematics
    updateTipPosition();
}

void Leg::setCurrentTipPositionGlobal(const Point3D &position) {
    // Delegate reachability adjustment to RobotModel to avoid duplicating workspace logic.
    // makeReachable implements OpenSHC-style constraint (workplane + geometric fallback).
    Point3D reachable = model_.makeReachable(leg_id_, position);

    // Update stored tip position reference (joint angles not updated here; IK applied elsewhere when needed).
    tip_position_ = reachable;
}

/**
 * @brief Apply inverse kinematics to reach a target position and update joint angles & tip position.
 * @param target_position Desired global tip position
 * @return True if IK succeeds within joint limits
 */
bool Leg::applyIK(const Point3D &target_position) {
    // OpenSHC approach: Make target reachable before applying IK
    Point3D reachable_target = model_.makeReachable(leg_id_, target_position);

    // Compute new joint angles via IK using the reachable target
    JointAngles new_angles = model_.inverseKinematicsCurrentGlobalCoordinates(
        leg_id_, joint_angles_, reachable_target);

    // Validate limits before updating member variables
    if (!model_.checkJointLimits(leg_id_, new_angles)) {
        return false;
    }

    // Update joint angles and tip position
    joint_angles_ = new_angles;
    updateTipPosition();
    return true;
}

/**
 * @brief Apply advanced IK implementation with delta calculation and joint optimization
 * This method uses a robust IK solver that includes delta calculation and joint optimization
 * @param target_position Desired global tip position
 * @return True if IK succeeds within joint limits
 */
bool Leg::applyAdvancedIK(const Point3D &target_position) {
    // Get current tip position
    Point3D current_position = getCurrentTipPositionGlobal();

    // Make target reachable before applying IK
    Point3D reachable_target = model_.makeReachable(leg_id_, target_position);

    // Use advanced IK implementation
    // Use unified global time delta from model parameters for velocity estimation
    double time_delta = model_.getTimeDelta();
    JointAngles new_angles = model_.applyAdvancedIK(leg_id_, current_position, reachable_target, joint_angles_, time_delta);

    // Validate limits before updating member variables
    if (!model_.checkJointLimits(leg_id_, new_angles)) {
        return false;
    }

    // Update joint angles and tip position
    joint_angles_ = new_angles;
    updateTipPosition();
    return true;
}
Eigen::Matrix4d Leg::getTransform() const {
    return model_.legTransform(leg_id_, joint_angles_);
}

Eigen::Matrix3d Leg::getJacobian() const {
    return model_.calculateJacobian(leg_id_, joint_angles_, tip_position_);
}

void Leg::initialize(const Pose &default_stance) {
    // Calculate default tip position from stance pose
    Point3D stance_tip = default_stance.position;

    // Calculate IK for default stance using zero angles as starting point
    JointAngles zero_angles(0, 0, 0);
    JointAngles stance_angles = model_.inverseKinematicsCurrentGlobalCoordinates(leg_id_, zero_angles, stance_tip);

    // Set default configuration
    default_angles_ = stance_angles;
    default_tip_position_ = stance_tip;

    // Set current configuration to default
    joint_angles_ = stance_angles;
    tip_position_ = stance_tip;

    // Update FK to ensure consistency
    updateTipPosition();
}

void Leg::reset() {
    // Reset to default configuration
    joint_angles_ = default_angles_;
    tip_position_ = default_tip_position_;
    step_phase_ = STANCE_PHASE;
    gait_phase_ = 0.0;

    // Reset FSR contact history
    resetFSRHistory();

    // Update FK
    updateTipPosition();
}

double Leg::getDistanceToTarget(const Point3D &target) const {
    double dx = target.x - tip_position_.x;
    double dy = target.y - tip_position_.y;
    double dz = target.z - tip_position_.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

Eigen::Vector3d Leg::getLegDirection() const {
    Eigen::Vector3d direction(tip_position_.x - base_position_.x,
                              tip_position_.y - base_position_.y,
                              tip_position_.z - base_position_.z);
    return direction.normalized();
}

bool Leg::isInDefaultStance(double tolerance) const {
    return math_utils::distance(tip_position_, default_tip_position_) <= tolerance;
}

// ===== FSR CONTACT HISTORY METHODS =====
void Leg::updateFSRHistory(bool in_contact, double pressure) {
    // Store contact state as 1.0 for contact, 0.0 for no contact
    double contact_value = in_contact ? 1.0 : 0.0;

    // Update circular buffer
    fsr_contact_history_[fsr_history_index_] = contact_value;

    // Update index (circular buffer)
    fsr_history_index_ = (fsr_history_index_ + 1) % 3;

    // Update current contact state and force
    in_contact_ = in_contact;
    contact_force_ = pressure;
}

bool Leg::getFilteredContactState(double fsr_touchdown_threshold, double fsr_liftoff_threshold) const {
    // Calculate average contact value from history
    double avg_contact = getAverageContactValue();

    // Apply hysteresis for contact detection
    if (in_contact_) {
        // Currently in contact - use release threshold
        return avg_contact >= fsr_liftoff_threshold;
    } else {
        // Currently not in contact - use contact threshold
        return avg_contact >= fsr_touchdown_threshold;
    }
}

double Leg::getFSRHistoryValue(int index) const {
    if (index >= 0 && index < 3) {
        return fsr_contact_history_[index];
    }
    return 0.0;
}

double Leg::getAverageContactValue() const {
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        sum += fsr_contact_history_[i];
    }
    return sum / 3.0;
}

void Leg::resetFSRHistory() {
    for (int i = 0; i < 3; ++i) {
        fsr_contact_history_[i] = 0.0;
    }
    fsr_history_index_ = 0;
    in_contact_ = false;
    contact_force_ = 0.0;
}

// ===== GAIT PHASE OFFSET METHODS =====

void Leg::setPhaseOffset(double offset) {
    // Normalize offset to 0.0-1.0 range
    leg_phase_offset_ = fmod(offset, 1.0);
    if (leg_phase_offset_ < 0.0) {
        leg_phase_offset_ += 1.0;
    }
}

double Leg::calculateLegPhase(double global_gait_phase) const {
    // Calculate leg-specific phase by adding offset to global phase
    double leg_phase = fmod(global_gait_phase + leg_phase_offset_, 1.0);
    if (leg_phase < 0.0) {
        leg_phase += 1.0;
    }
    return leg_phase;
}

bool Leg::shouldBeInStance(double global_gait_phase, double stance_duration) const {
    double leg_phase = calculateLegPhase(global_gait_phase);
    return leg_phase < stance_duration;
}

bool Leg::shouldBeInSwing(double global_gait_phase, double stance_duration) const {
    double leg_phase = calculateLegPhase(global_gait_phase);
    return leg_phase >= stance_duration;
}

// Update tip position via forward kinematics
void Leg::updateTipPosition() {
    tip_position_ = model_.forwardKinematicsGlobalCoordinates(leg_id_, joint_angles_);
}