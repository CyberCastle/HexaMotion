#include "leg.h"
#include "hexamotion_constants.h"

Leg::Leg(int leg_id, const RobotModel &model)
    : model_(model), leg_id_(leg_id), leg_name_("Leg_" + std::to_string(leg_id)), joint_angles_(0.0, 0.0, 0.0), tip_position_(0.0, 0.0, 0.0), base_position_(0.0, 0.0, 0.0), step_phase_(STANCE_PHASE), gait_phase_(0.0), in_contact_(false), contact_force_(0.0), fsr_history_index_(0), leg_phase_offset_(0.0), default_angles_(0.0, 0.0, 0.0), default_tip_position_(0.0, 0.0, 0.0) {

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
    calculateBasePosition();
}

void Leg::setJointAngles(const JointAngles &angles) {
    joint_angles_ = angles;
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
}

bool Leg::setCurrentTipPositionGlobal(const Point3D &position) {
    // Store the target position
    Point3D target = position;

    // For global coordinates, we need to check reachability relative to the leg's base position
    Point3D leg_base = model_.getLegBasePosition(leg_id_);

    // Calculate distance from leg base to target position
    double dx = target.x - leg_base.x;
    double dy = target.y - leg_base.y;
    double dz = target.z - leg_base.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    double max_reach = getLegReach();

    if (distance > max_reach) {
        // TODO: OpenSHC approach: could adjust position to be within reach
        // For now, we reject it
        return false; // Target too far
    }

    // Update tip position (IK will be calculated when needed)
    tip_position_ = target;
    return true;
}

/**
 * @brief Apply inverse kinematics to reach a target position and update joint angles & tip position.
 * @param target_position Desired global tip position
 * @return True if IK succeeds within joint limits
 */
bool Leg::applyIK(const Point3D &target_position) {
    // Compute new joint angles via IK
    JointAngles new_angles = model_.inverseKinematicsCurrentGlobalCoordinates(
        leg_id_, joint_angles_, target_position);
    // Validate limits
    if (!checkJointLimits()) {
        return false;
    }
    // Update state
    joint_angles_ = new_angles;
    tip_position_ = target_position;

    // Ensure consistency via FK
    updateTipPosition();
    return true;
}

Eigen::Matrix4d Leg::getTransform() const {
    return model_.legTransform(leg_id_, joint_angles_);
}

Eigen::Matrix3d Leg::getJacobian() const {
    return model_.calculateJacobian(leg_id_, joint_angles_, tip_position_);
}

bool Leg::isTargetReachable(const Point3D &target) const {
    // Use RobotModel's workspace validation
    // This is a simplified check - full validation should be implemented
    double distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    double max_reach = getLegReach();

    return distance <= max_reach;
}

Point3D Leg::constrainToWorkspace(const Point3D &target) const {
    // Simple workspace constraint - scale target to max reach if outside
    double distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    double max_reach = getLegReach();

    if (distance > max_reach) {
        double scale = max_reach / distance;
        return Point3D(target.x * scale, target.y * scale, target.z * scale);
    }

    return target;
}

bool Leg::checkJointLimits() const {
    const Parameters &params = model_.getParams();
    return (joint_angles_.coxa >= params.coxa_angle_limits[0] &&
            joint_angles_.coxa <= params.coxa_angle_limits[1] &&
            joint_angles_.femur >= params.femur_angle_limits[0] &&
            joint_angles_.femur <= params.femur_angle_limits[1] &&
            joint_angles_.tibia >= params.tibia_angle_limits[0] &&
            joint_angles_.tibia <= params.tibia_angle_limits[1]);
}

double Leg::getJointLimitProximity() const {
    const Parameters &params = model_.getParams();
    // Calculate proximity for each joint (1.0 = far from limits, 0.0 = at limits)
    double coxa_range = params.coxa_angle_limits[1] - params.coxa_angle_limits[0];
    double femur_range = params.femur_angle_limits[1] - params.femur_angle_limits[0];
    double tibia_range = params.tibia_angle_limits[1] - params.tibia_angle_limits[0];

    double coxa_proximity = 1.0 - abs(joint_angles_.coxa - (params.coxa_angle_limits[0] + coxa_range / 2)) / (coxa_range / 2);
    double femur_proximity = 1.0 - abs(joint_angles_.femur - (params.femur_angle_limits[0] + femur_range / 2)) / (femur_range / 2);
    double tibia_proximity = 1.0 - abs(joint_angles_.tibia - (params.tibia_angle_limits[0] + tibia_range / 2)) / (tibia_range / 2);

    // Return minimum proximity (worst case)
    return std::min({coxa_proximity, femur_proximity, tibia_proximity});
}

void Leg::constrainJointLimits() {
    const Parameters &params = model_.getParams();
    joint_angles_.coxa = std::max(params.coxa_angle_limits[0],
                                  std::min(params.coxa_angle_limits[1], joint_angles_.coxa));
    joint_angles_.femur = std::max(params.femur_angle_limits[0],
                                   std::min(params.femur_angle_limits[1], joint_angles_.femur));
    joint_angles_.tibia = std::max(params.tibia_angle_limits[0],
                                   std::min(params.tibia_angle_limits[1], joint_angles_.tibia));
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

double Leg::getLegReach() const {
    const Parameters &params = model_.getParams();
    // Maximum reach is femur + tibia lengths (coxa only provides lateral offset)
    // The coxa rotates around Z-axis and doesn't extend the radial reach
    return params.femur_length + params.tibia_length;
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

// Calculate position delta in leg frame for synchronization
Point3D Leg::calculatePositionDelta(const Point3D &desired_position, const Point3D &current_position) const {
    // Use current joint angles to define the leg frame
    JointAngles current_angles = joint_angles_;

    // Transform positions to leg local coordinates
    Point3D desired_local = model_.transformGlobalToLocalCoordinates(leg_id_, desired_position, current_angles);
    Point3D current_local = model_.transformGlobalToLocalCoordinates(leg_id_, current_position, current_angles);

    // Compute delta in leg frame
    return Point3D(desired_local.x - current_local.x,
                   desired_local.y - current_local.y,
                   desired_local.z - current_local.z);
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

bool Leg::getFilteredContactState(double contact_threshold, double release_threshold) const {
    // Calculate average contact value from history
    double avg_contact = getAverageContactValue();

    // Apply hysteresis for contact detection
    if (in_contact_) {
        // Currently in contact - use release threshold
        return avg_contact >= release_threshold;
    } else {
        // Currently not in contact - use contact threshold
        return avg_contact >= contact_threshold;
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

// Calculate base position from robot model
void Leg::calculateBasePosition() {
    double angle_rad = model_.getLegBaseAngleOffset(leg_id_);
    const Parameters &params = model_.getParams();
    base_position_.x = params.hexagon_radius * cos(angle_rad);
    base_position_.y = params.hexagon_radius * sin(angle_rad);
    base_position_.z = 0.0;
}

// Update tip position via forward kinematics
void Leg::updateTipPosition() {
    tip_position_ = model_.forwardKinematicsGlobalCoordinates(leg_id_, joint_angles_);
}