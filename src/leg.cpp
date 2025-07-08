#include "leg.h"
#include "hexamotion_constants.h"

Leg::Leg(int leg_id, const RobotModel &model)
    : leg_id_(leg_id), leg_name_("Leg_" + std::to_string(leg_id)), joint_angles_(0.0, 0.0, 0.0), tip_position_(0.0, 0.0, 0.0), base_position_(0.0, 0.0, 0.0), step_phase_(STANCE_PHASE), gait_phase_(0.0), in_contact_(false), contact_force_(0.0), fsr_history_index_(0), leg_phase_offset_(0.0), default_angles_(0.0, 0.0, 0.0), default_tip_position_(0.0, 0.0, 0.0) {

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
    calculateBasePosition(model);
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

    // Check if target is reachable
    double distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    double max_reach = 300.0; // Approximate max reach for hexapod

    if (distance > max_reach) {
        return false; // Target too far
    }

    // Update tip position (IK will be calculated when needed)
    tip_position_ = target;
    return true;
}

void Leg::updateForwardKinematics(const RobotModel &model) {
    // Calculate tip position from current joint angles
    updateTipPosition(model);
}

bool Leg::updateInverseKinematics(const RobotModel &model, const Point3D &target_position) {
    // Use current joint angles as starting point for IK
    JointAngles new_angles = model.inverseKinematicsCurrentGlobalCoordinates(leg_id_, joint_angles_, target_position);

    // Check if IK was successful (basic validation)
    if (checkJointLimits(model.getParams())) {
        joint_angles_ = new_angles;
        tip_position_ = target_position;
        return true;
    }

    return false;
}

Eigen::Matrix4d Leg::getTransform(const RobotModel &model) const {
    return model.legTransform(leg_id_, joint_angles_);
}

Eigen::Matrix3d Leg::getJacobian(const RobotModel &model) const {
    return model.calculateJacobian(leg_id_, joint_angles_, tip_position_);
}

bool Leg::isTargetReachable(const RobotModel &model, const Point3D &target) const {
    // Use RobotModel's workspace validation
    // This is a simplified check - full validation should be implemented
    double distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    double max_reach = getLegReach(model.getParams());

    return distance <= max_reach;
}

Point3D Leg::constrainToWorkspace(const RobotModel &model, const Point3D &target) const {
    // Simple workspace constraint - scale target to max reach if outside
    double distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    double max_reach = getLegReach(model.getParams());

    if (distance > max_reach) {
        double scale = max_reach / distance;
        return Point3D(target.x * scale, target.y * scale, target.z * scale);
    }

    return target;
}

bool Leg::checkJointLimits(const Parameters &params) const {
    return (joint_angles_.coxa >= params.coxa_angle_limits[0] &&
            joint_angles_.coxa <= params.coxa_angle_limits[1] &&
            joint_angles_.femur >= params.femur_angle_limits[0] &&
            joint_angles_.femur <= params.femur_angle_limits[1] &&
            joint_angles_.tibia >= params.tibia_angle_limits[0] &&
            joint_angles_.tibia <= params.tibia_angle_limits[1]);
}

double Leg::getJointLimitProximity(const Parameters &params) const {
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

void Leg::constrainJointLimits(const Parameters &params) {
    joint_angles_.coxa = std::max(params.coxa_angle_limits[0],
                                  std::min(params.coxa_angle_limits[1], joint_angles_.coxa));
    joint_angles_.femur = std::max(params.femur_angle_limits[0],
                                   std::min(params.femur_angle_limits[1], joint_angles_.femur));
    joint_angles_.tibia = std::max(params.tibia_angle_limits[0],
                                   std::min(params.tibia_angle_limits[1], joint_angles_.tibia));
}

void Leg::initialize(const RobotModel &model, const Pose &default_stance) {
    // Calculate default tip position from stance pose
    Point3D stance_tip = default_stance.position;

    // Calculate IK for default stance using zero angles as starting point
    JointAngles zero_angles(0, 0, 0);
    JointAngles stance_angles = model.inverseKinematicsCurrentGlobalCoordinates(leg_id_, zero_angles, stance_tip);

    // Set default configuration
    default_angles_ = stance_angles;
    default_tip_position_ = stance_tip;

    // Set current configuration to default
    joint_angles_ = stance_angles;
    tip_position_ = stance_tip;

    // Update FK to ensure consistency
    updateForwardKinematics(model);
}

void Leg::reset(const RobotModel &model) {
    // Reset to default configuration
    joint_angles_ = default_angles_;
    tip_position_ = default_tip_position_;
    step_phase_ = STANCE_PHASE;
    gait_phase_ = 0.0;

    // Reset FSR contact history
    resetFSRHistory();

    // Update FK
    updateForwardKinematics(model);
}

double Leg::getLegReach(const Parameters &params) const {
    // Maximum reach is sum of link lengths
    return params.coxa_length + params.femur_length + params.tibia_length;
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

// Set desired tip position (OpenSHC architecture)
void Leg::setDesiredTipPositionGlobal(const Point3D &desired_position) {
    desired_tip_position_ = desired_position;
}

// Apply inverse kinematics (OpenSHC architecture)
bool Leg::applyIK(const RobotModel &model) {
    // Use current joint angles as starting point for IK (OpenSHC approach)
    JointAngles new_angles = model.inverseKinematicsCurrentGlobalCoordinates(leg_id_, joint_angles_, desired_tip_position_);

    // Update joint angles
    setJointAngles(new_angles);

    // Update forward kinematics to synchronize current tip position
    updateForwardKinematics(model);

    // Check if IK was successful (tip position matches desired)
    double ik_error = math_utils::distance(tip_position_, desired_tip_position_);
    return ik_error < IK_TOLERANCE; // 1mm tolerance
}

void Leg::calculateBasePosition(const RobotModel &model) {
    // Calculate leg base position in world coordinates using RobotModel
    double angle_rad = model.getLegBaseAngleOffset(leg_id_);
    const Parameters &params = model.getParams();
    base_position_.x = params.hexagon_radius * cos(angle_rad);
    base_position_.y = params.hexagon_radius * sin(angle_rad);
    base_position_.z = 0.0; // Base is at ground level
}

void Leg::updateTipPosition(const RobotModel &model) {
    // Calculate tip position from current joint angles using FK
    Point3D new_tip = model.forwardKinematicsGlobalCoordinates(leg_id_, joint_angles_);
    tip_position_ = new_tip;
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