#ifndef LEG_H
#define LEG_H

#include "math_utils.h"
#include "robot_model.h"
#include <Arduino.h>
#include <ArduinoEigen.h>

/**
 * @brief Represents a single leg of the hexapod robot with all associated data and functionality.
 *
 * ---
 * \b Tip Position API: setTipPosition vs setDesiredTipPose
 *
 * - setTipPosition(const Point3D&):
 *   - Directly sets the tip position (tip_position_) of the leg, without updating joint angles or performing inverse kinematics (IK).
 *   - Does NOT guarantee that the position is physically reachable or consistent with the current joint configuration.
 *   - Use only for initialization, simulation, or forced state (not for real kinematic control).
 *
 * - setDesiredTipPose(const Point3D&):
 *   - Sets the desired tip pose (desired_tip_pose_) as a target for the leg.
 *   - To actually move the leg to this position, you must call applyIK(), which will compute the necessary joint angles and update the real tip position accordingly.
 *   - This is the recommended way to command leg movement in a physically consistent and realistic manner.
 *
 * Typical usage for kinematic control:
 *   1. leg.setDesiredTipPose(target_position);
 *   2. leg.applyIK(robot_model);
 *
 * ---
 *
 * This class encapsulates all information related to a leg including:
 * - Identification and configuration
 * - Joint angles and positions
 * - Kinematic state and transforms
 * - Gait phase and contact state
 * - DH parameters and workspace information
 *
 * Based on OpenSHC's Leg class concept but adapted for HexaMotion's architecture.
 */
class Leg {
  public:
    /**
     * @brief Constructor for a leg with given parameters.
     * @param leg_id The leg identification number (0-5)
     * @param model Robot model containing DH parameters and base angle offsets
     */
    explicit Leg(int leg_id, const RobotModel &model);

    /**
     * @brief Destructor.
     */
    ~Leg() = default;

    // ===== IDENTIFICATION AND CONFIGURATION =====

    /**
     * @brief Get the leg identification number.
     * @return Leg ID (0-5)
     */
    int getLegId() const { return leg_id_; }

    /**
     * @brief Get the leg name string.
     * @return Leg name (e.g., "Leg_0", "Leg_1", etc.)
     */
    String getLegName() const { return leg_name_; }

    /**
     * @brief Get the degrees of freedom for this leg.
     * @return Number of DOF (always 3 for hexapod)
     */
    int getDOF() const { return DOF_PER_LEG; }

    // ===== JOINT ANGLES AND POSITIONS =====

    /**
     * @brief Get current joint angles.
     * @return Current joint angles (coxa, femur, tibia)
     */
    JointAngles getJointAngles() const { return joint_angles_; }

    /**
     * @brief Set joint angles for this leg.
     * @param angles New joint angles
     */
    void setJointAngles(const JointAngles &angles);

    /**
     * @brief Get individual joint angle.
     * @param joint_index Joint index (0=coxa, 1=femur, 2=tibia)
     * @return Joint angle in degrees
     */
    double getJointAngle(int joint_index) const;

    /**
     * @brief Set individual joint angle.
     * @param joint_index Joint index (0=coxa, 1=femur, 2=tibia)
     * @param angle Joint angle in degrees
     */
    void setJointAngle(int joint_index, double angle);

    /**
     * @brief Get the current tip position in global coordinates
     */
    Point3D getCurrentTipPositionGlobal() const { return tip_position_; }

    /**
     * @brief Set the current tip position in global coordinates
     */
    bool setCurrentTipPositionGlobal(const RobotModel &model, const Point3D &position);

    /**
     * @brief Get leg base position in world coordinates.
     * @return Leg base position
     */
    Point3D getBasePosition() const { return base_position_; }

    /**
     * @brief Update tip position from current joint angles.
     * @param model Robot model for FK calculation
     */
    void updateTipPosition(const RobotModel &model);

    // ===== KINEMATIC STATE =====

    /**
     * @brief Update inverse kinematics and joint angles.
     * @param model Robot model for IK calculations
     * @param target_position Target tip position
     * @return True if IK succeeds
     */
    bool updateInverseKinematics(const RobotModel &model, const Point3D &target_position);

    /**
     * @brief Get current DH transform matrix.
     * @param model Robot model for transform calculation
     * @return 4x4 DH transform matrix
     */
    Eigen::Matrix4d getTransform(const RobotModel &model) const;

    /**
     * @brief Get Jacobian matrix for this leg.
     * @param model Robot model for Jacobian calculation
     * @return 3x3 Jacobian matrix
     */
    Eigen::Matrix3d getJacobian(const RobotModel &model) const;

    // ===== GAIT AND CONTACT STATE =====

    /**
     * @brief Get current step phase.
     * @return Current step phase
     */
    StepPhase getStepPhase() const { return step_phase_; }

    /**
     * @brief Set step phase.
     * @param phase New step phase
     */
    void setStepPhase(StepPhase phase) { step_phase_ = phase; }

    /**
     * @brief Get gait phase (0.0 to 1.0).
     * @return Current gait phase
     */
    double getGaitPhase() const { return gait_phase_; }

    /**
     * @brief Set gait phase.
     * @param phase Gait phase (0.0 to 1.0)
     */
    void setGaitPhase(double phase) { gait_phase_ = phase; }

    /**
     * @brief Check if leg is in contact with ground.
     * @return True if leg is touching ground
     */
    bool isInContact() const { return in_contact_; }

    /**
     * @brief Set contact state.
     * @param contact True if leg is touching ground
     */
    void setContactState(bool contact) { in_contact_ = contact; }

    /**
     * @brief Get contact force reading.
     * @return Contact force value
     */
    double getContactForce() const { return contact_force_; }

    /**
     * @brief Set contact force reading.
     * @param force Contact force value
     */
    void setContactForce(double force) { contact_force_ = force; }

    // ===== FSR CONTACT HISTORY =====

    /**
     * @brief Update FSR contact history with new reading.
     * @param in_contact Current contact state
     * @param pressure Current pressure reading
     */
    void updateFSRHistory(bool in_contact, double pressure);

    /**
     * @brief Get filtered contact state using history.
     * @param contact_threshold Threshold for contact detection (default 0.7)
     * @param release_threshold Threshold for contact release (default 0.3)
     * @return True if leg is in contact based on filtered history
     */
    bool getFilteredContactState(double contact_threshold = 0.7, double release_threshold = 0.3) const;

    /**
     * @brief Get current FSR history index.
     * @return Current index in circular buffer
     */
    int getFSRHistoryIndex() const { return fsr_history_index_; }

    /**
     * @brief Get FSR contact history value at specific index.
     * @param index Index in history buffer (0-2)
     * @return Contact value (1.0 for contact, 0.0 for no contact)
     */
    double getFSRHistoryValue(int index) const;

    /**
     * @brief Get average contact value from history.
     * @return Average contact value (0.0 to 1.0)
     */
    double getAverageContactValue() const;

    /**
     * @brief Reset FSR contact history.
     */
    void resetFSRHistory();

    // ===== GAIT PHASE OFFSET =====

    /**
     * @brief Set the phase offset for this leg in the gait cycle.
     * @param offset Phase offset (0.0 to 1.0)
     */
    void setPhaseOffset(double offset);

    /**
     * @brief Get the phase offset for this leg.
     * @return Phase offset (0.0 to 1.0)
     */
    double getPhaseOffset() const { return leg_phase_offset_; }

    /**
     * @brief Calculate the current phase for this leg given the global gait phase.
     * @param global_gait_phase Current global gait phase (0.0 to 1.0)
     * @return Leg-specific phase (0.0 to 1.0)
     */
    double calculateLegPhase(double global_gait_phase) const;

    /**
     * @brief Check if this leg should be in stance phase based on current gait phase.
     * @param global_gait_phase Current global gait phase (0.0 to 1.0)
     * @param stance_duration Duration of stance phase (0.0 to 1.0)
     * @return True if leg should be in stance phase
     */
    bool shouldBeInStance(double global_gait_phase, double stance_duration) const;

    /**
     * @brief Check if this leg should be in swing phase based on current gait phase.
     * @param global_gait_phase Current global gait phase (0.0 to 1.0)
     * @param stance_duration Duration of stance phase (0.0 to 1.0)
     * @return True if leg should be in swing phase
     */
    bool shouldBeInSwing(double global_gait_phase, double stance_duration) const;

    // ===== WORKSPACE AND LIMITS =====

    /**
     * @brief Check if target position is reachable.
     * @param model Robot model for workspace validation
     * @param target Target position to check
     * @return True if target is within workspace
     */
    bool isTargetReachable(const RobotModel &model, const Point3D &target) const;

    /**
     * @brief Constrain target to workspace boundary.
     * @param model Robot model for workspace calculation
     * @param target Target position to constrain
     * @return Constrained position within workspace
     */
    Point3D constrainToWorkspace(const RobotModel &model, const Point3D &target) const;

    /**
     * @brief Check if current joint angles are within limits.
     * @param params Robot parameters containing joint limits
     * @return True if all joints are within limits
     */
    bool checkJointLimits(const Parameters &params) const;

    /**
     * @brief Get joint limit proximity (1.0 = far from limits, 0.0 = at limits).
     * @param params Robot parameters containing joint limits
     * @return Proximity value (0.0 to 1.0)
     */
    double getJointLimitProximity(const Parameters &params) const;

    /**
     * @brief Constrain joint angles to limits.
     * @param params Robot parameters containing joint limits
     */
    void constrainJointLimits(const Parameters &params);

    // ===== INITIALIZATION =====

    /**
     * @brief Initialize leg with default stance position.
     * @param model Robot model for initialization
     * @param default_stance Default stance pose
     */
    void initialize(const RobotModel &model, const Pose &default_stance);

    /**
     * @brief Reset leg to default configuration.
     * @param model Robot model for reset
     */
    void reset(const RobotModel &model);

    // ===== UTILITY FUNCTIONS =====

    /**
     * @brief Get leg reach distance.
     * @param params Robot parameters
     * @return Maximum reach distance
     */
    double getLegReach(const Parameters &params) const;

    /**
     * @brief Calculate distance from current tip to target.
     * @param target Target position
     * @return Distance in mm
     */
    double getDistanceToTarget(const Point3D &target) const;

    /**
     * @brief Get leg direction vector.
     * @return Normalized direction vector from base to tip
     */
    Eigen::Vector3d getLegDirection() const;

    /**
     * @brief Check if leg is in default stance position.
     * @param tolerance Tolerance for position comparison in mm
     * @return True if leg is in default stance
     */
    bool isInDefaultStance(double tolerance = 5.0) const;

    // Set desired tip position in global coordinates
    void setDesiredTipPositionGlobal(const Point3D &desired_position);
    Point3D getDesiredTipPositionGlobal() const { return desired_tip_position_; }

    // Apply inverse kinematics (OpenSHC architecture)
    bool applyIK(const RobotModel &model);

    // Apply inverse kinematics with position delta (OpenSHC architecture)
    bool applyIKWithDelta(const RobotModel &model, const Point3D &position_delta);

  private:
    // ===== IDENTIFICATION =====
    int leg_id_;      //< Leg identification number (0-5)
    String leg_name_; //< Leg name string

    // ===== JOINT STATE =====
    JointAngles joint_angles_; //< Current joint angles (coxa, femur, tibia)
    Point3D tip_position_;     //< Current tip position in world coordinates
    Point3D base_position_;    //< Leg base position in world coordinates

    // ===== GAIT STATE =====
    StepPhase step_phase_; //< Current step phase
    double gait_phase_;    //< Gait phase (0.0 to 1.0)
    bool in_contact_;      //< Contact state with ground
    double contact_force_; //< Contact force reading

    // ===== FSR CONTACT HISTORY =====
    double fsr_contact_history_[3]; //< Circular buffer for FSR contact history (3 samples)
    int fsr_history_index_;         //< Current index in the circular buffer

    // ===== GAIT PHASE OFFSET =====
    double leg_phase_offset_; //< Phase offset for this leg in gait cycle (0.0 to 1.0)

    // ===== DEFAULT CONFIGURATION =====
    JointAngles default_angles_;   //< Default joint angles
    Point3D default_tip_position_; //< Default tip position

    // Desired tip position
    Point3D desired_tip_position_; //< Desired tip position

    /**
     * @brief Calculate base position from robot parameters.
     * @param params Robot parameters
     */
    void calculateBasePosition(const RobotModel &model);
};

#endif // LEG_H
