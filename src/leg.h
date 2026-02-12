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
 *   2. leg.applyIK(); (No need to pass RobotModel; it is stored as a reference.)
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
 * The class stores a reference to the RobotModel to avoid redundant parameter passing
 * and improve performance by eliminating duplicate model references in method calls.
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

    /** Identification and configuration. */

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

    /** Joint angles and positions. */

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
     * @brief Set desired joint velocities (driver units or rad/s depending on interface).
     * @param velocities Desired joint velocities
     */
    void setDesiredJointVelocity(const JointAngles &velocities) { desired_joint_velocity_ = velocities; }

    /**
     * @brief Get desired joint velocities.
     * @return Desired joint velocities
     */
    JointAngles getDesiredJointVelocity() const { return desired_joint_velocity_; }

    /**
     * @brief Set current joint velocities (rad/s) if available from hardware.
     * @param velocities Current joint velocities
     */
    void setCurrentJointVelocity(const JointAngles &velocities);

    /**
     * @brief Get current joint velocities.
     * @return Current joint velocities
     */
    JointAngles getCurrentJointVelocity() const { return current_joint_velocity_; }

    /**
     * @brief Set current joint efforts/torques if available from hardware.
     * @param efforts Current joint efforts
     */
    void setCurrentJointEffort(const JointAngles &efforts);

    /**
     * @brief Get current joint efforts/torques.
     * @return Current joint efforts
     */
    JointAngles getCurrentJointEffort() const { return current_joint_effort_; }

    /**
     * @brief Get the current tip position in global coordinates
     */
    Point3D getCurrentTipPositionGlobal() const { return tip_position_; }

    /**
     * @brief Set the current tip position in global coordinates
     */
    void setCurrentTipPositionGlobal(const Point3D &position);

    /**
     * @brief Get leg base position in world coordinates.
     * @return Leg base position
     */
    Point3D getBasePosition() const { return base_position_; }

    /**
     * @brief Update tip position from current joint angles.
     */
    void updateTipPosition();

    /** Kinematic state. */

    /**
     * @brief Apply inverse kinematics to reach a target position and update joint angles & tip position.
     * @param target_position Desired global tip position
     * @return True if IK succeeds within joint limits
     */
    bool applyIK(const Point3D &target_position);

    /**
     * @brief Apply advanced IK implementation with delta calculation and joint optimization
     * This method uses a robust IK solver that includes:
     * - Position delta calculation in leg frame
     * - DLS-based IK with joint limit cost function
     * - Joint position updates with proper clamping
     * @param target_position Desired global tip position
     * @return True if IK succeeds within joint limits
     */
    bool applyAdvancedIK(const Point3D &target_position);

    /**
     * @brief Get current DH transform matrix.
     * @return 4x4 DH transform matrix
     */
    Eigen::Matrix4d getTransform() const;

    /**
     * @brief Get Jacobian matrix for this leg.
     * @return 3x3 Jacobian matrix
     */
    Eigen::Matrix3d getJacobian() const;

    /**
     * @brief Calculate tip force from joint efforts using Jacobian transpose (OpenSHC equivalent).
     */
    void calculateTipForce();

    /**
     * @brief Get calculated tip force (Cartesian) from joint effort estimation.
     * @return Calculated tip force vector
     */
    Eigen::Vector3d getCalculatedTipForce() const { return tip_force_calculated_; }

    /** Leg state (OpenSHC equivalent). */

    /**
     * @brief Get current leg state (walking, manual, transitioning).
     * @return Current LegState
     */
    LegState getLegState() const { return leg_state_; }

    /**
     * @brief Set current leg state.
     * @param state New LegState
     */
    void setLegState(LegState state) { leg_state_ = state; }

    /**
     * @brief Get swing progress for this leg (0.0 to 1.0 during swing, -1.0 if not swinging).
     * @return Swing progress value
     */
    double getSwingProgress() const { return swing_progress_; }

    /**
     * @brief Set swing progress value.
     * @param progress Swing progress (0.0 to 1.0, or -1.0 if not swinging)
     */
    void setSwingProgress(double progress) { swing_progress_ = progress; }

    /** Gait and contact state. */

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

    /** OpenSHC-style desired position management. */

    /**
     * @brief Set desired tip position for OpenSHC-style batch IK processing
     * @param position Desired tip position from Bézier trajectory (pure, no IK applied yet)
     */
    void setDesiredTipPosition(const Point3D &position) { desired_tip_position_ = position; }

    /**
     * @brief Get desired tip position for OpenSHC-style batch IK processing
     * @return Desired tip position from Bézier trajectory
     */
    Point3D getDesiredTipPosition() const { return desired_tip_position_; }

    /** FSR contact history. */

    /**
     * @brief Update FSR contact history with new reading.
     * @param in_contact Current contact state
     * @param pressure Current pressure reading
     */
    void updateFSRHistory(bool in_contact, double pressure);

    /**
     * @brief Get filtered contact state using history.
     * @param fsr_touchdown_threshold Threshold for contact detection (default 0.7)
     * @param fsr_liftoff_threshold Threshold for contact release (default 0.3)
     * @return True if leg is in contact based on filtered history
     */
    bool getFilteredContactState(double fsr_touchdown_threshold = 0.7, double fsr_liftoff_threshold = 0.3) const;

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

    /** Gait phase offset. */

    /**
     * @brief Set the phase offset for this leg in the gait cycle.
     * @param offset Phase offset in iterations (0 to period-1).
     */
    void setPhaseOffset(int offset);

    /**
     * @brief Get the phase offset for this leg.
     * @return Phase offset in iterations (0 to period-1).
     */
    int getPhaseOffset() const { return leg_phase_offset_; }

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

    /** Initialization. */

    /**
     * @brief Initialize leg with default stance position.
     * @param default_stance Default stance pose
     */
    void initialize(const Pose &default_stance);

    /**
     * @brief Reset leg to default configuration.
     */
    void reset();

    /**
     * @brief Get default tip position (stance position).
     * @return Default tip position in world coordinates
     */
    Point3D getDefaultTipPosition() const { return default_tip_position_; }

    /** Utility functions. */

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

  private:
    /** Robot model reference. */
    const RobotModel &model_; /**< Reference to robot model for all calculations. */

    /** Identification. */
    int leg_id_;      /**< Leg identification number (0-5). */
    String leg_name_; /**< Leg name string. */

    /** Joint state. */
    JointAngles joint_angles_;           /**< Current joint angles (coxa, femur, tibia). */
    JointAngles desired_joint_velocity_; /**< Desired joint velocities (driver units or rad/s). */
    JointAngles current_joint_velocity_; /**< Current joint velocities (rad/s). */
    JointAngles current_joint_effort_;   /**< Current joint efforts/torques (driver units). */
    bool has_effort_data_ = false;       /**< True when current_joint_effort_ is populated. */
    Point3D tip_position_;               /**< Current tip position in world coordinates. */
    Point3D base_position_;              /**< Leg base position in world coordinates. */

    /** Leg state. */
    LegState leg_state_;    /**< Current leg state (walking, manual, transitioning). */
    double swing_progress_; /**< Swing progress (0.0-1.0 during swing, -1.0 otherwise). */

    /** Gait state. */
    StepPhase step_phase_;                 /**< Current step phase. */
    double gait_phase_;                    /**< Gait phase (0.0 to 1.0). */
    bool in_contact_;                      /**< Contact state with ground. */
    double contact_force_;                 /**< Contact force reading. */
    Eigen::Vector3d tip_force_calculated_; /**< Estimated tip force from joint effort. */

    /** FSR contact history. */
    double fsr_contact_history_[3]; /**< Circular buffer for FSR contact history (3 samples). */
    int fsr_history_index_;         /**< Current index in the circular buffer. */

    /** Gait phase offset. */
    int leg_phase_offset_; /**< Phase offset for this leg in gait cycle (iterations, 0 to period-1). */

    /** OpenSHC-style desired position. */
    Point3D desired_tip_position_; /**< Desired tip position from Bezier trajectory (OpenSHC-style). */

    /** Default configuration. */
    JointAngles default_angles_;   /**< Default joint angles. */
    Point3D default_tip_position_; /**< Default tip position. */
};

#endif /**< LEG_H */
