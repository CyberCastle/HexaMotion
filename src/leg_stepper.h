#ifndef LEG_STEPPER_H
#define LEG_STEPPER_H

#include "robot_model.h"
#include "leg.h"

// Forward declarations - REMOVED WalkController dependency
// class WalkController; // ❌ ELIMINAR esta dependencia

// Enum definitions needed by LegStepper
enum WalkState {
    WALK_STARTING,    ///< The walk controller cycle is in 'starting' state (transitioning from 'stopped' to 'moving')
    WALK_MOVING,      ///< The walk controller cycle is in a 'moving' state (the primary walking state)
    WALK_STOPPING,    ///< The walk controller cycle is in a 'stopping' state (transitioning from 'moving' to 'stopped')
    WALK_STOPPED,     ///< The walk controller cycle is in a 'stopped' state (state whilst velocity input is zero)
    WALK_STATE_COUNT, ///< Misc enum defining number of Walk States
};

enum StepState {
    STEP_SWING,        ///< The leg step cycle is in 'swing' state, the forward 'in air' progression of the step cycle
    STEP_STANCE,       ///< The leg step cycle is in 'stance' state, the backward 'on ground' regression of the step cycle
    STEP_FORCE_STANCE, ///< State used to force a 'stance' state in non-standard instances
    STEP_FORCE_STOP,   ///< State used to force the step cycle to stop iterating
    STEP_STATE_COUNT,  ///< Misc enum defining number of Step States
};

/**
 * @brief Step cycle timing parameters (OpenSHC equivalent)
 */
struct StepCycle {
    double frequency_;      ///< Step frequency in Hz
    int period_;           ///< Total step cycle length in iterations
    int swing_period_;     ///< Swing period length in iterations
    int stance_period_;    ///< Stance period length in iterations
    int stance_end_;       ///< Iteration when stance period ends
    int swing_start_;      ///< Iteration when swing period starts
    int swing_end_;        ///< Iteration when swing period ends
    int stance_start_;     ///< Iteration when stance period starts
};

/**
 * @brief External target for leg positioning (OpenSHC equivalent)
 */
struct LegStepperExternalTarget {
    Point3D position;           ///< Target position
    double swing_clearance;     ///< Swing clearance height
    std::string frame_id;       ///< Reference frame ID
    bool defined = false;       ///< Whether target is defined
};

/**
 * @brief Leg stepper class for individual leg trajectory control (OpenSHC equivalent)
 *
 * ✅ CORRECTED: LegStepper is now independent and does NOT depend on WalkController
 */
class LegStepper {
public:
    // ✅ CORRECTED: Constructor without WalkController dependency
    LegStepper(int leg_index, const Point3D& identity_tip_pose, Leg& leg, RobotModel& robot_model);

    // Accessors
    int getLegIndex() const { return leg_index_; }
    Point3D getCurrentTipPose() const { return current_tip_pose_; }
    Point3D getCurrentTipVelocity() const { return current_tip_velocity_; }
    JointAngles getJointAngles() const { return leg_.getJointAngles(); }
    Point3D getDefaultTipPose() const { return default_tip_pose_; }
    Point3D getIdentityTipPose() const { return identity_tip_pose_; }
    Point3D getTargetTipPose() const { return target_tip_pose_; }
    WalkState getWalkState() const { return current_walk_state_; }  // ✅ Internal state
    Point3D getWalkPlane() const { return walk_plane_; }
    Point3D getWalkPlaneNormal() const { return walk_plane_normal_; }
    StepState getStepState() const { return step_state_; }
    int getPhase() const { return phase_; }
    double getPhaseOffset() const { return leg_.getPhaseOffset(); }
    Point3D getStrideVector() const { return stride_vector_; }
    Point3D getSwingClearance() const { return swing_clearance_; }
    double getSwingProgress() const { return swing_progress_; }
    double getStanceProgress() const { return stance_progress_; }
    bool hasCompletedFirstStep() const { return completed_first_step_; }
    bool isAtCorrectPhase() const { return at_correct_phase_; }
    Point3D getSwing1ControlNode(int i) const { return swing_1_nodes_[i]; }
    Point3D getSwing2ControlNode(int i) const { return swing_2_nodes_[i]; }
    Point3D getStanceControlNode(int i) const { return stance_nodes_[i]; }
    LegStepperExternalTarget getExternalTarget() const { return external_target_; }
    LegStepperExternalTarget getExternalDefault() const { return external_default_; }

    // Modifiers
    void setCurrentTipPose(const Point3D& pose) { leg_.setTipPosition(pose); }
    void setDefaultTipPose(const Point3D& pose) { default_tip_pose_ = pose; }
    void setStepState(StepState state) { step_state_ = state; }
    void setPhase(int phase) { phase_ = phase; }
    void setSwingProgress(int progress) { swing_progress_ = progress; }
    void setStanceProgress(int progress) { stance_progress_ = progress; }
    void setPhaseOffset(double offset) { leg_.setPhaseOffset(offset); }
    void setCompletedFirstStep(bool completed) { completed_first_step_ = completed; }
    void setAtCorrectPhase(bool at_correct) { at_correct_phase_ = at_correct; }
    void setTouchdownDetection(bool detection) { touchdown_detection_ = detection; }
    void setExternalTarget(const LegStepperExternalTarget& target) { external_target_ = target; }
    void setExternalDefault(const LegStepperExternalTarget& default_pos) { external_default_ = default_pos; }
    void setWalkState(WalkState state) { current_walk_state_ = state; }  // ✅ Set by WalkController

    // ✅ CORRECTED: Core functionality without WalkController dependency
    void updatePhase(const StepCycle& step);  // ✅ StepCycle passed as parameter
    void iteratePhase(const StepCycle& step);  // ✅ StepCycle passed as parameter
    void updateStepState(const StepCycle& step);  // ✅ StepCycle passed as parameter
    void updateStride(double step_length);
    Point3D calculateStanceSpanChange();
    void updateDefaultTipPosition();
    void updateTipPosition(double step_length, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown);  // ✅ Parameters passed
    void generatePrimarySwingControlNodes();
    void generateSecondarySwingControlNodes(bool ground_contact);
    void generateStanceControlNodes(double stride_scaler);
    void forceNormalTouchdown();

    // ✅ CORRECTED: OpenSHC-like API without WalkController dependency
    void updateWithPhase(double local_phase, double step_length, double time_delta);

private:
    // ❌ REMOVED: WalkController* walker_;  // No more dependency on WalkController
    int leg_index_;
    Leg& leg_;
    RobotModel& robot_model_;  // ✅ Store robot model for kinematics
    Point3D identity_tip_pose_;
    Point3D default_tip_pose_;
    Point3D origin_tip_pose_;
    Point3D target_tip_pose_;
    Point3D current_tip_pose_;       ///< Current tip pose calculated by stepper

    // Walking state
    Point3D walk_plane_;
    Point3D walk_plane_normal_;
    Point3D stride_vector_;
    Point3D current_tip_velocity_;
    Point3D swing_origin_tip_position_;
    Point3D swing_origin_tip_velocity_;
    Point3D stance_origin_tip_position_;
    Point3D swing_clearance_;

    // Phase and state management
    bool at_correct_phase_;
    bool completed_first_step_;
    int phase_;
    double stance_progress_;
    double swing_progress_;
    double step_progress_;
    StepState step_state_;
    WalkState current_walk_state_;  // ✅ Internal walk state

    // Timing
    double swing_delta_t_;
    double stance_delta_t_;

    // External targets
    LegStepperExternalTarget external_target_;
    LegStepperExternalTarget external_default_;
    bool touchdown_detection_;

    // Bezier control nodes (5 nodes for quartic curves)
    Point3D swing_1_nodes_[5];
    Point3D swing_2_nodes_[5];
    Point3D stance_nodes_[5];
};

#endif // LEG_STEPPER_H