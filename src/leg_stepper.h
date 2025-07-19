#ifndef LEG_STEPPER_H
#define LEG_STEPPER_H

#include "gait_config.h" // For StepCycle definition
#include "leg.h"
#include "robot_model.h"
#include "walkspace_analyzer.h"
#include "workspace_validator.h"

enum StepState {
    STEP_SWING,        //< The leg step cycle is in 'swing' state, the forward 'in air' progression of the step cycle
    STEP_STANCE,       //< The leg step cycle is in 'stance' state, the backward 'on ground' regression of the step cycle
    STEP_FORCE_STANCE, //< State used to force a 'stance' state in non-standard instances
    STEP_FORCE_STOP,   //< State used to force the step cycle to stop iterating
    STEP_STATE_COUNT,  //< Misc enum defining number of Step States
};

/**
 * @brief External target for leg positioning (OpenSHC equivalent)
 */
struct LegStepperExternalTarget {
    Point3D position;       //< Target position
    double swing_clearance; //< Swing clearance height
    std::string frame_id;   //< Reference frame ID
    bool defined = false;   //< Whether target is defined
};

/**
 * @brief Leg stepper class for individual leg trajectory control (OpenSHC equivalent)
 */
class LegStepper {
  public:
    // Modificar el constructor para aceptar los validadores
    LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model,
               WalkspaceAnalyzer *walkspace_analyzer, WorkspaceValidator *workspace_validator);

    // Accessors
    int getLegIndex() const { return leg_index_; }
    Point3D getCurrentTipPose() const { return current_tip_pose_; }
    Point3D getCurrentTipVelocity() const { return current_tip_velocity_; }
    JointAngles getJointAngles() const { return leg_.getJointAngles(); }
    Point3D getDefaultTipPose() const { return default_tip_pose_; }
    Point3D getIdentityTipPose() const { return identity_tip_pose_; }
    Point3D getTargetTipPose() const { return target_tip_pose_; }
    StepState getStepState() const { return step_state_; }
    int getPhase() const { return phase_; }
    double getPhaseOffset() const { return leg_.getPhaseOffset(); }
    Point3D getStrideVector() const { return stride_vector_; }
    Point3D getSwingClearance() const { return swing_clearance_; }
    double getStepProgress() const { return step_progress_; }
    bool hasCompletedFirstStep() const { return completed_first_step_; }
    bool isAtCorrectPhase() const { return at_correct_phase_; }
    Point3D getSwing1ControlNode(int i) const { return swing_1_nodes_[i]; }
    Point3D getSwing2ControlNode(int i) const { return swing_2_nodes_[i]; }
    Point3D getStanceControlNode(int i) const { return stance_nodes_[i]; }
    LegStepperExternalTarget getExternalTarget() const { return external_target_; }
    LegStepperExternalTarget getExternalDefault() const { return external_default_; }

    // Modifiers
    void setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity);
    void setCurrentTipPose(const Point3D &pose) { leg_.setCurrentTipPositionGlobal(pose); }
    void setDefaultTipPose(const Point3D &pose) { default_tip_pose_ = pose; }
    void setStepState(StepState state) { step_state_ = state; }
    void setPhase(int phase) { phase_ = phase; }
    void setStepProgress(double progress) { step_progress_ = progress; }
    void setPhaseOffset(double offset) { leg_.setPhaseOffset(offset); }
    void setSwingOriginTipVelocity(const Point3D &velocity) { swing_origin_tip_velocity_ = velocity; }
    void setCompletedFirstStep(bool completed) { completed_first_step_ = completed; }
    void setAtCorrectPhase(bool at_correct) { at_correct_phase_ = at_correct; }
    void setTouchdownDetection(bool detection) { touchdown_detection_ = detection; }
    void setExternalTarget(const LegStepperExternalTarget &target) { external_target_ = target; }
    void setExternalDefault(const LegStepperExternalTarget &default_pos) { external_default_ = default_pos; }
    void setStepLength(double length) { step_length_ = length; }
    void setSwingHeight(double height) { swing_height_ = height; }
    void setBodyClearance(double clearance) { body_clearance_ = clearance; }
    void setStanceSpanModifier(double modifier) { stance_span_modifier_ = modifier; }
    void setSwingClearance(const Point3D &clearance) { swing_clearance_ = clearance; }

    // Main unified update method (OpenSHC equivalent)
    void updateStepCycle(double normalized_phase, double step_length, double time_delta);

    // Internal helper methods
    void updateStride();
    Point3D calculateStanceSpanChange();
    void updateDefaultTipPosition();
    void updateTipPosition(double step_length, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown);
    void generatePrimarySwingControlNodes();
    void generateSecondarySwingControlNodes(bool ground_contact);
    void generateStanceControlNodes(double stride_scaler);
    void forceNormalTouchdown();

  private:
    // Internal helper methods
    void updateStepState(const StepCycle &step);
    void updateDynamicTiming(double step_length, double time_delta);

    int leg_index_;
    Leg &leg_;
    RobotModel &robot_model_; // Store robot model for kinematics
    Point3D identity_tip_pose_;
    Point3D default_tip_pose_;
    Point3D origin_tip_pose_;
    Point3D target_tip_pose_;
    Point3D current_tip_pose_; //< Current tip pose calculated by stepper

    // Walking state
    Point3D desired_linear_velocity_;
    double desired_angular_velocity_;
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
    double step_progress_;
    StepState step_state_;

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
    double step_length_ = 0.0;
    double swing_height_ = 0.0;
    double body_clearance_ = 0.0;
    double stance_span_modifier_ = 0.0;

    WalkspaceAnalyzer *walkspace_analyzer_ = nullptr;
    WorkspaceValidator *workspace_validator_ = nullptr;
};

#endif // LEG_STEPPER_H