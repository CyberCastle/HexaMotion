#ifndef LEG_STEPPER_OPENSHC_H
#define LEG_STEPPER_OPENSHC_H

#include "../src/leg_stepper.h" // Include for enum definitions
#include "leg.h"
#include "robot_model.h"
#include "walkspace_analyzer.h"
#include "workspace_validator.h"

/**
 * @brief LegStepper implementación basada en flujo de OpenSHC
 *
 * Esta clase implementa el comportamiento exacto de OpenSHC:
 * - Flujo iterativo secuencial
 * - Acumulación de deltas incremental
 * - Timing correcto basado en iteraciones
 */
class LegStepperOpenSHC {
  public:
    // Constructor
    LegStepperOpenSHC(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model,
                      WalkspaceAnalyzer *walkspace_analyzer, WorkspaceValidator *workspace_validator);

    // Accessors
    int getLegIndex() const { return leg_index_; }
    Point3D getCurrentTipPose() const { return current_tip_pose_; }
    Point3D getCurrentTipVelocity() const { return current_tip_velocity_; }
    Point3D getDefaultTipPose() const { return default_tip_pose_; }
    Point3D getIdentityTipPose() const { return identity_tip_pose_; }
    Point3D getTargetTipPose() const { return target_tip_pose_; }
    Point3D getStrideVector() const { return stride_vector_; }
    Point3D getSwingClearance() const { return swing_clearance_; }
    StepState getStepState() const { return step_state_; }
    int getPhase() const { return phase_; }
    double getStepProgress() const { return step_progress_; }
    bool hasCompletedFirstStep() const { return completed_first_step_; }
    bool isAtCorrectPhase() const { return at_correct_phase_; }
    Point3D getSwing1ControlNode(int i) const { return swing_1_nodes_[i]; }
    Point3D getSwing2ControlNode(int i) const { return swing_2_nodes_[i]; }
    Point3D getStanceControlNode(int i) const { return stance_nodes_[i]; }

    // Modifiers
    void setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity);
    void setCurrentTipPose(const Point3D &pose) { current_tip_pose_ = pose; }
    void setDefaultTipPose(const Point3D &pose) { default_tip_pose_ = pose; }
    void setStepState(StepState state) { step_state_ = state; }
    void setPhase(int phase) { phase_ = phase; }
    void setStepProgress(double progress) { step_progress_ = progress; }
    void setSwingOriginTipVelocity(const Point3D &velocity) { swing_origin_tip_velocity_ = velocity; }
    void setCompletedFirstStep(bool completed) { completed_first_step_ = completed; }
    void setAtCorrectPhase(bool at_correct) { at_correct_phase_ = at_correct; }
    void setSwingClearance(const Point3D &clearance) { swing_clearance_ = clearance; }

    // Main OpenSHC-style methods
    void updateStride();
    void initializeSwingPeriod(int iteration);
    void updateTipPositionIterative(int iteration, double time_delta, bool rough_terrain_mode = false, bool force_normal_touchdown = false);

    // Control node generation (OpenSHC style)
    void generatePrimarySwingControlNodes();
    void generateSecondarySwingControlNodes(bool ground_contact = false);
    void generateStanceControlNodes(double stride_scaler = 1.0);

    // OpenSHC timing calculations
    void calculateSwingTiming(double time_delta);

    // Debug methods
    void printDebugInfo() const;

  private:
    // Basic properties
    int leg_index_;
    Leg &leg_;
    RobotModel &robot_model_;
    Point3D identity_tip_pose_;
    Point3D default_tip_pose_;
    Point3D origin_tip_pose_;
    Point3D target_tip_pose_;
    Point3D current_tip_pose_;

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

    // OpenSHC timing parameters
    double swing_delta_t_;
    double stance_delta_t_;
    int swing_iterations_;
    int stance_iterations_;
    int current_iteration_;

    // Bezier control nodes (5 nodes for quartic curves)
    Point3D swing_1_nodes_[5];
    Point3D swing_2_nodes_[5];
    Point3D stance_nodes_[5];

    WalkspaceAnalyzer *walkspace_analyzer_ = nullptr;
    WorkspaceValidator *workspace_validator_ = nullptr;
};

#endif // LEG_STEPPER_OPENSHC_H
