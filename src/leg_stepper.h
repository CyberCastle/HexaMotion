#ifndef LEG_STEPPER_H
#define LEG_STEPPER_H

#include "gait_config.h" // For StepCycle definition
#include "leg.h"
#include "robot_model.h"
#include "velocity_limits.h"
#include "workspace_analyzer.h"

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
 *
 * This implementation follows OpenSHC's exact philosophy:
 * - Iterative sequential flow (not normalized phase-based)
 * - Delta accumulation using quarticBezierDot derivatives
 * - Two-phase swing control (swing_1_nodes, swing_2_nodes)
 * - Timing based on iteration counters and delta_t values
 * - Precision control to minimize accumulated errors
 */
class LegStepper {
  public:
    // Constructor
    LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model);

    // Destructor
    ~LegStepper() = default;

    // Disable copy constructor and assignment operator (due to reference members)
    LegStepper(const LegStepper &) = delete;
    LegStepper &operator=(const LegStepper &) = delete;

    // Enable move constructor but disable move assignment operator (due to reference members)
    LegStepper(LegStepper &&) = default;
    LegStepper &operator=(LegStepper &&) = delete;

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
    double getStepProgress() const { return step_progress_; }

    // Debug getters for velocity troubleshooting
    Point3D getDesiredLinearVelocity() const { return desired_linear_velocity_; }
    double getDesiredAngularVelocity() const { return desired_angular_velocity_; }

    bool hasCompletedFirstStep() const { return completed_first_step_; }
    bool isAtCorrectPhase() const { return at_correct_phase_; }
    Point3D getSwing1ControlNode(int i) const { return swing_1_nodes_[i]; }
    Point3D getSwing2ControlNode(int i) const { return swing_2_nodes_[i]; }
    Point3D getStanceControlNode(int i) const { return stance_nodes_[i]; }
    Point3D getSwingClearance() const { return swing_clearance_; }

    // OpenSHC-specific accessors
    int getSwingIterations() const { return swing_iterations_; }
    int getStanceIterations() const { return stance_iterations_; }
    int getCurrentIteration() const { return current_iteration_; }
    double getSwingDeltaT() const { return swing_delta_t_; }
    double getStanceDeltaT() const { return stance_delta_t_; }

    // Modifiers
    void setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity);
    void setCurrentTipPose(const Point3D &pose) {
        current_tip_pose_ = pose;
        leg_.setCurrentTipPositionGlobal(pose);
    }
    void setDefaultTipPose(const Point3D &pose) { default_tip_pose_ = pose; }
    void setTargetTipPose(const Point3D &pose) { target_tip_pose_ = pose; }
    void setStepState(StepState state) { step_state_ = state; }
    void setPhase(int phase) { phase_ = phase; }
    void setStepProgress(double progress) { step_progress_ = progress; }
    void setPhaseOffset(double offset) { leg_.setPhaseOffset(offset); }
    void setSwingOriginTipVelocity(const Point3D &velocity) { swing_origin_tip_velocity_ = velocity; }
    void setCompletedFirstStep(bool completed) { completed_first_step_ = completed; }
    void setAtCorrectPhase(bool at_correct) { at_correct_phase_ = at_correct; }
    void setWalkPlane(const Point3D &walk_plane) { walk_plane_ = walk_plane; }
    Point3D getWalkPlane() const { return walk_plane_; }
    void setWalkPlaneNormal(const Point3D &walk_plane_normal) { walk_plane_normal_ = walk_plane_normal; }
    Point3D getWalkPlaneNormal() const { return walk_plane_normal_; }

    // OpenSHC-style StepCycle interface
    void setStepCycle(const StepCycle &step_cycle) { step_cycle_ = step_cycle; }
    const StepCycle &getStepCycle() const { return step_cycle_; }

    // Gait configuration parameters (not part of StepCycle)
    void setSwingWidth(double swing_width) { swing_width_ = swing_width; }
    double getSwingWidth() const { return swing_width_; }
    void setControlFrequency(double control_frequency) { control_frequency_ = control_frequency; }
    double getControlFrequency() const { return control_frequency_; }
    void setStepClearanceHeight(double step_clearance_height) { step_clearance_height_ = step_clearance_height; }
    double getStepClearanceHeight() const { return step_clearance_height_; }

    // OpenSHC-specific workflow methods
    void initializeSwingPeriod(int iteration);
    void calculateSwingTiming(double time_delta);
    void updateStride();
    void updateTipPositionIterative(int iteration, double time_delta, bool rough_terrain_mode = false, bool force_normal_touchdown = false);
    // Freeze stride & target (OpenSHC philosophical alignment) called on state transitions
    void beginSwingPhase();
    void beginStancePhase();

    // Workspace validation and safety methods (Step 1 implementation)
    bool validateTargetTipPose(const Point3D &target_pose) const;
    Point3D constrainToWorkspace(const Point3D &target_pose) const;
    Point3D calculateSafeTarget(const Point3D &desired_target) const;

    // Velocity limiting and validation methods (Step 2 implementation)
    Point3D validateAndLimitVelocities(const Point3D &linear_velocity, double angular_velocity);
    Point3D calculateSafeStride(const Point3D &desired_stride) const;

    // VelocityLimits integration for enhanced safety
    void setVelocityLimits(const VelocityLimits *velocity_limits) { velocity_limits_ = velocity_limits; }

    // Comprehensive safety validation (combines all 4 steps)
    bool validateCurrentTrajectory() const;

// Testing interface methods (only use in test code)
#ifdef TESTING_ENABLED
    void testGeneratePrimarySwingControlNodes() { generatePrimarySwingControlNodes(); }
    void testGenerateSecondarySwingControlNodes(bool ground_contact = false) { generateSecondarySwingControlNodes(ground_contact); }
    void testGenerateStanceControlNodes(double stride_scaler = 1.0) { generateStanceControlNodes(stride_scaler); }
#endif

  private:
    // Auxiliary method for velocity calculations
    Point3D calculateCurrentTipVelocity() const;

    // Control node generation (OpenSHC style)
    void generatePrimarySwingControlNodes();
    void generateSecondarySwingControlNodes(bool ground_contact = false);
    void generateStanceControlNodes(double stride_scaler = 1.0);

    // Control node validation methods (Step 4 implementation)
    void validateAndFixControlNodes(Point3D nodes[5]) const;

    // OpenSHC-style stride scaler calculation
    double calculateStanceStrideScaler();

  private:
    // Basic properties
    int leg_index_;
    Leg &leg_;
    RobotModel &robot_model_;
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

    // Phase and state management
    bool at_correct_phase_;
    bool completed_first_step_;
    int phase_;
    double step_progress_;
    StepState step_state_;

    // OpenSHC timing parameters - use StepCycle instead of individual variables
    StepCycle step_cycle_; // Complete step cycle timing (OpenSHC exact)
    double swing_delta_t_;
    double stance_delta_t_;
    int swing_iterations_;
    int stance_iterations_;
    int current_iteration_;

    // Gait configuration parameters (not part of StepCycle)
    double swing_width_;           // Lateral shift at mid-swing (OpenSHC mid_lateral_shift)
    double control_frequency_;     // Control loop frequency (equivalent to OpenSHC walker_->getTimeDelta())
    double step_clearance_height_; // Step clearance height (equivalent to OpenSHC walker_->getStepClearance())

    // Swing state management (OpenSHC style)
    bool swing_initialized_;
    bool nodes_generated_;
    int last_swing_iteration_;
    int last_swing_start_iteration_;

    // Additional swing trajectory variable (was missing)
    Point3D swing_clearance_; // Swing clearance vector (OpenSHC)

    // === OpenSHC philosophical alignment additions ===
    // We freeze (cache) stride components at the start of each phase to avoid intra-phase drift caused
    // by using the continuously-updated current_tip_pose_ (difference vs. prior HexaMotion implementation).
    Point3D frozen_stride_vector_linear_;
    Point3D frozen_stride_vector_angular_;
    Point3D frozen_stride_vector_total_;
    Point3D frozen_target_tip_pose_;
    bool stride_frozen_ = false;
    bool target_frozen_ = false;

    // Bezier control nodes (5 nodes for quartic curves)
    Point3D swing_1_nodes_[5];
    Point3D swing_2_nodes_[5];
    Point3D stance_nodes_[5];

    // Safety and validation systems
    const VelocityLimits *velocity_limits_; // Optional velocity limits for enhanced validation

    // Variables for velocity tracking
    Point3D previous_tip_pose_;
    bool has_previous_position_;
    double time_step_;
};

#endif // LEG_STEPPER_H