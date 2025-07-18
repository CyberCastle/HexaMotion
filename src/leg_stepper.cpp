#include "leg_stepper.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

#include "velocity_limits.h"

LegStepper::LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model,
                       WalkspaceAnalyzer *walkspace_analyzer, WorkspaceValidator *workspace_validator)
    : leg_index_(leg_index),
      leg_(leg),
      robot_model_(robot_model),
      identity_tip_pose_(identity_tip_pose),
      walkspace_analyzer_(walkspace_analyzer),
      workspace_validator_(workspace_validator) {
    // Inicialización estándar
    default_tip_pose_ = identity_tip_pose_;
    origin_tip_pose_ = identity_tip_pose_;
    target_tip_pose_ = identity_tip_pose_;
    current_tip_pose_ = identity_tip_pose_;
    at_correct_phase_ = false;
    completed_first_step_ = false;
    phase_ = 0;
    step_progress_ = 0.0;
    step_state_ = STEP_STANCE;
    current_walk_state_ = WALK_STOPPED;
    swing_delta_t_ = 0.0;
    stance_delta_t_ = 0.0;
    touchdown_detection_ = false;
    step_length_ = 0.0;
    swing_height_ = 0.0;
    body_clearance_ = 0.0;
    swing_clearance_ = Point3D(0.0, 0.0, 0.0);

    // Initialize walk plane normal (pointing up by default)
    walk_plane_normal_ = Point3D(0.0, 0.0, 1.0);

    // Initialize origin positions with identity pose instead of (0,0,0)
    swing_origin_tip_position_ = identity_tip_pose_;
    stance_origin_tip_position_ = identity_tip_pose_;
    swing_origin_tip_velocity_ = Point3D(0.0, 0.0, 0.0);

    for (int i = 0; i < 5; ++i) {
        swing_1_nodes_[i] = identity_tip_pose_;
        swing_2_nodes_[i] = identity_tip_pose_;
        stance_nodes_[i] = identity_tip_pose_;
    }
}

// Add setter implementation for desired velocity to ensure velocities are synced
void LegStepper::setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity) {
    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

// Main unified step cycle update method (OpenSHC equivalent)
void LegStepper::updateStepCycle(double normalized_phase, double step_length, double time_delta) {
    // Get current step cycle configuration
    const auto &config = robot_model_.getParams().dynamic_gait;

    // Create step cycle from configuration
    StepCycle step_cycle;
    step_cycle.period_ = config.stance_phase + config.swing_phase;
    step_cycle.stance_start_ = 0;
    step_cycle.stance_end_ = config.stance_phase;
    step_cycle.swing_start_ = config.stance_phase;
    step_cycle.swing_end_ = step_cycle.period_;
    step_cycle.frequency_ = config.frequency;

    // 1. Update step state (STANCE/SWING)
    updateStepState(step_cycle);

    // 2. Calculate step progress
    calculateStepProgress(normalized_phase, step_cycle);

    // 3. Update dynamic timing parameters
    updateDynamicTiming(step_length, time_delta);

    // 4. Update tip position and trajectories
    updateTipPosition(step_length, time_delta, false, false);
}

void LegStepper::calculateStepProgress(double normalized_phase, const StepCycle &step) {
    step_progress_ = normalized_phase;

    // Update at_correct_phase_ and completed_first_step_ based on walk state
    if (current_walk_state_ == WALK_STARTING) {
        // During starting, check if we're at the correct phase for this leg
        if (step_state_ == STEP_STANCE && step_progress_ < 0.1) {
            at_correct_phase_ = true;
            completed_first_step_ = true;
        }
    } else if (current_walk_state_ == WALK_STOPPING) {
        // During stopping, check if we're back to default position
        Point3D error = current_tip_pose_ - default_tip_pose_;
        if (error.norm() < 5.0) { // 5mm tolerance
            at_correct_phase_ = true;
        }
    } else if (current_walk_state_ == WALK_MOVING) {
        at_correct_phase_ = true;
        completed_first_step_ = true;
    }
}

void LegStepper::updateStepState(const StepCycle &step) {
    // Update step state from phase unless force stopped
    if (step_state_ == STEP_FORCE_STOP) {
        return;
    }

    if (phase_ >= step.swing_start_ && phase_ < step.swing_end_ && step_state_ != STEP_FORCE_STANCE) {
        // Transition to swing state
        if (step_state_ != STEP_SWING) {
            // Just transitioned to swing - update swing origin
            swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
            swing_origin_tip_velocity_ = current_tip_velocity_;
        }
        step_state_ = STEP_SWING;
    } else {
        // Transition to stance state
        if (step_state_ != STEP_STANCE) {
            // Just transitioned to stance - update stance origin
            stance_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
        }
        step_state_ = STEP_STANCE;
    }
}

void LegStepper::updateStride() {
    // Usar el método OpenSHC-equivalente
    stride_vector_ = VelocityLimits::calculateStrideVector(
        desired_linear_velocity_.x, desired_linear_velocity_.y, desired_angular_velocity_,
        getCurrentTipPose(), robot_model_.getParams().dynamic_gait.duty_factor, robot_model_.getParams().control_frequency);
}

Point3D LegStepper::calculateStanceSpanChange() {
    // Use body_clearance_ directly as the target workplane height
    // This avoids circular dependency on default_tip_pose_
    double target_workplane_height = -body_clearance_;

    // stance_span_modifier configurable por marcha
    double stance_span_modifier = stance_span_modifier_;

    // Determinar bearing (dirección lateral de la pierna)
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    int bearing = (positive_y_axis ^ (stance_span_modifier > 0.0)) ? 270 : 90;
    stance_span_modifier *= (positive_y_axis ? 1.0 : -1.0);

    // Consultar el workspace real usando WalkspaceAnalyzer
    double radius = 0.0;
    if (walkspace_analyzer_) {
        // Obtener workplane interpolado a la altura objetivo
        auto workplane = walkspace_analyzer_->getWorkplane(leg_index_, target_workplane_height);
        if (!workplane.empty() && workplane.count(bearing)) {
            radius = workplane.at(bearing);
        }
    }
    // Fallback: usar WorkspaceValidator si no hay WalkspaceAnalyzer
    if (radius == 0.0 && workspace_validator_) {
        WorkspaceBounds bounds = workspace_validator_->getWorkspaceBounds(leg_index_);
        radius = bounds.max_radius;
    }

    // TODO: Validate if workspace validator is available
    // Fallback final: usar estimación basada en geometría del robot si no hay workspace data
    if (radius == 0.0) {
        // Estimar un radio base basado en la posición de identidad
        double identity_radius = sqrt(identity_tip_pose_.x * identity_tip_pose_.x +
                                      identity_tip_pose_.y * identity_tip_pose_.y);
        radius = identity_radius * 0.8; // Use 80% of identity radius as safe estimate
    }

    // Aplicar el modificador de span
    double span = radius * stance_span_modifier;
    return Point3D(0.0, span, 0.0);
}

void LegStepper::updateDefaultTipPosition() {
    Point3D new_default_tip_pose;

    if (external_default_.defined) {
        new_default_tip_pose = external_default_.position;
    } else {
        // Start with identity position
        Point3D identity_tip_position = identity_tip_pose_;

        // Apply lateral stance span adjustment
        identity_tip_position = identity_tip_position + calculateStanceSpanChange();

        // Apply body clearance (maintain z = -body_clearance_)
        if (body_clearance_ > 0.0) {
            identity_tip_position.z = -body_clearance_;
        }

        // Use the adjusted position directly as the default tip pose
        // Remove erroneous projection logic that was canceling body_clearance_
        new_default_tip_pose = identity_tip_position;
    }
    double default_tip_position_delta = math_utils::distance(default_tip_pose_, new_default_tip_pose);
    default_tip_pose_ = new_default_tip_pose;
}

void LegStepper::updateTipPosition(double step_length, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // Usar el step_length configurado si no se pasa uno explícito
    double used_step_length = (step_length_ > 0.0) ? step_length_ : step_length;

    // TODO: Use dynamic gait parameters to adjust step length
    // Update dynamic timing parameters (OpenSHC equivalent)
    // updateDynamicTiming(used_step_length, time_delta);

    // Obtener parámetros de la configuración de marcha actual
    const auto &config = robot_model_.getParams().dynamic_gait;
    double stance_ratio = config.duty_factor;

    // Usar la frecuencia de control del sistema
    double control_frequency = robot_model_.getParams().control_frequency;

    // Actualizar stride vector
    updateStride();

    // En OpenSHC, el target se calcula desde la posición actual, no desde default_tip_pose_
    Point3D origin_position;
    if (step_state_ == STEP_SWING) {
        origin_position = swing_origin_tip_position_;
    } else {
        origin_position = stance_origin_tip_position_;
    }

    // Calcular target tip pose basado en el origen actual + stride vector (OpenSHC equivalent)
    target_tip_pose_ = origin_position + stride_vector_ * 0.5;

    // Período de Swing
    if (step_state_ == STEP_SWING) {
        // Generar nodos de control para swing
        generatePrimarySwingControlNodes();
        generateSecondarySwingControlNodes(false);

        // Usar step_progress_ para interpolación Bézier
        double time_input = step_progress_;

        // Determinar qué curva usar basado en el progreso
        Point3D bezier_position;
        if (time_input < 0.5) {
            // Primera mitad del swing - usar swing_1_nodes_
            double t = time_input * 2.0; // Mapear [0,0.5] a [0,1]
            bezier_position = math_utils::quarticBezier(swing_1_nodes_, t);
        } else {
            // Segunda mitad del swing - usar swing_2_nodes_
            double t = (time_input - 0.5) * 2.0; // Mapear [0.5,1] a [0,1]
            bezier_position = math_utils::quarticBezier(swing_2_nodes_, t);
        }

        // Set position from Bézier (z already encoded by control nodes)
        current_tip_pose_ = bezier_position;

        // Calculate velocity using bezier derivative
        Point3D bezier_velocity;
        if (time_input < 0.5) {
            double t = time_input * 2.0;
            bezier_velocity = math_utils::quarticBezierDot(swing_1_nodes_, t) * 2.0; // Chain rule
        } else {
            double t = (time_input - 0.5) * 2.0;
            bezier_velocity = math_utils::quarticBezierDot(swing_2_nodes_, t) * 2.0; // Chain rule
        }
        current_tip_velocity_ = bezier_velocity / time_delta;
    }
    // Período de Stance
    else if (step_state_ == STEP_STANCE || step_state_ == STEP_FORCE_STANCE) {
        // Generar nodos de control para stance
        generateStanceControlNodes(1.0);

        // Usar step_progress_ para interpolación Bézier
        double time_input = step_progress_;
        Point3D bezier_position = math_utils::quarticBezier(stance_nodes_, time_input);

        // La posición actual es la posición del Bézier
        current_tip_pose_ = bezier_position;

        // Calcular velocidad usando la derivada de la curva Bézier
        Point3D bezier_velocity = math_utils::quarticBezierDot(stance_nodes_, time_input);
        current_tip_velocity_ = bezier_velocity / time_delta;
    }

    // Synchronize with leg after trajectory generation
    // Synchronize leg state: apply IK to reach current tip pose and sync FK
    leg_.applyIK(current_tip_pose_);
}

void LegStepper::generatePrimarySwingControlNodes() {
    // If swing_origin_tip_position_ is at (0,0,0), initialize it with the current tip position
    if (swing_origin_tip_position_.norm() < 1e-6) {
        swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    }
    // Ensure swing origin at default tip z
    {
        Point3D origin = swing_origin_tip_position_;
        origin.z = default_tip_pose_.z;
        swing_origin_tip_position_ = origin;
    }

    // Calculate midpoint for swing using default tip z and configured swing height
    Point3D mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_) * 0.5;
    // Use configured swing clearance for midpoint height
    mid_tip_position.z = default_tip_pose_.z + swing_clearance_.z;

    // Get swing width parameter from robot model or use default
    double mid_lateral_shift = LEG_STEPPER_SWING_LATERAL_SHIFT; // Default swing width, configurable
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    // Avoid division by zero and use reasonable defaults
    Point3D stance_node_separation;
    if (swing_delta_t_ > LEG_STEPPER_FLOAT_TOLERANCE && swing_origin_tip_velocity_.norm() < LEG_STEPPER_MAX_SWING_VELOCITY) {
        stance_node_separation = swing_origin_tip_velocity_ * (1.0 / swing_delta_t_) * LEG_STEPPER_SWING_NODE_SCALER;
    } else {
        stance_node_separation = stride_vector_ * LEG_STEPPER_DEFAULT_NODE_SEPARATION;
    }

    // Nodos de control para curvas Bézier cuárticas de swing primario
    swing_1_nodes_[0] = swing_origin_tip_position_;
    swing_1_nodes_[1] = swing_origin_tip_position_ + stance_node_separation;
    swing_1_nodes_[2] = swing_origin_tip_position_ + stance_node_separation * 2.0;
    swing_1_nodes_[3] = (mid_tip_position + swing_1_nodes_[2]) * LEG_STEPPER_TOUCHDOWN_INTERPOLATION;
    swing_1_nodes_[3].z = mid_tip_position.z;
    swing_1_nodes_[4] = mid_tip_position;
}

void LegStepper::generateSecondarySwingControlNodes(bool ground_contact) {
    // If swing_origin_tip_position_ is at (0,0,0), initialize it with the current tip position
    if (swing_origin_tip_position_.norm() < 1e-6) {
        swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    }
    // Avoid division by zero and use reasonable defaults
    Point3D final_tip_velocity;
    Point3D stance_node_separation;

    if (stance_delta_t_ > LEG_STEPPER_FLOAT_TOLERANCE && swing_delta_t_ > LEG_STEPPER_FLOAT_TOLERANCE) {
        final_tip_velocity = stride_vector_ * (stance_delta_t_ / 1.0) * -1.0;
        stance_node_separation = final_tip_velocity * (1.0 / swing_delta_t_) * LEG_STEPPER_SWING_NODE_SCALER;
    } else {
        // Use default values when timing parameters are not properly initialized
        final_tip_velocity = stride_vector_ * LEG_STEPPER_DEFAULT_TOUCHDOWN_VELOCITY;
        stance_node_separation = stride_vector_ * LEG_STEPPER_DEFAULT_NODE_SEPARATION;
    }

    // Nodos de control para curvas Bézier cuárticas de swing secundario
    swing_2_nodes_[0] = swing_1_nodes_[4];
    swing_2_nodes_[1] = swing_1_nodes_[4] - (swing_1_nodes_[3] - swing_1_nodes_[4]);
    swing_2_nodes_[2] = target_tip_pose_ - stance_node_separation * 2.0;
    swing_2_nodes_[3] = target_tip_pose_ - stance_node_separation;
    swing_2_nodes_[4] = target_tip_pose_;

    // Detener movimiento adicional de la posición del tip en dirección normal al plano de marcha
    if (ground_contact) {
        Point3D current_pos = leg_.getCurrentTipPositionGlobal();
        swing_2_nodes_[0] = current_pos + stance_node_separation * 0.0;
        swing_2_nodes_[1] = current_pos + stance_node_separation * 1.0;
        swing_2_nodes_[2] = current_pos + stance_node_separation * 2.0;
        swing_2_nodes_[3] = current_pos + stance_node_separation * 3.0;
        swing_2_nodes_[4] = current_pos + stance_node_separation * 4.0;
    }
}

void LegStepper::generateStanceControlNodes(double stride_scaler) {
    // If stance_origin_tip_position_ is at (0,0,0), initialize it with the current tip position
    if (stance_origin_tip_position_.norm() < 1e-6) {
        stance_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    }

    // Calcular posición objetivo de stance basada en el stride vector
    Point3D stance_target_position = stance_origin_tip_position_ + stride_vector_ * stride_scaler;

    // Calcular separación de nodos para suavidad de curva
    Point3D stance_node_separation = stride_vector_ * stride_scaler * LEG_STEPPER_STANCE_NODE_SCALER;

    // Control nodes for quartic Bézier stance curve (OpenSHC equivalent)
    // Nodo 0: posición de origen
    stance_nodes_[0] = stance_origin_tip_position_;
    // Nodos 1-3: nodos intermedios para suavidad
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_separation * 0.25;
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_separation * 0.5;
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_separation * 0.75;
    // Nodo 4: posición objetivo
    stance_nodes_[4] = stance_target_position;
}

void LegStepper::forceNormalTouchdown() {
    // Avoid division by zero and use reasonable defaults
    Point3D final_tip_velocity;
    Point3D stance_node_separation;

    if (stance_delta_t_ > LEG_STEPPER_FLOAT_TOLERANCE && swing_delta_t_ > LEG_STEPPER_FLOAT_TOLERANCE) {
        final_tip_velocity = stride_vector_ * (stance_delta_t_ / 1.0) * -1.0;
        stance_node_separation = final_tip_velocity * (1.0 / swing_delta_t_) * LEG_STEPPER_SWING_NODE_SCALER;
    } else {
        // Use default values when timing parameters are not properly initialized
        final_tip_velocity = stride_vector_ * LEG_STEPPER_DEFAULT_TOUCHDOWN_VELOCITY;
        stance_node_separation = stride_vector_ * LEG_STEPPER_DEFAULT_NODE_SEPARATION;
    }

    Point3D bezier_target = target_tip_pose_;
    Point3D bezier_origin = target_tip_pose_ - stance_node_separation * LEG_STEPPER_TOUCHDOWN_NODE_MULTIPLIER;
    bezier_origin.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    bezier_origin = bezier_origin + swing_clearance_;

    swing_1_nodes_[4] = bezier_origin;
    swing_2_nodes_[0] = bezier_origin;
    swing_2_nodes_[2] = bezier_target - stance_node_separation * 2.0;
    swing_1_nodes_[3] = swing_2_nodes_[0] - (swing_2_nodes_[2] - bezier_origin) * LEG_STEPPER_TOUCHDOWN_INTERPOLATION;
    swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[2] - bezier_origin) * LEG_STEPPER_TOUCHDOWN_INTERPOLATION;
}

void LegStepper::updateDynamicTiming(double step_length, double time_delta) {
    const auto &config = robot_model_.getParams().dynamic_gait;

    if (config.enable_dynamic_iterations) {
        // Use dynamic timing based on step length
        double length_factor = std::max(0.5, std::min(2.0, step_length / 50.0)); // Normalize around 50mm
        swing_delta_t_ = config.swing_phase * config.swing_period_factor * length_factor * time_delta;
        stance_delta_t_ = config.stance_phase * config.stance_period_factor * length_factor * time_delta;
    } else {
        // Use static timing from configuration
        swing_delta_t_ = config.swing_phase * time_delta;
        stance_delta_t_ = config.stance_phase * time_delta;
    }
}