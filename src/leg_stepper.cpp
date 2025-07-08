#include "leg_stepper.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include "hexamotion_constants.h"

//Constructor without WalkController dependency
LegStepper::LegStepper(int leg_index, const Point3D& identity_tip_pose, Leg& leg, RobotModel& robot_model,
                       WalkspaceAnalyzer* walkspace_analyzer, WorkspaceValidator* workspace_validator)
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
    stance_progress_ = 0.0;
    swing_progress_ = 0.0;
    step_progress_ = 0.0;
    step_state_ = STEP_STANCE;
    current_walk_state_ = WALK_STOPPED;
    swing_delta_t_ = 0.0;
    stance_delta_t_ = 0.0;
    touchdown_detection_ = false;
    step_length_ = 0.0;
    swing_height_ = 0.0;
    body_clearance_ = 0.0;

    for (int i = 0; i < 5; ++i) {
        swing_1_nodes_[i] = identity_tip_pose_;
        swing_2_nodes_[i] = identity_tip_pose_;
        stance_nodes_[i] = identity_tip_pose_;
    }
}

// Core functionality implementations
void LegStepper::updatePhase(const StepCycle& step) {
    phase_ = static_cast<int>(step_progress_ * step.period_);
    updateStepState(step);
}

void LegStepper::iteratePhase(const StepCycle& step) {
    phase_ = (phase_ + 1) % step.period_;
    updateStepState(step);

    // Calcular progreso de stance/swing periods (0.0->1.0 o -1.0 si no está en estado específico)
    step_progress_ = double(phase_) / step.period_;
    if (step_state_ == STEP_SWING) {
        swing_progress_ = double(phase_ - step.swing_start_ + 1) / double(step.swing_end_ - step.swing_start_);
        swing_progress_ = std::max(0.0, std::min(1.0, swing_progress_));
        stance_progress_ = -1.0;
    } else if (step_state_ == STEP_STANCE) {
        stance_progress_ = double((phase_ + (step.period_ - step.stance_start_)) % step.period_ + 1) /
                          double((step.stance_end_ - step.stance_start_ + step.period_) % step.period_);
        stance_progress_ = std::max(0.0, std::min(1.0, stance_progress_));
        swing_progress_ = -1.0;
    } else if (step_state_ == STEP_FORCE_STOP) {
        stance_progress_ = 0.0;
        swing_progress_ = -1.0;
    }
}

void LegStepper::updateStepState(const StepCycle& step) {
    // Actualizar estado de paso desde la fase a menos que esté forzado a parar
    if (step_state_ == STEP_FORCE_STOP) {
        return;
    }
    if (phase_ >= step.swing_start_ && phase_ < step.swing_end_ &&
        step_state_ != STEP_FORCE_STANCE) {
        step_state_ = STEP_SWING;
    } else {
        step_state_ = STEP_STANCE;
    }
}

void LegStepper::updateStride(double step_length) {
    // Stride global en dirección X (avance del robot), igual para todas las piernas
    stride_vector_ = Point3D(step_length, 0.0, 0.0);
}

Point3D LegStepper::calculateStanceSpanChange() {
    // Obtener altura objetivo del plano de trabajo
    Point3D default_shift = default_tip_pose_ - identity_tip_pose_;
    double target_workplane_height = default_shift.z;

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

    // Aplicar el modificador de span
    double span = radius * stance_span_modifier;
    return Point3D(0.0, span, 0.0);
}

void LegStepper::updateDefaultTipPosition() {
    Point3D new_default_tip_pose;

    if (external_default_.defined) {
        new_default_tip_pose = external_default_.position;
    } else {
        Point3D identity_tip_position = identity_tip_pose_;
        identity_tip_position = identity_tip_position + calculateStanceSpanChange();
        // Usar body_clearance_ si está configurado
        if (body_clearance_ > 0.0) {
            identity_tip_position.z = -body_clearance_;
        }
        Point3D identity_to_stance_origin = stance_origin_tip_position_ - identity_tip_position;
        Point3D projection_to_walk_plane = math_utils::projectVector(identity_to_stance_origin, walk_plane_normal_);
        new_default_tip_pose = identity_tip_position + projection_to_walk_plane;
    }
    double default_tip_position_delta = math_utils::distance(default_tip_pose_, new_default_tip_pose);
    default_tip_pose_ = new_default_tip_pose;
}

void LegStepper::updateTipPosition(double step_length, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // Usar el step_length configurado si no se pasa uno explícito
    double used_step_length = (step_length_ > 0.0) ? step_length_ : step_length;
    target_tip_pose_ = default_tip_pose_ + stride_vector_ * 0.5;

    // Período de Swing
    if (step_state_ == STEP_SWING) {
        updateStride(used_step_length);
        double time_input = step_progress_;
        generatePrimarySwingControlNodes();
        generateSecondarySwingControlNodes(false);
        Point3D delta_pos = math_utils::quarticBezierDot(swing_1_nodes_, time_input);
        Point3D new_tip_position = swing_origin_tip_position_ + delta_pos;
        current_tip_pose_ = new_tip_position;
        current_tip_velocity_ = delta_pos / time_delta;
        leg_.setDesiredTipPositionGlobal(new_tip_position);
        leg_.applyIK(robot_model_);
    }
    // Período de Stance
    else if (step_state_ == STEP_STANCE || step_state_ == STEP_FORCE_STANCE) {
        updateStride(used_step_length);
        double time_input = step_progress_;
        generateStanceControlNodes(1.0);
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input);
        Point3D new_tip_position = stance_origin_tip_position_ + delta_pos;
        current_tip_pose_ = new_tip_position;
        current_tip_velocity_ = delta_pos / time_delta;
        leg_.setDesiredTipPositionGlobal(new_tip_position);
        leg_.applyIK(robot_model_);
    }
}

void LegStepper::generatePrimarySwingControlNodes() {
    Point3D mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_) * 0.5;
    // Usar swing_height_ si está configurado
    if (swing_height_ > 0.0) {
        mid_tip_position.z = swing_origin_tip_position_.z + swing_height_;
    } else {
        mid_tip_position.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
        mid_tip_position = mid_tip_position + swing_clearance_;
    }

    // Get swing width parameter from robot model or use default
    double mid_lateral_shift = LEG_STEPPER_SWING_LATERAL_SHIFT; // Default swing width, configurable
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    //Avoid division by zero and use reasonable defaults
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
    //Avoid division by zero and use reasonable defaults
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
    Point3D stance_node_separation = stride_vector_ * stride_scaler * LEG_STEPPER_STANCE_NODE_SCALER;

    // Nodos de control para curva Bézier cuártica de stance
    stance_nodes_[0] = stance_origin_tip_position_ + stance_node_separation * 0.0;
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_separation * 1.0;
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_separation * 2.0;
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_separation * 3.0;
    stance_nodes_[4] = stance_origin_tip_position_ + stance_node_separation * 4.0;
}

void LegStepper::forceNormalTouchdown() {
    //Avoid division by zero and use reasonable defaults
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

// These methods are now inline in the header file

// Nueva API OpenSHC-like
void LegStepper::updateWithPhase(double local_phase, double step_length, double time_delta) {
    // Duty factor para trípode (0.5: mitad del ciclo en stance, mitad en swing)
    constexpr double duty_factor = LEG_STEPPER_DUTY_FACTOR;

    // Actualizar posiciones de origen si es necesario
    if (step_state_ == STEP_STANCE && local_phase < duty_factor) {
        // Estamos en stance, actualizar posición de origen de stance
        stance_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    } else if (step_state_ == STEP_SWING && local_phase >= duty_factor) {
        // Estamos en swing, actualizar posición de origen de swing
        swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    }

    // Determinar estado y progreso
    if (local_phase < duty_factor) {
        // STANCE
        step_state_ = STEP_STANCE;
        double stance_progress = local_phase / duty_factor;
        step_progress_ = stance_progress;
        stance_progress_ = stance_progress;
        swing_progress_ = -1.0;
    } else {
        // SWING
        step_state_ = STEP_SWING;
        double swing_progress = (local_phase - duty_factor) / (1.0 - duty_factor);
        step_progress_ = swing_progress;
        swing_progress_ = swing_progress;
        stance_progress_ = -1.0;
    }

    // Actualizar la trayectoria y ángulos usando el progreso
    updateTipPosition(step_length, time_delta, false, false);  // Default terrain adaptation values
}