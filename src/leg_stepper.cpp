#include "leg_stepper.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

// ✅ CORRECTED: Constructor without WalkController dependency
LegStepper::LegStepper(int leg_index, const Point3D& identity_tip_pose, Leg& leg, RobotModel& robot_model)
            : leg_index_(leg_index), leg_(leg), robot_model_(robot_model), identity_tip_pose_(identity_tip_pose),
          default_tip_pose_(identity_tip_pose), origin_tip_pose_(identity_tip_pose), target_tip_pose_(identity_tip_pose),
          walk_plane_(Point3D(0, 0, 0)), walk_plane_normal_(Point3D(0, 0, 1)), stride_vector_(Point3D(0, 0, 0)),
          current_tip_velocity_(Point3D(0, 0, 0)), swing_origin_tip_position_(identity_tip_pose),
          swing_origin_tip_velocity_(Point3D(0, 0, 0)), stance_origin_tip_position_(identity_tip_pose),
          swing_clearance_(Point3D(0, 0, 30.0)), at_correct_phase_(false), completed_first_step_(false), phase_(0),
          stance_progress_(0.0), swing_progress_(0.0), step_progress_(0.0), step_state_(STEP_STANCE),
          current_walk_state_(WALK_STOPPED), swing_delta_t_(0.0), stance_delta_t_(0.0), touchdown_detection_(false)
{
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
    if (phase_ >= step.swing_start_ && phase_ < step.swing_end_ && step_state_ != STEP_FORCE_STANCE) {
        step_state_ = STEP_SWING;
    } else if (phase_ < step.stance_end_ || phase_ >= step.stance_start_) {
        step_state_ = STEP_STANCE;
    }
}

void LegStepper::updateStride(double step_length) {
    // Stride global en dirección X (avance del robot), igual para todas las piernas
    stride_vector_ = Point3D(step_length, 0.0, 0.0);
}

Point3D LegStepper::calculateStanceSpanChange() {
    // Calcular altura objetivo del plano dentro del workspace
    Point3D default_shift = default_tip_pose_ - identity_tip_pose_;
    double target_workplane_height = default_shift.z;

    // Get stance span modifier from robot model parameters
    double stance_span_modifier = 0.1; // Default value, can be made configurable
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    int bearing = (positive_y_axis ^ (stance_span_modifier > 0.0)) ? 270 : 90;
    stance_span_modifier *= (positive_y_axis ? 1.0 : -1.0);

    // Simplificado por ahora
    return Point3D(0.0, 50.0 * stance_span_modifier, 0.0);
}

void LegStepper::updateDefaultTipPosition() {
    Point3D new_default_tip_pose;

    // Generar nueva posición por defecto basada en posición del tip al final del período de swing
    if (external_default_.defined) {
        new_default_tip_pose = external_default_.position;
    } else {
        // Modificar posiciones de tip de identidad según el modificador de stance span
        Point3D identity_tip_position = identity_tip_pose_;
        identity_tip_position = identity_tip_position + calculateStanceSpanChange();

        // Actualizar posición por defecto como proyección de la posición del tip al inicio del período STANCE
        Point3D identity_to_stance_origin = stance_origin_tip_position_ - identity_tip_position;
        Point3D projection_to_walk_plane = math_utils::projectVector(identity_to_stance_origin, walk_plane_normal_);
        new_default_tip_pose = identity_tip_position + projection_to_walk_plane;
    }

    // Actualizar posición por defecto del tip
    double default_tip_position_delta = math_utils::distance(default_tip_pose_, new_default_tip_pose);
    default_tip_pose_ = new_default_tip_pose;
}

void LegStepper::updateTipPosition(double step_length, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // Generar target por defecto
    target_tip_pose_ = default_tip_pose_ + stride_vector_ * 0.5;

    // Período de Swing
    if (step_state_ == STEP_SWING) {
        updateStride(step_length);
        // Usar step_progress_ como parámetro de interpolación [0,1]
        double time_input = step_progress_;
        // Generar nodos de control si es necesario (puedes optimizar esto si lo deseas)
        generatePrimarySwingControlNodes();
        generateSecondarySwingControlNodes(false);
        // Interpolación Bézier para swing
        Point3D delta_pos = math_utils::quarticBezierDot(swing_1_nodes_, time_input);
        Point3D new_tip_position = swing_origin_tip_position_ + delta_pos;
        leg_.setTipPosition(new_tip_position);
        // ✅ CORRECTED: Use robot model for inverse kinematics
        JointAngles new_angles = robot_model_.inverseKinematics(leg_index_, new_tip_position);
        leg_.setJointAngles(new_angles);
        current_tip_velocity_ = delta_pos / time_delta;
    }
    // Período de Stance
    else if (step_state_ == STEP_STANCE || step_state_ == STEP_FORCE_STANCE) {
        updateStride(step_length);
        double time_input = step_progress_;
        generateStanceControlNodes(1.0); // Puedes ajustar el stride_scaler si es necesario
        // Interpolación Bézier para stance
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input);
        Point3D new_tip_position = stance_origin_tip_position_ + delta_pos;
        leg_.setTipPosition(new_tip_position);
        // ✅ CORRECTED: Use robot model for inverse kinematics
        JointAngles new_angles = robot_model_.inverseKinematics(leg_index_, new_tip_position);
        leg_.setJointAngles(new_angles);
        current_tip_velocity_ = delta_pos / time_delta;
    }
}

void LegStepper::generatePrimarySwingControlNodes() {
    Point3D mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_) * 0.5;
    mid_tip_position.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    mid_tip_position = mid_tip_position + swing_clearance_;

    // Get swing width parameter from robot model or use default
    double mid_lateral_shift = 10.0; // Default swing width, can be made configurable
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    Point3D stance_node_separation = swing_origin_tip_velocity_ * (1.0 / swing_delta_t_) * 0.25;

    // Nodos de control para curvas Bézier cuárticas de swing primario
    swing_1_nodes_[0] = swing_origin_tip_position_;
    swing_1_nodes_[1] = swing_origin_tip_position_ + stance_node_separation;
    swing_1_nodes_[2] = swing_origin_tip_position_ + stance_node_separation * 2.0;
    swing_1_nodes_[3] = (mid_tip_position + swing_1_nodes_[2]) * 0.5;
    swing_1_nodes_[3].z = mid_tip_position.z;
    swing_1_nodes_[4] = mid_tip_position;
}

void LegStepper::generateSecondarySwingControlNodes(bool ground_contact) {
    Point3D final_tip_velocity = stride_vector_ * (stance_delta_t_ / 1.0) * -1.0;
    Point3D stance_node_separation = final_tip_velocity * (1.0 / swing_delta_t_) * 0.25;

    // Nodos de control para curvas Bézier cuárticas de swing secundario
    swing_2_nodes_[0] = swing_1_nodes_[4];
    swing_2_nodes_[1] = swing_1_nodes_[4] - (swing_1_nodes_[3] - swing_1_nodes_[4]);
    swing_2_nodes_[2] = target_tip_pose_ - stance_node_separation * 2.0;
    swing_2_nodes_[3] = target_tip_pose_ - stance_node_separation;
    swing_2_nodes_[4] = target_tip_pose_;

    // Detener movimiento adicional de la posición del tip en dirección normal al plano de marcha
    if (ground_contact) {
        Point3D current_pos = leg_.getTipPosition();
        swing_2_nodes_[0] = current_pos + stance_node_separation * 0.0;
        swing_2_nodes_[1] = current_pos + stance_node_separation * 1.0;
        swing_2_nodes_[2] = current_pos + stance_node_separation * 2.0;
        swing_2_nodes_[3] = current_pos + stance_node_separation * 3.0;
        swing_2_nodes_[4] = current_pos + stance_node_separation * 4.0;
    }
}

void LegStepper::generateStanceControlNodes(double stride_scaler) {
    Point3D stance_node_separation = stride_vector_ * stride_scaler * -0.25;

    // Nodos de control para curva Bézier cuártica de stance
    stance_nodes_[0] = stance_origin_tip_position_ + stance_node_separation * 0.0;
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_separation * 1.0;
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_separation * 2.0;
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_separation * 3.0;
    stance_nodes_[4] = stance_origin_tip_position_ + stance_node_separation * 4.0;
}

void LegStepper::forceNormalTouchdown() {
    Point3D final_tip_velocity = stride_vector_ * (stance_delta_t_ / 1.0) * -1.0;
    Point3D stance_node_separation = final_tip_velocity * (1.0 / swing_delta_t_) * 0.25;

    Point3D bezier_target = target_tip_pose_;
    Point3D bezier_origin = target_tip_pose_ - stance_node_separation * 4.0;
    bezier_origin.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    bezier_origin = bezier_origin + swing_clearance_;

    swing_1_nodes_[4] = bezier_origin;
    swing_2_nodes_[0] = bezier_origin;
    swing_2_nodes_[2] = bezier_target - stance_node_separation * 2.0;
    swing_1_nodes_[3] = swing_2_nodes_[0] - (swing_2_nodes_[2] - bezier_origin) * 0.5;
    swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[2] - bezier_origin) * 0.5;
}

// These methods are now inline in the header file

// Nueva API OpenSHC-like
void LegStepper::updateWithPhase(double local_phase, double step_length, double time_delta) {
    // Duty factor para trípode (0.5: mitad del ciclo en stance, mitad en swing)
    constexpr double duty_factor = 0.5;

    // Actualizar posiciones de origen si es necesario
    if (step_state_ == STEP_STANCE && local_phase < duty_factor) {
        // Estamos en stance, actualizar posición de origen de stance
        stance_origin_tip_position_ = leg_.getTipPosition();
    } else if (step_state_ == STEP_SWING && local_phase >= duty_factor) {
        // Estamos en swing, actualizar posición de origen de swing
        swing_origin_tip_position_ = leg_.getTipPosition();
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