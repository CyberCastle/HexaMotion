#include "leg_stepper.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

#include "velocity_limits.h"

// Constructor without WalkController dependency
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

// Core functionality implementations
void LegStepper::updatePhase(const StepCycle &step) {
    phase_ = static_cast<int>(step_progress_ * step.period_);
    updateStepState(step);
}

void LegStepper::iteratePhase(const StepCycle &step) {
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

void LegStepper::updateStepState(const StepCycle &step) {
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

void LegStepper::updateStride(double linear_velocity_x, double linear_velocity_y, double angular_velocity, double stance_ratio, double step_frequency) {
    // Usar el método OpenSHC-equivalente
    stride_vector_ = VelocityLimits::calculateStrideVector(
        linear_velocity_x, linear_velocity_y, angular_velocity,
        getCurrentTipPose(), stance_ratio, step_frequency);
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

    // Update dynamic timing parameters (OpenSHC equivalent)
    updateDynamicTiming(used_step_length, time_delta);

    // Obtener parámetros de la configuración de marcha actual
    const auto &config = robot_model_.getParams().dynamic_gait;
    double stance_ratio = config.duty_factor;

    // Usar la frecuencia de control del sistema
    double control_frequency = robot_model_.getParams().control_frequency;

    // Calcular velocidad lineal basada en step_length y frecuencia de control
    double linear_velocity_x = used_step_length * control_frequency / 2.0;
    double linear_velocity_y = 0.0;
    double angular_velocity = 0.0;

    // Actualizar stride vector
    updateStride(linear_velocity_x, linear_velocity_y, angular_velocity, stance_ratio, control_frequency);

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

        // La posición actual es la posición del Bézier
        current_tip_pose_ = bezier_position;

        // Calcular velocidad usando la derivada de la curva Bézier
        Point3D bezier_velocity;
        if (time_input < 0.5) {
            double t = time_input * 2.0;
            bezier_velocity = math_utils::quarticBezierDot(swing_1_nodes_, t) * 2.0;
        } else {
            double t = (time_input - 0.5) * 2.0;
            bezier_velocity = math_utils::quarticBezierDot(swing_2_nodes_, t) * 2.0;
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

    leg_.setDesiredTipPositionGlobal(current_tip_pose_);
    leg_.applyIK(robot_model_);
}

void LegStepper::generatePrimarySwingControlNodes() {
    // If swing_origin_tip_position_ is at (0,0,0), initialize it with the current tip position
    if (swing_origin_tip_position_.norm() < 1e-6) {
        swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
    }
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

// These methods are now inline in the header file

// Nueva API OpenSHC-like
void LegStepper::updateWithPhase(double local_phase, double step_length, double time_delta) {
    const auto &config = robot_model_.getParams().dynamic_gait;

    // Use dynamic duty factor from configuration
    double duty_factor = config.duty_factor;

    // Determinar el nuevo estado basado en la fase
    StepState new_state;
    if (local_phase < duty_factor) {
        new_state = STEP_STANCE;
    } else {
        new_state = STEP_SWING;
    }

    // Actualizar posiciones de origen SOLO cuando hay transición de estado
    if (new_state != step_state_) {
        if (new_state == STEP_STANCE) {
            // Transición a STANCE: actualizar posición de origen de stance
            stance_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
        } else if (new_state == STEP_SWING) {
            // Transición a SWING: actualizar posición de origen de swing (OpenSHC equivalent)
            swing_origin_tip_position_ = leg_.getCurrentTipPositionGlobal();
            swing_origin_tip_velocity_ = current_tip_velocity_;
        }
    }

    // Actualizar estado y progreso
    step_state_ = new_state;

    if (step_state_ == STEP_STANCE) {
        double stance_progress = local_phase / duty_factor;
        step_progress_ = stance_progress;
        stance_progress_ = stance_progress;
        swing_progress_ = -1.0;
    } else {
        // SWING
        double swing_progress = (local_phase - duty_factor) / (1.0 - duty_factor);
        step_progress_ = swing_progress;
        swing_progress_ = swing_progress;
        stance_progress_ = -1.0;
    }

    // Actualizar la trayectoria y ángulos usando el progreso
    updateTipPosition(step_length, time_delta, false, false); // Default terrain adaptation values
}

// Dynamic iteration calculation implementations (OpenSHC equivalent)

int LegStepper::calculateSwingIterations(double step_length, double time_delta) const {
    const auto &config = robot_model_.getParams().dynamic_gait;

    if (!config.enable_dynamic_iterations) {
        // Fallback to static calculation
        return static_cast<int>(config.swing_phase / config.time_delta);
    }

    // OpenSHC-style dynamic calculation
    double swing_period = calculateSwingPeriod(step_length);
    double period = config.step_period;

    // Calculate iterations using OpenSHC formula
    int iterations = static_cast<int>((swing_period / period) / (config.frequency * time_delta));

    // Apply safety limits
    iterations = std::max(static_cast<int>(config.min_swing_iterations),
                          std::min(static_cast<int>(config.max_swing_iterations), iterations));

    return iterations;
}

int LegStepper::calculateStanceIterations(double step_length, double time_delta) const {
    const auto &config = robot_model_.getParams().dynamic_gait;

    if (!config.enable_dynamic_iterations) {
        // Fallback to static calculation
        return static_cast<int>(config.stance_phase / config.time_delta);
    }

    // OpenSHC-style dynamic calculation
    double stance_period = calculateStancePeriod(step_length);
    double period = config.step_period;

    // Calculate iterations using OpenSHC formula
    int iterations = static_cast<int>((stance_period / period) / (config.frequency * time_delta));

    // Apply safety limits
    iterations = std::max(static_cast<int>(config.min_stance_iterations),
                          std::min(static_cast<int>(config.max_stance_iterations), iterations));

    return iterations;
}

double LegStepper::calculateSwingPeriod(double step_length) const {
    const auto &config = robot_model_.getParams().dynamic_gait;

    // Base swing period from configuration
    double base_period = config.swing_phase;

    // Apply step length scaling (longer steps = longer swing period)
    double length_factor = std::max(0.5, std::min(2.0, step_length / 50.0)); // Normalize around 50mm

    // Apply swing period factor from configuration
    double scaled_period = base_period * config.swing_period_factor * length_factor;

    return scaled_period;
}

double LegStepper::calculateStancePeriod(double step_length) const {
    const auto &config = robot_model_.getParams().dynamic_gait;

    // Base stance period from configuration
    double base_period = config.stance_phase;

    // Apply step length scaling (longer steps = longer stance period)
    double length_factor = std::max(0.5, std::min(2.0, step_length / 50.0)); // Normalize around 50mm

    // Apply stance period factor from configuration
    double scaled_period = base_period * config.stance_period_factor * length_factor;

    return scaled_period;
}

void LegStepper::updateDynamicTiming(double step_length, double time_delta) {
    const auto &config = robot_model_.getParams().dynamic_gait;

    if (config.enable_dynamic_iterations) {
        // Calculate dynamic timing parameters
        double swing_period = calculateSwingPeriod(step_length);
        double stance_period = calculateStancePeriod(step_length);

        // Update timing deltas based on calculated periods
        swing_delta_t_ = swing_period * time_delta;
        stance_delta_t_ = stance_period * time_delta;
    } else {
        // Use static timing from configuration
        swing_delta_t_ = config.swing_phase * time_delta;
        stance_delta_t_ = config.stance_phase * time_delta;
    }
}

StepCycle LegStepper::calculateStepCycle(double step_length, double time_delta) const {
    const auto &config = robot_model_.getParams().dynamic_gait;

    StepCycle cycle;

    // Calculate dynamic iterations
    cycle.swing_period_ = calculateSwingIterations(step_length, time_delta);
    cycle.stance_period_ = calculateStanceIterations(step_length, time_delta);

    // Calculate total period
    cycle.period_ = cycle.swing_period_ + cycle.stance_period_;

    // Calculate phase boundaries
    cycle.stance_start_ = 0;
    cycle.stance_end_ = cycle.stance_period_;
    cycle.swing_start_ = cycle.stance_end_;
    cycle.swing_end_ = cycle.period_;

    // Set frequency from configuration
    cycle.frequency_ = config.frequency;

    return cycle;
}