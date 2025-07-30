#include "admittance_controller.h"
#include "hexamotion_constants.h"
#include "workspace_validator.h" // Use unified validator for workspace utilities

/**
 * @file admittance_controller.cpp
 * @brief Implements the admittance-based compliance controller.
 */

#include "math_utils.h"
#include <algorithm>
#include <cmath>

namespace {
// File-scope empty state to avoid static local initialization
static const AdmittanceController::LegAdmittanceState EMPTY_LEG_STATE;
} // namespace

AdmittanceController::AdmittanceController(RobotModel &model, IIMUInterface *imu, IFSRInterface *fsr,
                                           ComputeConfig config)
    : model_(model), imu_(imu), fsr_(fsr), config_(config),
      dynamic_stiffness_enabled_(false), swing_stiffness_scaler_(WORKSPACE_SCALING_FACTOR),
      load_stiffness_scaler_(1.5f), step_clearance_(40.0f),
      current_time_(0.0) {

    // Initialize workspace validator for position calculations
    ValidationConfig validator_config;
    validator_config.enable_collision_checking = false;   // Disable for performance in admittance control
    validator_config.enable_joint_limit_checking = false; // Not needed for stiffness calculations
    workspace_validator_ = std::make_unique<WorkspaceValidator>(model_, validator_config);

    delta_time_ = config_.getDeltaTime();
    selectIntegrationMethod();

    // Initialize state vectors and external forces
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_dynamics_state_[i] = math_utils::StateVector<Point3D>(Point3D(0, 0, 0), Point3D(0, 0, 0));
        external_forces_[i] = Point3D(0, 0, 0);
    }
}

AdmittanceController::~AdmittanceController() = default;

void AdmittanceController::initialize() {
    initializeDefaultParameters();
    resetAllDynamics();
}

void AdmittanceController::setLegAdmittance(int leg_index, double mass, double damping, double stiffness) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_states_[leg_index].params.virtual_mass = mass;
        leg_states_[leg_index].params.virtual_damping = damping;
        leg_states_[leg_index].params.virtual_stiffness = stiffness;
    }
}

Point3D AdmittanceController::applyForceAndIntegrate(int leg_index, const Point3D &applied_force) {
    if (leg_index < 0 || leg_index >= NUM_LEGS)
        return Point3D(0, 0, 0);

    LegAdmittanceState &state = leg_states_[leg_index];
    state.params.applied_force = applied_force;

    // Store external force for derivative calculation
    external_forces_[leg_index] = applied_force;

    // Always use derivative-based integration with math_utils functions
    Point3D position_delta = integrateDerivatives(leg_index);

    state.params.position_delta = position_delta;
    return position_delta;
}

void AdmittanceController::updateAllLegs(const Point3D forces[NUM_LEGS], Point3D position_deltas[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; i++) {
        position_deltas[i] = applyForceAndIntegrate(i, forces[i]);
    }

    // Update time for integration
    current_time_ += delta_time_;
}

void AdmittanceController::setDynamicStiffness(bool enabled, double swing_scaler, double load_scaler) {
    dynamic_stiffness_enabled_ = enabled;
    swing_stiffness_scaler_ = swing_scaler;
    load_stiffness_scaler_ = load_scaler;
}

void AdmittanceController::updateStiffness(const StepPhase leg_states[NUM_LEGS],
                                           const Point3D leg_positions[NUM_LEGS],
                                           double step_clearance) {
    if (!dynamic_stiffness_enabled_)
        return;

    step_clearance_ = step_clearance;

    // Reset all stiffness to default
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i].stiffness_scale = 1.0f;
    }

    // Apply dynamic stiffness based on leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_states[i] == SWING_PHASE) {
            // Calculate stiffness scaling for swing leg
            double scale = calculateStiffnessScale(i, leg_states[i], leg_positions[i]);
            leg_states_[i].stiffness_scale = scale * swing_stiffness_scaler_;

            // Update adjacent legs with increased stiffness
            updateAdjacentLegStiffness(i, load_stiffness_scaler_);
        }
    }

    // Apply stiffness scaling to virtual stiffness
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get base stiffness and apply scaling
        double base_stiffness = leg_states_[i].params.virtual_stiffness / leg_states_[i].stiffness_scale;
        leg_states_[i].params.virtual_stiffness = base_stiffness * leg_states_[i].stiffness_scale;
    }
}

const AdmittanceController::LegAdmittanceState &AdmittanceController::getLegState(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return leg_states_[leg_index];
    }
    return EMPTY_LEG_STATE;
}

void AdmittanceController::resetLegDynamics(int leg_index) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_states_[leg_index].params.velocity = Point3D(0, 0, 0);
        leg_states_[leg_index].params.acceleration = Point3D(0, 0, 0);
        leg_states_[leg_index].params.applied_force = Point3D(0, 0, 0);
        leg_states_[leg_index].params.position_delta = Point3D(0, 0, 0);
    }
}

void AdmittanceController::resetAllDynamics() {
    for (int i = 0; i < NUM_LEGS; i++) {
        resetLegDynamics(i);
    }
}

void AdmittanceController::setPrecisionConfig(const ComputeConfig &config) {
    config_ = config;
    delta_time_ = config_.getDeltaTime();
    selectIntegrationMethod();
}

void AdmittanceController::selectIntegrationMethod() {
    switch (config_.precision) {
    case PRECISION_LOW:
        integration_method_ = EULER_METHOD;
        break;
    case PRECISION_MEDIUM:
        integration_method_ = RUNGE_KUTTA_2;
        break;
    case PRECISION_HIGH:
        integration_method_ = RUNGE_KUTTA_4;
        break;
    }
}

void AdmittanceController::initializeDefaultParameters() {
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i].params.virtual_mass = WORKSPACE_SCALING_FACTOR;       // 500g virtual mass
        leg_states_[i].params.virtual_damping = ANGULAR_ACCELERATION_FACTOR; // Critical damping
        leg_states_[i].params.virtual_stiffness = 100.0f;                    // Medium stiffness
        leg_states_[i].active = true;
        leg_states_[i].stiffness_scale = DEFAULT_ANGULAR_SCALING;

        // Initialize state vectors for derivative-based integration
        leg_dynamics_state_[i] = math_utils::StateVector<Point3D>(Point3D(0, 0, 0), Point3D(0, 0, 0));
        external_forces_[i] = Point3D(0, 0, 0);
    }
    current_time_ = 0.0;
}

Point3D AdmittanceController::calculateAcceleration(const AdmittanceParams &params, const Point3D &position_error) {
    // Admittance equation: M*a + D*v + K*x = F
    // Solving for acceleration: a = (F - D*v - K*x) / M

    Point3D spring_force = position_error * (-params.virtual_stiffness);
    Point3D damping_force = params.velocity * (-params.virtual_damping);
    Point3D total_force = params.applied_force + spring_force + damping_force;

    return total_force * (DEFAULT_ANGULAR_SCALING / params.virtual_mass);
}

double AdmittanceController::calculateStiffnessScale(int leg_index, StepPhase leg_state,
                                                     const Point3D &leg_position) {
    if (leg_state != SWING_PHASE)
        return 1.0f;

    // Use WorkspaceValidator for default position calculation
    if (!workspace_validator_) {
        return 1.0f; // Safety fallback
    }

    // Get workspace bounds for more accurate default position
    auto bounds = workspace_validator_->getWorkspaceBounds(leg_index);
    Point3D default_pos = bounds.center_position; // Use workspace center as reference

    double z_diff = abs(leg_position.z - default_pos.z);

    // Scale based on step clearance using workspace height range
    double workspace_height_range = bounds.max_height - bounds.min_height;
    double normalized_clearance = std::max(step_clearance_, workspace_height_range * 0.1f); // Min 10% of workspace

    double step_reference = z_diff / normalized_clearance;
    step_reference = math_utils::clamp<double>(step_reference, 0.0, 1.0);

    return step_reference;
}

void AdmittanceController::updateAdjacentLegStiffness(int swing_leg_index, double load_scaling) {
    int adjacent1 = (swing_leg_index + 1) % NUM_LEGS;
    int adjacent2 = (swing_leg_index + NUM_LEGS - 1) % NUM_LEGS;

    // Add load stiffness to adjacent legs (additive as per OpenSHC)
    leg_states_[adjacent1].stiffness_scale += (load_scaling - 1.0f);
    leg_states_[adjacent2].stiffness_scale += (load_scaling - 1.0f);
}

// Legacy compatibility methods
Point3D AdmittanceController::orientationError(const Point3D &target) {
    if (!imu_)
        return Point3D(0, 0, 0);

    IMUData imu_data = imu_->readIMU();
    Point3D current(imu_data.roll, imu_data.pitch, imu_data.yaw);
    return target - current;
}

bool AdmittanceController::maintainOrientation(const Point3D &target, Point3D &current, double dt) {
    if (!imu_)
        return false;

    Point3D err = orientationError(target);
    current = current + err * (dt * WORKSPACE_SCALING_FACTOR);
    return true;
}

bool AdmittanceController::checkStability(const Point3D leg_pos[NUM_LEGS], const StepPhase leg_states[NUM_LEGS]) {
    if (!fsr_)
        return true;

    int contacts = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (leg_states[i] == STANCE_PHASE) {
            FSRData fsr_data = fsr_->readFSR(i);
            if (fsr_data.in_contact)
                contacts++;
        }
    }
    return contacts >= 3;
}

Point3D AdmittanceController::integrateDerivatives(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return Point3D(0, 0, 0);
    }

    LegAdmittanceState &state = leg_states_[leg_index];

    // For derivative-based integration, we work with position error from equilibrium
    // In a full implementation, this would get current leg position from locomotion system
    // For now, we'll use the tracked state position error
    Point3D position_error = leg_dynamics_state_[leg_index].position;

    // Setup parameters for derivative function
    AdmittanceDerivativeParams params;
    params.mass = state.params.virtual_mass;
    params.damping = state.params.virtual_damping;
    params.stiffness = state.params.virtual_stiffness * state.stiffness_scale;
    params.external_force = external_forces_[leg_index];
    params.equilibrium = state.equilibrium_position;

    // Choose integration method based on precision configuration using math_utils functions
    math_utils::StateVector<Point3D> new_state;

    switch (config_.precision) {
    case PRECISION_HIGH:
        // Use RK4 for maximum accuracy
        new_state = math_utils::rungeKutta4<Point3D>(
            admittanceDerivatives,
            leg_dynamics_state_[leg_index],
            current_time_,
            delta_time_,
            &params);
        break;

    case PRECISION_MEDIUM:
        // Use RK2 for balanced performance
        new_state = math_utils::rungeKutta2<Point3D>(
            admittanceDerivatives,
            leg_dynamics_state_[leg_index],
            current_time_,
            delta_time_,
            &params);
        break;

    case PRECISION_LOW:
    default:
        // Use Euler for fastest computation
        new_state = math_utils::forwardEuler<Point3D>(
            admittanceDerivatives,
            leg_dynamics_state_[leg_index],
            current_time_,
            delta_time_,
            &params);
        break;
    }

    // Update state
    leg_dynamics_state_[leg_index] = new_state;

    // Update legacy params for compatibility
    state.params.velocity = new_state.velocity;

    // Calculate position delta from velocity
    Point3D position_delta = new_state.velocity * delta_time_;

    // For validation and comparison, also calculate using direct acceleration method
    if (config_.precision == PRECISION_HIGH) {
        // Use calculateAcceleration for validation against derivative method
        Point3D direct_acceleration = calculateAcceleration(state.params, position_error);

        // The results should be mathematically equivalent
        // This can be used for debugging or switching integration methods
        (void)direct_acceleration; // Suppress unused variable warning for now
    }

    return position_delta;
}

math_utils::StateVector<Point3D> AdmittanceController::admittanceDerivatives(
    const math_utils::StateVector<Point3D> &state,
    double t,
    void *params) {

    // Cast parameters
    AdmittanceDerivativeParams *admittance_params =
        static_cast<AdmittanceDerivativeParams *>(params);

    if (!admittance_params) {
        return math_utils::StateVector<Point3D>(Point3D(0, 0, 0), Point3D(0, 0, 0));
    }

    // Extract state variables
    Point3D position = state.position; // x = displacement from equilibrium
    Point3D velocity = state.velocity; // ẋ = velocity

    // Admittance differential equation: M*ẍ + B*ẋ + K*x = F_ext
    // Solving for acceleration: ẍ = (F_ext - B*ẋ - K*x) / M

    Point3D spring_force = position * (-admittance_params->stiffness);
    Point3D damping_force = velocity * (-admittance_params->damping);
    Point3D net_force = admittance_params->external_force + spring_force + damping_force;

    Point3D acceleration = net_force * (DEFAULT_ANGULAR_SCALING / admittance_params->mass);

    // Return derivatives: [dx/dt, dv/dt] = [velocity, acceleration]
    return math_utils::StateVector<Point3D>(velocity, acceleration);
}
