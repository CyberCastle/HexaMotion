#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "robot_model.h"
#include "precision_config.h"
#include <memory>
#include <vector>

// Forward declaration to avoid circular dependency
class WorkspaceValidator;

/**
 * @brief Enhanced admittance controller with ODE integration
 * Equivalent to OpenSHC's admittance control theory implementation
 *
 * Features configurable precision vs computational complexity:
 * - Euler integration (fast, basic)
 * - RK2 integration (balanced)
 * - RK4 integration (accurate, slow)
 * - Dynamic stiffness adjustment
 * - Multi-leg coordination
 */
class AdmittanceController {
  public:
    /**
     * @brief Admittance parameters for virtual leg model
     */
    struct AdmittanceParams {
        double virtual_mass;      //< Virtual mass (kg)
        double virtual_damping;   //< Damping coefficient
        double virtual_stiffness; //< Spring stiffness
        Point3D velocity;        //< Current velocity
        Point3D acceleration;    //< Current acceleration
        Point3D applied_force;   //< Applied force
        Point3D position_delta;  //< Position change from admittance

        AdmittanceParams() : virtual_mass(0.5f), virtual_damping(2.0f),
                             virtual_stiffness(100.0f), velocity(0, 0, 0),
                             acceleration(0, 0, 0), applied_force(0, 0, 0), position_delta(0, 0, 0) {}
    };

    /**
     * @brief Parameters for derivative function in admittance equation
     */
    struct AdmittanceDerivativeParams {
        double mass;             //< Virtual mass
        double damping;          //< Damping coefficient
        double stiffness;        //< Spring stiffness
        Point3D external_force; //< External force applied
        Point3D equilibrium;    //< Equilibrium position

        AdmittanceDerivativeParams() : mass(0.5f), damping(2.0f), stiffness(100.0f),
                                       external_force(0, 0, 0), equilibrium(0, 0, 0) {}
    };

    /**
     * @brief ODE integration methods
     */
    enum IntegrationMethod {
        EULER_METHOD,  //< First-order Euler (fastest)
        RUNGE_KUTTA_2, //< Second-order RK (balanced)
        RUNGE_KUTTA_4  //< Fourth-order RK (most accurate)
    };

    /**
     * @brief Leg admittance state
     */
    struct LegAdmittanceState {
        AdmittanceParams params;
        Point3D equilibrium_position;
        bool active;
        double stiffness_scale; //< Dynamic stiffness scaling

        LegAdmittanceState() : equilibrium_position(0, 0, 0), active(true), stiffness_scale(1.0f) {}
    };

  private:
    RobotModel &model_;
    std::unique_ptr<WorkspaceValidator> workspace_validator_; // Workspace validation
    IIMUInterface *imu_;
    IFSRInterface *fsr_;
    ComputeConfig config_;
    IntegrationMethod integration_method_;

    // Per-leg admittance state
    LegAdmittanceState leg_states_[NUM_LEGS];
    double delta_time_;

    // Dynamic stiffness parameters
    bool dynamic_stiffness_enabled_;
    double swing_stiffness_scaler_;
    double load_stiffness_scaler_;
    double step_clearance_;

  public:
    /**
     * @brief Construct admittance controller
     * @param model Reference to robot model
     * @param imu IMU interface pointer
     * @param fsr FSR interface pointer
     * @param config Computational configuration
     */
    AdmittanceController(RobotModel &model, IIMUInterface *imu, IFSRInterface *fsr,
                         ComputeConfig config = ComputeConfig::medium());
    ~AdmittanceController(); // Needed for unique_ptr with forward declaration

    /**
     * @brief Initialize admittance controller
     */
    void initialize();

    /**
     * @brief Set admittance parameters for specific leg
     * @param leg_index Leg index (0-5)
     * @param mass Virtual mass
     * @param damping Damping coefficient
     * @param stiffness Spring stiffness
     */
    void setLegAdmittance(int leg_index, double mass, double damping, double stiffness);

    /**
     * @brief Apply force to leg and integrate dynamics
     * @param leg_index Leg index (0-5)
     * @param applied_force Force vector
     * @return Position delta from admittance response
     */
    Point3D applyForceAndIntegrate(int leg_index, const Point3D &applied_force);

    /**
     * @brief Update all legs simultaneously
     * @param forces Applied forces for each leg
     * @param position_deltas Output position deltas
     */
    void updateAllLegs(const Point3D forces[NUM_LEGS], Point3D position_deltas[NUM_LEGS]);

    /**
     * @brief Enable/disable dynamic stiffness adjustment
     * @param enabled Whether to enable dynamic stiffness
     * @param swing_scaler Stiffness scaling for swing legs
     * @param load_scaler Stiffness scaling for loaded legs
     */
    void setDynamicStiffness(bool enabled, double swing_scaler = 0.5f, double load_scaler = 1.5f);

    /**
     * @brief Update dynamic stiffness based on gait phase
     * @param leg_states Current leg states
     * @param leg_positions Current leg positions
     * @param step_clearance Current step clearance
     */
    void updateStiffness(const StepPhase leg_states[NUM_LEGS],
                         const Point3D leg_positions[NUM_LEGS],
                         double step_clearance);

    /**
     * @brief Get current leg admittance state
     * @param leg_index Leg index (0-5)
     * @return Leg admittance state
     */
    const LegAdmittanceState &getLegState(int leg_index) const;

    /**
     * @brief Reset leg dynamics
     * @param leg_index Leg index (0-5)
     */
    void resetLegDynamics(int leg_index);

    /**
     * @brief Reset all leg dynamics
     */
    void resetAllDynamics();

    /**
     * @brief Set computational precision configuration
     * @param config New configuration
     */
    void setPrecisionConfig(const ComputeConfig &config);

    // Legacy compatibility methods
    /**
     * @brief Compute orientation error with respect to a target pose.
     * @param target Desired roll, pitch and yaw in degrees.
     * @return Difference between target and current orientation.
     */
    Point3D orientationError(const Point3D &target);

    /**
     * @brief Maintain body orientation using a simple admittance filter.
     * @param target Desired orientation in degrees.
     * @param current Current orientation, updated in place.
     * @param dt      Time step in seconds.
     * @return True if the operation succeeded.
     */
    bool maintainOrientation(const Point3D &target, Point3D &current, double dt);

    /**
     * @brief Check static stability from FSR readings.
     * @param leg_pos  Current foot positions.
     * @param leg_states Leg state for each leg.
     * @return True if the robot is considered stable.
     */
    bool checkStability(const Point3D leg_pos[NUM_LEGS], const StepPhase leg_states[NUM_LEGS]);

  private:
    void selectIntegrationMethod();
    void initializeDefaultParameters();

    // Admittance equation calculation
    Point3D calculateAcceleration(const AdmittanceParams &params, const Point3D &position_error);

    // Derivative-based integration using math_utils functions
    Point3D integrateDerivatives(int leg_index);

    // Derivative function for admittance system
    static math_utils::StateVector<Point3D> admittanceDerivatives(
        const math_utils::StateVector<Point3D> &state,
        double t,
        void *params);

    // Current state tracking for derivative integration
    math_utils::StateVector<Point3D> leg_dynamics_state_[NUM_LEGS];
    Point3D external_forces_[NUM_LEGS];
    double current_time_;

    // Dynamic stiffness calculation
    double calculateStiffnessScale(int leg_index, StepPhase leg_state,
                                  const Point3D &leg_position);
    void updateAdjacentLegStiffness(int swing_leg_index, double load_scaling);
};

#endif // ADMITTANCE_CONTROLLER_H
