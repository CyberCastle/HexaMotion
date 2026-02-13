#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "robot_model.h"

/** Forward declarations. */
class Leg;
class WalkController;

/**
 * @brief Admittance controller (OpenSHC 1:1 equivalent).
 *
 * Models each leg as a mass-spring-damper system to compute tip position offsets.
 * Input:  tip forces scaled by force_gain, positive-clamped per axis.
 * Output: per-leg admittance_delta stored on the Leg object.
 *
 * Virtual damping is derived from the damping ratio:
 *   virtual_damping = damping_ratio * 2 * sqrt(mass * stiffness)
 *
 * Integration uses RK4 with 30 sub-steps over integrator_step_time (default 0.5 s).
 * Output per axis = -state[0] clamped to ±0.2, with optional deadbanding.
 *
 * Also manages dynamic stiffness during walk cycles and leg state transitions.
 */
class AdmittanceController {
  public:
    /**
     * @brief Construct admittance controller.
     * @param params Robot parameters (includes AdmittanceConfig)
     */
    explicit AdmittanceController(const Parameters &params);

    /**
     * @brief Integrate admittance ODE for all legs and set admittance deltas.
     *
     * For each leg reads tip force (calculated from joint effort or measured via FSR),
     * scales by force_gain, positive-clamps per axis, integrates the mass-spring-damper
     * ODE with RK4 (30 sub-steps over integrator_step_time), and stores the clamped
     * negated displacement as admittance_delta on the Leg.
     *
     * OpenSHC equivalent: AdmittanceController::updateAdmittance()
     *
     * @param legs Leg array
     * @param fsr  FSR interface for measured tip force (may be nullptr)
     */
    void updateAdmittance(Leg legs[NUM_LEGS], IFSRInterface *fsr);

    /**
     * @brief Update dynamic stiffness for all legs based on walk cycle.
     *
     * Resets all legs to default virtual_stiffness, then for each swing leg
     * scales stiffness down and adds load stiffness to adjacent legs (additive).
     *
     * OpenSHC equivalent: AdmittanceController::updateStiffness(walker)
     *
     * @param legs   Leg array
     * @param walker Walk controller for step state and clearance
     */
    void updateStiffness(Leg legs[NUM_LEGS], WalkController *walker);

    /**
     * @brief Update stiffness for a single leg and its adjacents during state transition.
     *
     * Used during WALKING_TO_MANUAL (scale 0->1) and MANUAL_TO_WALKING (scale 1->0).
     *
     * OpenSHC equivalent: AdmittanceController::updateStiffness(leg, scale_reference)
     *
     * @param legs            Leg array
     * @param leg_index       Index of the transitioning leg
     * @param scale_reference Transition progress (0.0 to 1.0)
     */
    void updateStiffness(Leg legs[NUM_LEGS], int leg_index, double scale_reference);

  private:
    const Parameters &params_;

    /**
     * @brief Single RK4 step for the mass-spring-damper ODE.
     *
     * ODE: m*x'' + c*x' + k*x = -F
     *   dxdt[0] = x[1]
     *   dxdt[1] = -F/m - c/m*x[1] - k/m*x[0]
     *
     * @param state     In/out [position, velocity]
     * @param force     Positive-clamped scaled force for this axis
     * @param mass      Virtual mass
     * @param damping   Virtual damping (= ratio * 2 * sqrt(m*k))
     * @param stiffness Virtual stiffness (per-leg, possibly dynamically scaled)
     * @param dt        Integration sub-step size
     */
    static void rk4Step(double state[2], double force, double mass,
                        double damping, double stiffness, double dt);
};

#endif /**< ADMITTANCE_CONTROLLER_H */
