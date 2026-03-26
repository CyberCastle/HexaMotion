#include "admittance_controller.h"
#include "leg.h"
#include "leg_stepper.h"
#include "walk_controller.h"
#include <algorithm>
#include <cmath>

/**
 * @file admittance_controller.cpp
 * @brief OpenSHC 1:1 admittance controller implementation.
 *
 * Each leg is modelled as a mass-spring-damper system:
 *   m*x'' + c*x' + k*x = -F
 * where c = damping_ratio * 2 * sqrt(m * k).
 *
 * Integration uses RK4 with 30 sub-steps per integrator_step_time (default 0.5 s).
 * Output per axis = -state[0] clamped ±0.2, with optional deadbanding.
 */

/** Deadband threshold (OpenSHC default: 0.0 — effectively disabled). */
static constexpr double ADMITTANCE_DEADBAND = 0.0;
/** Number of RK4 sub-steps per integration call (OpenSHC: step_time / 30). */
static constexpr int INTEGRATION_SUBSTEPS = 30;
/** Output clamp magnitude (OpenSHC: clamped(-state[0], -0.2, 0.2)). */
static constexpr double ADMITTANCE_CLAMP = 0.2;

AdmittanceController::AdmittanceController(const Parameters &params)
    : params_(params) {
}

void AdmittanceController::updateAdmittance(Leg legs[NUM_LEGS], IFSRInterface *fsr) {
    const auto &ac = params_.admittance;

    for (int leg_idx = 0; leg_idx < NUM_LEGS; ++leg_idx) {
        Leg &leg = legs[leg_idx];
        Eigen::Vector3d admittance_delta = Eigen::Vector3d::Zero();

        /** Get tip force: calculated from joint effort, or measured from FSR. */
        Eigen::Vector3d tip_force;
        if (ac.use_joint_effort) {
            tip_force = leg.getCalculatedTipForce();
        } else {
            /** Measured tip force from FSR (Z-axis only for single-axis sensors). */
            if (fsr) {
                FSRData fsr_data = fsr->readFSR(leg_idx);
                tip_force = Eigen::Vector3d(0.0, 0.0, fsr_data.pressure);
            } else {
                tip_force = Eigen::Vector3d::Zero();
            }
        }

        /** Scale by force_gain (OpenSHC: tip_force *= params_.force_gain.current_value). */
        tip_force *= ac.force_gain;

        /** Per-axis integration (OpenSHC: for i in 0..2). */
        for (int axis = 0; axis < 3; ++axis) {
            /** Positive-clamp per axis (OpenSHC: std::max(tip_force[i], 0.0)). */
            double force_input = std::max(tip_force[axis], 0.0);

            double mass = ac.virtual_mass;
            double stiffness = leg.getVirtualStiffness();
            /** Derive damping from ratio (OpenSHC: damping * 2 * sqrt(mass * stiffness)). */
            double virtual_damping = ac.virtual_damping_ratio * 2.0 * std::sqrt(mass * stiffness);
            double step_time = ac.integrator_step_time;
            double sub_dt = step_time / INTEGRATION_SUBSTEPS;

            /** Persistent per-axis ODE state [position, velocity]. */
            double *state = leg.getAdmittanceState(axis);

            /** RK4 integration: 30 sub-steps over step_time. */
            for (int s = 0; s < INTEGRATION_SUBSTEPS; ++s) {
                rk4Step(state, force_input, mass, virtual_damping, stiffness, sub_dt);
            }

            /** Output: negate position, clamp ±0.2 (OpenSHC: clamped(-state[0], -0.2, 0.2)). */
            double delta = std::max(-ADMITTANCE_CLAMP, std::min(-state[0], ADMITTANCE_CLAMP));

            /** Deadbanding (OpenSHC: ADMITTANCE_DEADBAND = 0.0, effectively disabled). */
            if (ADMITTANCE_DEADBAND > 0.0 && std::abs(delta) > ADMITTANCE_DEADBAND) {
                double sign = (delta > 0.0) ? 1.0 : -1.0;
                admittance_delta[axis] = sign * (std::abs(delta) - ADMITTANCE_DEADBAND) /
                                         (1.0 - ADMITTANCE_DEADBAND);
            } else if (ADMITTANCE_DEADBAND <= 0.0) {
                /** Deadband disabled; pass through directly. */
                admittance_delta[axis] = delta;
            }
            /** else: within deadband, stays 0. */
        }

        /**
         * OpenSHC projects admittance_delta onto the tip X-axis via:
         *   admittance_delta_ = getProjection(delta, tip_rotation * UnitX)
         * For 3DOF legs without explicit tip rotation tracking, the delta is
         * stored directly (HexaMotion architectural simplification; functional
         * outcome is equivalent for planar compliance).
         */
        leg.setAdmittanceDelta(admittance_delta);
    }
}

void AdmittanceController::updateStiffness(Leg legs[NUM_LEGS], WalkController *walker) {
    const auto &ac = params_.admittance;

    /** Reset all legs to default virtual stiffness each cycle (OpenSHC pattern). */
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].setVirtualStiffness(ac.virtual_stiffness);
    }

    /** Calculate dynamic virtual stiffness for swing legs. */
    for (int i = 0; i < NUM_LEGS; ++i) {
        auto leg_stepper = walker->getLegStepper(i);
        if (!leg_stepper)
            continue;

        if (leg_stepper->getStepState() == STEP_SWING) {
            /** Z difference between current swing position and default position. */
            double z_diff = leg_stepper->getCurrentTipPose().z - leg_stepper->getDefaultTipPose().z;
            double step_reference = std::abs(z_diff / walker->getStepClearance());

            int adjacent_1 = ((i - 1) + NUM_LEGS) % NUM_LEGS;
            int adjacent_2 = (i + 1) % NUM_LEGS;

            double virtual_stiffness = ac.virtual_stiffness;
            /** Swing leg: stiffness decreases (scaler < 1 => stiffness drops). */
            double swing_stiffness = virtual_stiffness *
                                     (step_reference * (ac.swing_stiffness_scaler - 1.0) + 1.0);
            /** Load stiffness delta for adjacent legs (additive). */
            double load_stiffness = virtual_stiffness *
                                    (step_reference * (ac.load_stiffness_scaler - 1.0));

            double current_s1 = legs[adjacent_1].getVirtualStiffness();
            double current_s2 = legs[adjacent_2].getVirtualStiffness();

            legs[i].setVirtualStiffness(swing_stiffness);
            legs[adjacent_1].setVirtualStiffness(current_s1 + load_stiffness);
            legs[adjacent_2].setVirtualStiffness(current_s2 + load_stiffness);
        }
    }
}

void AdmittanceController::updateStiffness(Leg legs[NUM_LEGS], int leg_index,
                                           double scale_reference) {
    const auto &ac = params_.admittance;

    int adjacent_1 = ((leg_index - 1) + NUM_LEGS) % NUM_LEGS;
    int adjacent_2 = (leg_index + 1) % NUM_LEGS;

    double virtual_stiffness = ac.virtual_stiffness;
    double swing_stiffness = virtual_stiffness *
                             (scale_reference * (ac.swing_stiffness_scaler - 1.0) + 1.0);
    double load_stiffness = virtual_stiffness *
                            (scale_reference * (ac.load_stiffness_scaler - 1.0) + 1.0);

    legs[leg_index].setVirtualStiffness(swing_stiffness);

    /** Only modify adjacent legs that are not in MANUAL state (OpenSHC check). */
    if (legs[adjacent_1].getLegState() != LEG_MANUAL) {
        legs[adjacent_1].setVirtualStiffness(load_stiffness);
    }
    if (legs[adjacent_2].getLegState() != LEG_MANUAL) {
        legs[adjacent_2].setVirtualStiffness(load_stiffness);
    }
}

void AdmittanceController::rk4Step(double state[2], double force, double mass,
                                   double damping, double stiffness, double dt) {
    /**
     * ODE (OpenSHC):
     *   dxdt[0] = x[1]
     *   dxdt[1] = -force/mass - damping/mass * x[1] - stiffness/mass * x[0]
     */
    auto deriv = [force, mass, damping, stiffness](const double s[2], double d[2]) {
        d[0] = s[1];
        d[1] = -force / mass - damping / mass * s[1] - stiffness / mass * s[0];
    };

    double k1[2], k2[2], k3[2], k4[2];
    double tmp[2];

    /** k1 = f(state). */
    deriv(state, k1);

    /** k2 = f(state + 0.5*dt*k1). */
    tmp[0] = state[0] + 0.5 * dt * k1[0];
    tmp[1] = state[1] + 0.5 * dt * k1[1];
    deriv(tmp, k2);

    /** k3 = f(state + 0.5*dt*k2). */
    tmp[0] = state[0] + 0.5 * dt * k2[0];
    tmp[1] = state[1] + 0.5 * dt * k2[1];
    deriv(tmp, k3);

    /** k4 = f(state + dt*k3). */
    tmp[0] = state[0] + dt * k3[0];
    tmp[1] = state[1] + dt * k3[1];
    deriv(tmp, k4);

    /** Update state: state += dt/6 * (k1 + 2*k2 + 2*k3 + k4). */
    state[0] += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
    state[1] += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
}
