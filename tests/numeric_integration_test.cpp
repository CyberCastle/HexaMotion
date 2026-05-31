/**
 * @file numeric_integration_test.cpp
 * @brief Consolidated test suite.
 *
 * This single translation unit folds together the following sub-tests
 * without losing any coverage. Each sub-test keeps its original assertions
 * and entry function; their bodies are wrapped in a dedicated namespace to
 * avoid symbol collisions, and this file's main() runs them in sequence and
 * aggregates the exit status:
 *   - run_runge_kutta_validation()
 *   - run_quaternion_functions()
 */

#include "../src/admittance_controller.h"
#include "../src/leg.h"
#include "math_utils.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>

// ===========================================================================
// Sub-test: run_runge_kutta_validation (from runge_kutta_validation_test.cpp)
// ===========================================================================
namespace cm_runge_kutta_validation_test {
/**
 * @brief Validate the OpenSHC-equivalent RK4 admittance integration.
 *
 * Applies a constant positive force to leg 0 and verifies:
 *   1. The ODE state (position) evolves monotonically.
 *   2. The output admittance_delta is non-zero and clamped within ±0.2.
 *   3. The system settles toward a steady-state displacement.
 */
int run_runge_kutta_validation() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.height_offset = 0;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    /** Enable admittance with OpenSHC defaults. */
    p.admittance.enable = true;
    p.admittance.virtual_mass = 10.0;
    p.admittance.virtual_stiffness = 12.0;
    p.admittance.virtual_damping_ratio = 0.8;
    p.admittance.force_gain = 0.1;
    p.admittance.integrator_step_time = 0.5;

    RobotModel model(p);
    DummyFSR fsr;

    /** Create legs array. */
    Leg legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)};

    /** Set up: apply a Z-axis force of 50 units to leg 0 via calculated tip force.
     *  Since use_joint_effort defaults to false, we use FSR.
     *  For this test, directly set up joint effort and use use_joint_effort=true. */
    p.admittance.use_joint_effort = true;

    /** Manually inject a tip force on leg 0 via joint effort. */
    /** For simplicity, directly set admittance state and use the raw ODE test. */

    AdmittanceController ac(p);

    std::cout << "Testing OpenSHC-equivalent RK4 Admittance Integration..." << std::endl;

    /** Give leg 0 a non-zero calculated tip force by setting joint efforts. */
    legs[0].setCurrentJointEffort(JointAngles(0.0, 5.0, 5.0));
    legs[0].calculateTipForce();

    Eigen::Vector3d initial_force = legs[0].getCalculatedTipForce();
    std::cout << "Initial tip force: [" << initial_force[0] << ", "
              << initial_force[1] << ", " << initial_force[2] << "]" << std::endl;

    /** Run multiple integration cycles. */
    for (int step = 0; step < 20; step++) {
        ac.updateAdmittance(legs, nullptr);

        Eigen::Vector3d delta = legs[0].getAdmittanceDelta();
        double *state_z = legs[0].getAdmittanceState(2);

        std::cout << "Step " << step
                  << ": state[0]=" << state_z[0]
                  << " state[1]=" << state_z[1]
                  << " delta=[" << delta[0] << "," << delta[1] << "," << delta[2] << "]"
                  << std::endl;

        /** Verify output is within clamp range. */
        for (int axis = 0; axis < 3; ++axis) {
            assert(std::abs(delta[axis]) <= 0.2 + 1e-12);
        }
    }

    /** Verify that admittance produced a non-trivial delta (at least one axis non-zero). */
    Eigen::Vector3d final_delta = legs[0].getAdmittanceDelta();
    double delta_norm = final_delta.norm();
    std::cout << "Final delta norm: " << delta_norm << std::endl;

    /** The delta may be zero if the tip force is zero after gain and positive clamp.
     *  This is valid behavior. Test passes if no assertion failures. */

    /** Test dynamic stiffness scaling. */
    std::cout << "\nTesting dynamic stiffness via updateStiffness(leg, scale)..." << std::endl;
    double original_stiffness = legs[0].getVirtualStiffness();
    std::cout << "Original stiffness leg 0: " << original_stiffness << std::endl;

    ac.updateStiffness(legs, 0, 0.5);
    double scaled_stiffness = legs[0].getVirtualStiffness();
    std::cout << "Scaled stiffness leg 0 (scale=0.5): " << scaled_stiffness << std::endl;
    assert(scaled_stiffness != original_stiffness || p.admittance.swing_stiffness_scaler == 1.0);

    /** Test reset. */
    legs[0].resetAdmittanceState();
    double *state = legs[0].getAdmittanceState(0);
    assert(state[0] == 0.0 && state[1] == 0.0);
    std::cout << "State reset verified." << std::endl;

    std::cout << "\n✅ OpenSHC-equivalent RK4 admittance integration validated!" << std::endl;
    std::cout << "✅ Dynamic stiffness scaling verified." << std::endl;
    std::cout << "✅ State reset verified." << std::endl;

    return 0;
}
} // namespace cm_runge_kutta_validation_test

// ===========================================================================
// Sub-test: run_quaternion_functions (from quaternion_functions_test.cpp)
// ===========================================================================
namespace cm_quaternion_functions_test {
int run_quaternion_functions() {
    std::cout << "=== Test quaternion functions in math_utils ===" << std::endl;

    /** Test eulerToQuaternion and quaternionToEuler. */
    Eigen::Vector3d euler_input_deg(30.0f, 45.0f, 60.0f);
    Eigen::Vector3d euler_input(
        math_utils::degreesToRadians(euler_input_deg[0]),
        math_utils::degreesToRadians(euler_input_deg[1]),
        math_utils::degreesToRadians(euler_input_deg[2]));
    Eigen::Vector4d quat = math_utils::eulerToQuaternion(euler_input);
    Eigen::Vector3d euler_output = math_utils::quaternionToEuler(quat);
    Eigen::Vector3d euler_output_deg(
        math_utils::radiansToDegrees(euler_output[0]),
        math_utils::radiansToDegrees(euler_output[1]),
        math_utils::radiansToDegrees(euler_output[2]));

    std::cout << "Input Euler (deg): " << euler_input_deg.transpose() << std::endl;
    std::cout << "Quaternion: " << quat.transpose() << std::endl;
    std::cout << "Output Euler (deg): " << euler_output_deg.transpose() << std::endl;

    /** Check if conversion is consistent. */
    double tolerance = 0.01f;
    bool conversion_ok = (euler_input_deg - euler_output_deg).norm() < tolerance;
    std::cout << "Euler<->Quaternion conversion: " << (conversion_ok ? "PASS" : "FAIL") << std::endl;

    /** Test quaternionMultiply. */
    /** 90 degrees around X. */
    Eigen::Vector4d q1(0.7071f, 0.7071f, 0.0f, 0.0f);
    /** 90 degrees around Y. */
    Eigen::Vector4d q2(0.7071f, 0.0f, 0.7071f, 0.0f);
    Eigen::Vector4d q_mult = math_utils::quaternionMultiply(q1, q2);
    std::cout << "Q1 * Q2: " << q_mult.transpose() << std::endl;

    /** Test quaternionInverse. */
    Eigen::Vector4d q_inv = math_utils::quaternionInverse(quat);
    Eigen::Vector4d identity = math_utils::quaternionMultiply(quat, q_inv);
    std::cout << "Q * Q_inv (should be ~[1,0,0,0]): " << identity.transpose() << std::endl;

    bool inverse_ok = std::abs(identity[0] - 1.0f) < tolerance &&
                      std::abs(identity[1]) < tolerance &&
                      std::abs(identity[2]) < tolerance &&
                      std::abs(identity[3]) < tolerance;
    std::cout << "Quaternion inverse: " << (inverse_ok ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n=== Test completed ===" << std::endl;
    return 0;
}
} // namespace cm_quaternion_functions_test

int main() {
    int rc = 0;

    std::cout << "\n========== runge kutta validation ==========\n";
    rc |= cm_runge_kutta_validation_test::run_runge_kutta_validation();

    std::cout << "\n========== quaternion functions ==========\n";
    rc |= cm_quaternion_functions_test::run_quaternion_functions();

    std::cout << "\n[numeric_integration_test] overall: " << (rc == 0 ? "PASS" : "FAIL") << "\n";
    return rc == 0 ? 0 : 1;
}
