/**
 * @file tripod_linearity_test.cpp
 * @brief Validates forward linearity of the tripod gait (minimal lateral drift).
 *
 * Rationale:
 * A balanced tripod gait should produce near-zero cumulative lateral (Y) drift of the
 * stance support centroid and near-zero mean global Y across all leg tip samples when
 * commanded to walk strictly forward (no angular or lateral velocity).
 *
 * Metrics collected (gait symmetry + body trajectory):
 *  Gait-level:
 *    - overall_mean_tip_y: Mean of all leg tip Y coordinates across the sampled window.
 *    - max_abs_stance_centroid_y: Max absolute deviation of stance-centroid Y from its initial value.
 *    - cycle_mean_centroid_y: Mean stance-centroid Y per cycle (informational).
 *  Body-level (added):
 *    - effective_forward_progress_x: surrogate forward progress derived from stance centroid X drift (body may be in a fixed world frame in simulation)
 *    - body_mean_lateral_y: Mean |body_position.y - initial_body_y| across samples.
 *    - body_max_lateral_y: Max absolute lateral deviation.
 *    - body_yaw_drift: |final_yaw - initial_yaw|
 *
 * Pass criteria (tolerances chosen conservatively; tighten after hardware validation):
 *  Gait symmetry:
 *    - |overall_mean_tip_y| <= 2.0 mm
 *    - max_abs_stance_centroid_y <= 5.0 mm
 *  Body rectilinearity:
 *    - body_max_lateral_y <= 3.0 mm
 *    - body_yaw_drift <= 1.0 deg
 *    - effective_forward_progress_x >= 1.0 mm (sanity: motion occurred)
 * These thresholds can be tightened once hardware validation confirms stability margins.
 *
 * Test flow:
 *  1. Initialize LocomotionSystem with default parameters.
 *  2. Set tripod gait; command a forward velocity (walkForward) with zero lateral/rotational components.
 *  3. Run startup sequence to enter RUNNING state.
 *  4. Execute N full gait cycles (default 3) collecting metrics every update.
 *  5. Compute metrics; print a summary; return 0 on success, 1 on failure.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h" // For BASE_THETA_OFFSETS (real leg base angles)
#include "../src/locomotion_system.h"    // Use header; implementation linked via object list in Makefile
#include "robot_model.h"
#include "test_stubs.h"
#include <array>
#include <bitset>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

// Forward test configuration
static constexpr double FORWARD_VELOCITY_MM_S = 100.0;   // Same magnitude as visualization test
static constexpr int NUM_CYCLES_TO_SAMPLE = 500;         // Long-run evaluation window
static constexpr double MEAN_Y_TOLERANCE = 2.0;          // mm
static constexpr double STANCE_CENTROID_DRIFT_TOL = 5.0; // mm
static constexpr double BODY_MAX_LATERAL_TOL = 3.0;      // mm
static constexpr double BODY_YAW_DRIFT_TOL_DEG = 1.0;    // deg

// Eliminated estructura detallada de ciclo para reducir verbosidad

int main() {
    Parameters params = createDefaultParameters();
    params.enable_body_translation = true;  // Activate body pose integration for forward motion validation
    params.preserve_swing_end_pose = false; // Force anti-drift reset of stance origin to evaluate lateral drift mitigation
    LocomotionSystem sys(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Initialization failed" << std::endl;
        return 1;
    }
    if (!sys.setStandingPose()) {
        std::cerr << "ERROR: Standing pose failed" << std::endl;
        return 1;
    }
    if (!sys.setGaitType(TRIPOD_GAIT)) {
        std::cerr << "ERROR: Failed to set tripod gait" << std::endl;
        return 1;
    }
    sys.walkForward(FORWARD_VELOCITY_MM_S); // Only forward component
    if (!sys.startWalking()) {
        std::cerr << "ERROR: startWalking failed" << std::endl;
        return 1;
    }

    // Run startup sequence until system enters RUNNING state (flag now auto-clears internally)
    int startup_loops = 0;
    const int STARTUP_MAX_LOOPS = 600; // Allow ample iterations
    while (startup_loops < STARTUP_MAX_LOOPS && sys.getSystemState() != SYSTEM_RUNNING) {
        sys.executeStartupSequence();
        sys.update();
        startup_loops++;
    }
    if (sys.getSystemState() != SYSTEM_RUNNING) {
        std::cerr << "ERROR: Startup sequence did not reach RUNNING state (loops=" << startup_loops << ")" << std::endl;
        return 1;
    }

    // Obtain step cycle parameters from first leg stepper
    auto leg_stepper0 = sys.getWalkController()->getLegStepper(0);
    if (!leg_stepper0) {
        std::cerr << "ERROR: Unable to access leg stepper" << std::endl;
        return 1;
    }
    StepCycle sc = leg_stepper0->getStepCycle();

    // Derive iteration counts exactly like other tests
    double time_delta = params.time_delta; // unified global timestep
    int swing_iterations = int((double(sc.swing_period_) / sc.period_) / (sc.frequency_ * time_delta));
    if (swing_iterations % 2 != 0)
        swing_iterations++;
    int stance_iterations = int((double(sc.stance_period_) / sc.period_) / (sc.frequency_ * time_delta));
    int cycle_iterations = swing_iterations + stance_iterations;

    const int total_iterations_target = NUM_CYCLES_TO_SAMPLE * cycle_iterations;

    std::vector<double> all_tip_y_samples;
    all_tip_y_samples.reserve(total_iterations_target * NUM_LEGS);
    std::vector<double> stance_centroid_y_samples;
    stance_centroid_y_samples.reserve(total_iterations_target);
    // Se eliminan estadísticas por ciclo para salida compacta

    // Ya no se hace agrupación detallada; sólo métricas globales

    double initial_centroid_y = 0.0;
    bool initial_centroid_set = false;

    int iteration = 0;
    int cycle_index = 0;
    double accum_centroid_y_current_cycle = 0.0;
    int centroid_samples_current_cycle = 0;

    // Body tracking buffers
    std::vector<double> body_y_samples;
    body_y_samples.reserve(total_iterations_target);
    double initial_body_x = 0.0;
    bool body_initial_set = false;
    double initial_body_y = 0.0;
    double initial_body_yaw_deg = 0.0;
    double final_body_x = 0.0;
    double final_body_y = 0.0;
    double final_body_yaw_deg = 0.0;
    double initial_stance_centroid_x = 0.0;
    bool init_centroid_x_set = false;
    double last_stance_centroid_x = 0.0;

    // Sin seguimiento de alternancia detallado (se asume configuración válida del gait)

    while (iteration < total_iterations_target) {
        if (!sys.update()) {
            std::cerr << "WARNING: update() failed at iteration " << iteration << std::endl;
            continue;
        }

        // Capture body position & orientation
        Eigen::Vector3d body_pos = sys.getBodyPosition();
        Eigen::Vector3d body_ori = sys.getBodyOrientation(); // roll, pitch, yaw (deg? check convention)
        // Assuming internal orientation units are degrees (adjust if radians)
        double yaw_deg = body_ori[2];
        if (!body_initial_set) {
            initial_body_x = body_pos[0];
            initial_body_y = body_pos[1];
            initial_body_yaw_deg = yaw_deg;
            body_initial_set = true;
        }
        body_y_samples.push_back(body_pos[1] - initial_body_y);
        final_body_x = body_pos[0];
        final_body_y = body_pos[1];
        final_body_yaw_deg = yaw_deg;

        // Collect per-leg Y samples
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const Leg &L = sys.getLeg(leg);
            Point3D tip = L.getCurrentTipPositionGlobal();
            all_tip_y_samples.push_back(tip.y);
        }

        // Construir centroide de apoyo (piernas en STANCE_PHASE)
        double sum_y = 0.0;
        double sum_x = 0.0;
        int stance_count = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const Leg &L = sys.getLeg(leg);
            if (L.getStepPhase() == STANCE_PHASE) {
                Point3D tip = L.getCurrentTipPositionGlobal();
                sum_y += tip.y;
                sum_x += tip.x;
                stance_count++;
            }
        }
        double centroid_y = (stance_count > 0) ? (sum_y / stance_count) : 0.0;
        stance_centroid_y_samples.push_back(centroid_y);
        if (!initial_centroid_set) {
            initial_centroid_y = centroid_y;
            initial_centroid_set = true;
        }
        double centroid_x = (stance_count > 0) ? (sum_x / stance_count) : 0.0;
        if (!init_centroid_x_set) {
            initial_stance_centroid_x = centroid_x;
            init_centroid_x_set = true;
        }
        last_stance_centroid_x = centroid_x; // continuously updated

        accum_centroid_y_current_cycle += centroid_y;
        centroid_samples_current_cycle++;

        iteration++;
        if (iteration % cycle_iterations == 0) {
            // Progreso: imprimir un punto cada ciclo (sin saltos de línea hasta el resumen final)
            if (cycle_index == 0) {
                std::cout << "[RUNNING] tripod_linearity_test: " << NUM_CYCLES_TO_SAMPLE
                          << " ciclos..." << std::flush;
            }
            std::cout << '.' << std::flush;
            // Reiniciar acumuladores por ciclo
            accum_centroid_y_current_cycle = 0.0;
            centroid_samples_current_cycle = 0;
            cycle_index++;
        }
    }

    // Compute metrics
    double sum_all_y = std::accumulate(all_tip_y_samples.begin(), all_tip_y_samples.end(), 0.0);
    double overall_mean_tip_y = (all_tip_y_samples.empty() ? 0.0 : sum_all_y / all_tip_y_samples.size());

    double max_abs_centroid_y = 0.0;
    for (double v : stance_centroid_y_samples) {
        double deviation = std::fabs(v - initial_centroid_y);
        if (deviation > max_abs_centroid_y)
            max_abs_centroid_y = deviation;
    }

    // Body metrics
    double body_sum_abs_y = 0.0;
    double body_max_abs_y = 0.0;
    for (double dy : body_y_samples) {
        double ady = std::fabs(dy);
        body_sum_abs_y += ady;
        if (ady > body_max_abs_y)
            body_max_abs_y = ady;
    }
    double body_mean_abs_y = body_y_samples.empty() ? 0.0 : body_sum_abs_y / body_y_samples.size();
    double total_forward_progress_x = final_body_x - initial_body_x;
    double body_yaw_drift = std::fabs(final_body_yaw_deg - initial_body_yaw_deg);

    // Report
    // Asegurar salto de línea tras la línea de progreso
    std::cout << "\n=== Tripod Linearity Metrics ===" << std::endl;
    std::cout << "Cycles sampled: " << NUM_CYCLES_TO_SAMPLE << " (" << cycle_iterations << " iterations each)" << std::endl;
    std::cout << "Overall mean tip Y: " << overall_mean_tip_y << " mm" << std::endl;
    std::cout << "Initial stance centroid Y: " << initial_centroid_y << " mm" << std::endl;
    std::cout << "Max abs stance centroid Y drift: " << max_abs_centroid_y << " mm" << std::endl;
    // Salida compacta sin listado por ciclo
    std::cout << "-- Body trajectory --" << std::endl;
    std::cout << "Forward progress X: " << total_forward_progress_x << " mm" << std::endl;
    std::cout << "Body mean |Y deviation|: " << body_mean_abs_y << " mm" << std::endl;
    std::cout << "Body max |Y deviation|: " << body_max_abs_y << " mm" << std::endl;
    std::cout << "Body yaw drift: " << body_yaw_drift << " deg" << std::endl;

    bool pass = std::fabs(overall_mean_tip_y) <= MEAN_Y_TOLERANCE &&
                max_abs_centroid_y <= STANCE_CENTROID_DRIFT_TOL &&
                body_max_abs_y <= BODY_MAX_LATERAL_TOL &&
                body_yaw_drift <= BODY_YAW_DRIFT_TOL_DEG &&
                total_forward_progress_x > 5.0; // require some forward motion

    if (!pass) {
        std::cerr << "\nFAIL: Tripod gait exhibits lateral bias beyond tolerance." << std::endl;
        std::cerr << "  |overall_mean_tip_y| <= " << MEAN_Y_TOLERANCE << " ? => " << std::fabs(overall_mean_tip_y) << std::endl;
        std::cerr << "  max_abs_centroid_y <= " << STANCE_CENTROID_DRIFT_TOL << " ? => " << max_abs_centroid_y << std::endl;
        std::cerr << "  body_max_lateral_y <= " << BODY_MAX_LATERAL_TOL << " ? => " << body_max_abs_y << std::endl;
        std::cerr << "  body_yaw_drift <= " << BODY_YAW_DRIFT_TOL_DEG << " ? => " << body_yaw_drift << std::endl;
        std::cerr << "  forward_progress_x > 5 ? => " << total_forward_progress_x << std::endl;
        return 1;
    }

    std::cout << "\nPASS: Tripod gait linearity within tolerances." << std::endl;
    return 0;
}
