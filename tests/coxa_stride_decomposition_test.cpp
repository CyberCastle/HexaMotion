/**
 * @file coxa_stride_decomposition_test.cpp
 * @brief Diagnostic test that exposes stride composition and per-leg deltas during tripod gait.
 *
 * The goal is to reproduce the coxa angle asymmetry observed in tripod stance phases by inspecting
 * the internal LegStepper debug state. The test prints, for each leg in stance:
 *   - The default/base tip pose used as reference
 *   - The active stride vector applied (frozen or live)
 *   - The composed pose (default + stride) that feeds the target generation
 *   - The incremental delta integrated on the current iteration
 *   - The delta expressed in the local leg frame (forward/lateral components)
 *   - The resulting coxa angle in degrees
 *
 * This step-by-step breakdown lets us verify how a shared global displacement translates into
 * different local motions depending on each leg's base orientation, highlighting the geometric
 * source of the stance-phase deviation.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/math_utils.h"
#include "robot_model.h"
#include "test_stubs.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

static std::string formatXY(const Point3D &p) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "(" << p.x << ", " << p.y << ")";
    return oss.str();
}

static std::string formatDelta(const Point3D &p) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "(" << p.x << ", " << p.y << ")";
    return oss.str();
}

static std::pair<double, double> projectToLocal(const Point3D &delta, double base_angle_rad) {
    double c = std::cos(base_angle_rad);
    double s = std::sin(base_angle_rad);
    double forward = delta.x * c + delta.y * s;
    double lateral = -delta.x * s + delta.y * c;
    return {forward, lateral};
}

static std::string padLeft(const std::string &value, int width) {
    if (static_cast<int>(value.size()) >= width) {
        return value;
    }
    return std::string(width - value.size(), ' ') + value;
}

static std::string formatInt(int value, int width) {
    std::ostringstream oss;
    oss << value;
    return padLeft(oss.str(), width);
}

static std::string formatNumber(double value, int width, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return padLeft(oss.str(), width);
}

static std::string formatLocal(double forward, double lateral, int width) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "(" << forward << ", " << lateral << ")";
    return padLeft(oss.str(), width);
}

static std::string formatLegLocal(const char *name, double forward, double lateral, int width) {
    std::ostringstream oss;
    oss << name << " " << std::fixed << std::setprecision(3) << "(" << forward << ", " << lateral << ")";
    return padLeft(oss.str(), width);
}

struct LegSample {
    int leg = -1;
    int iteration = -1;
    double base_deg = 0.0;
    double forward = 0.0;
    double lateral = 0.0;
    double coxa_deg = 0.0;
    double delta_x = 0.0;
    double delta_y = 0.0;
};

struct PairReport {
    int pair_index = -1;
    int iteration = -1;
    LegSample primary;
    LegSample secondary;
};

struct PairBuffer {
    bool has_primary = false;
    LegSample primary;
    bool has_secondary = false;
    LegSample secondary;
};

int main() {
    Parameters params = createDefaultParameters();
    /** Keep velocity limits permissive for diagnostics. */
    params.max_velocity = 1000.0;

    LocomotionSystem system(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!system.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Unable to initialize locomotion system" << std::endl;
        return 1;
    }

    if (!system.setStandingPose()) {
        std::cerr << "ERROR: Unable to set standing pose" << std::endl;
        return 1;
    }

    GaitConfiguration tripod = createGaitConfig(TRIPOD_GAIT, params);
    double reach = RobotModel::computeStandingHorizontalReach(params);
    /** Replicate locomotion test configuration. */
    tripod.step_length = reach * 2.0;
    tripod.time_to_max_stride = 0.2;

    if (!system.setGaitConfiguration(tripod)) {
        std::cerr << "ERROR: Unable to configure tripod gait" << std::endl;
        return 1;
    }

    system.walkForward(600.0);
    if (!system.startWalking()) {
        std::cerr << "ERROR: Failed to start walking" << std::endl;
        return 1;
    }

    /** Execute startup sequence exactly once before sampling debug data. */
    const int MAX_STARTUP_ITERATIONS = 400;
    int startup_iterations = 0;
    bool startup_completed = false;
    while (system.isStartupInProgress() && startup_iterations < MAX_STARTUP_ITERATIONS) {
        if (system.executeStartupSequence()) {
            startup_completed = true;
            break;
        }
        startup_iterations++;
    }

    if (!startup_completed && system.isStartupInProgress()) {
        std::cerr << "ERROR: Startup sequence did not complete" << std::endl;
        return 1;
    }

    WalkController *walk_ctrl = system.getWalkController();
    if (!walk_ctrl) {
        std::cerr << "ERROR: WalkController not available" << std::endl;
        return 1;
    }

    std::vector<std::shared_ptr<LegStepper>> steppers;
    steppers.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        auto stepper = walk_ctrl->getLegStepper(i);
        if (!stepper) {
            std::cerr << "ERROR: Missing LegStepper for leg " << i << std::endl;
            return 1;
        }
        steppers.push_back(stepper);
    }

    const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    const char *PAIR_NAMES[3] = {"AR-AL", "BR-BL", "CR-CL"};
    const int LEG_TO_PAIR[NUM_LEGS] = {0, 1, 2, 2, 1, 0};
    const bool LEG_IS_PRIMARY[NUM_LEGS] = {true, true, true, false, false, false};

    RobotModel &model = system.getRobotModel();
    double base_angle_rad[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        base_angle_rad[i] = model.getLegBaseAngleOffset(i);
    }

    PairBuffer pair_buffers[3];
    std::vector<PairReport> pair_reports;
    pair_reports.reserve(12);

    std::cout << "=== Coxa Stride Decomposition Test ===" << std::endl;
    std::cout << "Iter | Leg | Phase | BaseDeg |   DefaultXY (mm) |  ActiveStride (mm) |    Base+Stride (mm) |    DeltaXY (mm) | Local(fwd,lat) (mm) | CoxaDeg" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;

    StepPhase previous_phase[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phase[i] = system.getLeg(i).getStepPhase();
    }

    const int REQUIRED_STANCE_TRANSITIONS = 100;
    const int MAX_UPDATE_STEPS = 4800;
    const int MAX_PAIR_ROWS = 12;
    int stance_transition_samples = 0;
    int total_phase_changes = 0;
    int rows_emitted = 0;

    for (int step = 0; step < MAX_UPDATE_STEPS && stance_transition_samples < REQUIRED_STANCE_TRANSITIONS; ++step) {
        if (!system.update()) {
            std::cerr << "WARNING: System update failed at step " << step << std::endl;
            continue;
        }

        for (int leg = 0; leg < NUM_LEGS && stance_transition_samples < REQUIRED_STANCE_TRANSITIONS; ++leg) {
            const StepPhase phase = system.getLeg(leg).getStepPhase();
            if (phase != previous_phase[leg]) {
                total_phase_changes++;
                previous_phase[leg] = phase;

                if (phase != STANCE_PHASE) {
                    /** Only inspect transitions into stance for precise drift checks. */
                    continue;
                }

                stance_transition_samples++;

                const auto &debug = steppers[leg]->getDebugState();

                auto local_components = projectToLocal(debug.last_delta, base_angle_rad[leg]);
                double coxa_deg = math_utils::radiansToDegrees(system.getLeg(leg).getJointAngles().coxa);
                double base_deg = math_utils::radiansToDegrees(base_angle_rad[leg]);

                std::string row;
                row.reserve(180);
                row += formatInt(debug.iteration, 4);
                row += " | ";
                row += padLeft(std::string(LEG_NAMES[leg]), 3);
                row += " | ";
                row += padLeft(std::string("S"), 5);
                row += " | ";
                row += formatNumber(base_deg, 7, 1);
                row += " | ";
                row += padLeft(formatXY(debug.default_tip_pose), 17);
                row += " | ";
                row += padLeft(formatXY(debug.active_stride), 17);
                row += " | ";
                row += padLeft(formatXY(debug.composed_pose), 17);
                row += " | ";
                row += padLeft(formatDelta(debug.last_delta), 16);
                row += " | ";
                row += formatLocal(local_components.first, local_components.second, 18);
                row += " | ";
                row += formatNumber(coxa_deg, 7, 2);

                /** Register sample for pairwise comparison when matching iterations are observed. */
                LegSample sample;
                sample.leg = leg;
                sample.iteration = debug.iteration;
                sample.base_deg = base_deg;
                sample.forward = local_components.first;
                sample.lateral = local_components.second;
                sample.coxa_deg = coxa_deg;
                sample.delta_x = debug.last_delta.x;
                sample.delta_y = debug.last_delta.y;

                int pair_index = LEG_TO_PAIR[leg];
                PairBuffer &buffer = pair_buffers[pair_index];
                if (LEG_IS_PRIMARY[leg]) {
                    buffer.primary = sample;
                    buffer.has_primary = true;
                } else {
                    buffer.secondary = sample;
                    buffer.has_secondary = true;
                }

                if (buffer.has_primary && buffer.has_secondary && buffer.primary.iteration == buffer.secondary.iteration) {
                    PairReport report;
                    report.pair_index = pair_index;
                    report.iteration = buffer.primary.iteration;
                    report.primary = buffer.primary;
                    report.secondary = buffer.secondary;
                    if (static_cast<int>(pair_reports.size()) < MAX_PAIR_ROWS) {
                        pair_reports.push_back(report);
                    }
                    buffer.has_primary = false;
                    buffer.has_secondary = false;
                }
                std::cout << row << std::endl;

                rows_emitted++;
            }
        }
    }

    if (stance_transition_samples < REQUIRED_STANCE_TRANSITIONS) {
        std::cerr << "ERROR: Only captured " << stance_transition_samples
                  << " stance transitions across " << total_phase_changes << " phase changes." << std::endl;
        return 1;
    }

    if (!pair_reports.empty()) {
        std::cout << "\nPairwise stance comparisons (matched iterations)" << std::endl;
        std::cout << "Pair | Iter |       LegA (fwd,lat mm)       |       LegB (fwd,lat mm)       |   Δfwd |   Δlat | CoxaA | CoxaB |  ΔCoxa |   ΔδX |   ΔδY" << std::endl;
        std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
        for (const auto &report : pair_reports) {
            const LegSample &a = report.primary;
            const LegSample &b = report.secondary;
            double delta_forward = a.forward - b.forward;
            double delta_lateral = a.lateral - b.lateral;
            double delta_coxa = a.coxa_deg - b.coxa_deg;
            double delta_dx = a.delta_x - b.delta_x;
            double delta_dy = a.delta_y - b.delta_y;

            std::string row;
            row.reserve(180);
            row += padLeft(std::string(PAIR_NAMES[report.pair_index]), 6);
            row += " | ";
            row += formatInt(report.iteration, 4);
            row += " | ";
            row += formatLegLocal(LEG_NAMES[a.leg], a.forward, a.lateral, 30);
            row += " | ";
            row += formatLegLocal(LEG_NAMES[b.leg], b.forward, b.lateral, 30);
            row += " | ";
            row += formatNumber(delta_forward, 7, 3);
            row += " | ";
            row += formatNumber(delta_lateral, 7, 3);
            row += " | ";
            row += formatNumber(a.coxa_deg, 6, 2);
            row += " | ";
            row += formatNumber(b.coxa_deg, 6, 2);
            row += " | ";
            row += formatNumber(delta_coxa, 8, 3);
            row += " | ";
            row += formatNumber(delta_dx, 7, 3);
            row += " | ";
            row += formatNumber(delta_dy, 7, 3);

            std::cout << row << std::endl;
        }
        std::cout << "Comparisons emitted: " << pair_reports.size() << std::endl;
    } else {
        std::cout << "\nNo matching stance iterations captured for opposing leg pairs." << std::endl;
    }

    std::cout << "\nStance transitions inspected: " << stance_transition_samples << " (out of " << total_phase_changes
              << " total phase changes)" << std::endl;
    if (rows_emitted == 0) {
        std::cout << "NOTE: No stance samples captured; check gait configuration." << std::endl;
    }

    system.stopWalking();

    return 0;
}
