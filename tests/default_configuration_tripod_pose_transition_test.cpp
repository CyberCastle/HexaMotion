#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/math_utils.h"
#include "test_stubs.h"

#include <array>
#include <cmath>
#include <iostream>
#include <string>

namespace {

struct TripodWindowMetrics {
    int samples = 0;
    int coherent_samples = 0;
    int valid_partition_samples = 0;
    int group_a_stance_samples = 0;
    int group_b_stance_samples = 0;
    double max_tip_jump_mm = 0.0;
    double max_workspace_projection_error_mm = 0.0;
};

double distance3D(const Point3D &a, const Point3D &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool runUntilRunning(LocomotionSystem &sys, int max_loops) {
    for (int i = 0; i < max_loops; ++i) {
        if (!sys.update()) {
            return false;
        }
        if (sys.getRobotState() == ROBOT_RUNNING) {
            return true;
        }
    }
    return false;
}

bool isTripodCoherentSample(const LocomotionSystem &sys, bool &group_a_stance, bool &group_b_stance) {
    int stance_count = 0;
    int swing_count = 0;

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLegState(i) == STANCE_PHASE) {
            stance_count++;
        } else if (sys.getLegState(i) == SWING_PHASE) {
            swing_count++;
        }
    }

    const bool partition_ok = (stance_count == 3 && swing_count == 3);

    group_a_stance = (sys.getLegState(0) == STANCE_PHASE) &&
                     (sys.getLegState(2) == STANCE_PHASE) &&
                     (sys.getLegState(4) == STANCE_PHASE);

    group_b_stance = (sys.getLegState(1) == STANCE_PHASE) &&
                     (sys.getLegState(3) == STANCE_PHASE) &&
                     (sys.getLegState(5) == STANCE_PHASE);

    const bool pattern_ok = (group_a_stance && !group_b_stance) ||
                            (group_b_stance && !group_a_stance);

    return partition_ok && pattern_ok;
}

bool collectTripodWindow(LocomotionSystem &sys,
                         int iterations,
                         TripodWindowMetrics &metrics,
                         std::string &error_message) {
    std::array<Point3D, NUM_LEGS> previous_tips;
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_tips[i] = Point3D(0.0, 0.0, 0.0);
    }
    bool previous_valid = false;

    int accepted_samples = 0;
    const int max_attempts = iterations * 8;

    for (int attempt = 0; attempt < max_attempts && accepted_samples < iterations; ++attempt) {
        if (!sys.update()) {
            error_message = "update() failed while collecting tripod window";
            return false;
        }

        WalkController *walker = sys.getWalkController();
        if (!walker || walker->getWalkState() != WALK_MOVING) {
            continue;
        }

        bool group_a_stance = false;
        bool group_b_stance = false;
        bool coherent = isTripodCoherentSample(sys, group_a_stance, group_b_stance);

        int stance_count = 0;
        int swing_count = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (sys.getLegState(i) == STANCE_PHASE) {
                stance_count++;
            } else if (sys.getLegState(i) == SWING_PHASE) {
                swing_count++;
            }
        }

        metrics.samples++;
        if (coherent) {
            metrics.coherent_samples++;
        }
        if (stance_count == 3 && swing_count == 3) {
            metrics.valid_partition_samples++;
        }
        if (group_a_stance) {
            metrics.group_a_stance_samples++;
        }
        if (group_b_stance) {
            metrics.group_b_stance_samples++;
        }

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            Point3D tip = sys.getLegPosition(leg);

            Point3D projected = sys.constrainToWorkspace(leg, tip);
            double workspace_error = distance3D(projected, tip);
            if (workspace_error > metrics.max_workspace_projection_error_mm) {
                metrics.max_workspace_projection_error_mm = workspace_error;
            }

            if (previous_valid) {
                double jump = distance3D(previous_tips[leg], tip);
                if (jump > metrics.max_tip_jump_mm) {
                    metrics.max_tip_jump_mm = jump;
                }
            }

            previous_tips[leg] = tip;
        }

        previous_valid = true;
        accepted_samples++;
    }

    if (accepted_samples < iterations) {
        error_message = "insufficient stable WALK_MOVING samples for tripod window";
        return false;
    }

    return true;
}

Point3D computeTipCentroid(const LocomotionSystem &sys) {
    Point3D centroid(0.0, 0.0, 0.0);
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Point3D tip = sys.getLegPosition(i);
        centroid.x += tip.x;
        centroid.y += tip.y;
        centroid.z += tip.z;
    }
    centroid.x /= NUM_LEGS;
    centroid.y /= NUM_LEGS;
    centroid.z /= NUM_LEGS;
    return centroid;
}

bool validateTripodWindow(const TripodWindowMetrics &metrics,
                          const std::string &label,
                          std::string &error_message) {
    if (metrics.samples <= 0) {
        error_message = label + ": no samples were collected";
        return false;
    }

    const double coherent_ratio = static_cast<double>(metrics.coherent_samples) / metrics.samples;
    const double partition_ratio = static_cast<double>(metrics.valid_partition_samples) / metrics.samples;

    if (coherent_ratio < 0.95) {
        error_message = label + ": tripod coherence ratio below 95%";
        return false;
    }

    if (partition_ratio < 0.98) {
        error_message = label + ": stance/swing partition ratio below 98%";
        return false;
    }

    if (metrics.group_a_stance_samples == 0 || metrics.group_b_stance_samples == 0) {
        error_message = label + ": both tripod groups were not observed in stance";
        return false;
    }

    if (metrics.max_workspace_projection_error_mm > 1.0) {
        error_message = label + ": workspace projection error exceeded 1.0 mm";
        return false;
    }

    if (metrics.max_tip_jump_mm > 120.0) {
        error_message = label + ": excessive per-step tip jump detected";
        return false;
    }

    return true;
}

void printMetrics(const TripodWindowMetrics &metrics, const std::string &label) {
    const double coherent_ratio = (metrics.samples > 0)
                                      ? static_cast<double>(metrics.coherent_samples) / metrics.samples
                                      : 0.0;
    const double partition_ratio = (metrics.samples > 0)
                                       ? static_cast<double>(metrics.valid_partition_samples) / metrics.samples
                                       : 0.0;

    std::cout << "[" << label << "] samples=" << metrics.samples
              << " coherent_ratio=" << coherent_ratio
              << " partition_ratio=" << partition_ratio
              << " groupA_stance_samples=" << metrics.group_a_stance_samples
              << " groupB_stance_samples=" << metrics.group_b_stance_samples
              << " max_tip_jump_mm=" << metrics.max_tip_jump_mm
              << " max_workspace_projection_error_mm=" << metrics.max_workspace_projection_error_mm
              << std::endl;
}

} // namespace

int main() {
    Parameters params = createDefaultParameters();
    params.use_fsr_contact = false;

    LocomotionSystem system(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!system.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "FAIL: initialization failed" << std::endl;
        return 1;
    }

    if (!system.setStandingPose()) {
        std::cerr << "FAIL: setStandingPose failed" << std::endl;
        return 1;
    }

    GaitConfiguration tripod = createGaitConfig(TRIPOD_GAIT, params);
    if (!system.setGaitConfiguration(tripod)) {
        std::cerr << "FAIL: setGaitConfiguration(TRIPOD_GAIT) failed" << std::endl;
        return 1;
    }

    const Eigen::Vector3d baseline_position = system.getBodyPosition();

    const Eigen::Vector3d pose_a_position = baseline_position + Eigen::Vector3d(8.0, -6.0, 0.0);
    const Eigen::Vector3d pose_a_orientation(
        math_utils::degreesToRadians(2.0),
        math_utils::degreesToRadians(-1.5),
        math_utils::degreesToRadians(1.0));

    if (!system.setBodyPose(pose_a_position, pose_a_orientation)) {
        std::cerr << "FAIL: setBodyPose for initial pose failed" << std::endl;
        return 1;
    }

    const Eigen::Vector3d position_after_pose_a = system.getBodyPosition();
    if ((position_after_pose_a - pose_a_position).norm() > 1e-6) {
        std::cerr << "FAIL: initial pose was not applied" << std::endl;
        return 1;
    }

    if (!system.walkForward(80.0)) {
        std::cerr << "FAIL: walkForward command failed" << std::endl;
        return 1;
    }

    if (!system.startWalking()) {
        std::cerr << "FAIL: startWalking failed" << std::endl;
        return 1;
    }

    if (!runUntilRunning(system, 1000)) {
        std::cerr << "FAIL: robot did not reach RUNNING state" << std::endl;
        return 1;
    }

    for (int i = 0; i < 180; ++i) {
        if (!system.update()) {
            std::cerr << "FAIL: warm-up update failed before first validation window" << std::endl;
            return 1;
        }
    }

    TripodWindowMetrics before_metrics;
    std::string window_error;
    if (!collectTripodWindow(system, 280, before_metrics, window_error)) {
        std::cerr << "FAIL: before-update window collection failed: " << window_error << std::endl;
        return 1;
    }

    if (!validateTripodWindow(before_metrics, "before_update", window_error)) {
        printMetrics(before_metrics, "before_update");
        std::cerr << "FAIL: " << window_error << std::endl;
        return 1;
    }

    const Point3D centroid_before_update = computeTipCentroid(system);

    const Eigen::Vector3d pose_b_position = baseline_position + Eigen::Vector3d(-10.0, 9.0, 0.0);
    const Eigen::Vector3d pose_b_orientation(
        math_utils::degreesToRadians(-2.5),
        math_utils::degreesToRadians(1.0),
        math_utils::degreesToRadians(-1.2));

    if (!system.setBodyPose(pose_b_position, pose_b_orientation)) {
        std::cerr << "FAIL: setBodyPose for post-update pose failed" << std::endl;
        return 1;
    }

    if (!system.updateDefaultConfiguration()) {
        std::cerr << "FAIL: updateDefaultConfiguration failed" << std::endl;
        return 1;
    }

    for (int i = 0; i < 120; ++i) {
        if (!system.update()) {
            std::cerr << "FAIL: warm-up update failed after default update" << std::endl;
            return 1;
        }
    }

    const Eigen::Vector3d position_after_pose_b = system.getBodyPosition();
    const Point3D centroid_after_update = computeTipCentroid(system);

    const double body_pose_shift_mm = (position_after_pose_b - position_after_pose_a).norm();
    const double centroid_shift_mm = distance3D(centroid_before_update, centroid_after_update);

    if (body_pose_shift_mm < 5.0) {
        std::cerr << "FAIL: body pose change after updateDefaultConfiguration is too small: "
                  << body_pose_shift_mm << " mm" << std::endl;
        return 1;
    }

    if (centroid_shift_mm < 2.0) {
        std::cerr << "FAIL: tip centroid change after new pose is too small: "
                  << centroid_shift_mm << " mm" << std::endl;
        return 1;
    }

    TripodWindowMetrics after_metrics;
    if (!collectTripodWindow(system, 320, after_metrics, window_error)) {
        std::cerr << "FAIL: after-update window collection failed: " << window_error << std::endl;
        return 1;
    }

    if (!validateTripodWindow(after_metrics, "after_update", window_error)) {
        printMetrics(after_metrics, "after_update");
        std::cerr << "FAIL: " << window_error << std::endl;
        return 1;
    }

    if (after_metrics.max_tip_jump_mm > (before_metrics.max_tip_jump_mm + 45.0)) {
        std::cerr << "FAIL: post-update trajectory continuity degraded too much" << std::endl;
        std::cerr << "  before max_tip_jump_mm=" << before_metrics.max_tip_jump_mm << std::endl;
        std::cerr << "  after  max_tip_jump_mm=" << after_metrics.max_tip_jump_mm << std::endl;
        return 1;
    }

    printMetrics(before_metrics, "before_update");
    printMetrics(after_metrics, "after_update");

    std::cout << "body_pose_shift_mm=" << body_pose_shift_mm
              << " centroid_shift_mm=" << centroid_shift_mm << std::endl;

    std::cout << "PASS: Tripod gait remained coherent across updateDefaultConfiguration with pose change." << std::endl;
    return 0;
}
