#include "gait_config_factory.h"
#include "robot_model.h"
#include "stride_deviation_limits.h"
#include "velocity_limits.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

static Parameters makeParams() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    p.max_velocity = 600.0;
    p.step_frequency = 1.0;
    return p;
}

static VelocityLimits::LimitMap makeWalkspace() {
    VelocityLimits::LimitMap map;
    for (int bearing = 0; bearing < 360; bearing += BEARING_STEP) {
        if (bearing == 90 || bearing == 270) {
            map[bearing] = 58.0;
        } else if (bearing == 0 || bearing == 180) {
            map[bearing] = 52.0;
        } else {
            map[bearing] = 55.0;
        }
    }
    return map;
}

static double mapMinimumFinite(const VelocityLimits::LimitMap &m) {
    double min_v = 1e18;
    for (VelocityLimits::LimitMap::const_iterator it = m.begin(); it != m.end(); ++it) {
        if (std::isfinite(it->second) && it->second < UNASSIGNED_VALUE) {
            min_v = std::min(min_v, it->second);
        }
    }
    return (min_v == 1e18) ? 0.0 : min_v;
}

struct TestReporter {
    int total = 0;
    int passed = 0;

    void printHeader(const Parameters &p, const GaitConfiguration &gait, const VelocityLimits::LimitMap &walkspace) {
        std::cout << "================================================================================" << std::endl;
        std::cout << "                  STRIDE DEVIATION LIMITS - DETAILED VALIDATION" << std::endl;
        std::cout << "================================================================================" << std::endl;
        std::cout << "Inputs:" << std::endl;
        std::cout << "  Morphology: hex_radius=" << p.hexagon_radius
                  << " coxa=" << p.coxa_length
                  << " femur=" << p.femur_length
                  << " tibia=" << p.tibia_length << " [mm]" << std::endl;
        std::cout << "  Heights: robot_height=" << p.robot_height
                  << " standing_height=" << p.standing_height
                  << " default_height_offset=" << p.default_height_offset << " [mm]" << std::endl;
        std::cout << "  Limits(deg): coxa=[" << p.coxa_angle_limits[0] << ", " << p.coxa_angle_limits[1]
                  << "] femur=[" << p.femur_angle_limits[0] << ", " << p.femur_angle_limits[1]
                  << "] tibia=[" << p.tibia_angle_limits[0] << ", " << p.tibia_angle_limits[1] << "]" << std::endl;
        std::cout << "  Gait: " << gait.gait_name << " step_freq=" << gait.step_frequency
                  << "Hz time_delta=" << p.time_delta << "s" << std::endl;
        std::cout << "  Walkspace bearings: " << walkspace.size() << " entries" << std::endl;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
    }

    void checkCondition(const std::string &name, bool condition, const std::string &details) {
        ++total;
        std::cout << "[TEST " << std::setw(2) << total << "] " << name << std::endl;
        std::cout << "         " << details << std::endl;
        std::cout << "         Result: " << (condition ? "PASS" : "FAIL") << std::endl;
        if (condition) {
            ++passed;
        }
        std::cout << std::endl;
        assert(condition);
    }

    void printSummary() const {
        std::cout << "================================================================================" << std::endl;
        std::cout << "Summary: " << passed << "/" << total << " checks passed" << std::endl;
        std::cout << "Stride deviation limits test PASSED" << std::endl;
        std::cout << "================================================================================" << std::endl;
    }
};

int main() {
    Parameters p = makeParams();
    RobotModel model(p);
    GaitConfiguration tripod = createTripodGaitConfig(p);
    VelocityLimits::LimitMap walkspace = makeWalkspace();

    StrideDeviationLimits limits(model);
    TestReporter reporter;
    reporter.printHeader(p, tripod, walkspace);

    std::pair<double, double> height_range = limits.calculateBodyHeightRange();
    reporter.checkCondition(
        "Body height range is valid",
        (height_range.first > 0.0) && (height_range.second > height_range.first),
        "Input: joint limits from Parameters | Expected: min>0 and max>min | Got: min=" +
            std::to_string(height_range.first) + " mm, max=" + std::to_string(height_range.second) + " mm");

    reporter.checkCondition(
        "Body min/max wrappers match range",
        std::fabs(limits.calculateMinimumBodyHeight() - height_range.first) < 1e-9 &&
            std::fabs(limits.calculateMaximumBodyHeight() - height_range.second) < 1e-9,
        "Expected: wrapper values equal calculateBodyHeightRange() | Got: min_wrapper=" +
            std::to_string(limits.calculateMinimumBodyHeight()) + ", max_wrapper=" +
            std::to_string(limits.calculateMaximumBodyHeight()));

    double standing_reach = limits.calculateStandingHorizontalReach();
    double standing_reach_expected = RobotModel::computeStandingHorizontalReach(p);
    reporter.checkCondition(
        "Standing horizontal reach parity",
        std::fabs(standing_reach - standing_reach_expected) < 1e-9,
        "Expected (RobotModel::computeStandingHorizontalReach)=" + std::to_string(standing_reach_expected) +
            " mm | Got=" + std::to_string(standing_reach) + " mm");

    double min_planar = limits.calculateMinimumPlanarReach();
    double max_planar = limits.calculateMaximumPlanarReach();
    double kinematic_upper = p.coxa_length + p.femur_length + p.tibia_length + 1e-6;
    reporter.checkCondition(
        "Planar reach bounds",
        (min_planar >= 0.0) && (max_planar > min_planar) && (max_planar <= kinematic_upper),
        "Expected: 0<=min<max<=coxa+femur+tibia(" + std::to_string(kinematic_upper) +
            ") | Got: min=" + std::to_string(min_planar) + ", max=" + std::to_string(max_planar));

    double offset_got = limits.calculateDefaultHeightOffset();
    reporter.checkCondition(
        "Default height offset passthrough",
        std::fabs(offset_got - p.default_height_offset) < 1e-9,
        "Expected=" + std::to_string(p.default_height_offset) + " mm | Got=" + std::to_string(offset_got) + " mm");

    double class_max_linear = limits.calculateOpenSHCMaxLinearSpeedFromWalkspace(tripod, walkspace);
    double class_max_angular = limits.calculateOpenSHCMaxAngularSpeedFromWalkspace(tripod, walkspace);
    reporter.checkCondition(
        "OpenSHC walkspace limits are positive",
        class_max_linear > 0.0 && class_max_angular > 0.0,
        "Expected: linear>0 and angular>0 | Got: linear=" + std::to_string(class_max_linear) +
            " mm/s, angular=" + std::to_string(class_max_angular) + " rad/s");

    VelocityLimits raw_limits(model);
    raw_limits.setWalkspace(walkspace);
    CalculatedServoAngles calc = RobotModel::calculateServoAnglesForHeight(p.standing_height, p);
    JointAngles nominal(0.0, calc.valid ? calc.femur : 0.0, calc.valid ? calc.tibia : 0.0);
    raw_limits.setReferenceTipPosition(model.forwardKinematicsGlobalCoordinates(0, nominal));

    VelocityLimits::LimitMap linear_map;
    VelocityLimits::LimitMap angular_map;
    raw_limits.generateLimits(tripod, &linear_map, &angular_map, NULL, NULL);

    double linear_min_expected = mapMinimumFinite(linear_map);
    double angular_min_expected = mapMinimumFinite(angular_map);

    reporter.checkCondition(
        "Linear limit parity vs VelocityLimits",
        std::fabs(class_max_linear - linear_min_expected) < 1e-9,
        "Expected(min map)=" + std::to_string(linear_min_expected) + " mm/s | Got=" +
            std::to_string(class_max_linear) + " mm/s");

    reporter.checkCondition(
        "Angular limit parity vs VelocityLimits",
        std::fabs(class_max_angular - angular_min_expected) < 1e-9,
        "Expected(min map)=" + std::to_string(angular_min_expected) + " rad/s | Got=" +
            std::to_string(class_max_angular) + " rad/s");

    double tol_small_deg = 1.0;
    double tol_large_deg = 6.0;
    double v_small = limits.calculateMaxLinearSpeedForMinimalAngularDeviation(tripod, tol_small_deg, &walkspace);
    double v_large = limits.calculateMaxLinearSpeedForMinimalAngularDeviation(tripod, tol_large_deg, &walkspace);

    reporter.checkCondition(
        "Deviation-constrained speeds are ordered and bounded",
        v_small >= 0.0 && v_large >= 0.0 && v_small <= v_large + 1e-6 && v_large <= class_max_linear + 1e-6,
        "Input tolerances: small=" + std::to_string(tol_small_deg) + " deg, large=" + std::to_string(tol_large_deg) +
            " deg | Got: v_small=" + std::to_string(v_small) + " mm/s, v_large=" + std::to_string(v_large) +
            " mm/s, class_max_linear=" + std::to_string(class_max_linear) + " mm/s");

    if (v_small > 0.0) {
        double dev_small = limits.calculateWorstFemurTibiaDeviationDeg(tripod, v_small);
        reporter.checkCondition(
            "Small tolerance deviation satisfied",
            dev_small <= tol_small_deg + 0.25,
            "Input: v_small=" + std::to_string(v_small) + " mm/s, tol=" + std::to_string(tol_small_deg) +
                " deg | Expected: dev<=tol+0.25 | Got dev=" + std::to_string(dev_small) + " deg");
    } else {
        reporter.checkCondition(
            "Small tolerance branch handled (zero speed)",
            true,
            "Input: tol=" + std::to_string(tol_small_deg) + " deg produced v_small=0, so direct deviation check is skipped.");
    }

    if (v_large > 0.0) {
        double dev_large = limits.calculateWorstFemurTibiaDeviationDeg(tripod, v_large);
        reporter.checkCondition(
            "Large tolerance deviation satisfied",
            dev_large <= tol_large_deg + 0.25,
            "Input: v_large=" + std::to_string(v_large) + " mm/s, tol=" + std::to_string(tol_large_deg) +
                " deg | Expected: dev<=tol+0.25 | Got dev=" + std::to_string(dev_large) + " deg");
    } else {
        reporter.checkCondition(
            "Large tolerance branch handled (zero speed)",
            true,
            "Input: tol=" + std::to_string(tol_large_deg) + " deg produced v_large=0, so direct deviation check is skipped.");
    }

    double dev_at_zero = limits.calculateWorstFemurTibiaDeviationDeg(tripod, 0.0);
    reporter.checkCondition(
        "Zero speed produces zero deviation",
        std::fabs(dev_at_zero) < 1e-12,
        "Input: speed=0 mm/s | Expected deviation=0 | Got deviation=" + std::to_string(dev_at_zero) + " deg");

    reporter.printSummary();
    return 0;
}
