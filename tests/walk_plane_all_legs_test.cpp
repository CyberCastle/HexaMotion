/**
 * @file walk_plane_all_legs_test.cpp
 * @brief Validates the OpenSHC-aligned walk-plane fit over ALL legs' default poses (§2.4).
 *
 * BodyPoseController::calculateWalkPlaneNormal/Height now fit a least-squares plane (z = a*x + b*y + c)
 * over the default tip positions of every leg (matching WalkController::updateWalkPlane in OpenSHC),
 * not just a subset of stance legs. The normal is (-a, -b, 1).normalized() and the height is the plane
 * offset c. This test exercises the exact public helper (math_utils::solveLeastSquaresPlane) the
 * controller uses, fed with all six legs' positions on a known tilted plane, and asserts the recovered
 * coefficients and the OpenSHC normal formula.
 */

#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iostream>
#include <vector>

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

int main() {
    std::cout << "=== Walk Plane All-Legs Fit Test (\u00a72.4) ===\n";

    // Known tilted plane: z = a0*x + b0*y + c0.
    const double a0 = 0.05, b0 = -0.02, c0 = -150.0;

    // Six default tip positions (one per leg) spread around the hexagon, all on the plane.
    std::vector<Point3D> default_tips;
    const double R = 332.8; // approx default stance radius (AGENTS.md)
    for (int i = 0; i < NUM_LEGS; ++i) {
        double theta = (-30.0 + 60.0 * i) * M_PI / 180.0;
        double x = R * std::cos(theta);
        double y = R * std::sin(theta);
        double z = a0 * x + b0 * y + c0;
        default_tips.emplace_back(x, y, z);
    }

    // Replicate BodyPoseController's exact fit input layout.
    std::vector<double> raw_A;
    std::vector<double> raw_B;
    for (const auto &p : default_tips) {
        raw_A.push_back(p.x);
        raw_A.push_back(p.y);
        raw_A.push_back(1.0);
        raw_B.push_back(p.z);
    }

    double a, b, c;
    bool ok = math_utils::solveLeastSquaresPlane(raw_A.data(), raw_B.data(), NUM_LEGS, a, b, c);
    check(ok, "least-squares plane fit succeeds over all 6 legs");

    std::cout << "fitted a=" << a << " b=" << b << " c=" << c << "\n";
    check(std::fabs(a - a0) < 1e-6, "recovered a matches known plane slope");
    check(std::fabs(b - b0) < 1e-6, "recovered b matches known plane slope");
    check(std::fabs(c - c0) < 1e-3, "recovered c (height) matches known plane offset");

    // OpenSHC normal: (-a, -b, 1).normalized(), flipped to point up.
    double mag = std::sqrt(a * a + b * b + 1.0);
    Point3D normal(-a / mag, -b / mag, 1.0 / mag);
    if (normal.z < 0) {
        normal.x = -normal.x;
        normal.y = -normal.y;
        normal.z = -normal.z;
    }
    Point3D expected(-a0, -b0, 1.0);
    expected = expected.normalized();
    std::cout << "normal=(" << normal.x << "," << normal.y << "," << normal.z << ")\n";
    check(std::fabs(normal.x - expected.x) < 1e-6 &&
              std::fabs(normal.y - expected.y) < 1e-6 &&
              std::fabs(normal.z - expected.z) < 1e-6,
          "walk-plane normal matches OpenSHC (-a,-b,1).normalized()");
    check(normal.z > 0, "normal points up");

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
