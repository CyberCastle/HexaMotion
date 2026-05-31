/**
 * @file rough_terrain_adaptation_test.cpp
 * @brief Validates the rough-terrain swing-target adaptation ported to TerrainAdaptation (§2.6/§3).
 *
 * The logic that previously lived inline inside LegStepper now lives in
 * TerrainAdaptation::computeRoughTerrainSwingTarget:
 *   - PROACTIVE: when a valid step plane is known, project the target onto that plane along the
 *     walk-plane normal.
 *   - REACTIVE: when no valid plane is known (touchdown probing), lower the target by step_depth.
 */

#include "terrain_adaptation.h"
#include "math_utils.h"
#include <cmath>
#include <iostream>

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

static bool approxEqual(const Point3D &a, const Point3D &b, double eps = 1e-6) {
    return std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps && std::fabs(a.z - b.z) < eps;
}

int main() {
    std::cout << "=== Rough Terrain Adaptation Test (\u00a72.6) ===\n";

    const double step_depth = 20.0;
    Point3D current_target(100.0, 50.0, -150.0);

    // REACTIVE case: no valid step plane -> probe downward by step_depth.
    {
        Point3D reactive = TerrainAdaptation::computeRoughTerrainSwingTarget(
            current_target, Point3D(0, 0, 0), Point3D(0, 0, 1), false, step_depth);
        Point3D expected(current_target.x, current_target.y, current_target.z - step_depth);
        check(approxEqual(reactive, expected),
              "reactive: target lowered by step_depth when plane invalid");
    }

    // PROACTIVE case (flat plane, normal +Z): projection onto plane at given height.
    {
        Point3D step_plane_position(0.0, 0.0, -140.0); // plane reference point
        Point3D normal(0, 0, 1);
        Point3D proactive = TerrainAdaptation::computeRoughTerrainSwingTarget(
            current_target, step_plane_position, normal, true, step_depth);
        // projectVector(step_plane_position - current_target, +Z) keeps only the Z component.
        Point3D diff = step_plane_position - current_target;
        Point3D expected = current_target + math_utils::projectVector(diff, normal);
        check(approxEqual(proactive, expected),
              "proactive: target projected onto step plane along normal");
        // For a +Z plane, x/y unchanged and z snaps to plane height delta.
        check(std::fabs(proactive.x - current_target.x) < 1e-6 &&
                  std::fabs(proactive.y - current_target.y) < 1e-6,
              "proactive: x/y preserved for vertical normal");
    }

    // PROACTIVE case (tilted plane normal): projection follows the tilted normal direction.
    {
        Point3D step_plane_position(0.0, 0.0, -130.0);
        Point3D normal = Point3D(0.2, 0.0, 1.0).normalized();
        Point3D proactive = TerrainAdaptation::computeRoughTerrainSwingTarget(
            current_target, step_plane_position, normal, true, step_depth);
        Point3D diff = step_plane_position - current_target;
        Point3D expected = current_target + math_utils::projectVector(diff, normal);
        check(approxEqual(proactive, expected),
              "proactive: tilted-normal projection matches projectVector");
    }

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
