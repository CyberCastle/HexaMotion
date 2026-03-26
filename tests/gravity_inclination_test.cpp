/**
 * @file gravity_inclination_test.cpp
 * @brief Validates gravity-driven inclination compensation in BodyPoseController.
 *
 * Unlike a tautological formula-vs-formula check, this test validates the
 * PHYSICAL EFFECT of inclination compensation:
 *
 * 1. **CoG projection moves uphill**: When the terrain tilts, the body shift
 *    must move the projected center of gravity toward the uphill side of the
 *    support polygon, increasing the static stability margin.
 *
 * 2. **All IK solutions remain valid**: After applying the compensated pose,
 *    every leg must still reach a valid IK solution within joint limits.
 *
 * 3. **Stability margin improves**: The minimum distance from the projected
 *    CoG to the nearest edge of the support polygon must be larger WITH
 *    compensation than WITHOUT, for every non-zero inclination.
 *
 * 4. **Visualization**: Each test case prints a top-down diagram of the
 *    hexapod on the inclined surface, showing leg joint angles, tip positions,
 *    CoG shift, and stability margins.
 *
 * IMU simulation:
 *   accel_x = -g·sin(pitch),  accel_y = g·sin(roll)·cos(pitch),
 *   accel_z = g·cos(roll)·cos(pitch)
 *
 * Test angles are randomly generated (fixed seed) within physical limits.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/hexamotion_constants.h"
#include "../src/locomotion_types.h"
#include "../src/math_utils.h"
#include "../src/robot_model.h"
#include "test_pose_helpers.h"
#include "test_stubs.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// Gravity in m/s^2
static constexpr double G_MS2 = math_utils::GRAVITY_ACCELERATION / 1000.0;

static int tests_passed = 0;
static int tests_failed = 0;

static void checkResult(bool condition, const std::string &test_name) {
    if (condition) {
        std::cout << "  [PASS] " << test_name << std::endl;
        tests_passed++;
    } else {
        std::cout << "  [FAIL] " << test_name << std::endl;
        tests_failed++;
    }
}

static std::string fmt(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    return std::string(buf);
}

// ────────────────────────────────────────────────────────────────────────────
// IMU simulation
// ────────────────────────────────────────────────────────────────────────────

static IMUData buildTiltedIMUData(double roll_deg, double pitch_deg, bool use_absolute) {
    double roll_rad = math_utils::degreesToRadians(roll_deg);
    double pitch_rad = math_utils::degreesToRadians(pitch_deg);

    IMUData data{};
    data.roll = roll_deg;
    data.pitch = pitch_deg;
    data.yaw = 0.0;

    data.accel_x = -G_MS2 * std::sin(pitch_rad);
    data.accel_y = G_MS2 * std::sin(roll_rad) * std::cos(pitch_rad);
    data.accel_z = G_MS2 * std::cos(roll_rad) * std::cos(pitch_rad);

    data.gyro_x = 0.0;
    data.gyro_y = 0.0;
    data.gyro_z = 0.0;
    data.is_valid = true;
    data.mode = use_absolute ? IMU_MODE_ABSOLUTE_POS : IMU_MODE_RAW_DATA;
    data.has_absolute_capability = use_absolute;

    if (use_absolute) {
        data.absolute_data.absolute_roll = roll_deg;
        data.absolute_data.absolute_pitch = pitch_deg;
        data.absolute_data.absolute_yaw = 0.0;
        data.absolute_data.linear_accel_x = 0.0;
        data.absolute_data.linear_accel_y = 0.0;
        data.absolute_data.linear_accel_z = 0.0;

        Eigen::Vector3d euler_rad(roll_rad, pitch_rad, 0.0);
        Eigen::Quaterniond q = math_utils::eulerAnglesToQuaterniond(euler_rad);
        data.absolute_data.quaternion_w = q.w();
        data.absolute_data.quaternion_x = q.x();
        data.absolute_data.quaternion_y = q.y();
        data.absolute_data.quaternion_z = q.z();

        data.absolute_data.absolute_orientation_valid = true;
        data.absolute_data.linear_acceleration_valid = true;
        data.absolute_data.quaternion_valid = true;
        data.absolute_data.calibration_status = 3;
        data.absolute_data.system_status = 5;
        data.absolute_data.self_test_result = 0x0;
    }

    return data;
}

// ────────────────────────────────────────────────────────────────────────────
// Geometry helpers for stability analysis
// ────────────────────────────────────────────────────────────────────────────

/** @brief 2D point for ground-plane geometry. */
struct Vec2 {
    double x, y;
};

/** @brief Minimum distance from a 2D point to a line segment. */
static double pointToSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12) {
        double ex = p.x - a.x, ey = p.y - a.y;
        return std::sqrt(ex * ex + ey * ey);
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = math_utils::clamp(t, 0.0, 1.0);
    double cx = a.x + t * dx - p.x;
    double cy = a.y + t * dy - p.y;
    return std::sqrt(cx * cx + cy * cy);
}

/** @brief Sort 2D points in counter-clockwise convex order around centroid. */
static void convexOrder(std::vector<Vec2> &pts) {
    double cx = 0, cy = 0;
    for (const auto &v : pts) {
        cx += v.x;
        cy += v.y;
    }
    cx /= static_cast<double>(pts.size());
    cy /= static_cast<double>(pts.size());
    std::sort(pts.begin(), pts.end(), [cx, cy](const Vec2 &a, const Vec2 &b) {
        return std::atan2(a.y - cy, a.x - cx) < std::atan2(b.y - cy, b.x - cx);
    });
}

/**
 * @brief Stability margin: minimum distance from a point to any edge
 *        of a convex polygon (the support polygon formed by tip contacts).
 */
static double stabilityMargin(const std::vector<Vec2> &polygon, Vec2 cog) {
    double min_dist = 1e9;
    int n = static_cast<int>(polygon.size());
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double d = pointToSegmentDistance(cog, polygon[i], polygon[j]);
        if (d < min_dist)
            min_dist = d;
    }
    return min_dist;
}

/**
 * @brief Project body CoG onto the ground plane through the gravity vector.
 *
 * When the surface is tilted by (roll, pitch), the gravity vector is no longer
 * perpendicular to the ground. The CoG at height h above ground projects to:
 *   proj_x = body_x + h * tan(pitch)
 *   proj_y = body_y - h * tan(roll)
 *
 * @param body_x Body X position (inclination shift)
 * @param body_y Body Y position (inclination shift)
 * @param height Body height above ground (positive)
 * @param pitch_rad Pitch angle in radians
 * @param roll_rad Roll angle in radians
 */
static Vec2 projectCoG(double body_x, double body_y, double height,
                       double pitch_rad, double roll_rad) {
    return {body_x + height * std::tan(pitch_rad),
            body_y - height * std::tan(roll_rad)};
}

// ────────────────────────────────────────────────────────────────────────────
// Visualization
// ────────────────────────────────────────────────────────────────────────────

static const char *LEG_TAG[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};

/**
 * @brief Print a top-down diagram + per-leg angle table for a single scenario.
 */
static void printRobotVisualization(const std::string &label,
                                    double roll_deg, double pitch_deg,
                                    Vec2 cog_proj,
                                    double stability_margin_val,
                                    const Point3D tips[NUM_LEGS],
                                    const JointAngles angles[NUM_LEGS],
                                    const Pose &body_pose) {
    std::cout << "\n  +----- " << label << " -----+" << std::endl;
    std::cout << "  | Terrain tilt : roll=" << fmt(roll_deg)
              << " deg  pitch=" << fmt(pitch_deg) << " deg" << std::endl;
    std::cout << "  | Body shift   : X=" << fmt(body_pose.position.x)
              << " Y=" << fmt(body_pose.position.y) << " mm" << std::endl;
    std::cout << "  | CoG proj     : X=" << fmt(cog_proj.x)
              << " Y=" << fmt(cog_proj.y) << " mm" << std::endl;
    std::cout << "  | Stab. margin : " << fmt(stability_margin_val) << " mm" << std::endl;
    std::cout << "  |" << std::endl;

    // Top-down hex diagram — map tip positions to an ASCII grid
    const int W = 48, H = 24;
    const int CX = W / 2, CY = H / 2;
    std::vector<std::string> grid(H, std::string(W, ' '));

    // Scale factor from mm to grid cells
    double max_r = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        double r = std::sqrt(tips[i].x * tips[i].x + tips[i].y * tips[i].y);
        if (r > max_r)
            max_r = r;
    }
    double scale = (max_r > 1) ? (std::min(CX, CY) - 2) / max_r : 1.0;

    // Plot tip positions as 2-char leg tags
    for (int i = 0; i < NUM_LEGS; i++) {
        int gx = CX + static_cast<int>(tips[i].y * scale); // Y -> horizontal
        int gy = CY - static_cast<int>(tips[i].x * scale); // X -> vertical (up=+X)
        gx = static_cast<int>(math_utils::clamp(static_cast<double>(gx), 0.0, static_cast<double>(W - 3)));
        gy = static_cast<int>(math_utils::clamp(static_cast<double>(gy), 0.0, static_cast<double>(H - 1)));
        grid[gy][gx] = LEG_TAG[i][0];
        if (gx + 1 < W)
            grid[gy][gx + 1] = LEG_TAG[i][1];
    }

    // Plot CoG projection as '*'
    {
        int gx = CX + static_cast<int>(cog_proj.y * scale);
        int gy = CY - static_cast<int>(cog_proj.x * scale);
        gx = static_cast<int>(math_utils::clamp(static_cast<double>(gx), 0.0, static_cast<double>(W - 1)));
        gy = static_cast<int>(math_utils::clamp(static_cast<double>(gy), 0.0, static_cast<double>(H - 1)));
        grid[gy][gx] = '*';
    }

    // Plot body center as '+'
    grid[CY][CX] = '+';

    for (const auto &row : grid) {
        std::cout << "  | " << row << std::endl;
    }

    // Per-leg angle table
    std::cout << "  |" << std::endl;
    std::cout << "  | Leg  Coxa(deg) Femur(deg) Tibia(deg)    Tip X      Tip Y      Tip Z" << std::endl;
    std::cout << "  | ---  --------- ---------- ---------- ---------  ---------  ---------" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "  |  %s   %8.2f   %8.2f   %8.2f  %9.2f  %9.2f  %9.2f",
                      LEG_TAG[i],
                      math_utils::radiansToDegrees(angles[i].coxa),
                      math_utils::radiansToDegrees(angles[i].femur),
                      math_utils::radiansToDegrees(angles[i].tibia),
                      tips[i].x, tips[i].y, tips[i].z);
        std::cout << line << std::endl;
    }
    std::cout << "  +----------------------------------------------+" << std::endl;
}

// ────────────────────────────────────────────────────────────────────────────
// Scenario runner — physical validation core
// ────────────────────────────────────────────────────────────────────────────

struct ScenarioResult {
    bool all_ik_valid = true;
    double margin_with = 0;    ///< Stability margin WITH compensation
    double margin_without = 0; ///< Stability margin WITHOUT compensation
    Vec2 cog_with = {0, 0};    ///< CoG projection WITH
    Vec2 cog_without = {0, 0}; ///< CoG projection WITHOUT
    JointAngles angles_with[NUM_LEGS];
    JointAngles angles_without[NUM_LEGS];
    Point3D tips_with[NUM_LEGS];
    Point3D tips_without[NUM_LEGS];
    Pose pose_with;
    Pose pose_without;
};

/**
 * @brief Run one inclination scenario comparing WITH vs WITHOUT compensation.
 *
 * Steps:
 * 1. Set standing pose -> get standing tip positions (= support polygon)
 * 2. Create BPC WITHOUT inclination -> pose is identity -> tips unchanged
 * 3. Create BPC WITH inclination -> apply body shift -> IK for all legs
 * 4. Project CoG for both cases onto tilted ground plane
 * 5. Compute stability margins
 * 6. Print visualization
 */
static ScenarioResult runScenario(const Parameters &p, RobotModel &model,
                                  const BodyPoseConfiguration &base_config,
                                  double roll_deg, double pitch_deg,
                                  bool use_absolute, bool visualize) {
    ScenarioResult result;

    double roll_rad = math_utils::degreesToRadians(roll_deg);
    double pitch_rad = math_utils::degreesToRadians(pitch_deg);
    double height = base_config.body_clearance;

    // ── WITHOUT compensation ──────────────────────────────────────────────
    {
        Leg legs_nc[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                                 Leg(3, model), Leg(4, model), Leg(5, model)};
        BodyPoseConfiguration nc_config = base_config;
        nc_config.inclination_posing_enabled = false;
        BodyPoseController pc_nc(model, nc_config);
        testSetStandingPose(pc_nc, model, legs_nc);

        IMUData imu = buildTiltedIMUData(roll_deg, pitch_deg, use_absolute);
        pc_nc.setIMUData(imu);
        pc_nc.updateCurrentPose(2, legs_nc);

        result.pose_without = pc_nc.getCurrentBodyPose();
        for (int i = 0; i < NUM_LEGS; i++) {
            result.tips_without[i] = legs_nc[i].getCurrentTipPositionGlobal();
            result.angles_without[i] = legs_nc[i].getJointAngles();
        }
        result.cog_without = projectCoG(result.pose_without.position.x,
                                        result.pose_without.position.y,
                                        height, pitch_rad, roll_rad);
    }

    // ── WITH compensation ─────────────────────────────────────────────────
    {
        Leg legs_wc[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                                 Leg(3, model), Leg(4, model), Leg(5, model)};
        BodyPoseConfiguration wc_config = base_config;
        wc_config.inclination_posing_enabled = true;
        BodyPoseController pc_wc(model, wc_config);
        testSetStandingPose(pc_wc, model, legs_wc);

        // Store standing tips BEFORE pose application (support polygon reference)
        Point3D standing_tips[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++) {
            standing_tips[i] = legs_wc[i].getCurrentTipPositionGlobal();
        }

        IMUData imu = buildTiltedIMUData(roll_deg, pitch_deg, use_absolute);
        pc_wc.setIMUData(imu);
        pc_wc.updateCurrentPose(2, legs_wc);

        result.pose_with = pc_wc.getCurrentBodyPose();

        // Apply the compensated body pose to standing tips via IK
        for (int i = 0; i < NUM_LEGS; i++) {
            Point3D posed_tip = result.pose_with.inverseTransformVector(standing_tips[i]);
            bool ik_ok = legs_wc[i].applyIK(posed_tip);
            if (!ik_ok)
                result.all_ik_valid = false;
            result.tips_with[i] = standing_tips[i]; // global tip position unchanged
            result.angles_with[i] = legs_wc[i].getJointAngles();
        }
        result.cog_with = projectCoG(result.pose_with.position.x,
                                     result.pose_with.position.y,
                                     height, pitch_rad, roll_rad);
    }

    // ── Build support polygon from standing tips ──────────────────────────
    std::vector<Vec2> polygon;
    for (int i = 0; i < NUM_LEGS; i++) {
        polygon.push_back({result.tips_without[i].x, result.tips_without[i].y});
    }
    convexOrder(polygon);

    result.margin_without = stabilityMargin(polygon, result.cog_without);
    result.margin_with = stabilityMargin(polygon, result.cog_with);

    // ── Visualization ─────────────────────────────────────────────────────
    if (visualize) {
        printRobotVisualization("WITHOUT compensation", roll_deg, pitch_deg,
                                result.cog_without, result.margin_without,
                                result.tips_without, result.angles_without,
                                result.pose_without);
        printRobotVisualization("WITH compensation", roll_deg, pitch_deg,
                                result.cog_with, result.margin_with,
                                result.tips_with, result.angles_with,
                                result.pose_with);
    }

    return result;
}

// ────────────────────────────────────────────────────────────────────────────
// Main tests
// ────────────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Gravity Inclination Physical Validation Test ===" << std::endl;
    std::cout << "Validates that inclination compensation PHYSICALLY improves" << std::endl;
    std::cout << "static stability (CoG projection closer to support polygon center)." << std::endl;
    std::cout << std::endl;

    Parameters p = createDefaultParameters();
    p.inclination_posing = true;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer();

    BodyPoseConfiguration config = getDefaultBodyPoseConfig(p);
    config.inclination_posing_enabled = true;

    const double body_clearance = config.body_clearance;
    const double max_tx = config.max_translation.x;
    const double max_ty = config.max_translation.y;

    const double clamp_angle_deg = math_utils::radiansToDegrees(
        std::atan(max_tx / body_clearance));

    std::cout << "  body_clearance = " << body_clearance << " mm" << std::endl;
    std::cout << "  max_translation = {" << max_tx << ", " << max_ty << "} mm" << std::endl;
    std::cout << "  clamp_angle = " << fmt(clamp_angle_deg) << " deg" << std::endl;
    std::cout << std::endl;

    // PRNG with fixed seed
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> sub_clamp_dist(1.0, clamp_angle_deg - 0.5);
    std::uniform_real_distribution<double> super_clamp_dist(clamp_angle_deg + 1.0, 40.0);
    std::uniform_real_distribution<double> full_dist(1.0, 40.0);

    // ======================================================================
    // TEST 1: Level baseline — no shift, identical margins
    // ======================================================================
    std::cout << "--- Test 1: Level baseline (0 deg tilt) ---" << std::endl;
    {
        auto r = runScenario(p, model, config, 0.0, 0.0, false, true);
        checkResult(std::abs(r.pose_with.position.x) < 0.5,
                    "Level: body X shift ~0 (got " + fmt(r.pose_with.position.x) + ")");
        checkResult(std::abs(r.pose_with.position.y) < 0.5,
                    "Level: body Y shift ~0 (got " + fmt(r.pose_with.position.y) + ")");
        checkResult(std::abs(r.margin_with - r.margin_without) < 0.5,
                    "Level: margins equal (" + fmt(r.margin_with) + " vs " +
                        fmt(r.margin_without) + ")");
        checkResult(r.all_ik_valid, "Level: all IK solutions valid");
    }

    // ======================================================================
    // TEST 2: Sub-clamping cardinal tilts — physical stability improvement
    // ======================================================================
    std::cout << "\n--- Test 2: Sub-clamping cardinal inclinations (physical validation) ---"
              << std::endl;
    {
        struct Case {
            std::string label;
            double roll;
            double pitch;
        };
        double a1 = sub_clamp_dist(rng), a2 = sub_clamp_dist(rng);
        double a3 = sub_clamp_dist(rng), a4 = sub_clamp_dist(rng);
        Case cases[] = {
            {"North (pitch +" + fmt(a1) + " deg)", 0.0, a1},
            {"South (pitch -" + fmt(a2) + " deg)", 0.0, -a2},
            {"East (roll +" + fmt(a3) + " deg)", a3, 0.0},
            {"West (roll -" + fmt(a4) + " deg)", -a4, 0.0},
        };

        for (const auto &tc : cases) {
            std::cout << "  -- " << tc.label << " --" << std::endl;
            auto r = runScenario(p, model, config, tc.roll, tc.pitch, false, true);

            // Physical check 1: CoG projection closer to center with compensation
            double dist_without = std::sqrt(r.cog_without.x * r.cog_without.x +
                                            r.cog_without.y * r.cog_without.y);
            double dist_with = std::sqrt(r.cog_with.x * r.cog_with.x +
                                         r.cog_with.y * r.cog_with.y);
            checkResult(dist_with < dist_without,
                        tc.label + ": CoG closer to center WITH comp (" +
                            fmt(dist_with) + " < " + fmt(dist_without) + " mm)");

            // Physical check 2: stability margin improves
            checkResult(r.margin_with > r.margin_without,
                        tc.label + ": stability margin improves (" +
                            fmt(r.margin_with) + " > " + fmt(r.margin_without) + " mm)");

            // Physical check 3: all IK valid
            checkResult(r.all_ik_valid, tc.label + ": all IK solutions valid");
        }
    }

    // ======================================================================
    // TEST 3: Super-clamping — still improves stability even when clamped
    // ======================================================================
    std::cout << "\n--- Test 3: Super-clamping cardinal inclinations ---" << std::endl;
    {
        struct Case {
            std::string label;
            double roll;
            double pitch;
        };
        double a1 = super_clamp_dist(rng), a2 = super_clamp_dist(rng);
        Case cases[] = {
            {"Forward steep (pitch +" + fmt(a1) + " deg)", 0.0, a1},
            {"Rightward steep (roll +" + fmt(a2) + " deg)", a2, 0.0},
        };

        for (const auto &tc : cases) {
            std::cout << "  -- " << tc.label << " --" << std::endl;
            auto r = runScenario(p, model, config, tc.roll, tc.pitch, false, true);

            // Even clamped, compensation still improves stability
            checkResult(r.margin_with >= r.margin_without - 0.1,
                        tc.label + ": stability margin no worse (" +
                            fmt(r.margin_with) + " >= " + fmt(r.margin_without) + ")");
            checkResult(r.all_ik_valid, tc.label + ": all IK solutions valid");

            // Verify body shift is clamped within limits
            double body_shift_mag = std::sqrt(
                r.pose_with.position.x * r.pose_with.position.x +
                r.pose_with.position.y * r.pose_with.position.y);
            checkResult(body_shift_mag <= std::sqrt(max_tx * max_tx + max_ty * max_ty) + 0.5,
                        tc.label + ": body shift within clamped limits (" +
                            fmt(body_shift_mag) + " mm)");
        }
    }

    // ======================================================================
    // TEST 4: Absolute IMU mode (BNO055) — same physical improvement
    // ======================================================================
    std::cout << "\n--- Test 4: Absolute IMU mode (BNO055) ---" << std::endl;
    {
        double roll = sub_clamp_dist(rng), pitch = sub_clamp_dist(rng);
        std::cout << "  Tilt: roll=" << fmt(roll) << " deg  pitch=" << fmt(pitch)
                  << " deg" << std::endl;

        auto r_abs = runScenario(p, model, config, roll, pitch, true, true);
        auto r_raw = runScenario(p, model, config, roll, pitch, false, false);

        // Absolute mode gives same improvement
        checkResult(r_abs.margin_with > r_abs.margin_without,
                    "Absolute: margin improves (" + fmt(r_abs.margin_with) +
                        " > " + fmt(r_abs.margin_without) + ")");
        checkResult(r_abs.all_ik_valid, "Absolute: all IK valid");

        // Raw and absolute produce same body shift
        checkResult(std::abs(r_abs.pose_with.position.x - r_raw.pose_with.position.x) < 0.5,
                    "Raw vs Abs: X shift match (" + fmt(r_abs.pose_with.position.x) +
                        " vs " + fmt(r_raw.pose_with.position.x) + ")");
        checkResult(std::abs(r_abs.pose_with.position.y - r_raw.pose_with.position.y) < 0.5,
                    "Raw vs Abs: Y shift match");
    }

    // ======================================================================
    // TEST 5: Diagonal inclinations — combined pitch + roll
    // ======================================================================
    std::cout << "\n--- Test 5: Diagonal inclinations (combined pitch+roll) ---" << std::endl;
    {
        double a1 = sub_clamp_dist(rng), a2 = sub_clamp_dist(rng);
        struct Case {
            std::string label;
            double roll;
            double pitch;
        };
        Case cases[] = {
            {"NE (+" + fmt(a1) + " deg both)", a1, a1},
            {"SW (-" + fmt(a2) + " deg both)", -a2, -a2},
        };
        for (const auto &tc : cases) {
            std::cout << "  -- " << tc.label << " --" << std::endl;
            auto r = runScenario(p, model, config, tc.roll, tc.pitch, false, true);

            double dist_without = std::sqrt(r.cog_without.x * r.cog_without.x +
                                            r.cog_without.y * r.cog_without.y);
            double dist_with = std::sqrt(r.cog_with.x * r.cog_with.x +
                                         r.cog_with.y * r.cog_with.y);
            checkResult(dist_with < dist_without,
                        tc.label + ": CoG closer WITH comp (" +
                            fmt(dist_with) + " < " + fmt(dist_without) + ")");
            checkResult(r.margin_with > r.margin_without,
                        tc.label + ": margin improves (" +
                            fmt(r.margin_with) + " > " + fmt(r.margin_without) + ")");
            checkResult(r.all_ik_valid, tc.label + ": all IK valid");
        }
    }

    // ======================================================================
    // TEST 6: Symmetry — opposite tilts produce opposite body shifts
    // ======================================================================
    std::cout << "\n--- Test 6: Symmetry (opposite tilts = opposite shifts) ---" << std::endl;
    {
        double pitch_a = sub_clamp_dist(rng);
        double roll_a = sub_clamp_dist(rng);

        auto r_fwd = runScenario(p, model, config, 0.0, pitch_a, false, false);
        auto r_bwd = runScenario(p, model, config, 0.0, -pitch_a, false, false);
        auto r_right = runScenario(p, model, config, roll_a, 0.0, false, false);
        auto r_left = runScenario(p, model, config, -roll_a, 0.0, false, false);

        checkResult(std::abs(r_fwd.pose_with.position.x + r_bwd.pose_with.position.x) < 0.5,
                    "Pitch symmetry: +" + fmt(pitch_a) + " deg vs -" + fmt(pitch_a) +
                        " deg shifts cancel");
        checkResult(std::abs(r_right.pose_with.position.y + r_left.pose_with.position.y) < 0.5,
                    "Roll symmetry: +" + fmt(roll_a) + " deg vs -" + fmt(roll_a) +
                        " deg shifts cancel");

        // Equal stability margins for symmetric tilts
        checkResult(std::abs(r_fwd.margin_with - r_bwd.margin_with) < 0.5,
                    "Pitch symmetry: equal margins (" + fmt(r_fwd.margin_with) +
                        " vs " + fmt(r_bwd.margin_with) + ")");
        checkResult(std::abs(r_right.margin_with - r_left.margin_with) < 0.5,
                    "Roll symmetry: equal margins");
    }

    // ======================================================================
    // TEST 7: Inclination disabled — no shift regardless of tilt
    // ======================================================================
    std::cout << "\n--- Test 7: Inclination disabled ---" << std::endl;
    {
        double angle = full_dist(rng);
        std::cout << "  Tilt angle = " << fmt(angle) << " deg (should be ignored)" << std::endl;

        Leg legs_dis[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                                  Leg(3, model), Leg(4, model), Leg(5, model)};
        BodyPoseConfiguration disabled_config = config;
        disabled_config.inclination_posing_enabled = false;
        BodyPoseController pc_dis(model, disabled_config);
        testSetStandingPose(pc_dis, model, legs_dis);

        IMUData imu = buildTiltedIMUData(angle, angle, false);
        pc_dis.setIMUData(imu);
        pc_dis.updateCurrentPose(2, legs_dis);

        const Pose &pose = pc_dis.getCurrentBodyPose();
        checkResult(std::abs(pose.position.x) < 0.5,
                    "Disabled: no X shift (got " + fmt(pose.position.x) + ")");
        checkResult(std::abs(pose.position.y) < 0.5,
                    "Disabled: no Y shift (got " + fmt(pose.position.y) + ")");
    }

    // ======================================================================
    // TEST 8: Invalid IMU data — graceful fallback
    // ======================================================================
    std::cout << "\n--- Test 8: Invalid IMU data (graceful fallback) ---" << std::endl;
    {
        Leg legs_inv[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model),
                                  Leg(3, model), Leg(4, model), Leg(5, model)};
        BodyPoseController pc_inv(model, config);
        testSetStandingPose(pc_inv, model, legs_inv);

        IMUData invalid_data{};
        invalid_data.is_valid = false;
        pc_inv.setIMUData(invalid_data);
        pc_inv.updateCurrentPose(2, legs_inv);

        const Pose &pose = pc_inv.getCurrentBodyPose();
        checkResult(std::abs(pose.position.x) < 0.5,
                    "Invalid IMU: X shift ~0 (" + fmt(pose.position.x) + ")");
        checkResult(std::abs(pose.position.y) < 0.5,
                    "Invalid IMU: Y shift ~0 (" + fmt(pose.position.y) + ")");
    }

    // ======================================================================
    // TEST 9: Monotonicity — larger tilt -> larger body shift
    // ======================================================================
    std::cout << "\n--- Test 9: Monotonicity (larger tilt -> larger body shift) ---" << std::endl;
    {
        std::vector<double> angles;
        for (int i = 0; i < 5; i++)
            angles.push_back(sub_clamp_dist(rng));
        std::sort(angles.begin(), angles.end());

        double prev_shift = 0.0;
        for (double angle : angles) {
            auto r = runScenario(p, model, config, 0.0, angle, false, false);
            double shift = std::abs(r.pose_with.position.x);

            checkResult(shift >= prev_shift - 0.1,
                        "Monotonic at " + fmt(angle) + " deg: shift " +
                            fmt(shift) + " >= " + fmt(prev_shift) + " mm");
            checkResult(r.margin_with > r.margin_without - 0.1,
                        "Monotonic at " + fmt(angle) + " deg: margin improves");

            prev_shift = shift;
        }
    }

    // ======================================================================
    // TEST 10: Near-perfect compensation at sub-clamping angles
    //   body_clearance == standing_height => shift exactly cancels gravity offset
    //   => compensated CoG projection returns to ~(0,0)
    // ======================================================================
    std::cout << "\n--- Test 10: Near-perfect compensation at sub-clamping ---" << std::endl;
    {
        double angle = sub_clamp_dist(rng);
        auto r = runScenario(p, model, config, 0.0, angle, false, true);

        checkResult(std::abs(r.cog_with.x) < 1.0,
                    "Sub-clamp pitch " + fmt(angle) + " deg: compensated CoG proj X ~0 (got " +
                        fmt(r.cog_with.x) + ")");
        checkResult(r.margin_with > r.margin_without,
                    "Sub-clamp: margin WITH > WITHOUT (" +
                        fmt(r.margin_with) + " > " + fmt(r.margin_without) + ")");
    }

    // ── Summary ──────────────────────────────────────────────────────────────
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Passed: " << tests_passed << std::endl;
    std::cout << "Failed: " << tests_failed << std::endl;

    if (tests_failed > 0) {
        std::cerr << "GRAVITY INCLINATION TEST FAILED" << std::endl;
        return 1;
    }

    std::cout << "ALL GRAVITY INCLINATION TESTS PASSED" << std::endl;
    return 0;
}
