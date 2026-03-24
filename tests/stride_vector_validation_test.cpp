/**
 * @file stride_vector_validation_test.cpp
 * @brief Validates that the stride vector calculation in HexaMotion
 *        faithfully reproduces OpenSHC's logic and, consequently, the expected
 *        tangential coxa movement (angular + linear components).
 *
 * Methodology:
 *  1. Builds a LegStepper in its initial state where current_tip_pose == default_tip_pose.
 *  2. Applies linear velocities (vx, vy) and angular velocity (omega_z).
 *  3. Calls HexaMotion's updateStride().
 *  4. Recomputes the "expected" stride vector using the OpenSHC formula:
 *       stride_linear  = (vx, vy, 0)
 *       radius         = tip rejection onto Z axis  (=> (x, y, 0) with z removed)
 *       stride_angular = omega_z * k̂  X  radius = (-omega_z * y, omega_z * x, 0)
 *       stride_total   = (stride_linear + stride_angular) * (on_ground_ratio / frequency)
 *  5. Compares each component with the HexaMotion result.
 *  6. Repeats with various cases (linear only, angular only, combined, different radii and signs).
 *
 * Success: maximum error < 1e-9 (strict tolerance because the calculations are deterministic).
 */

#include "gait_config.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

struct StrideTestCase {
    Point3D identity_tip;    // Identity position (initial default/current)
    Point3D linear_velocity; // (vx, vy, 0)
    double angular_velocity; // omega_z (rad/s)
    double frequency;        // step_cycle_.frequency_
    int stance_period;       // step_cycle_.stance_period_
    int swing_period;        // step_cycle_.swing_period_
    std::string name;        // Label
};

// Forward declaration of the OpenSHC calculation for use in validateAllLegs
static Point3D computeExpectedOpenSHCStride(const StrideTestCase &tc);

// Executes the same validation logic on all 6 legs using each one's default position.
static bool validateAllLegs(const StrideTestCase &tc, RobotModel &model, const Parameters &params, double tol) {
    std::cout << "  [SubTest] Radial validation and hexagonal symmetry (6 coxas)" << std::endl;
    bool all_ok = true;
    // Build Leg objects (one per index)
    std::vector<std::unique_ptr<Leg>> legs;
    legs.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs.emplace_back(std::make_unique<Leg>(i, model));
    }

    // For zero angles: the planar reach to the tip includes hexagon_radius + coxa_length + femur_length.
    double r = params.hexagon_radius + params.coxa_length + params.femur_length;
    double z0 = params.default_height_offset;

    // For each leg, build the analytic identity: (r cos(theta_i), r sin(theta_i), z0) using BASE_THETA_OFFSETS
    // Storage for tangential symmetry (populated inside the loop)
    std::vector<Point3D> angular_components(NUM_LEGS, Point3D(0, 0, 0));
    std::vector<double> angular_mags(NUM_LEGS, 0.0);
    std::vector<double> coxa_deltas(NUM_LEGS, 0.0);

    for (int i = 0; i < NUM_LEGS; ++i) {
        double theta = BASE_THETA_OFFSETS[i];
        Point3D analytic_identity(r * std::cos(theta), r * std::sin(theta), z0);

        // Position obtained by FK (built internally by the model with the same DH offsets)
        Point3D fk_default = model.getLegDefaultPosition(i);
        fk_default.z = z0; // normalize height to compare XY plane only
        double dx = fk_default.x - analytic_identity.x;
        double dy = fk_default.y - analytic_identity.y;
        double planar_geom_err = std::sqrt(dx * dx + dy * dy);

        // Create specific stepper
        LegStepper stepper(i, analytic_identity, *legs[i], model);

        StepCycle cycle{};
        cycle.frequency_ = tc.frequency;
        cycle.period_ = tc.stance_period + tc.swing_period;
        cycle.stance_period_ = tc.stance_period;
        cycle.swing_period_ = tc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = tc.stance_period;
        cycle.swing_start_ = tc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepper.setStepCycle(cycle);

        stepper.setDesiredVelocity(tc.linear_velocity, tc.angular_velocity);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.updateStride();
        // Instrumentation layer getters (raw/arc_pre/post) no longer exist.
        // We directly obtain the final calculated stride (after safety validations if applicable).
        Point3D got = stepper.getStrideVector();

        // Note: The current version of LegStepper::updateStride directly implements the combined formula
        // (v + ω×r) * (stance_ratio / frequency) without publicly exposed intermediate stages; therefore
        // the E0/E1/E2 analysis is removed and only the comparison against the expected OpenSHC model remains.

        // Recalculate expected stride using analytic_identity as the planar radius
        StrideTestCase local_tc = tc;
        local_tc.identity_tip = analytic_identity;
        Point3D expected = computeExpectedOpenSHCStride(local_tc);
        Point3D diff = got - expected;
        double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

        double period = tc.stance_period + tc.swing_period;
        double on_ground_ratio = (period > 0) ? (double)tc.stance_period / period : 0.0;
        Point3D scaled_linear = tc.linear_velocity * (on_ground_ratio / tc.frequency);
        Point3D angular_component_expected = expected - scaled_linear;
        Point3D angular_component_got = got - scaled_linear;
        Point3D angular_diff = angular_component_got - angular_component_expected;
        double angular_err = std::sqrt(angular_diff.x * angular_diff.x + angular_diff.y * angular_diff.y + angular_diff.z * angular_diff.z);

        // Coxa delta and approximate tangential translation validation
        double stance_ratio = on_ground_ratio;
        double coxa_delta_expected = tc.angular_velocity * (stance_ratio / tc.frequency);
        Point3D radius_vec(analytic_identity.x, analytic_identity.y, 0.0);
        double radius_norm = std::sqrt(radius_vec.x * radius_vec.x + radius_vec.y * radius_vec.y);
        double coxa_delta_got = 0.0;
        bool coxa_valid = radius_norm > 1e-9;
        if (coxa_valid) {
            Point3D tangent_unit(-radius_vec.y / radius_norm, radius_vec.x / radius_norm, 0.0);
            double arc_len = angular_component_got.x * tangent_unit.x + angular_component_got.y * tangent_unit.y;
            coxa_delta_got = arc_len / radius_norm;
        }
        double coxa_err = std::fabs(coxa_delta_got - coxa_delta_expected);

        // (New) Tangential validations:
        //  a) Orthogonality: angular component ⋅ radius ≈ 0
        //  b) Magnitude: |stride_angular| ≈ |coxa_delta_expected| * radius_norm
        double tangential_dot = angular_component_got.x * radius_vec.x + angular_component_got.y * radius_vec.y; // should be ~0
        double tangential_dot_abs = std::fabs(tangential_dot);
        double expected_arc_len = std::fabs(coxa_delta_expected) * radius_norm;
        double got_arc_len = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        double arc_len_err = std::fabs(got_arc_len - expected_arc_len);
        double tangential_tol = tol * std::max(1.0, radius_norm);
        double arc_len_tol = tol * std::max(1.0, radius_norm);

        // mm to degrees ratio (linear planar scaled, shared across all legs)
        double linear_planar_mm = std::sqrt(scaled_linear.x * scaled_linear.x + scaled_linear.y * scaled_linear.y);
        double coxa_delta_deg_exp = math_utils::radiansToDegrees(coxa_delta_expected);
        double coxa_delta_deg_got = math_utils::radiansToDegrees(coxa_delta_got);

        // Store for symmetry analysis (only the pure angular part is considered)
        angular_components[i] = angular_component_got;
        angular_mags[i] = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        coxa_deltas[i] = coxa_delta_got;

        bool tangential_ok = (!coxa_valid) || (tangential_dot_abs <= tangential_tol && arc_len_err <= arc_len_tol);
        bool pass = (err <= tol && angular_err <= tol && (!coxa_valid || coxa_err <= tol) && planar_geom_err <= tol && tangential_ok);
        std::cout << "    Leg " << i
                  << ": stride_err=" << err
                  << " ang_err=" << angular_err
                  << " coxa_err=" << coxa_err
                  << " geom_err=" << planar_geom_err
                  << " tangential_dot=" << tangential_dot_abs
                  << " arc_len_err=" << arc_len_err
                  << " | linear(mm)=" << linear_planar_mm
                  << " coxaΔexp(deg)=" << coxa_delta_deg_exp
                  << " coxaΔgot(deg)=" << coxa_delta_deg_got
                  << (pass ? " ✓" : " ❌") << std::endl;
        if (!pass)
            all_ok = false;
    }

    // ================= Symmetry Validation (180° Opposition and 60° Reflection) =================
    // NEW: The two notions are separated.
    //  A) Opposition (pairs truly separated by 180° in the hexagon): (0,3), (1,4), (2,5)
    //     AR(+30°)↔CL(-150°), BR(+90°)↔BL(-90°), CR(+150°)↔AL(-30°)
    //  B) Reflection (mirror pairs by index, separated ~60°): (0,5), (1,4), (2,3)

    struct Pair {
        int a;
        int b;
        const char *label;
    };

    // ---- Block A: True opposition (we expect opposite angular vectors -> dot ≈ -1) ----
    Pair opposite_pairs[3] = {{0, 3, "(0,3)"}, {1, 4, "(1,4)"}, {2, 5, "(2,5)"}};
    std::cout << "    BaseAngles(deg):";
    for (int i = 0; i < NUM_LEGS; ++i) {
        double deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[i]);
        std::cout << " L" << i << "=" << std::fixed << std::setprecision(1) << deg;
    }
    std::cout << std::endl;
    std::cout << "    Symmetry Opposite Pairs (Δθ≈180° -> dot≈-1):" << std::endl;
    double dir_tol = 1e-6; // strict directional tolerance
    for (const auto &p : opposite_pairs) {
        Point3D va = angular_components[p.a];
        Point3D vb = angular_components[p.b];
        double ma = angular_mags[p.a];
        double mb = angular_mags[p.b];
        bool trivial = (ma < 1e-12 && mb < 1e-12);
        double rel_mag_err = 0.0, dir_dot_norm = 0.0;
        bool mag_ok = true, dir_ok = true;
        if (!trivial) {
            rel_mag_err = std::fabs(ma - mb) / std::max(1e-12, (ma + mb) * 0.5);
            if (ma > 0 && mb > 0)
                dir_dot_norm = (va.x * vb.x + va.y * vb.y + va.z * vb.z) / (ma * mb);
            mag_ok = (rel_mag_err <= 1e-9 || trivial);
            dir_ok = (std::fabs(dir_dot_norm + 1.0) <= dir_tol) || trivial;
        }
        bool pair_ok = trivial || (mag_ok && dir_ok);
        double theta_a_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.a]);
        double theta_b_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.b]);
        double delta_theta = std::fmod(std::fabs(theta_a_deg - theta_b_deg), 360.0);
        if (delta_theta > 180.0)
            delta_theta = 360.0 - delta_theta;
        std::cout << "      OppPair " << p.label
                  << " (θa=" << theta_a_deg << ", θb=" << theta_b_deg << ", Δθ=" << delta_theta << ")"
                  << " |mag_a|=" << ma << " |mag_b|=" << mb
                  << " rel_mag_err=" << rel_mag_err
                  << " dir_dot(expect -1)=" << dir_dot_norm
                  << (trivial ? " (trivial: no rotation)" : "")
                  << (pair_ok ? " ✓" : " ❌") << std::endl;
        if (!pair_ok)
            all_ok = false; // Only this block affects the global result
    }

    // ---- Block B: Reflection (informational, does NOT affect all_ok) ----
    // Mirror pairs by index (0↔5, 1↔4, 2↔3): separated ~60°, not 180°.
    // Equal magnitudes but directions NOT necessarily opposite — informational only.
    Pair reflection_pairs[3] = {{0, 5, "(0,5)"}, {1, 4, "(1,4)"}, {2, 3, "(2,3)"}};
    std::cout << "    Symmetry Reflection Pairs (Δθ≈60° or 180° -> dot≈+1 expected):" << std::endl;
    for (const auto &p : reflection_pairs) {
        Point3D va = angular_components[p.a];
        Point3D vb = angular_components[p.b];
        double ma = angular_mags[p.a];
        double mb = angular_mags[p.b];
        bool trivial = (ma < 1e-12 && mb < 1e-12);
        double rel_mag_err = 0.0, dir_dot_norm = 0.0;
        bool mag_ok = true, dir_ok = true;
        if (!trivial) {
            rel_mag_err = std::fabs(ma - mb) / std::max(1e-12, (ma + mb) * 0.5);
            if (ma > 0 && mb > 0)
                dir_dot_norm = (va.x * vb.x + va.y * vb.y + va.z * vb.z) / (ma * mb);
            // relaxed criterion: nearly equal magnitudes and dot close to +1 (±0.5 margin for linear mixing)
            mag_ok = (rel_mag_err <= 1e-9 || trivial);
            dir_ok = (dir_dot_norm >= 0.3) || trivial; // informational only
        }
        double theta_a_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.a]);
        double theta_b_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.b]);
        double delta_theta = std::fmod(std::fabs(theta_a_deg - theta_b_deg), 360.0);
        if (delta_theta > 180.0)
            delta_theta = 360.0 - delta_theta;
        std::cout << "      ReflPair " << p.label
                  << " (θa=" << theta_a_deg << ", θb=" << theta_b_deg << ", Δθ=" << delta_theta << ")"
                  << " |mag_a|=" << ma << " |mag_b|=" << mb
                  << " rel_mag_err=" << rel_mag_err
                  << " dir_dot(expect +)=" << dir_dot_norm
                  << (trivial ? " (trivial: no rotation)" : "")
                  << ((mag_ok && dir_ok) ? " (info ✓)" : " (info ⚠)") << std::endl;
    }

    // Similar magnitude between axial blocks (informational, does not affect all_ok)
    double avg05 = 0.5 * (angular_mags[0] + angular_mags[5]);
    double avg23 = 0.5 * (angular_mags[2] + angular_mags[3]);
    double rel_block_err = std::fabs(avg05 - avg23) / std::max(1e-12, (avg05 + avg23) * 0.5);
    std::cout << "      Block magnitudes opos ( (0,5) vs (2,3) ) avg05=" << avg05 << " avg23=" << avg23
              << " rel_err=" << rel_block_err << " (info)" << std::endl;

    return all_ok;
}

// ---------------------------------------------------------------------------
// New validation: demonstrates that the current algorithm assumes rotation around
// the GLOBAL origin instead of the true geometric center of the body.
// Idea: the entire hexagon is translated by a vector 'shift'. If the rotation
// were correctly centered, the expected angular stride (ω×r_rel) would NOT
// change when adding a constant shift to the center (because r_rel = p_i - center).
// However, the current code uses absolute (x,y) directly => a constant
// bias = ω × shift appears and is added to ALL legs.
// We verify:
//  1) got_stride - expected_center_stride is equal (≈) for all legs.
//  2) That common vector matches bias = ω×shift * (stance_ratio/frequency).
//  3) After subtracting bias, each leg satisfies the correct centered formula.
// This confirms: "the current stride is internally consistent, but assumes
// a center and a homogeneous frame that is not respected downstream".
// ---------------------------------------------------------------------------
static bool validateFrameCenterAssumption(const StrideTestCase &baseTc, RobotModel &model, const Parameters &params, const Point3D &shift, double tol) {
    std::cout << "\n  [FrameTest] Applied shift = (" << shift.x << ", " << shift.y << ") mm" << std::endl;

    // Prepara legs
    std::vector<std::unique_ptr<Leg>> legs;
    legs.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs.emplace_back(std::make_unique<Leg>(i, model));
    }

    double r = params.hexagon_radius + params.coxa_length + params.femur_length; // analytic radius
    double z0 = params.default_height_offset;
    double period = baseTc.stance_period + baseTc.swing_period;
    double stance_ratio = (period > 0.0) ? (double)baseTc.stance_period / period : 0.0;
    double scale = (stance_ratio / baseTc.frequency); // common time factor

    // Shifted geometric center (ideal)
    Point3D center = Point3D(shift.x, shift.y, z0);
    // Theoretical bias induced by using the incorrect global origin
    // ω×shift (ω on k̂) => (-ω*shift.y, ω*shift.x, 0)
    Point3D bias(-baseTc.angular_velocity * shift.y, baseTc.angular_velocity * shift.x, 0.0);
    bias = bias * scale; // scaled by stance time

    std::vector<Point3D> diffs;
    diffs.reserve(NUM_LEGS);
    bool all_ok = true;

    for (int i = 0; i < NUM_LEGS; ++i) {
        double theta = BASE_THETA_OFFSETS[i];
        Point3D identity_unshifted(r * std::cos(theta), r * std::sin(theta), z0);
        Point3D identity_shifted(identity_unshifted.x + shift.x, identity_unshifted.y + shift.y, z0);

        // Build local test case for both
        StrideTestCase tcUn = baseTc;
        tcUn.identity_tip = identity_unshifted;
        StrideTestCase tcSh = baseTc;
        tcSh.identity_tip = identity_shifted;

        // Stepper unshifted
        LegStepper stepperUn(i, identity_unshifted, *legs[i], model);
        StepCycle cycle{};
        cycle.frequency_ = baseTc.frequency;
        cycle.period_ = baseTc.stance_period + baseTc.swing_period;
        cycle.stance_period_ = baseTc.stance_period;
        cycle.swing_period_ = baseTc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = baseTc.stance_period;
        cycle.swing_start_ = baseTc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepperUn.setStepCycle(cycle);
        stepperUn.setDesiredVelocity(baseTc.linear_velocity, baseTc.angular_velocity);
        stepperUn.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepperUn.updateStride();
        Point3D strideUn = stepperUn.getStrideVector();

        // Stepper shifted
        LegStepper stepperSh(i, identity_shifted, *legs[i], model);
        stepperSh.setStepCycle(cycle);
        stepperSh.setDesiredVelocity(baseTc.linear_velocity, baseTc.angular_velocity);
        stepperSh.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepperSh.updateStride();
        Point3D strideSh = stepperSh.getStrideVector();

        Point3D diff = strideSh - strideUn; // should be equal to bias
        diffs.push_back(diff);
        Point3D residual = diff - bias; // close to zero
        double diff_err = std::sqrt((diff.x - bias.x) * (diff.x - bias.x) + (diff.y - bias.y) * (diff.y - bias.y));
        double residual_norm = std::sqrt(residual.x * residual.x + residual.y * residual.y + residual.z * residual.z);
        double bias_mag = std::sqrt(bias.x * bias.x + bias.y * bias.y);
        bool pass_local = (diff_err <= tol * std::max(1.0, bias_mag) && residual_norm <= tol * std::max(1.0, bias_mag));
        if (!pass_local)
            all_ok = false;
        std::cout << "    Leg " << i << " diff=(" << diff.x << "," << diff.y << ") bias=(" << bias.x << "," << bias.y << ") residual=(" << residual.x << "," << residual.y << ")" << (pass_local ? " ✓" : " ❌") << std::endl;
    }

    // Verify that all diffs are (nearly) equal to each other (internal consistency)
    double uniform_err_max = 0.0;
    Point3D ref = diffs[0];
    for (size_t i = 1; i < diffs.size(); ++i) {
        Point3D d = diffs[i] - ref;
        double dn = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        uniform_err_max = std::max(uniform_err_max, dn);
    }
    bool uniform_ok = uniform_err_max <= tol * 1e3; // relaxed tolerance (algorithm may clip stride)
    if (!uniform_ok) {
        std::cout << "    ⚠️  Uniformity NOT strict: uniform_err_max=" << uniform_err_max << std::endl;
        all_ok = false;
    }

    double bias_mag = std::sqrt(bias.x * bias.x + bias.y * bias.y);
    Point3D diff_meas = ref;
    double diff_mag = std::sqrt(diff_meas.x * diff_meas.x + diff_meas.y * diff_meas.y);
    double dot = diff_meas.x * bias.x + diff_meas.y * bias.y;
    double dir_cos = (bias_mag > 1e-12 && diff_mag > 1e-12) ? dot / (bias_mag * diff_mag) : 1.0;
    double angle_deg = math_utils::radiansToDegrees(std::acos(std::max(-1.0, std::min(1.0, dir_cos))));
    double mag_ratio = (bias_mag > 1e-12) ? diff_mag / bias_mag : 0.0;
    bool bias_detected = (diff_mag > tol * 100.0); // the measured value must be significant
    bool direction_ok = angle_deg <= 15.0;         // reasonable directional alignment
    std::cout << "    Theoretical bias=(" << bias.x << "," << bias.y << ") mag=" << bias_mag
              << " | diff_medido=(" << diff_meas.x << "," << diff_meas.y << ") mag=" << diff_mag
              << " angle_diff_deg=" << angle_deg << " mag_ratio=" << mag_ratio << std::endl;

    if (uniform_ok && bias_detected && direction_ok) {
        std::cout << "    ✓ Confirmed: internal stride consistent + origin dependency (uniform diff aligned to ω×shift)." << std::endl;
        return true;
    } else {
        std::cout << "    ❌ Not fully confirmed (uniform_ok=" << uniform_ok
                  << ", bias_detected=" << bias_detected << ", direction_ok=" << direction_ok << ")" << std::endl;
        return false;
    }
}

// Computes the expected stride vector using the OpenSHC formula (identical to walk_controller.cpp in OpenSHC).
// NOTE: This function replicates the same formula as the SUT (LegStepper::updateStride).
// It is used as an algorithmic cross-check, NOT as an independent oracle.
// Independent validation is done with the hardcoded values in hardcoded_expected_strides[].
static Point3D computeExpectedOpenSHCStride(const StrideTestCase &tc) {
    // Linear components
    Point3D stride_linear(tc.linear_velocity.x, tc.linear_velocity.y, 0.0);

    // Radius: rejection onto Z axis (x,y,0) because current_tip_pose == identity_tip and z is zeroed.
    Point3D radius(tc.identity_tip.x, tc.identity_tip.y, 0.0);

    // Angular velocity (0,0,omega)
    Point3D angular_velocity_vec(0.0, 0.0, tc.angular_velocity);

    // Cross product: omega_k x radius => (-omega*y, omega*x, 0)
    Point3D stride_angular;
    stride_angular.x = angular_velocity_vec.y * radius.z - angular_velocity_vec.z * radius.y; // = -omega * y
    stride_angular.y = angular_velocity_vec.z * radius.x - angular_velocity_vec.x * radius.z; // =  omega * x
    stride_angular.z = 0.0;                                                                   // planar

    Point3D stride_total = stride_linear + stride_angular;
    double period = tc.stance_period + tc.swing_period;
    double on_ground_ratio = static_cast<double>(tc.stance_period) / period;
    stride_total = stride_total * (on_ground_ratio / tc.frequency);
    return stride_total;
}

int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Stride Vector Validation Test (HexaMotion vs OpenSHC) ===\n";

    // --- Minimum RobotModel parameter configuration ---
    Parameters params{};
    params.hexagon_radius = 160.0;             // mm
    params.coxa_length = 45.0;                 // mm
    params.femur_length = 90.0;                // mm
    params.tibia_length = 150.0;               // mm
    params.default_height_offset = -150.0;     // mm (same as -tibia_length for neutral vertical pose)
    params.robot_height = 150.0;               // mm
    params.time_delta = 1.0 / 50.0;            // 50 Hz
    params.standing_height = 120.0;            // mm (for internal validations)
    params.enable_workspace_constrain = false; // Disable to avoid altering stride

    RobotModel model(params);
    model.workspaceAnalyzerInitializer(); // Initialize analyzer (even though we don't use it directly here)

    // Create a Leg for leg_index 0
    Leg leg0(0, model);

    // Test cases
    std::vector<StrideTestCase> cases = {
        {Point3D(100.0, 0.0, params.default_height_offset), Point3D(50.0, 30.0, 0.0), 0.50, 1.0, 3, 1, "Combinado_PositiveY"},
        {Point3D(80.0, 60.0, params.default_height_offset), Point3D(40.0, -10.0, 0.0), 0.25, 1.2, 3, 1, "AnguloY_PositiveX"},
        {Point3D(120.0, -50.0, params.default_height_offset), Point3D(0.0, 20.0, 0.0), -0.40, 0.8, 2, 2, "SoloAngular_NegOmega"},
        {Point3D(60.0, 40.0, params.default_height_offset), Point3D(30.0, 0.0, 0.0), 0.00, 1.5, 3, 1, "SoloLineal"},
        {Point3D(90.0, -70.0, params.default_height_offset), Point3D(-25.0, 15.0, 0.0), 0.75, 2.0, 4, 2, "AltaFrecuencia"}};

    // ======================================================================
    // Hardcoded expected strides — hand-computed from first principles.
    // Formula: stride = (v_linear + omega_z_hat × radius_xy) * (stance/period) / freq
    //   where radius_xy = (tip.x, tip.y, 0), omega_z_hat × r = (-ω*y, ω*x, 0)
    // These serve as the PRIMARY independent oracle (not derived from SUT).
    // ======================================================================
    // Case 0 "Combinado_PositiveY": v=(50,30,0), ω=0.5, tip=(100,0,z), scale=0.75/1.0=0.75
    //   angular = (-0.5*0, 0.5*100, 0) = (0, 50, 0), total = (50,80,0)*0.75 = (37.5, 60.0, 0)
    // Case 1 "AnguloY_PositiveX": v=(40,-10,0), ω=0.25, tip=(80,60,z), scale=0.75/1.2=0.625
    //   angular = (-0.25*60, 0.25*80, 0) = (-15, 20, 0), total = (25,10,0)*0.625 = (15.625, 6.25, 0)
    // Case 2 "SoloAngular_NegOmega": v=(0,20,0), ω=-0.4, tip=(120,-50,z), scale=0.5/0.8=0.625
    //   angular = (0.4*(-50), -0.4*120, 0) = (-20, -48, 0), total = (-20,-28,0)*0.625 = (-12.5, -17.5, 0)
    // Case 3 "SoloLineal": v=(30,0,0), ω=0.0, tip=(60,40,z), scale=0.75/1.5=0.5
    //   angular = (0, 0, 0), total = (30,0,0)*0.5 = (15.0, 0.0, 0.0)
    // Case 4 "AltaFrecuencia": v=(-25,15,0), ω=0.75, tip=(90,-70,z), scale=(4/6)/2.0=1/3
    //   angular = (0.75*70, 0.75*90, 0) = (52.5, 67.5, 0), total = (27.5,82.5,0)/3 = (55/6, 27.5, 0)
    const Point3D hardcoded_expected_strides[] = {
        Point3D(37.5, 60.0, 0.0),       // Case 0
        Point3D(15.625, 6.25, 0.0),     // Case 1
        Point3D(-12.5, -17.5, 0.0),     // Case 2
        Point3D(15.0, 0.0, 0.0),        // Case 3
        Point3D(55.0 / 6.0, 27.5, 0.0), // Case 4
    };
    static_assert(sizeof(hardcoded_expected_strides) / sizeof(hardcoded_expected_strides[0]) == 5,
                  "Hardcoded expected strides must match number of test cases");

    const double TOL = 1e-9; // Strict tolerance
    bool all_passed = true;
    int case_idx = 0;

    for (const auto &tc : cases) {
        // Prepare LegStepper
        LegStepper stepper(0, tc.identity_tip, leg0, model);

        // Configure StepCycle
        StepCycle cycle{};
        cycle.frequency_ = tc.frequency;
        cycle.period_ = tc.stance_period + tc.swing_period;
        cycle.stance_period_ = tc.stance_period;
        cycle.swing_period_ = tc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = tc.stance_period; // [0, stance_period)
        cycle.swing_start_ = tc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepper.setStepCycle(cycle);

        // Set desired velocities and walk plane
        stepper.setDesiredVelocity(tc.linear_velocity, tc.angular_velocity);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));

        // updateStride computes and potentially freezes stride; we call it once.
        stepper.updateStride();

        Point3D got = stepper.getStrideVector();
        Point3D expected = computeExpectedOpenSHCStride(tc);

        // PRIMARY oracle: validate against hand-computed hardcoded expected values
        Point3D hc = hardcoded_expected_strides[case_idx];
        Point3D hc_diff = got - hc;
        double hc_err = std::sqrt(hc_diff.x * hc_diff.x + hc_diff.y * hc_diff.y + hc_diff.z * hc_diff.z);

        // SECONDARY cross-check: formula reimplementation (same math as SUT, catches refactoring regressions)
        Point3D diff = got - expected;
        double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

        // Extra: separate linear and angular scaled components for coxa movement diagnostics.
        double period = tc.stance_period + tc.swing_period;
        double on_ground_ratio = static_cast<double>(tc.stance_period) / period;
        Point3D scaled_linear = tc.linear_velocity * (on_ground_ratio / tc.frequency);
        Point3D angular_component_expected = expected - scaled_linear;
        Point3D angular_component_got = got - scaled_linear;
        Point3D angular_diff = angular_component_got - angular_component_expected;
        double angular_err = std::sqrt(angular_diff.x * angular_diff.x + angular_diff.y * angular_diff.y + angular_diff.z * angular_diff.z);

        std::cout << "\n[Test] " << tc.name << "\n";
        std::cout << " Identity Tip: (" << tc.identity_tip.x << ", " << tc.identity_tip.y << ", " << tc.identity_tip.z << ")" << std::endl;
        std::cout << " Linear Vel:   (" << tc.linear_velocity.x << ", " << tc.linear_velocity.y << ") mm/s  Angular Vel: " << tc.angular_velocity << " rad/s" << std::endl;
        std::cout << " freq=" << tc.frequency << " stance_period=" << tc.stance_period << " swing_period=" << tc.swing_period << std::endl;
        std::cout << " Expected Stride: (" << expected.x << ", " << expected.y << ", " << expected.z << ")" << std::endl;
        std::cout << " Hardcoded Exp:   (" << hc.x << ", " << hc.y << ", " << hc.z << ")" << std::endl;
        std::cout << " Got Stride:      (" << got.x << ", " << got.y << ", " << got.z << ")" << std::endl;
        std::cout << " |Hardcoded Err|=" << hc_err << "  |Formula Err|=" << err << "  Angular |Error|=" << angular_err << std::endl;

        // ===================== Comparison of effect on COXA =====================
        // In OpenSHC, the angular part of the stride represents the tangential translation of the tip
        // that comes from a body rotation (yaw) during the stance phase.
        // Let:
        //   body_yaw_angle = omega_z * (stance_ratio / frequency)
        // The magnitude of the angular stride component must satisfy:
        //   |stride_angular| = |omega_z| * (stance_ratio / frequency) * radius
        // where radius = sqrt(x^2 + y^2) of the identity position (XY plane).
        // Therefore, the expected coxa angle delta is:
        //   coxa_delta_expected = omega_z * (stance_ratio / frequency)
        // We can derive the angle delta from the obtained stride by projecting the
        // angular component onto the tangent vector and dividing by the radius.

        double stance_ratio = (period > 0.0) ? (static_cast<double>(tc.stance_period) / period) : 0.0;
        double coxa_delta_expected = tc.angular_velocity * (stance_ratio / tc.frequency); // rad

        // Derive coxa delta from the stride calculated by HexaMotion:
        // stride_angular_got already computed above (angular_component_got)
        Point3D radius_vec(tc.identity_tip.x, tc.identity_tip.y, 0.0);
        double radius_norm = std::sqrt(radius_vec.x * radius_vec.x + radius_vec.y * radius_vec.y);
        double coxa_delta_got = 0.0;
        bool coxa_delta_valid = radius_norm > 1e-9; // avoid division by zero
        if (coxa_delta_valid) {
            // Unit tangent vector ( -y, x ) / r
            Point3D tangent_unit(-radius_vec.y / radius_norm, radius_vec.x / radius_norm, 0.0);
            // Projection of the obtained angular component onto the tangent => arc (length) traveled
            double arc_length_got = angular_component_got.x * tangent_unit.x + angular_component_got.y * tangent_unit.y;
            coxa_delta_got = arc_length_got / radius_norm; // rad (con signo)
        }

        double coxa_err = std::fabs(coxa_delta_got - coxa_delta_expected);

        // Additional metric: linear displacement (mm) -> coxa degrees ratio
        double linear_planar_mm = std::sqrt(scaled_linear.x * scaled_linear.x + scaled_linear.y * scaled_linear.y);
        double coxa_delta_deg_expected = math_utils::radiansToDegrees(coxa_delta_expected);
        double coxa_delta_deg_got = math_utils::radiansToDegrees(coxa_delta_got);
        // Additional tangential validations (identical to multi-leg section) for the base case leg0
        double tangential_dot = angular_component_got.x * radius_vec.x + angular_component_got.y * radius_vec.y;
        double tangential_dot_abs = std::fabs(tangential_dot);
        double expected_arc_len = std::fabs(coxa_delta_expected) * radius_norm;
        double got_arc_len = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        double arc_len_err = std::fabs(got_arc_len - expected_arc_len);
        std::cout << " Coxa Δexpected(rad): " << coxa_delta_expected
                  << "  Coxa Δgot(rad): " << coxa_delta_got
                  << "  |Δerr|=" << coxa_err << std::endl;
        std::cout << " Tangential: |ω×r|exp=" << expected_arc_len
                  << " |ω×r|got=" << got_arc_len
                  << " arc_len_err=" << arc_len_err
                  << " dot(radius,stride_ang)=" << tangential_dot_abs << std::endl;
        std::cout << " Mapping: linear_planar_scaled=" << linear_planar_mm
                  << " mm -> coxaΔexpected=" << coxa_delta_deg_expected
                  << " deg  coxaΔgot=" << coxa_delta_deg_got << " deg" << std::endl;

        bool tangential_ok = (!coxa_delta_valid) || (tangential_dot_abs <= TOL * std::max(1.0, radius_norm) && std::fabs(got_arc_len - expected_arc_len) <= TOL * std::max(1.0, radius_norm));
        bool pass = (hc_err <= TOL && err <= TOL && angular_err <= TOL && (!coxa_delta_valid || coxa_err <= TOL) && tangential_ok);
        if (!pass) {
            std::cout << "  ❌ Mismatch exceeds tolerance." << std::endl;
            all_passed = false;
        } else {
            std::cout << "  ✓ OK" << std::endl;
        }

        // Additional validation on all 6 legs with radial geometry using their default positions
        bool radial_ok = validateAllLegs(tc, model, params, TOL);
        if (!radial_ok) {
            all_passed = false;
        }
        // Note: Symmetry errors already reflect that the deviation originates from E0 in most cases.

        // --- New frame/center test: only if there is an angular component (to observe bias) ---
        if (std::fabs(tc.angular_velocity) > 1e-9) {
            // Artificial shift (displaces the true center).
            Point3D shiftA(25.0, -25.0, 0.0);
            std::cout << "\n  [FrameTestSuite] Case: " << tc.name << " (with shiftA)" << std::endl;
            bool frame_ok = validateFrameCenterAssumption(tc, model, params, shiftA, 1e-9);
            if (!frame_ok)
                all_passed = false;
        }
        ++case_idx;
    }

    std::cout << "\nGlobal Result: " << (all_passed ? "✓ ALL CASES OK" : "❌ SOME CASE FAILED") << std::endl;
    return all_passed ? 0 : 1;
}
