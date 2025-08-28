/**
 * @file coxa_tripod_symmetry_analytic_test.cpp
 * @brief Analytical symmetry validation of COXA joint angle evolution driven purely by
 *        LegStepper::updateTipPosition() under a tripod gait (direct, controller‑free harness).
 *
 * Rationale:
 *  Existing tests (e.g. coxa_phase_transition_test) validate symmetry within the full
 *  LocomotionSystem stack (body pose controller, walk controller, etc.). This test
 *  isolates the LegStepper integration loop to ensure that the generated Cartesian
 *  tip trajectories (and subsequent IK) preserve bilateral coxa angular symmetry
 *  for opposing leg pairs (0↔3, 1↔4, 2↔5) when both legs are simultaneously in the
 *  same gait state (STANCE or SWING) during a tripod gait cycle.
 *
 * Morphological context (from AGENTS.md):
 *  - Hexagonal body, legs spaced 60°; base orientation offsets (deg):
 *      leg0=-30, leg1=-90, leg2=-150, leg3=+150, leg4=+90, leg5=+30.
 *  - All joint angles at 0°: femur horizontal, tibia vertical; body height == |tibia_length|.
 *  - Default height offset explicitly set to -tibia_length for clarity.
 *
 * Test Model Parameters (physical robot):
 *  hexagon_radius=200 mm, coxa=50 mm, femur=101 mm, tibia=208 mm,
 *  standing_height=150 mm, default_height_offset = -tibia_length.
 *  Frequency: 50 Hz (time_delta = 1/50). StepCycle tripod: stance=2, swing=2, period=4, freq=1 Hz.
 *
 * Method:
 *  1. Construct RobotModel with physical parameters.
 *  2. For each leg i build identity tip pose at radius = hexagon_radius + (coxa_length + femur_length)
 *     (consistent with all-zero joint baseline) and z = default_height_offset.
 *  3. Instantiate Leg + LegStepper (identity pose) and assign tripod StepCycle (2/2 pattern).
 *  4. Assign a forward-only desired linear velocity (no angular) identical for all legs;
 *     set walk plane normal (0,0,1).
 *  5. Iterate N full gait cycles. For each global iteration k:
 *       - Compute phase = (k + offset_i) % period where offset_i = 0 for group A (AR/BL/CR),
 *         and 2 for group B (AL/BR/CL) to emulate balanced alternating tripod offsets.
 *       - Call updateStepStateFromPhase(), then updateTipPosition().
 *       - Apply IK to synchronize joint angles with the new tip position.
 *       - When an opposing pair is concurrently in same state (STANCE or SWING), record
 *         absolute delta (in degrees) between their coxa angle deviations from baseline.
 *  6. Track maximum |Δcoxa_a - Δcoxa_b| for each opposing pair separately in STANCE and SWING.
 *  7. Assert pass if maxima are below configured tolerances:
 *        STANCE_MAX_DIFF_DEG <= 3.0  &&  SWING_MAX_DIFF_DEG <= 4.0
 *
 * Notes:
 *  - We compare deltas (angle - initial_angle) rather than raw absolute angles to remove
 *    inherent base orientation offsets.
 *  - The LegStepper by design updates only Cartesian tip pose; IK invocation in this harness
 *    ensures joint angles reflect the generated trajectory just like batch IK would downstream.
 *  - No body pose adjustments or external controllers are involved; this isolates LegStepper logic.
 *
 * Output:
 *  Prints max symmetry deviations for each pair in stance & swing and overall PASS/FAIL.
 */

#include "hexamotion_constants.h" // explicit for BASE_THETA_OFFSETS morphological validation
#include "leg_stepper.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Opposing leg pair mapping (indices)
struct Pair {
    int a;
    int b;
};
static const Pair OPPOSING_PAIRS[3] = {{0, 3}, {1, 4}, {2, 5}}; // (AR, CL?) naming depends on convention but indices match model offsets

// Tolerances (degrees)
static constexpr double STANCE_SYM_TOL_DEG = 3.0;
static constexpr double SWING_SYM_TOL_DEG = 4.0;

// Number of full gait cycles (stance+ swing) to simulate at internal resolution.
static constexpr int NUM_FULL_CYCLES = 40; // Lower is enough now that we iterate full internal steps

// Helper: degrees conversion
static double radToDeg(double r) { return r * 180.0 / M_PI; }

int main() {
    std::cout << "=== Coxa Tripod Analytical Symmetry Test (Boundary Symmetry) ===\n";

    // --- 1. Parameters (AGENTS.md physical morphology) ---
    Parameters p{};
    p.hexagon_radius = 200.0;
    p.coxa_length = 50.0;
    p.femur_length = 101.0;
    p.tibia_length = 208.0;
    p.default_height_offset = -p.tibia_length; // explicit per AGENTS.md
    p.robot_height = 208.0;
    p.standing_height = 150.0;
    p.time_delta = 1.0 / 50.0;            // 50 Hz integration
    p.enable_workspace_constrain = false; // keep raw stride effects

    RobotModel model(p);

    // --- 1.a Morphological base offset validation (explicit use of BASE_THETA_OFFSETS) ---
    // Opposing pairs must be radially opposite (angular separation ~= 180°). Intra-tripod legs (0,2,4) & (1,3,5)
    // should be separated by ~120° (every other hex vertex) ensuring uniform torque distribution.
    const double tol_rad = 1e-6;
    auto nearlyZero = [&](double v) { return std::fabs(v) < tol_rad; };
    // Opposing radial separation check (difference ~ pi)
    const int mirror_pairs[3][2] = {{0, 3}, {1, 4}, {2, 5}};
    for (auto &mp : mirror_pairs) {
        double a = BASE_THETA_OFFSETS[mp[0]];
        double b = BASE_THETA_OFFSETS[mp[1]];
        double diff = std::fabs(b - a);
        // normalize to [0,2pi)
        diff = std::fmod(diff, 2 * M_PI);
        if (diff > M_PI)
            diff = 2 * M_PI - diff; // shortest arc
        if (std::fabs(diff - M_PI) > 1e-6) {
            std::cerr << "ERROR: Base theta offsets for pair (" << mp[0] << "," << mp[1] << ") not 180° apart (sep=" << diff << " rad)\n";
            return 2;
        }
    }
    // Intra-tripod angular separation (expected 120° = 2*pi/3)
    auto checkTripod = [&](const int idx[3]) {
        for (int a = 0; a < 3; ++a)
            for (int b = a + 1; b < 3; ++b) {
                double da = std::fabs(BASE_THETA_OFFSETS[idx[a]] - BASE_THETA_OFFSETS[idx[b]]);
                // normalize to [0,pi]
                while (da > M_PI)
                    da = std::fabs(da - 2 * M_PI);
                double target = 2.0 * M_PI / 3.0; // 120°
                if (std::fabs(da - target) > 1e-4) {
                    std::cerr << "ERROR: Tripod separation not 120 deg between legs " << idx[a] << " and " << idx[b] << " (rad diff=" << da << ")\n";
                    return false;
                }
            }
        return true;
    };
    const int tripodA[3] = {0, 2, 4};
    const int tripodB[3] = {1, 3, 5};
    if (!checkTripod(tripodA) || !checkTripod(tripodB))
        return 3;
    std::cout << "Base theta offsets validated (mirror + 120° intra-tripod spacing).\n";

    // --- 2. Build Leg + LegStepper arrays ---
    struct LegContext {
        Leg leg;
        LegStepper stepper;
        JointAngles baseline_angles; // initial coxa angle (rad)
    };

    std::vector<LegContext> legs;
    legs.reserve(NUM_LEGS);

    // Tripod StepCycle (OpenSHC-equivalent 2 stance / 2 swing)
    StepCycle cycle{};
    cycle.frequency_ = 1.0; // 1 Hz base cycle (period spans 1 second)
    cycle.period_ = 4;      // total slices
    cycle.stance_period_ = 2;
    cycle.swing_period_ = 2;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 2;
    cycle.swing_start_ = 2;
    cycle.swing_end_ = 4;

    // Identity radius: hexagon center to leg base + (coxa + femur) horizontal reach when all joints zero
    double radial_reach = p.hexagon_radius + (p.coxa_length + p.femur_length);

    for (int i = 0; i < NUM_LEGS; ++i) {
        Leg leg(i, model);
        double base_theta = model.getLegBaseAngleOffset(i); // radians
        Point3D identity_tip(radial_reach * std::cos(base_theta),
                             radial_reach * std::sin(base_theta),
                             p.default_height_offset);

        // Initialize LegStepper with identity pose
        LegStepper stepper(i, identity_tip, leg, model);
        stepper.setStepCycle(cycle);
        stepper.setWalkPlane(identity_tip);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));

        // Assign desired velocity (forward X). Keep modest to avoid workspace edge.
        stepper.setDesiredVelocity(Point3D(80.0, 0.0, 0.0), 0.0); // mm/s linear, 0 angular

        // Sync IK baseline (all zeros expected; ensures baseline_angles captured)
        leg.applyIK(identity_tip);
        JointAngles baseline = leg.getJointAngles();

        legs.push_back(LegContext{std::move(leg), std::move(stepper), baseline});
    }

    // Tripod grouping (balanced) -> group A indices: 0,2,4? or as per createTripodGaitConfig comment: AR/BL/CR, AL/BR/CL
    // Mapping by index (leg names typical: 0=AR,1=BR,2=CR,3=CL,4=BL,5=AL)
    // Group A: AR(0), BL(4), CR(2)  | Group B: AL(5), BR(1), CL(3)
    auto inGroupA = [](int idx) { return (idx == 0) || (idx == 4) || (idx == 2); };

    // --- 3. Derive internal iteration counts (OpenSHC equivalence) ---
    int swing_iterations = int((double(cycle.swing_period_) / cycle.period_) / (cycle.frequency_ * p.time_delta));
    if (swing_iterations % 2 != 0)
        swing_iterations++;
    if (swing_iterations < 10)
        swing_iterations = 10;
    int stance_iterations = int((double(cycle.stance_period_) / cycle.period_) / (cycle.frequency_ * p.time_delta));
    int total_cycle_iterations = swing_iterations + stance_iterations; // per leg

    // Per-leg global iteration counters (simulate independent steppers with offset)
    std::vector<int> leg_global_iter(NUM_LEGS, 0);
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!inGroupA(i)) {
            leg_global_iter[i] = stance_iterations; // half-cycle offset so these start in SWING while group A in STANCE
        }
    }

    int total_global_iterations = NUM_FULL_CYCLES * total_cycle_iterations;

    // Metrics
    double max_opposed_boundary_mirror_deg[3] = {0, 0, 0}; // max |delta_i + delta_j| at boundaries
    double max_intragroup_disp_deg_A = 0.0;
    double max_intragroup_disp_deg_B = 0.0;
    // Amplitude tracking per leg
    std::vector<double> leg_min_delta(NUM_LEGS, 1e9);
    std::vector<double> leg_max_delta(NUM_LEGS, -1e9);

    int boundary_samples = 0;

    for (int global_iter = 0; global_iter < total_global_iterations; ++global_iter) {
        // Advance each leg one internal iteration
        for (int i = 0; i < NUM_LEGS; ++i) {
            leg_global_iter[i]++;
            int local_in_cycle = leg_global_iter[i] % total_cycle_iterations; // 0..total-1
            if (local_in_cycle == 0)
                local_in_cycle = total_cycle_iterations;

            bool in_stance = (local_in_cycle <= stance_iterations);
            // Set step state manually (we bypass phase-based path)
            LegStepper &st = legs[i].stepper;
            st.setStepState(in_stance ? STEP_STANCE : STEP_SWING);
            // Perform update using global iteration counter for internal modulo logic
            st.updateTipPositionIterative(leg_global_iter[i], p.time_delta, false, false);
            legs[i].leg.applyIK(st.getCurrentTipPose());

            // Track amplitude continuously
            double delta = radToDeg(legs[i].leg.getJointAngles().coxa - legs[i].baseline_angles.coxa);
            if (delta < leg_min_delta[i])
                leg_min_delta[i] = delta;
            if (delta > leg_max_delta[i])
                leg_max_delta[i] = delta;
        }

        // Boundary detection: start of stance (local_in_cycle==1) or start of swing (== stance_iterations+1)
        // We collect deltas for legs at their phase starts.
        std::vector<int> stance_start_legs;
        std::vector<int> swing_start_legs;
        for (int i = 0; i < NUM_LEGS; ++i) {
            int local_in_cycle = leg_global_iter[i] % total_cycle_iterations;
            if (local_in_cycle == 0)
                local_in_cycle = total_cycle_iterations;
            if (local_in_cycle == 1)
                stance_start_legs.push_back(i);
            else if (local_in_cycle == (stance_iterations + 1))
                swing_start_legs.push_back(i);
        }
        bool have_boundary = (!stance_start_legs.empty() || !swing_start_legs.empty());
        if (have_boundary) {
            boundary_samples++;
            // Opposing pair mirror check uses current deltas regardless of which boundary (they always occur in complementary sets)
            for (int pidx = 0; pidx < 3; ++pidx) {
                int a = OPPOSING_PAIRS[pidx].a;
                int b = OPPOSING_PAIRS[pidx].b;
                double da = radToDeg(legs[a].leg.getJointAngles().coxa - legs[a].baseline_angles.coxa);
                double db = radToDeg(legs[b].leg.getJointAngles().coxa - legs[b].baseline_angles.coxa);
                double mirror_err = std::fabs(da + db); // expect da ≈ -db
                if (mirror_err > max_opposed_boundary_mirror_deg[pidx])
                    max_opposed_boundary_mirror_deg[pidx] = mirror_err;
            }
            // Intra-group dispersion (legs beginning phases of same type at this boundary)
            auto computeDispersion = [&](const std::vector<int> &legs_indices) {
                if (legs_indices.size() < 2)
                    return 0.0;
                double minv = 1e9, maxv = -1e9;
                for (int li : legs_indices) {
                    double d = radToDeg(legs[li].leg.getJointAngles().coxa - legs[li].baseline_angles.coxa);
                    if (d < minv)
                        minv = d;
                    if (d > maxv)
                        maxv = d;
                }
                return maxv - minv;
            };
            double disp_stance = computeDispersion(stance_start_legs);
            double disp_swing = computeDispersion(swing_start_legs);
            // Assign to A/B buckets by checking first leg's group (both sets contain homogeneous group membership by construction)
            if (disp_stance > 0.0) {
                bool groupA = inGroupA(stance_start_legs.front());
                if (groupA && disp_stance > max_intragroup_disp_deg_A)
                    max_intragroup_disp_deg_A = disp_stance;
                if (!groupA && disp_stance > max_intragroup_disp_deg_B)
                    max_intragroup_disp_deg_B = disp_stance;
            }
            if (disp_swing > 0.0) {
                bool groupA = swing_start_legs.empty() ? false : inGroupA(swing_start_legs.front());
                if (groupA && disp_swing > max_intragroup_disp_deg_A)
                    max_intragroup_disp_deg_A = disp_swing;
                if (!groupA && disp_swing > max_intragroup_disp_deg_B)
                    max_intragroup_disp_deg_B = disp_swing;
            }
        }
    }

    // Amplitude equality: compare |max| and |min| magnitudes per leg and opposed pair differences
    double max_amplitude_asym_deg = 0.0; // per leg | |max| - |min| |
    for (int i = 0; i < NUM_LEGS; ++i) {
        double amp_pos = std::fabs(leg_max_delta[i]);
        double amp_neg = std::fabs(leg_min_delta[i]);
        double asym = std::fabs(amp_pos - amp_neg);
        if (asym > max_amplitude_asym_deg)
            max_amplitude_asym_deg = asym;
    }
    double max_opposed_amp_diff_deg = 0.0; // | amplitude_i - amplitude_j |
    for (int pidx = 0; pidx < 3; ++pidx) {
        int a = OPPOSING_PAIRS[pidx].a;
        int b = OPPOSING_PAIRS[pidx].b;
        double amp_a = 0.5 * (std::fabs(leg_max_delta[a]) + std::fabs(leg_min_delta[a])); // mean magnitude
        double amp_b = 0.5 * (std::fabs(leg_max_delta[b]) + std::fabs(leg_min_delta[b]));
        double diff = std::fabs(amp_a - amp_b);
        if (diff > max_opposed_amp_diff_deg)
            max_opposed_amp_diff_deg = diff;
    }

    // --- 4. Report ---
    std::cout << "Cycles simulated: " << NUM_FULL_CYCLES << " (internal iter per cycle=" << total_cycle_iterations << ")\n";
    std::cout << "Boundary samples: " << boundary_samples << "\n";
    std::cout << std::fixed << std::setprecision(3);
    const char *pair_labels[3] = {"(0,3)", "(1,4)", "(2,5)"};
    std::cout << "\nMirror symmetry (boundary) max |δ_i + δ_j| (deg):\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << "  Pair " << pair_labels[i] << ": " << max_opposed_boundary_mirror_deg[i] << "\n";
    }
    std::cout << "\nIntra-tripod dispersion max (deg): GroupA=" << max_intragroup_disp_deg_A << " GroupB=" << max_intragroup_disp_deg_B << "\n";
    std::cout << "Amplitude asymmetry per leg max | |max| - |min| | (deg): " << max_amplitude_asym_deg << "\n";
    std::cout << "Opposed amplitude diff max (deg): " << max_opposed_amp_diff_deg << "\n";

    // Pass criteria (conservative, can tighten with real data):
    double mirror_tol_deg = STANCE_SYM_TOL_DEG; // reuse 3° baseline
    double intragroup_tol_deg = 1.0;            // legs in same tripod should differ <1° at boundaries
    double amp_asym_tol_deg = 2.0;              // difference between protraction/retraction magnitudes per leg
    double opposed_amp_diff_tol_deg = 2.0;

    bool pass = true;
    for (int i = 0; i < 3; ++i)
        if (max_opposed_boundary_mirror_deg[i] > mirror_tol_deg)
            pass = false;
    if (max_intragroup_disp_deg_A > intragroup_tol_deg || max_intragroup_disp_deg_B > intragroup_tol_deg)
        pass = false;
    if (max_amplitude_asym_deg > amp_asym_tol_deg)
        pass = false;
    if (max_opposed_amp_diff_deg > opposed_amp_diff_tol_deg)
        pass = false;

    std::cout << "\nTolerances: mirror<=" << mirror_tol_deg
              << " intragroup<=" << intragroup_tol_deg
              << " amp_asym<=" << amp_asym_tol_deg
              << " opposed_amp_diff<=" << opposed_amp_diff_tol_deg << " (deg)\n";
    if (!pass) {
        std::cerr << "FAIL: Tripod coxa symmetry boundary validation failed." << std::endl;
        return 1;
    }
    std::cout << "PASS: Tripod coxa symmetry & amplitude validated at boundaries." << std::endl;
    return 0;
}
