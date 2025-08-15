#include "gait_config.h"
#include "hexamotion_constants.h"
#include "robot_model.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

// ============================================================================
// Balanced tripod multiplier computation
// Derives two alternating tripod groups from BASE_THETA_OFFSETS ensuring that
// legs assigned to the same phase are spaced (approximately) every other leg
// when traversing the body perimeter in angular order. This reduces risk of
// lateral bias without reindexing or introducing mapping layers.
// ============================================================================
namespace {

LegOffsetMultipliers computeBalancedTripodOffsets() {
    // PURPOSE: Build two balanced tripod phase groups by
    // 1) collecting each leg's base orientation angle,
    // 2) sorting legs by that angle around the hexagon,
    // 3) alternating assignment (even/odd) to produce maximally interleaved groups.
    // This avoids lateral bias caused by using the raw internal index ordering.

    // Internal leg name order used throughout the codebase
    static const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"}; // Constant array mapping index->name

    struct LegAngle { // Small POD struct to pair a leg index with its normalized angle
        int index;    // Original leg index in internal ordering
        double angle; // Normalized (0..2π) absolute base orientation angle
    };
    std::vector<LegAngle> legs;           // Container to hold all leg-angle pairs for sorting
    legs.reserve(NUM_LEGS);               // Reserve exact capacity (6) to avoid reallocations
    for (int i = 0; i < NUM_LEGS; ++i) {  // Iterate over each leg index
        double a = BASE_THETA_OFFSETS[i]; // Fetch the configured base orientation (may be negative)
        if (a < 0.0)                      // If angle is negative
            a += 2.0 * M_PI;              //   wrap it into the [0,2π) range to enable a single circular sort
        legs.push_back({i, a});           // Store the (index, normalized angle) pair
    }

    // Sort legs by increasing angle so we traverse them clockwise/counter‑clockwise uniformly
    std::sort(legs.begin(), legs.end(), [](const LegAngle &l1, const LegAngle &l2) { return l1.angle < l2.angle; });

    int group_for_index[NUM_LEGS];                                        // Array storing computed group (0 or 1) for each original index
    for (size_t sorted_pos = 0; sorted_pos < legs.size(); ++sorted_pos) { // Walk legs in angular order
        const auto &la = legs[sorted_pos];                                // Reference current sorted leg-angle record
        group_for_index[la.index] =                                       // Assign group for the original leg index
            (sorted_pos % 2 == 0) ? 0 : 1;                                // Even positions -> group 0, odd positions -> group 1 (alternation)
    }

    LegOffsetMultipliers m;                               // Result structure to be returned
    for (int i = 0; i < NUM_LEGS; ++i) {                  // Reconstruct mapping in the internal leg name order
        m.multipliers[LEG_NAMES[i]] = group_for_index[i]; // Store multiplier (phase group) under its name key
    }
    return m; // Return the balanced two-group mapping
}

// Generic balance validator: only actively corrects 2-group (tripod-style) patterns.
// Multi-step sequential gaits (wave, ripple, metachronal) are inherently asymmetric by design
// but should distribute legs across halves; we simply avoid altering their OpenSHC mapping.
bool isTwoGroupAndImbalanced(const LegOffsetMultipliers &m) {
    // PURPOSE: Detect if a two-group gait (e.g. tripod) assigns legs such that any group
    // occupies only one lateral half of the body (all y >= 0 or all y < 0), which would
    // create lateral imbalance. Returns true only for that imbalance case.

    std::map<int, std::vector<int>> groups;                                        // Map phase multiplier -> list of leg indices
    static const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"}; // Internal lookup order
    for (int i = 0; i < NUM_LEGS; ++i) {                                           // Iterate fixed set of legs
        int mult = 0;                                                              // Default multiplier if missing
        auto it = m.multipliers.find(LEG_NAMES[i]);                                // Locate leg name in provided mapping
        if (it != m.multipliers.end())                                             // If mapping entry exists
            mult = it->second;                                                     //   extract its multiplier value
        groups[mult].push_back(i);                                                 // Append leg index to its multiplier group
    }
    if (groups.size() != 2) // Only care about strictly two phase groups
        return false;       // Multi-group gaits are not auto-balanced here

    // Lambda: returns true if every leg in idxs is on same lateral half-plane (all y>=0 or all y<0)
    auto allSameHalf = [](const std::vector<int> &idxs) {
        if (idxs.size() < 2) // Single-element group can't be imbalanced on its own
            return false;
        int pos = 0, neg = 0;                   // Counters for legs with positive/negative sine(angle)
        for (int idx : idxs) {                  // Examine each leg index in the group
            double a = BASE_THETA_OFFSETS[idx]; // Retrieve base orientation angle (may be negative)
            if (a < 0.0)                        // Normalize if negative
                a += 2.0 * M_PI;
            (std::sin(a) >= 0.0 ? pos : neg)++; // Increment pos if y>=0 else neg
        }
        return (pos == 0 || neg == 0); // True if all fell into only one half-plane
    };

    for (auto &kv : groups) {       // Check each phase group
        if (allSameHalf(kv.second)) // If any group is laterally one-sided
            return true;            // Report imbalance
    }
    return false; // Otherwise balanced (or acceptable distribution)
}

void ensureBalancedIfNeeded(GaitConfiguration &cfg) {
    if (isTwoGroupAndImbalanced(cfg.offsets)) {
        cfg.offsets = computeBalancedTripodOffsets();
    }
}

} // namespace

/**
 * @file gait_config_factory.cpp
 * @brief OpenSHC-equivalent gait configuration factory for HexaMotion robot
 *
 * This implementation follows OpenSHC's gait.yaml configuration structure where:
 * - Each gait has specific stance_phase, swing_phase, and phase_offset parameters
 * - Leg offset multipliers define the phase timing for each leg
 * - Configuration matches OpenSHC's gait.yaml structure exactly
 *
 * Gait configurations based on OpenSHC/config/gait.yaml:
 * - wave_gait: AR,CL,BL,AL,CR,BR (most stable, slowest)
 * - tripod_gait: AR/BL/CR,AL/BR/CL (balanced speed/stability)
 * - ripple_gait: AR,CL,BR,AL,CR,BL (faster, less stable)
 * - metachronal_gait: AR,CL,BL,AL,CR,BR (adaptive, HexaMotion specific)
 */

/**
 * @brief Create wave gait configuration (OpenSHC equivalent)
 * Step order: AR,CL,BL,AL,CR,BR
 * Most stable gait, suitable for rough terrain and slow movement
 */
GaitConfiguration createWaveGaitConfig(const Parameters &params) {
    GaitConfiguration config;
    config.gait_name = "wave_gait";
    config.phase_config.stance_phase = 10;
    config.phase_config.swing_phase = 2;
    config.phase_config.phase_offset = 2;
    config.offsets.multipliers = {
        {"AR", 2}, {"BR", 3}, {"CR", 4}, {"CL", 1}, {"BL", 0}, {"AL", 5}};

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_WAVE_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_WAVE_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 3.0;                       // mm - smaller lateral shift for wave gait (more conservative)
    config.step_frequency = DEFAULT_STEP_FREQUENCY; // Hz - OpenSHC default step frequency

    config.max_velocity = 50.0;
    config.stability_factor = 0.95;
    config.supports_rough_terrain = true;
    config.time_to_max_stride = 2.0;   // Default conservative value for wave gait
    config.stance_span_modifier = 0.1; // OpenSHC: valor por defecto para wave gait

    config.description = "Wave gait: Most stable gait with sequential leg movement";
    ensureBalancedIfNeeded(config); // No-op (more than 2 groups)
    config.step_order = {"AR", "CL", "BL", "AL", "CR", "BR"};
    return config;
}

/**
 * @brief Create tripod gait configuration (OpenSHC equivalent)
 * Step order: AR/BL/CR,AL/BR/CL
 * Balanced gait for moderate speed and stability
 */
GaitConfiguration createTripodGaitConfig(const Parameters &params) {
    GaitConfiguration config;
    config.gait_name = "tripod_gait";
    config.phase_config.stance_phase = 2;
    config.phase_config.swing_phase = 2;
    config.phase_config.phase_offset = 2;
    // Option 1 implementation: compute balanced alternating tripod groups
    // Replaces prior hardcoded pattern to minimize lateral bias.
    config.offsets = computeBalancedTripodOffsets();
    ensureBalancedIfNeeded(config); // Ensures future modifications remain balanced

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_TRIPOD_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_TRIPOD_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 5.0;                       // mm - OpenSHC standard lateral shift at mid-swing
    config.step_frequency = DEFAULT_STEP_FREQUENCY; // Hz - OpenSHC default step frequency

    config.max_velocity = 100.0;
    config.stability_factor = 0.75;
    config.supports_rough_terrain = false;
    config.time_to_max_stride = 1.5;    // Faster than wave gait
    config.stance_span_modifier = 0.25; // OpenSHC: valor por defecto para tripod gait

    config.description = "Tripod gait: Balanced speed and stability with alternating tripods";
    config.step_order = {"AR/BL/CR", "AL/BR/CL"};
    return config;
}

/**
 * @brief Create ripple gait configuration (OpenSHC equivalent)
 * Step order: AR,CL,BR,AL,CR,BL (overlapping)
 * Faster gait with overlapping movements
 */
GaitConfiguration createRippleGaitConfig(const Parameters &params) {
    GaitConfiguration config;
    config.gait_name = "ripple_gait";

    // OpenSHC gait.yaml parameters
    config.phase_config.stance_phase = 4;
    config.phase_config.swing_phase = 2;
    config.phase_config.phase_offset = 1;

    // OpenSHC offset_multiplier mapping
    config.offsets.multipliers = {
        {"AR", 2}, {"BR", 0}, {"CR", 4}, {"CL", 1}, {"BL", 3}, {"AL", 5}}; // OpenSHC mapping
    ensureBalancedIfNeeded(config);                                        // No-op (6 distinct groups)

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_RIPPLE_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_RIPPLE_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 7.0;                       // mm - larger lateral shift for ripple gait (more dynamic)
    config.step_frequency = DEFAULT_STEP_FREQUENCY; // Hz - OpenSHC default step frequency

    config.max_velocity = 150.0;           // Faster movement
    config.stability_factor = 0.60;        // Moderate stability
    config.supports_rough_terrain = false; // Less suitable for rough terrain
    config.stance_span_modifier = 0.2;     // OpenSHC: valor por defecto para tripod gait
    config.time_to_max_stride = 1.0;       // Fast acceleration

    // Description
    config.description = "Ripple gait: Faster gait with overlapping leg movements";
    config.step_order = {"AR", "CL", "BR", "AL", "CR", "BL"};

    return config;
}

/**
 * @brief Create metachronal gait configuration (HexaMotion specific)
 * Step order: AR,CL,BL,AL,CR,BR with adaptive timing
 * Adaptive gait that adjusts to terrain conditions
 */
GaitConfiguration createMetachronalGaitConfig(const Parameters &params) {
    GaitConfiguration config;
    config.gait_name = "metachronal_gait";

    // Adaptive parameters based on wave gait
    config.phase_config.stance_phase = 8; // Slightly faster than wave
    config.phase_config.swing_phase = 2;
    config.phase_config.phase_offset = 2;

    // Same offset pattern as wave gait
    config.offsets.multipliers = {
        {"AR", 2}, {"BR", 3}, {"CR", 4}, {"CL", 1}, {"BL", 0}, {"AL", 5}}; // Wave-style mapping
    ensureBalancedIfNeeded(config);                                        // No-op (multi-group)

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_METACHRONAL_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_METACHRONAL_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 4.0;                       // mm - moderate lateral shift for metachronal gait (adaptive)
    config.step_frequency = DEFAULT_STEP_FREQUENCY; // Hz - OpenSHC default step frequency

    config.max_velocity = 80.0;     // Adaptive speed
    config.stability_factor = 0.85; // High stability with adaptation
    config.supports_rough_terrain = true;
    config.stance_span_modifier = 0.15; // OpenSHC: valor por defecto para ripple gait
    config.time_to_max_stride = 1.8;    // Adaptive acceleration

    // Description
    config.description = "Metachronal gait: Adaptive gait that adjusts to terrain conditions";
    config.step_order = {"AR", "CL", "BL", "AL", "CR", "BR"};

    return config;
}

/**
 * @brief Create gait selection configuration with all available gaits
 * @param params Robot parameters to use for gait configuration
 * @return Complete gait selection configuration
 */
GaitSelectionConfig createGaitSelectionConfig(const Parameters &params) {
    GaitSelectionConfig config;

    // Use the received parameters
    // Add all available gaits
    config.available_gaits["wave_gait"] = createWaveGaitConfig(params);
    config.available_gaits["tripod_gait"] = createTripodGaitConfig(params);
    config.available_gaits["ripple_gait"] = createRippleGaitConfig(params);
    config.available_gaits["metachronal_gait"] = createMetachronalGaitConfig(params);

    // Default configuration
    config.default_gait = "tripod_gait";
    config.current_gait = "tripod_gait";

    // Transition parameters
    config.transition_time = 1.0; // 1 second transition time
    config.smooth_transitions = true;

    // Selection criteria
    config.min_stability_threshold = 0.5;
    config.max_velocity_threshold = 150.0;

    return config;
}