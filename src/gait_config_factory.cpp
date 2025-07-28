#include "gait_config.h"
#include "hexamotion_constants.h"
#include "robot_model.h"
#include <cmath>
#include <map>

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
    config.swing_width = 3.0;                            // mm - smaller lateral shift for wave gait (more conservative)
    config.control_frequency = params.control_frequency; // Hz - use robot's control frequency

    config.max_velocity = 50.0;
    config.stability_factor = 0.95;
    config.supports_rough_terrain = true;
    config.time_to_max_stride = 2.0;   // Default conservative value for wave gait
    config.stance_span_modifier = 0.1; // OpenSHC: valor por defecto para wave gait

    config.description = "Wave gait: Most stable gait with sequential leg movement";
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
    config.offsets.multipliers = {
        {"AR", 0}, {"BR", 1}, {"CR", 0}, {"CL", 1}, {"BL", 0}, {"AL", 1}};

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_TRIPOD_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_TRIPOD_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 5.0;                            // mm - OpenSHC standard lateral shift at mid-swing
    config.control_frequency = params.control_frequency; // Hz - use robot's control frequency

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
        {"AR", 2}, {"BR", 0}, {"CR", 4}, {"CL", 1}, {"BL", 3}, {"AL", 5}};

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_RIPPLE_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_RIPPLE_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 7.0;                            // mm - larger lateral shift for ripple gait (more dynamic)
    config.control_frequency = params.control_frequency; // Hz - use robot's control frequency

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
        {"AR", 2}, {"BR", 3}, {"CR", 4}, {"CL", 1}, {"BL", 0}, {"AL", 5}};

    // Calculated parameters using OpenSHC equivalent constants
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * GAIT_METACHRONAL_LENGTH_FACTOR;
    config.swing_height = params.standing_height * GAIT_METACHRONAL_HEIGHT_FACTOR;
    config.body_clearance = params.standing_height;

    // OpenSHC trajectory parameters
    config.swing_width = 4.0;                            // mm - moderate lateral shift for metachronal gait (adaptive)
    config.control_frequency = params.control_frequency; // Hz - use robot's control frequency

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