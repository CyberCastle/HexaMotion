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
 * - amble_gait: AR/CL,BR/BL,AL/CR (fastest, least stable)
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
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * params.gait_factors.wave_length_factor;
    config.swing_height = params.robot_height * params.gait_factors.wave_height_factor;
    config.body_clearance = params.robot_height;
    int total_phase = config.phase_config.stance_phase + config.phase_config.swing_phase;
    config.step_frequency = 1.0 / (total_phase * 0.01);
    config.max_velocity = 50.0;
    config.stability_factor = 0.95;
    config.supports_rough_terrain = true;

    // Velocity limits parameters (unified configuration)
    config.stance_ratio = (double)config.phase_config.stance_phase / total_phase;
    config.swing_ratio = (double)config.phase_config.swing_phase / total_phase;
    config.time_to_max_stride = 2.0;   // Default conservative value for wave gait
    config.stance_span_modifier = 0.0; // OpenSHC: configurable, por defecto 0.0

    config.description = "Wave gait: Most stable gait with sequential leg movement";
    config.step_order = {"AR", "CL", "BL", "AL", "CR", "BR"};
    config.step_cycle.frequency_ = config.step_frequency;
    config.step_cycle.period_ = total_phase;
    config.step_cycle.stance_period_ = config.phase_config.stance_phase;
    config.step_cycle.swing_period_ = config.phase_config.swing_phase;
    config.step_cycle.stance_start_ = 0;
    config.step_cycle.stance_end_ = config.phase_config.stance_phase;
    config.step_cycle.swing_start_ = config.phase_config.stance_phase;
    config.step_cycle.swing_end_ = total_phase;
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
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * params.gait_factors.tripod_length_factor;
    config.swing_height = params.robot_height * params.gait_factors.tripod_height_factor;
    config.body_clearance = params.robot_height;
    int total_phase = config.phase_config.stance_phase + config.phase_config.swing_phase;
    config.step_frequency = 1.0 / (total_phase * 0.01);
    config.max_velocity = 100.0;
    config.stability_factor = 0.75;
    config.supports_rough_terrain = true;

    // Velocity limits parameters (unified configuration)
    config.stance_ratio = (double)config.phase_config.stance_phase / total_phase;
    config.swing_ratio = (double)config.phase_config.swing_phase / total_phase;
    config.time_to_max_stride = 1.5;   // Faster than wave gait
    config.stance_span_modifier = 0.0; // OpenSHC: configurable, por defecto 0.0

    config.description = "Tripod gait: Balanced speed and stability with alternating tripods";
    config.step_order = {"AR/BL/CR", "AL/BR/CL"};
    // StepCycle
    config.step_cycle.frequency_ = config.step_frequency;
    config.step_cycle.period_ = total_phase;
    config.step_cycle.stance_period_ = config.phase_config.stance_phase;
    config.step_cycle.swing_period_ = config.phase_config.swing_phase;
    config.step_cycle.stance_start_ = 0;
    config.step_cycle.stance_end_ = config.phase_config.stance_phase;
    config.step_cycle.swing_start_ = config.phase_config.stance_phase;
    config.step_cycle.swing_end_ = total_phase;
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

    // Calculated parameters
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * params.gait_factors.ripple_length_factor;
    config.swing_height = params.robot_height * params.gait_factors.ripple_height_factor;
    config.body_clearance = params.robot_height;
    int total_phase = config.phase_config.stance_phase + config.phase_config.swing_phase;
    config.step_frequency = 1.0 / (total_phase * 0.01); // Assuming 10ms iterations
    config.max_velocity = 150.0;                        // Faster movement
    config.stability_factor = 0.60;                     // Moderate stability
    config.supports_rough_terrain = false;              // Less suitable for rough terrain
    config.stance_span_modifier = 0.0;                  // OpenSHC: configurable, por defecto 0.0

    // Velocity limits parameters (unified configuration)
    config.stance_ratio = (double)config.phase_config.stance_phase / total_phase;
    config.swing_ratio = (double)config.phase_config.swing_phase / total_phase;
    config.time_to_max_stride = 1.0; // Fast acceleration

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

    // Calculated parameters
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    config.step_length = leg_reach * params.gait_factors.metachronal_length_factor;
    config.swing_height = params.robot_height * params.gait_factors.metachronal_height_factor;
    config.body_clearance = params.robot_height;
    int total_phase = config.phase_config.stance_phase + config.phase_config.swing_phase;
    config.step_frequency = 1.0 / (total_phase * 0.01);
    config.max_velocity = 80.0;     // Adaptive speed
    config.stability_factor = 0.85; // High stability with adaptation
    config.supports_rough_terrain = true;
    config.stance_span_modifier = 0.0; // OpenSHC: configurable, por defecto 0.0

    // Velocity limits parameters (unified configuration)
    config.stance_ratio = (double)config.phase_config.stance_phase / total_phase;
    config.swing_ratio = (double)config.phase_config.swing_phase / total_phase;
    config.time_to_max_stride = 1.8; // Adaptive acceleration

    // Description
    config.description = "Metachronal gait: Adaptive gait that adjusts to terrain conditions";
    config.step_order = {"AR", "CL", "BL", "AL", "CR", "BR"};

    return config;
}

/**
 * @brief Create gait selection configuration with all available gaits
 * @param params Parámetros del robot a usar para la configuración de los gaits
 * @return Complete gait selection configuration
 */
GaitSelectionConfig createGaitSelectionConfig(const Parameters &params) {
    GaitSelectionConfig config;

    // Usar los parámetros recibidos
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