#ifndef GAIT_CONFIG_FACTORY_H
#define GAIT_CONFIG_FACTORY_H

#include "gait_config.h"
#include "leg_stepper.h"
#include "locomotion_types.h" /**< For GaitType enum. */
#include "robot_model.h"      /**< For model access. */
#include <string>
#include <vector>

/**
 * @file gait_config_factory.h
 * @brief OpenSHC-equivalent gait configuration factory declarations
 *
 * This header provides factory functions to create gait configurations
 * that match OpenSHC's gait.yaml structure and parameters.
 */

/** Gait configuration creation functions. */
GaitConfiguration createWaveGaitConfig(const Parameters &params);
GaitConfiguration createTripodGaitConfig(const Parameters &params);
GaitConfiguration createRippleGaitConfig(const Parameters &params);
GaitConfiguration createMetachronalGaitConfig(const Parameters &params);

/**
 * @brief Create gait configuration based on GaitType enum
 * @param gait_type Type of gait to create
 * @param params Robot parameters
 * @return Configured gait of the specified type
 */
GaitConfiguration createGaitConfig(GaitType gait_type, const Parameters &params);

#endif /**< GAIT_CONFIG_FACTORY_H */