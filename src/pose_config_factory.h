#ifndef POSE_CONFIG_FACTORY_H
#define POSE_CONFIG_FACTORY_H

#include "HexaModel.h"
#include "pose_config.h"

/**
 * @file pose_config_factory.h
 * @brief Factory functions for creating pose configurations
 *
 * This module separates the pose configuration creation logic from the data structures,
 * allowing pose_controller to only depend on data structures defined in pose_config.h
 */

/**
 * @brief Calculate hexagonal leg stance positions based on robot parameters
 * @param hexagon_radius Body hexagon radius in mm
 * @param coxa_length Coxa length in mm
 * @return Array of calculated stance positions in meters
 */
std::array<LegStancePosition, NUM_LEGS> calculateHexagonalStancePositions(
    float hexagon_radius, float coxa_length);

/**
 * @brief Get default standing pose joint angles (OpenSHC equivalent)
 * For a hexagonal robot in static equilibrium, this should return:
 * - Coxa ≈ 0° (radially aligned with mounting)
 * - Femur and Tibia calculated using inverse kinematics for target height
 * @param params Robot parameters containing dimensions and joint limits
 * @return Array of standing pose joint configurations
 */
std::array<StandingPoseJoints, NUM_LEGS> getDefaultStandingPoseJoints(const Parameters &params);

/**
 * @brief Get default pose configuration using robot parameters (OpenSHC equivalent)
 * @param params Robot parameters from HexaModel
 * @return Default pose configuration calculated from robot specifications
 */
PoseConfiguration getDefaultPoseConfig(const Parameters &params);

/**
 * @brief Get conservative pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Conservative pose configuration with reduced limits for safety
 */
PoseConfiguration getConservativePoseConfig(const Parameters &params);

/**
 * @brief Get high-speed pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return High-speed pose configuration optimized for faster locomotion
 */
PoseConfiguration getHighSpeedPoseConfig(const Parameters &params);

#endif // POSE_CONFIG_FACTORY_H
