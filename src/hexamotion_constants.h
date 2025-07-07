#ifndef HEXAMOTION_CONSTANTS_H
#define HEXAMOTION_CONSTANTS_H

/**
 * @file hexamotion_constants.h
 * @brief Global constants for HexaMotion robot system
 *
 * This file contains all the commonly used constants throughout the HexaMotion
 * system, including velocity control, angular limits, and robot geometry values.
 * Centralizing these constants improves maintainability and consistency.
 */

// ========================================================================
// VELOCITY CONTROL CONSTANTS
// ========================================================================

// Servo speed limits and defaults
#define SERVO_SPEED_MIN 0.1f     // Minimum servo speed multiplier
#define SERVO_SPEED_MAX 3.0f     // Maximum servo speed multiplier
#define SERVO_SPEED_DEFAULT 1.0f // Default servo speed multiplier

// Velocity scaling limits
#define VELOCITY_SCALE_MIN 0.1f // Minimum velocity scaling factor
#define VELOCITY_SCALE_MAX 5.0f // Maximum velocity scaling factor

// Speed ratio limits
#define SPEED_RATIO_MIN 0.05f            // Minimum speed ratio (5%)
#define SPEED_RATIO_MAX_VALIDATION 0.95f // Maximum for minimum_speed_ratio validation
#define SPEED_RATIO_MIN_VALIDATION 1.05f // Minimum for maximum_speed_ratio validation
#define SPEED_RATIO_MAX 3.0f             // Maximum speed ratio (300%)

// Gait modifier limits
#define GAIT_MODIFIER_MIN 0.1f // Minimum gait speed modifier
#define GAIT_MODIFIER_MAX 2.0f // Maximum gait speed modifier

// Leg compensation limits
#define LEG_COMPENSATION_MIN 0.5f // Minimum leg speed compensation
#define LEG_COMPENSATION_MAX 2.0f // Maximum leg speed compensation

// Default velocity scaling factors
#define DEFAULT_LINEAR_VELOCITY_SCALE 2.0f  // Default linear velocity scaling factor
#define DEFAULT_ANGULAR_VELOCITY_SCALE 1.5f // Default angular velocity scaling factor
#define DEFAULT_MIN_SPEED_RATIO 0.2f        // Default minimum speed ratio (20%)
#define DEFAULT_MAX_SPEED_RATIO 1.8f        // Default maximum speed ratio (180%)

// Default gait speed factors
#define DEFAULT_TRIPOD_SPEED_FACTOR 1.2f      // Default tripod gait speed factor
#define DEFAULT_WAVE_SPEED_FACTOR 0.8f        // Default wave gait speed factor
#define DEFAULT_RIPPLE_SPEED_FACTOR 0.9f      // Default ripple gait speed factor
#define DEFAULT_METACHRONAL_SPEED_FACTOR 1.0f // Default metachronal gait speed factor
#define DEFAULT_ADAPTIVE_SPEED_FACTOR 1.1f    // Default adaptive gait speed factor

// Velocity multipliers for different joints
#define FEMUR_VELOCITY_MULTIPLIER 1.1f // Femur velocity scaling multiplier
#define TIBIA_VELOCITY_MULTIPLIER 1.2f // Tibia velocity scaling multiplier

// ========================================================================
// ANGULAR POSITION CONSTANTS
// ========================================================================

// Joint angle limits (degrees)
#define COXA_ANGLE_MIN -65.0f  // Minimum coxa angle (degrees)
#define COXA_ANGLE_MAX 65.0f   // Maximum coxa angle (degrees)
#define FEMUR_ANGLE_MIN -75.0f // Minimum femur angle (degrees)
#define FEMUR_ANGLE_MAX 75.0f  // Maximum femur angle (degrees)
#define TIBIA_ANGLE_MIN -45.0f // Minimum tibia angle (degrees)
#define TIBIA_ANGLE_MAX 45.0f  // Maximum tibia angle (degrees)

// Robot geometry angles
#define FULL_ROTATION 360.0f    // Full rotation in degrees

// Inverse kinematics starting poses (degrees)
#define IK_PRIMARY_FEMUR_ANGLE -45.0f  // Primary IK guess femur angle
#define IK_PRIMARY_TIBIA_ANGLE 60.0f   // Primary IK guess tibia angle
#define IK_HIGH_FEMUR_ANGLE 30.0f      // High pose femur angle
#define IK_HIGH_TIBIA_ANGLE -60.0f     // High pose tibia angle
#define IK_EXTENDED_FEMUR_ANGLE -60.0f // Extended pose femur angle
#define IK_EXTENDED_TIBIA_ANGLE 80.0f  // Extended pose tibia angle

// ========================================================================
// ROBOT PARAMETER DEFAULTS
// ========================================================================

// Default velocity limits
#define DEFAULT_MAX_LINEAR_VELOCITY 200.0f // Default maximum linear velocity (mm/s)
#define DEFAULT_MAX_ANGULAR_VELOCITY 90.0f // Default maximum angular velocity (degrees/s)

// Control system defaults
#define DEFAULT_CONTROL_FREQUENCY 50.0 // Default control frequency (Hz)

// Velocity scaling and coupling factors
#define DEFAULT_ANGULAR_SCALING 1.0     // Default angular velocity scaling
#define ANGULAR_LINEAR_COUPLING 0.3     // Coupling factor between angular and linear velocity
#define ANGULAR_ACCELERATION_FACTOR 2000.0 // Angular acceleration threshold (mm/s²)
#define WORKSPACE_SCALING_FACTOR 0.5    // Workspace scaling factor
#define WALKSPACE_SCALING_FACTOR 0.7    // Walkspace scaling factor
#define MIN_SERVO_VELOCITY 0.1          // Minimum servo velocity (10% of max)
#define FULL_ROTATION_DEGREES 360.0f     // Full rotation in degrees
#define HALF_ROTATION_DEGREES 180.0f     // Half rotation in degrees
#define BEARING_STEP_DEGREES 30.0f       // Bearing sampling step in degrees

// ========================================================================
// MATHEMATICAL CONSTANTS
// ========================================================================

#define DEGREES_TO_RADIANS_FACTOR (M_PI / 180.0f) // Conversion factor degrees to radians
#define RADIANS_TO_DEGREES_FACTOR (180.0f / M_PI) // Conversion factor radians to degrees

// Physics constants
#define GRAVITY_ACCELERATION 9806.65f // Standard gravity acceleration (mm/s²) - BIPM definition
                                      // Reference: https://en.wikipedia.org/wiki/Standard_gravity

// Sampling and analysis constants
#define RADIUS_STEP_DEFAULT 20.0f // Default radius step for sampling (mm)

// ========================================================================
// INVERSE KINEMATICS CONSTANTS
// ========================================================================

// DLS (Damped Least Squares) IK parameters (OpenSHC-style)
#define IK_DLS_COEFFICIENT 0.02f       // Damping factor for numerical stability in DLS method
#define IK_TOLERANCE 1.0f              // Position tolerance for IK convergence (1mm)
#define IK_DEFAULT_MAX_ITERATIONS 30   // Default maximum iterations for IK solver
#define IK_MAX_ANGLE_STEP 5.0f         // Maximum angle change per IK iteration (degrees)

// Workspace analysis parameters
#define WORKSPACE_RESOLUTION 10 // Discretization resolution for workspace analysis

// ========================================================================
// TERRAIN ADAPTATION CONSTANTS
// ========================================================================

// Default FSR threshold values for terrain contact detection
#define DEFAULT_FSR_TOUCHDOWN_THRESHOLD 10.0f // Default FSR touchdown threshold (N or ADC units)
#define DEFAULT_FSR_LIFTOFF_THRESHOLD 5.0f    // Default FSR liftoff threshold (N or ADC units)

// Terrain analysis parameters
#define STEP_DEPTH_DEFAULT 20.0f   // Default step depth for terrain analysis (mm)
#define STEP_PLANE_CONFIDENCE 0.8f // Default confidence level for step plane validation

// ========================================================================
// TOLERANCE AND PRECISION CONSTANTS
// ========================================================================

#define FLOAT_TOLERANCE 1e-6f     // Standard floating point tolerance
#define ANGLE_TOLERANCE 0.1f      // Angular tolerance (degrees)
#define POSITION_TOLERANCE 1.0f   // Position tolerance (mm)
#define VELOCITY_THRESHOLD 1.0f // Minimum velocity to consider as moving (mm/s)

// ========================================================================
// SYSTEM STATE CONSTANTS
// ========================================================================

// System states (OpenSHC equivalent)
enum SystemState {
    SYSTEM_UNKNOWN = 0,  // Unknown state
    SYSTEM_PACKED = 1,   // Robot is packed/disabled
    SYSTEM_READY = 2,    // Robot is ready but not walking
    SYSTEM_RUNNING = 3   // Robot is running/walking
};

// Startup sequence constants
#define STARTUP_SEQUENCE_TIME 6.0f      // Time for startup sequence (seconds)
#define SHUTDOWN_SEQUENCE_TIME 4.0f     // Time for shutdown sequence (seconds)
#define STANCE_TRANSITION_HEIGHT 30.0f  // Height for stance transition (mm)
#define STANCE_TRANSITION_TIME 0.5f     // Time for stance transition (seconds)

// Progress constants
#define PROGRESS_COMPLETE 100    // Sequence completed
#define PROGRESS_GENERATING -1   // Sequence is being generated

#endif // HEXAMOTION_CONSTANTS_H
