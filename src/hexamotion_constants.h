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

#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_LEGS 6
#define DEGREES_TO_RADIANS_FACTOR (M_PI / 180.0)

// ========================================================================
// VELOCITY CONTROL CONSTANTS
// ========================================================================

// Servo speed limits and defaults
#define SERVO_SPEED_MIN 0.1     // Minimum servo speed multiplier
#define SERVO_SPEED_MAX 3.0     // Maximum servo speed multiplier
#define SERVO_SPEED_DEFAULT 1.0 // Default servo speed multiplier

// Velocity scaling limits
#define VELOCITY_SCALE_MIN 0.1 // Minimum velocity scaling factor
#define VELOCITY_SCALE_MAX 5.0 // Maximum velocity scaling factor

// Speed ratio limits
#define SPEED_RATIO_MIN 0.05            // Minimum speed ratio (5%)
#define SPEED_RATIO_MAX_VALIDATION 0.95 // Maximum for minimum_speed_ratio validation
#define SPEED_RATIO_MIN_VALIDATION 1.05 // Minimum for maximum_speed_ratio validation
#define SPEED_RATIO_MAX 3.0             // Maximum speed ratio (300%)

// Gait modifier limits
#define GAIT_MODIFIER_MIN 0.1 // Minimum gait speed modifier
#define GAIT_MODIFIER_MAX 2.0 // Maximum gait speed modifier

// Leg compensation limits
#define LEG_COMPENSATION_MIN 0.5 // Minimum leg speed compensation
#define LEG_COMPENSATION_MAX 2.0 // Maximum leg speed compensation

// Default velocity scaling factors
#define DEFAULT_LINEAR_VELOCITY_SCALE 2.0  // Default linear velocity scaling factor
#define DEFAULT_ANGULAR_VELOCITY_SCALE 1.5 // Default angular velocity scaling factor
#define DEFAULT_MIN_SPEED_RATIO 0.2        // Default minimum speed ratio (20%)
#define DEFAULT_MAX_SPEED_RATIO 1.8        // Default maximum speed ratio (180%)

// Default gait speed factors
#define DEFAULT_TRIPOD_SPEED_FACTOR 1.2      // Default tripod gait speed factor
#define DEFAULT_WAVE_SPEED_FACTOR 0.8        // Default wave gait speed factor
#define DEFAULT_RIPPLE_SPEED_FACTOR 0.9      // Default ripple gait speed factor
#define DEFAULT_METACHRONAL_SPEED_FACTOR 1.0 // Default metachronal gait speed factor
#define DEFAULT_ADAPTIVE_SPEED_FACTOR 1.1    // Default adaptive gait speed factor

// Velocity multipliers for different joints
#define FEMUR_VELOCITY_MULTIPLIER 1.1 // Femur velocity scaling multiplier
#define TIBIA_VELOCITY_MULTIPLIER 1.2 // Tibia velocity scaling multiplier

// ========================================================================
// ANGULAR POSITION CONSTANTS
// ========================================================================

// Joint angle limits (degrees)
#define COXA_ANGLE_MIN -65.0  // Minimum coxa angle (degrees)
#define COXA_ANGLE_MAX 65.0   // Maximum coxa angle (degrees)
#define FEMUR_ANGLE_MIN -75.0 // Minimum femur angle (degrees)
#define FEMUR_ANGLE_MAX 75.0  // Maximum femur angle (degrees)
#define TIBIA_ANGLE_MIN -45.0 // Minimum tibia angle (degrees)
#define TIBIA_ANGLE_MAX 45.0  // Maximum tibia angle (degrees)

// Robot geometry angles
#define FULL_ROTATION 360.0 // Full rotation in degrees

// Inverse kinematics starting poses (degrees)
#define IK_PRIMARY_FEMUR_ANGLE -45.0  // Primary IK guess femur angle
#define IK_PRIMARY_TIBIA_ANGLE 60.0   // Primary IK guess tibia angle
#define IK_HIGH_FEMUR_ANGLE 30.0      // High pose femur angle
#define IK_HIGH_TIBIA_ANGLE -60.0     // High pose tibia angle
#define IK_EXTENDED_FEMUR_ANGLE -60.0 // Extended pose femur angle
#define IK_EXTENDED_TIBIA_ANGLE 80.0  // Extended pose tibia angle

// ========================================================================
// ROBOT PARAMETER DEFAULTS
// ========================================================================

// Default velocity limits
#define DEFAULT_MAX_LINEAR_VELOCITY 200.0 // Default maximum linear velocity (mm/s)
#define DEFAULT_MAX_ANGULAR_VELOCITY 90.0 // Default maximum angular velocity (degrees/s)

// Control system defaults
#define DEFAULT_CONTROL_FREQUENCY 50.0 // Default control frequency (Hz)
#define DEFAULT_STEP_FREQUENCY 1.0     // Default step frequency (Hz)

// ========================================================================
// GAIT CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Step length factors as percentage of leg reach (OpenSHC equivalent)
#define GAIT_TRIPOD_LENGTH_FACTOR 0.35      // 35% for speed (OpenSHC: balanced speed/stability)
#define GAIT_WAVE_LENGTH_FACTOR 0.25        // 25% for stability (OpenSHC: most stable)
#define GAIT_RIPPLE_LENGTH_FACTOR 0.30      // 30% for balance (OpenSHC: faster movement)
#define GAIT_METACHRONAL_LENGTH_FACTOR 0.28 // 28% for smooth motion (HexaMotion specific)
#define GAIT_ADAPTIVE_LENGTH_FACTOR 0.30    // 30% base for adaptation (HexaMotion specific)

// Step height factors as percentage of robot height (OpenSHC equivalent)
#define GAIT_TRIPOD_HEIGHT_FACTOR 0.30      // 30% for obstacle clearance
#define GAIT_WAVE_HEIGHT_FACTOR 0.20        // 20% for stability
#define GAIT_RIPPLE_HEIGHT_FACTOR 0.25      // 25% for balance
#define GAIT_METACHRONAL_HEIGHT_FACTOR 0.25 // 25% for smooth motion
#define GAIT_ADAPTIVE_HEIGHT_FACTOR 0.25    // 25% base for adaptation

// Safety limits as percentage of leg reach/height (OpenSHC equivalent)
#define GAIT_MIN_LENGTH_FACTOR 0.12 // 12% minimum step length
#define GAIT_MAX_LENGTH_FACTOR 0.50 // 50% maximum step length
#define GAIT_MIN_HEIGHT_FACTOR 0.12 // 12% minimum step height
#define GAIT_MAX_HEIGHT_FACTOR 0.40 // 40% maximum step height

// ========================================================================
// BODY POSE CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Default swing height factor for body pose configuration (OpenSHC default.yaml: swing_height default 0.020)
// This is gait-independent, unlike gait-specific step heights
#define BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR 0.10 // 10% of standing height (OpenSHC: 0.020m for typical robot)

// Body clearance factor (OpenSHC default.yaml: body_clearance 0.100)
#define BODY_POSE_DEFAULT_CLEARANCE_FACTOR 1.0 // 100% of standing height (OpenSHC equivalent)

// Velocity scaling and coupling factors
#define DEFAULT_ANGULAR_SCALING 1.0        // Default angular velocity scaling
#define ANGULAR_LINEAR_COUPLING 0.3        // Coupling factor between angular and linear velocity
#define ANGULAR_ACCELERATION_FACTOR 2000.0 // Angular acceleration threshold (mm/s²)
#define WORKSPACE_SCALING_FACTOR 0.5       // Workspace scaling factor
#define WALKSPACE_SCALING_FACTOR 0.7       // Walkspace scaling factor
#define MIN_SERVO_VELOCITY 0.1             // Minimum servo velocity (10% of max)
#define FULL_ROTATION_DEGREES 360.0        // Full rotation in degrees
#define HALF_ROTATION_DEGREES 180.0        // Half rotation in degrees
#define BEARING_STEP_DEGREES 30.0          // Bearing sampling step in degrees

// ========================================================================
// MATHEMATICAL CONSTANTS
// ========================================================================

#define DEGREES_TO_RADIANS_FACTOR (M_PI / 180.0) // Conversion factor degrees to radians
#define RADIANS_TO_DEGREES_FACTOR (180.0 / M_PI) // Conversion factor radians to degrees

// Physics constants
#define GRAVITY_ACCELERATION 9806.65 // Standard gravity acceleration (mm/s²) - BIPM definition
                                     // Reference: https://en.wikipedia.org/wiki/Standard_gravity

// Sampling and analysis constants
#define RADIUS_STEP_DEFAULT 20.0 // Default radius step for sampling (mm)

// ========================================================================
// INVERSE KINEMATICS CONSTANTS
// ========================================================================

// DLS (Damped Least Squares) IK parameters (OpenSHC-style)
#define IK_DLS_COEFFICIENT 0.02        // Damping factor for numerical stability in DLS method
#define IK_JOINT_LIMIT_COST_WEIGHT 0.1 // Gain used in determining cost weight for joints approaching limits (OpenSHC)
#define IK_TOLERANCE 1.0               // Position tolerance for IK convergence (1mm)
#define IK_DEFAULT_MAX_ITERATIONS 30   // Default maximum iterations for IK solver
#define IK_MAX_ANGLE_STEP 5.0          // Maximum angle change per IK iteration (degrees)

// Workspace analysis parameters (OpenSHC equivalent, converted to mm)
#define MAX_POSITION_DELTA 2.0      // Position delta increment for workspace generation (2mm, OpenSHC uses 0.002m)
#define MAX_WORKSPACE_RADIUS 1000.0 // Maximum radius in workspace polygon plane (1000mm, from OpenSHC 1.0m)
#define BEARING_STEP 45             // Bearing step for workspace generation (45°, from OpenSHC)
#define WORKSPACE_LAYERS 10         // Number of height layers for workspace generation (from OpenSHC)
#define WORKSPACE_RESOLUTION 10     // Discretization resolution for workspace analysis

// ========================================================================
// TERRAIN ADAPTATION CONSTANTS
// ========================================================================

// Default FSR threshold values for terrain contact detection
#define DEFAULT_FSR_TOUCHDOWN_THRESHOLD 10.0 // Default FSR touchdown threshold (N or ADC units)
#define DEFAULT_FSR_LIFTOFF_THRESHOLD 5.0    // Default FSR liftoff threshold (N or ADC units)

// Terrain analysis parameters
#define STEP_DEPTH_DEFAULT 20.0   // Default step depth for terrain analysis (mm)
#define STEP_PLANE_CONFIDENCE 0.8 // Default confidence level for step plane validation

// ========================================================================
// TOLERANCE AND PRECISION CONSTANTS
// ========================================================================

#define FLOAT_TOLERANCE 1e-6   // Standard floating point tolerance
#define ANGLE_TOLERANCE 0.1    // Angular tolerance (degrees)
#define POSITION_TOLERANCE 1.0 // Position tolerance (mm)
#define VELOCITY_THRESHOLD 1.0 // Minimum velocity to consider as moving (mm/s)

// ========================================================================
// AUTO-POSE CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Auto-pose phase conversion constants
#define AUTO_POSE_PHASE_CONVERSION_FACTOR 100.0   // Conversion factor from gait phase (0.0-1.0) to phase (0-100)
#define AUTO_POSE_GAIT_PHASE_THRESHOLD 0.5        // Threshold for determining tripod gait group phases
#define AUTO_POSE_BODY_COMPENSATION_REDUCTION 0.5 // Reduction factor for body-level compensation amplitudes

// Auto-pose default amplitudes (OpenSHC auto_pose.yaml equivalent)
#define AUTO_POSE_DEFAULT_ROLL_AMPLITUDE 0.015  // Default roll compensation amplitude (radians)
#define AUTO_POSE_DEFAULT_PITCH_AMPLITUDE 0.020 // Default pitch compensation amplitude (radians)
#define AUTO_POSE_DEFAULT_Z_AMPLITUDE 0.020     // Default Z compensation amplitude (millimeters)

// ========================================================================
// SYSTEM STATE CONSTANTS
// ========================================================================

// System states (OpenSHC equivalent)
enum SystemState {
    SYSTEM_UNKNOWN = 0, // Unknown state
    SYSTEM_PACKED = 1,  // Robot is packed/disabled
    SYSTEM_READY = 2,   // Robot is ready but not walking
    SYSTEM_RUNNING = 3  // Robot is running/walking
};

// Startup sequence constants
#define STARTUP_SEQUENCE_TIME 6.0     // Time for startup sequence (seconds)
#define SHUTDOWN_SEQUENCE_TIME 4.0    // Time for shutdown sequence (seconds)
#define STANCE_TRANSITION_HEIGHT 30.0 // Height for stance transition (mm)
#define STANCE_TRANSITION_TIME 0.5    // Time for stance transition (seconds)

// Progress constants
#define PROGRESS_COMPLETE 100  // Sequence completed
#define PROGRESS_GENERATING -1 // Sequence is being generated

#define LEG_STEPPER_DUTY_FACTOR 0.5
#define LEG_STEPPER_FLOAT_TOLERANCE 1e-6
#define LEG_STEPPER_MAX_SWING_VELOCITY 100.0
#define LEG_STEPPER_DEFAULT_NODE_SEPARATION 0.05
#define LEG_STEPPER_DEFAULT_TOUCHDOWN_VELOCITY -0.2
#define LEG_STEPPER_STANCE_NODE_SCALER -0.25
#define LEG_STEPPER_SWING_LATERAL_SHIFT 10.0
#define LEG_STEPPER_SWING_NODE_SCALER 0.25
#define LEG_STEPPER_TOUCHDOWN_NODE_MULTIPLIER 4.0
#define LEG_STEPPER_TOUCHDOWN_INTERPOLATION 0.5

// ========================================================================
// STANCE CONFIGURATION CONSTANTS (OpenSHC equivalent)
// ========================================================================

// Default stance radius factor (OpenSHC equivalent)
// OpenSHC uses 65% of total leg reach for conservative stance positioning
#define DEFAULT_STANCE_RADIUS_FACTOR 0.65 // 65% of leg reach for safe stance position

// Per-leg base orientation offsets in radians - symmetric for opposite leg pairs
// Pairs: (0,3)=(-30°,30°), (1,4)=(-90°,90°), (2,5)=(-150°,150°)
// This matches the DH parameter orientation
const double BASE_THETA_OFFSETS[NUM_LEGS] = {
    -30.0 * DEGREES_TO_RADIANS_FACTOR,
    -90.0 * DEGREES_TO_RADIANS_FACTOR,
    -150.0 * DEGREES_TO_RADIANS_FACTOR,
    150.0 * DEGREES_TO_RADIANS_FACTOR,
    90.0 * DEGREES_TO_RADIANS_FACTOR,
    30.0 * DEGREES_TO_RADIANS_FACTOR};

#endif // HEXAMOTION_CONSTANTS_H
