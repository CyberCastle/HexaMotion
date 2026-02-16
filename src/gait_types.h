// Dedicated header for shared type enumerations to avoid circular dependencies
#ifndef GAIT_TYPES_H
#define GAIT_TYPES_H

/**
 * @brief Enumeration of supported gait types.
 */
enum GaitType {
    NO_GAIT,
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

/**
 * @brief Leg states for state machine control (OpenSHC equivalent)
 */
enum LegState {
    LEG_WALKING,                //< The leg is in a 'walking' state - participates in walking cycle
    LEG_MANUAL,                 //< The leg is in a 'manual' state - able to move via manual manipulation inputs
    LEG_STATE_COUNT,            //< Misc enum defining number of LegStates
    LEG_WALKING_TO_MANUAL = -1, //< The leg is in 'walking to manual' state
    LEG_MANUAL_TO_WALKING = -2, //< The leg is in 'manual to walking' state
};

/**
 * @brief Walk states for walk controller cycle (OpenSHC equivalent)
 */
enum WalkState {
    WALK_STARTING,    //< Transitioning from 'stopped' to 'moving'
    WALK_MOVING,      //< Primary walking state
    WALK_STOPPING,    //< Transitioning from 'moving' to 'stopped'
    WALK_STOPPED,     //< State whilst velocity input is zero
    WALK_STATE_COUNT, //< Misc enum defining number of Walk States
};

/**
 * @brief Designation for potential posing states used in auto-posing (OpenSHC equivalent).
 *
 * Shared between BodyPoseController, AutoPoser, StateController and WalkController.
 */
enum PosingState {
    POSING,             /**< Auto-poser objects should start their posing cycle. */
    STOP_POSING,        /**< Auto-poser objects should end their posing cycle. */
    POSING_COMPLETE,    /**< All auto-poser objects have completed their cycles. */
    POSING_STATE_COUNT, /**< Number of posing states. */
};

/** @brief Sequence execution types for pose controller transitions (OpenSHC equivalent). */
enum SequenceSelection {
    START_UP,       /**< Start-up sequence from ready to running. */
    SHUT_DOWN,      /**< Shut-down sequence from running to ready. */
    SEQUENCE_COUNT, /**< Number of sequence types. */
};

/** @brief Manual pose reset modes (OpenSHC equivalent). */
enum PoseResetMode {
    NO_RESET,              /**< No manual body pose reset requested. */
    Z_AND_YAW_RESET,       /**< Reset z translation or yaw rotation to zero. */
    X_AND_Y_RESET,         /**< Reset x or y translation to zero. */
    PITCH_AND_ROLL_RESET,  /**< Reset roll or pitch rotation to zero. */
    ALL_RESET,             /**< Reset all manual body posing to zero. */
    IMMEDIATE_ALL_RESET,   /**< Immediately reset all manual body posing to zero. */
    POSE_RESET_MODE_COUNT, /**< Number of pose reset modes. */
};

#endif // GAIT_TYPES_H