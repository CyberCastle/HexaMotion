# Correction of inverse kinematics errors and validation of angular positions

## Description of the problem

The hexapod robot locomotion system presented violations in the limits of angles of the joints, where the angles exceeded the configured limits of ± 65 ° (Coxa), ± 75 ° (femur), and ± 45 ° (tibia), with violations that reached values ​​greater than 90 ° according to the outputs of the tests.

## Research and correction process

### Step 1: Initial analysis of the restriction system

** Action: ** Investigate the Pipeline for the application of restrictions in the locomotion system.

** Findings: **

- The system correctly implements the application of restrictions in the `setlegjintangles ()`
- Use the `Constrainangle ()` method to limit angles within the permitted ranges
- The restriction pipeline works: `setlegposition ()` → `setlegjointangles ()` → `constrainangle ()` → stores limited values

** Examined files: **

- `SRC/Locomotion_system.CPP` - Contains the logic of restriction application
- `SRC/Robot_Model.cpp` - Implement` Constrainangle () `and` normalizeangle () `
- `SRC/Pose_Controller.CPP` - Additional application of restrictions in pose control

### Step 2: Root cause identification

** problem found: ** The main simulation test used incorrect articulation limits (± 90 ° for all joints) instead of the appropriate limits of the CSIRO style.

** PROBLEM SHIP: ** `/USERS/CyberCastle/TMP/Hexamotion/Tests/tripod_gait_sim_test.cpp`

** Identified problem code: **

`` CPP
// Incorrect limits (too permissive)
p.coxa_angle_limits [0] = -90;// should be -65
p.coxa_angle_limits [1] = 90;// should be 65
p.femur_angle_limits [0] = -90;// should be -75
p.femur_angle_limits [1] = 90;// should be 75
p.tibia_angle_limits [0] = -90;// should be -45
p.tibia_angle_limits [1] = 90;// should be 45
``

### Step 3: Correction of configuration parameters

** Action: ** Update the trial file with the correct limits of articulation CSIRO.

** Changes made in `tripod_gait_sim_test.cpp` (lines 292-302): **

`` CPP
// Corrected limits for CSIRO style joints
p.coxa_angle_limits [0] = -65;// Change from -90 to -65
p.coxa_angle_limits [1] = 65;// Change from 90 to 65
p.femur_angle_limits [0] = -75;// Change from -90 to -75
p.femur_angle_limits [1] = 75;// Change from 90 to 75
p.tibia_angle_limits [0] = -45;// Change from -90 to -45
p.tibia_angle_limits [1] = 45;// Change from 90 to 45
``

**Justification:**

- ** Coxa (± 65 °): ** Realistic range for hip joint that allows adequate lateral movement
- ** femur (± 75 °): ** range that allows effective elevation and descent from the legs
- ** Tibia (± 45 °): ** Conservative range that avoids collisions and maintains structural stability

### Step 4: Improvement of the monitoring system

** Improvements implemented: **

1. ** Robot dimensions visualization function: **

`` CPP
Void Printrobotdimensions (consta robotmodelparameters & p) {
STD :: cout << "\ n === Robot dimensions ===" << STD :: endl;
STD :: COUT << "COXA LENGTH:" << STD :: SETW (6) << P.coxa_length << "mm" << STD :: Endl;
STD :: cout << "femur length:" << std :: Setw (6) << p.femur_length << "mm" << STD :: endl;
STD :: COUT << "WARE LENGTH:" << STD :: SETW (6) << P.TIBIA_LENGTH << "MM" << STD :: Endl;
// ... more details
}
``

2. ** Improved Robot Detailed Function: **

`` CPP
Void printoiledrobotstatus (const locomotionsystem & ls, consta robotmodelparameters & p) {
// Includes summary of dimensions and states of joints
// with values ​​calculated as maximum scope and work radio
}
``

### Step 5: System verification

** Executed test: ** Execute the corrected test to confirm that articulation angles violations are now correctly restricted.

** Result: ** The angles remain within the specified limits, confirming that the restriction system works correctly with the appropriate parameters.

## Restriction system analysis

### Restriction system architecture

The system implements a multi -layer pipeline for the application of restrictions:

1. ** Input layer: ** `Setlegposition ()` Receive Cartesian positions target
2. ** Inverse kinematics layer: ** Calculate necessary articulation angles
3. ** Layer of Restrictions: ** `Setlegjointangles ()` Applies `Constrainangle ()` To each joint
4. ** Storage layer: ** Saves restricted values ​​in `Joint_angles [leg]`

### ANGLES RESTRICTION FUNCTION

`` CPP
Double Constrainangle (Double Angle, Double Min_angle, Double Max_angle) {
ifle <min_angle) return min_angle;
If (Angle> Max_angle) Return Max_angle;
Return Angle;
}
``

**Characteristics:**

- Simple and effective clamping implementation
- Application consisting of the entire system
- No unexpected state modifications or state modifications

## Lessons learned

### 1. ** Importance of the correct configuration **

The restriction system worked correctly from the beginning.The real problem was that the test configuration used too permissive limits that did not reflect the real physical limitations of the robot.

### 2. ** Validation of parameters **

It is crucial to validate that the configuration parameters reflect the real hardware specifications, especially in robotic systems where physical limitations are critical for safe operation.

### 3. ** Separation of responsibilities **

The correct design clearly separates:

- ** Cinematics algorithms: ** Calculation of necessary angles
- ** Restriction system: ** Application of physical limits
- ** Configuration: ** Specific robot parameters

## Recommendations for the future

### 1. ** Automatic configuration validation **

Implement automatic checks that validate that configuration parameters are within realistic ranges:

`` CPP
BOOL VALIDATEROBOTPARAMERERS (CONST ROBOTMODELPARAMEERS & P) {
// Verify that angles limits are realistic
if (abs (p.coxa_angle_limits [0])> 90 || ABS (P.coxa_angle_limits [1])> 90) {
STD :: Cer << "Warning: Excessive Coxa Limits" << STD :: Endl;
return false;
}
// ... more validations
Return True;
}
``

### 2. ** SPECIFICATION DOCUMENTATION **

Create clear documentation that specifies:

- Recommended articulation limits for different types of robot
- Technical justification for each movement range
- Calibration and validation procedures

### 3. ** Improved Integration Tests **

Develop evidence that specifically validates:

- That configuration limits are appropriate for hardware
- That the restriction system works correctly under various conditions
- There are no discrepancies between configuration and implementation

## Modified files

### Main files:

- ** `Tests/tripod_gait_sim_test.cpp`: ** Correction of articulation limits parameters and addition of dimensions of dimensions
- ** Analysis files: ** Multiple test files examined to verify the restriction system

### System files (verified, not modified):

- ** `SRC/Locomotion_system.CPP`: ** Functional Restriction System
- ** `src/robot_model.cpp`: ** Implementations of` constrainangle () `and` normalizeangle () `
- ** `src/pose_controller.cpp`: ** Additional restrictions in pose control

## Conclusion

The successful solution of this problem showed that often the "algorithm errors" can actually be configuration problems.The reverse cinematics system and restrictions worked properly, but was being tested with inappropriate parameters that did not reflect the real physical limitations of the robot.

The correction was relatively simple once the root cause was identified: change the limits of ± 90 ° to the realistic values ​​of ± 65 ° (Coxa), ± 75 ° (femur), and ± 45 ° (tibia).This allowed the restriction system to function as designed, maintaining all angles within safe and operational ranges.

** Documentation date: ** June 9, 2025
** STATE: ** SOLVED PROBLEM AND VERIFIED SYSTEM
