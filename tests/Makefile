CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../src -I. -I$(EIGEN_PATH) -std=c++17 -DTESTING_ENABLED -DCOXA_STRIDE_TESTING_ENABLED

SOURCES = ../src/math_utils.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/cartesian_velocity_controller.cpp ../src/workspace_analyzer.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp ../src/stride_deviation_limits.cpp

# ----------------------------------------------------------------------------
# Consolidated test executables (see AGENTS.md "Test coverage matrix").
# Each consolidated target links a thin driver (run_*() dispatcher) together
# with the original per-test translation units. The original tests keep all of
# their coverage: their former `int main()` is renamed to `int run_<name>()`
# and invoked by the driver. No coverage was removed during consolidation.
# ----------------------------------------------------------------------------
CONSOLIDATED = numeric_integration_test ik_test dh_kinematics_test bezier_test coxa_stride_test trajectory_test tripod_walk_test runtime_api_test

# Standalone tests that were NOT consolidated.
STANDALONE = math_utils_test pose_controller_test walk_controller_test kinematics_validation_test brute_force_workspace_test jacobian_validation_test pose_gait_integration_test finetune_angles_test virtual_hardware_sim_test workspace_analyzer_fusion_test step_frequency_regeneration_test default_configuration_tripod_pose_transition_test gravity_inclination_test timing_rounding_parity_test force_normal_touchdown_parity_test strict_parity_legstepper_test rough_terrain_adaptation_test walk_plane_all_legs_test manual_leg_toggle_dual_test cartesian_velocity_controller_test accessor_coverage_test terrain_adaptation_test leg_stepper_accessor_test factory_pose_admittance_test walk_controller_accessor_test

all: $(CONSOLIDATED) $(STANDALONE)

# ======================== Consolidated targets ========================

# Numeric integration & quaternion math (Runge-Kutta + quaternion functions).
numeric_integration_test: numeric_integration_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Inverse kinematics suite (simple, advanced, tracking diagnostic, internal loop).
ik_test: ik_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# DH / analytic kinematics suite (simple DH, DH-vs-analytic, physical offset, joint calibration).
dh_kinematics_test: dh_kinematics_test.cpp ../src/analytic_robot_model.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Bezier curve suite (validation, deterministic, transition single/all legs).
bezier_test: bezier_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Coxa / stride suite (phase transition, decomposition, tripod symmetry, swing orientation, stride vector, deviation limits).
coxa_stride_test: coxa_stride_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Trajectory suite (tip position, all legs, hexapod analysis).
trajectory_test: trajectory_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Tripod walk suite (linearity, visualization).
tripod_walk_test: tripod_walk_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Runtime API suite (runtime parameter setter, convenience motion API, publication accessor).
runtime_api_test: runtime_api_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# ======================== Standalone targets ========================

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

gravity_inclination_test: gravity_inclination_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

kinematics_validation_test: kinematics_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) -I../src -I. -I/usr/local/opt/eigen/include/eigen3 $^ -o $@

brute_force_workspace_test: brute_force_workspace_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

jacobian_validation_test: jacobian_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_gait_integration_test: pose_gait_integration_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp ../src/state_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

finetune_angles_test: finetune_angles_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/analytic_robot_model.cpp ../src/body_pose_config_factory.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

virtual_hardware_sim_test: virtual_hardware_sim_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/cartesian_velocity_controller.cpp ../src/admittance_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

workspace_analyzer_fusion_test: workspace_analyzer_fusion_test.cpp ../src/workspace_analyzer.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/velocity_limits.cpp ../src/gait_config_factory.cpp ../src/leg_stepper.cpp ../src/leg.cpp ../src/body_pose_config_factory.cpp ../src/leg_poser.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/body_pose_controller.cpp ../src/admittance_controller.cpp ../src/cartesian_velocity_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

step_frequency_regeneration_test: step_frequency_regeneration_test.cpp ../src/gait_config_factory.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

default_configuration_tripod_pose_transition_test: default_configuration_tripod_pose_transition_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/cartesian_velocity_controller.cpp ../src/admittance_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

# --- OpenSHC 1:1 parity reconciliation tests ---
timing_rounding_parity_test: timing_rounding_parity_test.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/velocity_limits.cpp ../src/terrain_adaptation.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

force_normal_touchdown_parity_test: force_normal_touchdown_parity_test.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/velocity_limits.cpp ../src/terrain_adaptation.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

strict_parity_legstepper_test: strict_parity_legstepper_test.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/velocity_limits.cpp ../src/terrain_adaptation.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

rough_terrain_adaptation_test: rough_terrain_adaptation_test.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_plane_all_legs_test: walk_plane_all_legs_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

manual_leg_toggle_dual_test: manual_leg_toggle_dual_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# --- TODO_TEST_COVERAGE additions ---

# CartesianVelocityController dedicated coverage ([A]).
cartesian_velocity_controller_test: cartesian_velocity_controller_test.cpp ../src/cartesian_velocity_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

# Accessor/mutator coverage for LocomotionSystem/StateController/Leg/RobotModel ([C][F][H][K]).
accessor_coverage_test: accessor_coverage_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# TerrainAdaptation proactive/reactive/projection coverage ([D]).
terrain_adaptation_test: terrain_adaptation_test.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/leg_stepper.cpp ../src/velocity_limits.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

# LegStepper getter/setter and trajectory-helper coverage ([J]).
leg_stepper_accessor_test: leg_stepper_accessor_test.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/velocity_limits.cpp ../src/terrain_adaptation.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

# gait_config_factory, body_pose_controller residuals, admittance updateStiffness
# and auto_poser coverage ([G][I][L][M]).
factory_pose_admittance_test: factory_pose_admittance_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

# WalkController residual accessor/gait/terrain coverage ([B]).
walk_controller_accessor_test: walk_controller_accessor_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(CONSOLIDATED) $(STANDALONE)
