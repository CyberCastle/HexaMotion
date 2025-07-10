CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../src -I. -I$(EIGEN_PATH) -std=c++17

SOURCES = ../src/math_utils.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/cartesian_velocity_controller.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp

all: math_utils_test model_test pose_controller_test leg_poser_bezier_test walk_controller_test walk_controller_validation_test admittance_controller_test locomotion_system_test state_controller_test cruise_control_comprehensive_test tripod_gait_sim_test tripod_gait_validation_test tripod_gait_detailed_analysis height_range_test geometry_test dls_validation_test workspace_analysis actual_workspace_test angle_range_test edge_case_analysis angle_normalization_test terrain_adaptation_test fsr_threshold_test velocity_limits_test state_controller_runtime_test openshc_equivalent_features_test full_feature_sim_test test_metachronal_adaptive_gaits final_validation_test quaternion_pose_system_test quaternion_functions_test pose_control_comprehensive_test pose_control_equivalence_test absolute_positioning_compile_test admittance_derivatives_test test_velocity_control test_smooth_movement ik_fk_validation_test simple_kinematics_test kinematics_validation_test collision_diagnostics_test simple_dh_test dh_base_position_test simple_ik_test brute_force_workspace_test jacobian_validation_test openshc_target_calculation_test walkspace_integration_test walkspace_workspace_integration_test transform_debug_test workspace_check_test comprehensive_kinematics_validation_test angle_test ik_analysis_test pose_controller_ik_test timing_test tripod_pose_consistency_test locomotion_position_validation_test pose_limits_test

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

model_test: model_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

body_pose_controller_gait_test: body_pose_controller_gait_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

leg_poser_bezier_test: leg_poser_bezier_test.cpp ../src/leg_poser.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/leg.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_validation_test: walk_controller_validation_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

admittance_controller_test: admittance_controller_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

locomotion_system_test: locomotion_system_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

state_controller_test: state_controller_test.cpp \
    ../src/state_controller.cpp \
    ../src/locomotion_system.cpp \
    ../src/robot_model.cpp \
    ../src/body_pose_controller.cpp \
    ../src/walk_controller.cpp \
    ../src/leg_stepper.cpp \
    ../src/terrain_adaptation.cpp \
    ../src/admittance_controller.cpp \
    ../src/velocity_limits.cpp \
    ../src/cartesian_velocity_controller.cpp \
    ../src/gait_config_factory.cpp \
    ../src/body_pose_config_factory.cpp \
    ../src/math_utils.cpp \
    ../src/workspace_validator.cpp \
    ../src/body_pose.cpp \
    ../src/leg.cpp \
    ../src/walkspace_analyzer.cpp \
    ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

tripod_gait_sim_test: tripod_gait_sim_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_gait_validation_test: tripod_gait_validation_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_gait_detailed_analysis: tripod_gait_detailed_analysis.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

height_range_test: height_range_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

geometry_test: geometry_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_fk_validation_test: ik_fk_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

dls_validation_test: dls_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

workspace_analysis: workspace_analysis.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

actual_workspace_test: actual_workspace_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test3_analysis: test3_analysis.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

angle_range_test: angle_range_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

edge_case_analysis: edge_case_analysis.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

angle_normalization_test: angle_normalization_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

terrain_adaptation_test: terrain_adaptation_test.cpp ../src/terrain_adaptation.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

fsr_threshold_test: fsr_threshold_test.cpp ../src/terrain_adaptation.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

velocity_limits_test: velocity_limits_test.cpp ../src/velocity_limits.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

state_controller_runtime_test: state_controller_runtime_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

openshc_equivalent_features_test: openshc_equivalent_features_test.cpp ../src/walkspace_analyzer.cpp ../src/admittance_controller.cpp ../src/imu_auto_pose.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

full_feature_sim_test: full_feature_sim_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/imu_auto_pose.cpp ../src/walkspace_analyzer.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test_metachronal_adaptive_gaits: test_metachronal_adaptive_gaits.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

final_validation_test: final_validation_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

quaternion_pose_system_test: quaternion_pose_system_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

quaternion_functions_test: quaternion_functions_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_control_equivalence_test: pose_control_equivalence_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

pose_control_comprehensive_test: pose_control_comprehensive_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $< $(SOURCES) -o $@

cruise_control_comprehensive_test: cruise_control_comprehensive_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $< $(SOURCES) -o $@

absolute_positioning_compile_test: absolute_positioning_compile_test.cpp ../src/imu_auto_pose.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

enhanced_imu_integration_test: enhanced_imu_integration_test.cpp ../src/terrain_adaptation.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

admittance_derivatives_test: admittance_derivatives_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test_velocity_control: test_velocity_control.cpp ../src/cartesian_velocity_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test_smooth_movement: test_smooth_movement.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

enhanced_kinematics_test: enhanced_kinematics_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

simple_kinematics_test: simple_kinematics_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

kinematics_validation_test: kinematics_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp
	$(CXX) $(CXXFLAGS) -I../src -I. -I/usr/local/opt/eigen/include/eigen3 $^ -o $@

collision_diagnostics_test: collision_diagnostics_test.cpp ../src/workspace_validator.cpp ../src/math_utils.cpp ../src/robot_model.cpp ../src/collision_diagnostics.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

simple_dh_test: simple_dh_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

dh_base_position_test: dh_base_position_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

simple_ik_test: simple_ik_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

brute_force_workspace_test: brute_force_workspace_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

jacobian_validation_test: jacobian_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

openshc_target_calculation_test: openshc_target_calculation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walkspace_integration_test: walkspace_integration_test.cpp ../src/walk_controller.cpp ../src/workspace_validator.cpp ../src/walkspace_analyzer.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/leg.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walkspace_workspace_integration_test: walkspace_workspace_integration_test.cpp ../src/walk_controller.cpp ../src/workspace_validator.cpp ../src/walkspace_analyzer.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/leg.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

transform_debug_test: transform_debug_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

workspace_check_test: workspace_check_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

comprehensive_kinematics_validation_test: comprehensive_kinematics_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

angle_test: angle_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_analysis_test: ik_analysis_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_ik_test: pose_controller_ik_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

timing_test: timing_test.cpp ../src/leg_stepper.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/workspace_validator.cpp ../src/gait_config_factory.cpp ../src/velocity_limits.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_pose_consistency_test: tripod_pose_consistency_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

locomotion_position_validation_test: locomotion_position_validation_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_validator.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/walkspace_analyzer.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_limits_test: pose_limits_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

leg_poser_auto_pose_test: leg_poser_auto_pose_test.cpp \
    ../src/robot_model.cpp ../src/leg.cpp ../src/body_pose_controller.cpp \
    ../src/leg_poser.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp
	g++ -I../src -I. -I/usr/local/opt/eigen/include/eigen3 -std=c++17 \
	leg_poser_auto_pose_test.cpp \
	../src/robot_model.cpp ../src/leg.cpp ../src/body_pose_controller.cpp \
	../src/leg_poser.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp \
	-o leg_poser_auto_pose_test

clean:
	rm -f math_utils_test model_test pose_controller_test leg_poser_bezier_test walk_controller_test walk_controller_validation_test admittance_controller_test locomotion_system_test state_controller_test tripod_gait_sim_test tripod_gait_validation_test tripod_gait_detailed_analysis height_range_test geometry_test dls_validation_test workspace_analysis actual_workspace_test test3_analysis angle_range_test edge_case_analysis angle_normalization_test terrain_adaptation_test fsr_threshold_test velocity_limits_test state_controller_runtime_test openshc_equivalent_features_test full_feature_sim_test test_metachronal_adaptive_gaits final_validation_test quaternion_pose_system_test quaternion_functions_test pose_control_comprehensive_test pose_control_equivalence_test cruise_control_comprehensive_test absolute_positioning_compile_test enhanced_imu_integration_test admittance_derivatives_test test_velocity_control test_smooth_movement ik_fk_validation_test enhanced_kinematics_test simple_kinematics_test kinematics_validation_test collision_diagnostics_test simple_dh_test simple_ik_test brute_force_workspace_test jacobian_validation_test openshc_target_calculation_test walkspace_integration_test walkspace_workspace_integration_test transform_debug_test workspace_check_test comprehensive_kinematics_validation_test angle_test timing_test tripod_pose_consistency_test locomotion_position_validation_test pose_limits_test


