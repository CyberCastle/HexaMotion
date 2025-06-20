CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../src -I. -I$(EIGEN_PATH) -std=c++17

SOURCES = ../src/math_utils.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/manual_pose_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp

all: math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test state_controller_test cruise_control_comprehensive_test tripod_gait_sim_test ik_debug_test height_range_test geometry_test ik_analytical_fix ik_coordinate_test coord_test joint_test ik_debug_detailed dls_validation_test ik_debug_analysis workspace_analysis actual_workspace_test comprehensive_ik_test angle_range_test edge_case_analysis angle_normalization_test terrain_adaptation_test velocity_limits_test state_controller_runtime_test openshc_equivalent_features_test full_feature_sim_test test_metachronal_adaptive_gaits final_validation_test quaternion_pose_system_test quaternion_functions_test pose_control_comprehensive_test pose_control_equivalence_test absolute_positioning_compile_test admittance_derivatives_test

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

model_test: model_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

admittance_controller_test: admittance_controller_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

locomotion_system_test: locomotion_system_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

state_controller_test: state_controller_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_gait_sim_test: tripod_gait_sim_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_debug_test: ik_debug_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

height_range_test: height_range_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

geometry_test: geometry_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_analytical_fix: ik_analytical_fix.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_coordinate_test: ik_coordinate_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

coord_test: coord_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

joint_test: joint_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_debug_detailed: ik_debug_detailed.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

dls_validation_test: dls_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

ik_debug_analysis: ik_debug_analysis.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

workspace_analysis: workspace_analysis.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

actual_workspace_test: actual_workspace_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

comprehensive_ik_test: comprehensive_ik_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test3_analysis: test3_analysis.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

angle_range_test: angle_range_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

edge_case_analysis: edge_case_analysis.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

angle_normalization_test: angle_normalization_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

terrain_adaptation_test: terrain_adaptation_test.cpp ../src/terrain_adaptation.cpp ../src/walk_controller.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

velocity_limits_test: velocity_limits_test.cpp ../src/velocity_limits.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

state_controller_runtime_test: state_controller_runtime_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

openshc_equivalent_features_test: openshc_equivalent_features_test.cpp ../src/walkspace_analyzer.cpp ../src/manual_pose_controller.cpp ../src/admittance_controller.cpp ../src/imu_auto_pose.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

full_feature_sim_test: full_feature_sim_test.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/manual_pose_controller.cpp ../src/imu_auto_pose.cpp ../src/walkspace_analyzer.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test_metachronal_adaptive_gaits: test_metachronal_adaptive_gaits.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

final_validation_test: final_validation_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

quaternion_pose_system_test: quaternion_pose_system_test.cpp ../src/pose_controller.cpp ../src/manual_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

quaternion_functions_test: quaternion_functions_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_control_equivalence_test: pose_control_equivalence_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

pose_control_comprehensive_test: pose_control_comprehensive_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $< $(SOURCES) -o $@

cruise_control_comprehensive_test: cruise_control_comprehensive_test.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $< $(SOURCES) -o $@

absolute_positioning_compile_test: absolute_positioning_compile_test.cpp ../src/imu_auto_pose.cpp ../src/robot_model.cpp ../src/manual_pose_controller.cpp ../src/pose_controller.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

enhanced_imu_integration_test: enhanced_imu_integration_test.cpp ../src/terrain_adaptation.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

admittance_derivatives_test: admittance_derivatives_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test state_controller_test tripod_gait_sim_test ik_debug_test height_range_test geometry_test ik_analytical_fix ik_coordinate_test coord_test joint_test ik_debug_detailed dls_validation_test ik_debug_analysis workspace_analysis actual_workspace_test comprehensive_ik_test test3_analysis angle_range_test edge_case_analysis angle_normalization_test terrain_adaptation_test velocity_limits_test state_controller_runtime_test openshc_equivalent_features_test full_feature_sim_test test_metachronal_adaptive_gaits final_validation_test quaternion_pose_system_test quaternion_functions_test pose_control_comprehensive_test pose_control_equivalence_test cruise_control_comprehensive_test absolute_positioning_compile_test enhanced_imu_integration_test admittance_derivatives_test
