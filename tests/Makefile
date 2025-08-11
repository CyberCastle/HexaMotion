CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../src -I. -I$(EIGEN_PATH) -std=c++17 -DTESTING_ENABLED

SOURCES = ../src/math_utils.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp ../src/cartesian_velocity_controller.cpp ../src/workspace_analyzer.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp

all: math_utils_test pose_controller_test walk_controller_test quaternion_functions_test kinematics_validation_test simple_dh_test simple_ik_test simple_advanced_ik_test brute_force_workspace_test jacobian_validation_test dh_vs_analytic_test pose_gait_integration_test trajectory_tip_position_test trajectory_all_legs_test finetune_angles_test tripod_walk_visualization_test tripod_linearity_test virtual_hardware_sim_test hexapod_trajectory_analysis_test workspace_analyzer_fusion_test bezier_validation_test test_complete_physical_offset

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/body_pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

quaternion_functions_test: quaternion_functions_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

kinematics_validation_test: kinematics_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/body_pose_config_factory.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) -I../src -I. -I/usr/local/opt/eigen/include/eigen3 $^ -o $@

simple_dh_test: simple_dh_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

simple_ik_test: simple_ik_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

simple_advanced_ik_test: simple_advanced_ik_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/leg.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

dh_vs_analytic_test: dh_vs_analytic_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/analytic_robot_model.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) dh_vs_analytic_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/analytic_robot_model.cpp ../src/workspace_analyzer.cpp -o dh_vs_analytic_test

brute_force_workspace_test: brute_force_workspace_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

jacobian_validation_test: jacobian_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_gait_integration_test: pose_gait_integration_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/body_pose_controller.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/admittance_controller.cpp ../src/velocity_limits.cpp ../src/cartesian_velocity_controller.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/body_pose_config_factory.cpp ../src/body_pose.cpp ../src/leg.cpp ../src/leg_poser.cpp ../src/gait_config_factory.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

finetune_angles_test: finetune_angles_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/analytic_robot_model.cpp ../src/body_pose_config_factory.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

trajectory_tip_position_test: trajectory_tip_position_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

trajectory_all_legs_test: trajectory_all_legs_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_walk_visualization_test: tripod_walk_visualization_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/cartesian_velocity_controller.cpp ../src/admittance_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_linearity_test: tripod_linearity_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/cartesian_velocity_controller.cpp ../src/admittance_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

virtual_hardware_sim_test: virtual_hardware_sim_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp ../src/cartesian_velocity_controller.cpp ../src/admittance_controller.cpp ../src/state_controller.cpp ../src/locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

hexapod_trajectory_analysis_test: hexapod_trajectory_analysis_test.cpp ../src/walk_controller.cpp ../src/leg_stepper.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/velocity_limits.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/leg.cpp ../src/gait_config_factory.cpp ../src/body_pose_config_factory.cpp ../src/body_pose_controller.cpp ../src/leg_poser.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

workspace_analyzer_fusion_test: workspace_analyzer_fusion_test.cpp ../src/workspace_analyzer.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/velocity_limits.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

bezier_validation_test: bezier_validation_test.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

test_complete_physical_offset: test_complete_physical_offset.cpp ../src/robot_model.cpp ../src/math_utils.cpp ../src/workspace_analyzer.cpp ../src/velocity_limits.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f math_utils_test pose_controller_test walk_controller_test quaternion_functions_test kinematics_validation_test simple_dh_test simple_ik_test simple_advanced_ik_test brute_force_workspace_test jacobian_validation_test dh_vs_analytic_test pose_gait_integration_test trajectory_tip_position_test trajectory_all_legs_test finetune_angles_test tripod_walk_visualization_test tripod_linearity_test virtual_hardware_sim_test hexapod_trajectory_analysis_test workspace_analyzer_fusion_test bezier_validation_test test_complete_physical_offset
