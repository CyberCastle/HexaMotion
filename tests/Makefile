CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../include -I. -I$(EIGEN_PATH) -std=c++17

all: math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test tripod_gait_sim_test

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

model_test: model_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

admittance_controller_test: admittance_controller_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

locomotion_system_test: locomotion_system_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/admittance_controller.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

tripod_gait_sim_test: tripod_gait_sim_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/admittance_controller.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test tripod_gait_sim_test
