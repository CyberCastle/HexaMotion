.RECIPEPREFIX := >
CXX=g++
CXXFLAGS=-I../include -I. -I/usr/include/eigen3 -std=c++17

all: math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test

math_utils_test: math_utils_test.cpp ../src/math_utils.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

model_test: model_test.cpp ../src/math_utils.cpp ../src/robot_model.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

pose_controller_test: pose_controller_test.cpp ../src/pose_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

walk_controller_test: walk_controller_test.cpp ../src/walk_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

admittance_controller_test: admittance_controller_test.cpp ../src/admittance_controller.cpp ../src/robot_model.cpp ../src/math_utils.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

locomotion_system_test: locomotion_system_test.cpp ../src/locomotion_system.cpp ../src/robot_model.cpp ../src/pose_controller.cpp ../src/walk_controller.cpp ../src/admittance_controller.cpp ../src/math_utils.cpp
>$(CXX) $(CXXFLAGS) $^ -o $@

clean:
>rm -f math_utils_test model_test pose_controller_test walk_controller_test admittance_controller_test locomotion_system_test
