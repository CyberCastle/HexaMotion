CXX=g++
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EIGEN_PATH := $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
else
EIGEN_PATH := /usr/include/eigen3
endif
CXXFLAGS=-I../include -I. -I$(EIGEN_PATH) -std=c++17

all: velocity_limits_demo

velocity_limits_demo: velocity_limits_demo.cpp ../src/velocity_limits.cpp ../src/walk_controller.cpp ../src/terrain_adaptation.cpp ../src/robot_model.cpp ../src/math_utils.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f velocity_limits_demo
