CXX=g++
CXXFLAGS=-I../include -std=c++17 -I.

all: utils_test
	
utils_test: utils_test.cpp ../src/hexapod_locomotion_system.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ -lstdc++

clean:
	rm -f utils_test
