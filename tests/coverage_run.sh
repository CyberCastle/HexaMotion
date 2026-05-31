#!/bin/bash
# Temporary coverage harness: compiles every src/*.cpp once with --coverage into
# shared objects, links every test against them, runs all tests so gcda data
# accumulates, then emits per-source gcov reports.
set -u
cd "$(dirname "$0")"

EIGEN_PATH=/tmp/eigen-master
CXX=g++
CXXFLAGS="-I../src -I. -I$EIGEN_PATH -std=c++17 -DTESTING_ENABLED -DCOXA_STRIDE_TESTING_ENABLED -O0 -g --coverage"

OBJ=cov_obj
rm -rf "$OBJ"
mkdir -p "$OBJ"

SRC_FILES=$(ls ../src/*.cpp)

echo "== Compiling source objects (instrumented) =="
SRC_OBJS=""
for s in $SRC_FILES; do
    b=$(basename "$s" .cpp)
    o="$OBJ/$b.o"
    $CXX $CXXFLAGS -c "$s" -o "$o" || { echo "FAILED compiling $s"; exit 1; }
    SRC_OBJS="$SRC_OBJS $o"
done

TESTS="numeric_integration_test ik_test dh_kinematics_test bezier_test coxa_stride_test trajectory_test tripod_walk_test runtime_api_test math_utils_test pose_controller_test walk_controller_test kinematics_validation_test brute_force_workspace_test jacobian_validation_test pose_gait_integration_test finetune_angles_test virtual_hardware_sim_test workspace_analyzer_fusion_test step_frequency_regeneration_test default_configuration_tripod_pose_transition_test gravity_inclination_test timing_rounding_parity_test force_normal_touchdown_parity_test strict_parity_legstepper_test rough_terrain_adaptation_test walk_plane_all_legs_test manual_leg_toggle_dual_test cartesian_velocity_controller_test accessor_coverage_test terrain_adaptation_test leg_stepper_accessor_test factory_pose_admittance_test walk_controller_accessor_test"

echo "== Building and running each test =="
PASS=0; FAIL=0; FAILED=""
for t in $TESTS; do
    exe="$OBJ/$t"
    $CXX $CXXFLAGS "$t.cpp" $SRC_OBJS -o "$exe" 2> "$OBJ/$t.build.log"
    if [ $? -ne 0 ]; then
        echo "BUILD-FAIL: $t"; FAILED="$FAILED build:$t"; FAIL=$((FAIL+1)); continue
    fi
    ( cd "$OBJ" && ./"$t" > "$t.run.log" 2>&1 )
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "RUN-FAIL: $t (rc=$rc)"; FAILED="$FAILED run:$t"; FAIL=$((FAIL+1))
    else
        echo "OK: $t"; PASS=$((PASS+1))
    fi
done
echo "PASS=$PASS FAIL=$FAIL FAILED=$FAILED"

echo "== gcov per source =="
( cd "$OBJ" && gcov -b -n *.gcno 2>/dev/null > gcov_summary.txt )
echo "Done. Summary in $OBJ/gcov_summary.txt"
