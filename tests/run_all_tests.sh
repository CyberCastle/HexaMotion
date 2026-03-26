#!/bin/bash
cd /Users/cybercastle/tmp/HexaMotion/tests
PASS=0
FAIL=0
FAILED=""
for t in *_test; do
    ./"$t" > /dev/null 2>&1
    rc=$?
    if [ $rc -ne 0 ]; then
        FAIL=$((FAIL+1))
        FAILED="$FAILED\nFAIL: $t (rc=$rc)"
    else
        PASS=$((PASS+1))
    fi
done
echo "PASS=$PASS FAIL=$FAIL"
echo -e "$FAILED"
