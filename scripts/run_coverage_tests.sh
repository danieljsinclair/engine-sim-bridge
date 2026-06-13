#!/bin/bash
# Run all test binaries with LLVM coverage profiling, merge, and export
set -e

BUILD_DIR="$1"
PROFRAW_DIR="$BUILD_DIR/profraw"
LLVM_COV="${LLVM_COV:-$(xcrun --find llvm-cov 2>/dev/null || which llvm-cov 2>/dev/null)}"
LLVM_PROFDATA="${LLVM_PROFDATA:-$(xcrun --find llvm-profdata 2>/dev/null || which llvm-profdata 2>/dev/null)}"

mkdir -p "$PROFRAW_DIR"

echo "=== Running tests with coverage instrumentation ==="
for test_bin in "$BUILD_DIR"/*test*; do
    if [ -x "$test_bin" ] && [ -f "$test_bin" ]; then
        echo "  Running: $(basename $test_bin)"
        LLVM_PROFILE_FILE="$PROFRAW_DIR/coverage-%p.profraw" "$test_bin" || true
    fi
done

echo "=== Merging coverage profiles ==="
$LLVM_PROFDATA merge -sparse "$PROFRAW_DIR"/*.profraw -o "$BUILD_DIR/coverage.profdata"

echo "=== Generating llvm-cov text report ==="
# Build -object args for all test binaries
OBJECT_ARGS=""
MAIN_BIN=""
for test_bin in "$BUILD_DIR"/*test*; do
    if [ -x "$test_bin" ] && [ -f "$test_bin" ]; then
        if [ -z "$MAIN_BIN" ]; then
            MAIN_BIN="$test_bin"
        else
            OBJECT_ARGS="$OBJECT_ARGS -object $test_bin"
        fi
    fi
done

$LLVM_COV show --show-branches=count \
    -instr-profile "$BUILD_DIR/coverage.profdata" \
    "$MAIN_BIN" \
    $OBJECT_ARGS \
    > "$BUILD_DIR/coverage.txt" 2>/dev/null || true

echo "=== Coverage report generated ==="
wc -l "$BUILD_DIR/coverage.txt" | awk '{print "Lines:", $1}'
