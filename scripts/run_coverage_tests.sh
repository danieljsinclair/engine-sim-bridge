#!/bin/bash
# Run all test binaries with LLVM coverage profiling, merge, and export
set -e

BUILD_DIR="$1"
PROFRAW_DIR="$BUILD_DIR/profraw"
LLVM_COV="${LLVM_COV:-$(xcrun --find llvm-cov 2>/dev/null || which llvm-cov 2>/dev/null)}"
LLVM_PROFDATA="${LLVM_PROFDATA:-$(xcrun --find llvm-profdata 2>/dev/null || which llvm-profdata 2>/dev/null)}"

mkdir -p "$PROFRAW_DIR"

# Discover test binaries recursively so this works for both Make (binaries at
# $BUILD_DIR root) and Ninja (binaries in $BUILD_DIR/test/{unit,integration,...}
# subdirs) generators. Excludes build internals that aren't real test binaries:
# googletest artifacts under _deps have test-like names, as do CMakeFiles/
# CMakeScripts helpers and *.cmake/Makefile scripts. Sorted for determinism so
# MAIN_BIN (first) + OBJECT_ARGS (rest) are stable across runs.
TEST_BINS=$(find "$BUILD_DIR" -type f -name '*test*' -perm +111 \
    -not -path '*/_deps/*' \
    -not -path '*/CMakeFiles/*' \
    -not -path '*/CMakeScripts/*' \
    -not -name '*.cmake' \
    -not -name 'Makefile' \
    2>/dev/null | sort)

# Track failures without aborting early: every binary runs so all profraw is
# collected, but we exit non-zero at the end if any binary failed.
FAILED_COUNT=0
FAILED_BINS=""

echo "=== Running tests with coverage instrumentation ==="
for test_bin in $TEST_BINS; do
    if [ -x "$test_bin" ] && [ -f "$test_bin" ]; then
        echo "  Running: $(basename $test_bin)"
        if ! LLVM_PROFILE_FILE="$PROFRAW_DIR/coverage-%p.profraw" "$test_bin"; then
            echo "  FAILED: $(basename $test_bin) (exit non-zero) — continuing to collect remaining coverage"
            FAILED_COUNT=$((FAILED_COUNT + 1))
            FAILED_BINS="$FAILED_BINS $(basename $test_bin)"
        fi
    fi
done

echo "=== Merging coverage profiles ==="
$LLVM_PROFDATA merge -sparse "$PROFRAW_DIR"/*.profraw -o "$BUILD_DIR/coverage.profdata"

echo "=== Generating llvm-cov text report ==="
# Build -object args for all test binaries (reuse the recursive TEST_BINS list)
OBJECT_ARGS=""
MAIN_BIN=""
for test_bin in $TEST_BINS; do
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

echo "=== Generating lcov report (for coverage-gutters) ==="
$LLVM_COV export -format=lcov \
    -instr-profile "$BUILD_DIR/coverage.profdata" \
    "$MAIN_BIN" \
    $OBJECT_ARGS \
    > "$BUILD_DIR/lcov.info" 2>/dev/null || true

echo "=== Coverage report generated ==="
wc -l "$BUILD_DIR/coverage.txt" | awk '{print "Lines:", $1}'

# Honesty gate: if any test binary failed during the run loop above, surface it
# now (after coverage artefacts have been collected) and exit non-zero. The
# merge/export steps above completed successfully under `set -e`.
if [ "$FAILED_COUNT" -gt 0 ]; then
    echo "=== FAILED: $FAILED_COUNT test binary(ies) during coverage run:$FAILED_BINS ===" >&2
    exit 1
fi
