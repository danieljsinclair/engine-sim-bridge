.DEFAULT_GOAL := all
.PHONY: all build clean clean-test scrub help remove-orphans check test test-core test-isomorphism             test-deep presets clean-presets          sonar-scan sonar-clean coverage-clean coverage-run sonar-summary test-nosonar

BUILD_DIR ?= build
BUILD_TYPE ?= Release
# Set to 1 to allow Debug builds (needed for coverage instrumentation).
# Without this flag, any existing Debug CMakeCache.txt is a hard error.
ALLOW_DEBUG_BUILD ?= 0
CTEST_VERBOSE ?= 0
CTEST_QUIET ?= 0
CTEST_UI_FLAGS := $(if $(filter 1,$(CTEST_VERBOSE)),-V,$(if $(filter 1,$(CTEST_QUIET)),-Q,))
BRIDGE_TEST_CMAKE_FLAGS := \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DBUILD_PRESET_ENGINE_TESTS=ON \
	-DBUILD_IOS_ADAPTER_TESTS=ON \
	-DBUILD_PHASE0_SPIKES=OFF
BRIDGE_TEST_EXCLUDE := (_NOT_BUILT|_spike$$)
BRIDGE_TEST_ISOMORPHISM_MATCH := (IsomorphismFixtures/EndToEndPresetTest\.PresetLoadsAndProducesValidEngine|IsomorphismPresets/ParameterIsomorphismTest|AllEngines/RoundTripIsomorphismTest)
BRIDGE_TEST_CORE_EXCLUDE := $(BRIDGE_TEST_EXCLUDE)|($(BRIDGE_TEST_ISOMORPHISM_MATCH)|(EnginePresets/.*|Phase4FactoryPresets/.*|twin_foundation_tests))
CTEST_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
BUILD_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
CMAKE_BUILD_PARALLEL_FLAG := $(if $(strip $(BUILD_PARALLEL_LEVEL)),--parallel $(BUILD_PARALLEL_LEVEL),)
BUILD_STAMP := $(BUILD_DIR)/.build-ready.stamp
SONAR_STAMP := $(BUILD_DIR)/.sonar-scan.stamp
SONAR_PROJECT_PROPERTIES := sonar-project.properties
COMPILE_DB := $(BUILD_DIR)/compile_commands.json
COVERAGE_REPORT := $(BUILD_DIR)/coverage.txt
SONAR_TOKEN ?= ${SONAR_TOKEN_ES}

# Source inputs that affect the bridge build. This ensures the stamp is invalidated
# when bridge or engine-sim sources change, so rebuilds happen when necessary.
BUILD_INPUTS := $(shell find Makefile CMakeLists.txt src include test tools engine-sim -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' -o -name '*.cmake' \) | sort)

ISOMORPHISM_STAMP := $(BUILD_DIR)/.test-isomorphism.stamp

define build_bridge_targets
	cmake --build $(BUILD_DIR) $(CMAKE_BUILD_PARALLEL_FLAG)
endef

bridge_print_result = printf '\033[0;$(1)m=== [engine-sim-bridge] RESULT: $(2) ===\033[0m\n'

define bridge_print_hint
	if [ -n "$(strip $(SKIP_HINT_MESSAGE))" ]; then printf '%s\n' "$(SKIP_HINT_MESSAGE)"; fi;
endef

define run_bridge_ctest_suite
	@if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) $(CTEST_SELECTOR); then \
		echo "$(SUMMARY_PASS_MESSAGE)"; \
	else \
		echo "$(SUMMARY_FAIL_MESSAGE)"; \
		exit 1; \
	fi
endef

# Preset compilation
PRESET_DIR := preset
ENGINE_SIM_ROOT := engine-sim
PRESET_COMPILER := $(BUILD_DIR)/engine-sim-preset-compiler

# Default target - build + presets (test/coverage require explicit `make test`)
all: build presets

# Compile everything (cmake configure + build)
build: $(BUILD_STAMP)

# Guard: refuse to compile a Debug build unless ALLOW_DEBUG_BUILD=1.
# The physics solver is 3-5× slower in Debug; building without Release is
# almost always accidental (e.g. left behind by the coverage target, which
# reconfigures the bridge as Debug for llvm-cov instrumentation).
# Fix: run `make build` (Release) or `make scrub && build` (clean slate).
# Override (coverage/special): `make build ALLOW_DEBUG_BUILD=1`.
check_build_type = { if [ "$(ALLOW_DEBUG_BUILD)" != "1" ] && grep -q "CMAKE_BUILD_TYPE:STRING=Debug" $(BUILD_DIR)/CMakeCache.txt 2>/dev/null; then printf '\nERROR: bridge build dir is Debug. Physics 3-5x slower.\n  Run: make scrub && build  (clean slate)\n  Or:  make build ALLOW_DEBUG_BUILD=1  (override)\n\n'; exit 1; fi; }

$(BUILD_STAMP): $(BUILD_INPUTS) $(BUILD_DIR)/CMakeCache.txt
	+$(call check_build_type)
	+$(call build_bridge_targets)
	@touch $@

$(BUILD_DIR)/CMakeCache.txt: CMakeLists.txt
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake $(BRIDGE_TEST_CMAKE_FLAGS) -DCMAKE_BUILD_TYPE=Release ..

# Remove orphaned binaries, symlinks, and stray cmake junk from source dirs
remove-orphans:
	@rm -f *.dylib libenginesim*.dylib 2>/dev/null || true
	@find . -name "*.dylib*" -type l -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "CMakeFiles" -type d -print -exec rm -rf {} + 2>/dev/null || true
	@find . -path ./build -prune -o -name "CMakeCache.txt" -type f -print -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "cmake_install.cmake" -type f -print -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "CTestTestfile.cmake" -type f -print -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "*_include.cmake" -type f -print -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "*.a" -type f -print -delete 2>/dev/null || true
	@find . -path ./build -prune -o -name "_deps" -type d -print -exec rm -rf {} + 2>/dev/null || true

# Clean build artifacts (cascades to engine-sim via cmake)
clean: remove-orphans clean-presets clean-test-fixtures sonar-clean coverage-clean
	@if [ -d $(BUILD_DIR) ]; then cmake --build $(BUILD_DIR) --target clean >/dev/null 2>&1 || true; fi
	@rm -f $(BUILD_DIR)/*.stamp
	@rm -rf tmp
	@rm -rf .scannerwork

# Remove only stamp files so tests can be rerun without full clean.
clean-test:
	@rm -f $(BUILD_DIR)/.*.stamp $(BUILD_DIR)/*.stamp

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf $(BUILD_DIR) preset tmp
	@$(MAKE) remove-orphans
	@rm -rf .scannerwork

# Remove runtime-generated preset fixtures so next test run regenerates them.
# Needed if .mr source scripts change.
clean-test-fixtures:
	@echo "Removing cached preset fixtures from $(BUILD_DIR)/test/runtime-fixtures..."
	@rm -rf $(BUILD_DIR)/test/runtime-fixtures
	@echo "Done."

# Run the bridge suite in explicit tiers.
test: sonar-scan test-core test-deep sonar-summary

test-core: CTEST_SELECTOR := -E '$(BRIDGE_TEST_CORE_EXCLUDE)'
test-core: BLOCK_START_MESSAGE := === [engine-sim-bridge] START: core bridge suite (factory + preset regression coverage) ===
test-core: SKIP_HINT_MESSAGE := === [engine-sim-bridge] HINT: this is the always-on core suite in make test; there is no core-only skip target inside the safe path. Use app run-notest or run-quick only when you intentionally want to bypass the full test gate. ===
test-core: SUMMARY_PASS_MESSAGE := === [engine-sim-bridge] SUMMARY: PASS (core) ===
test-core: SUMMARY_FAIL_MESSAGE := === [engine-sim-bridge] SUMMARY: FAIL (core) ===
test-core: build presets
	$(run_bridge_ctest_suite)

test-isomorphism: CTEST_SELECTOR := -R '$(BRIDGE_TEST_ISOMORPHISM_MATCH)'
test-isomorphism: BLOCK_START_MESSAGE := === [engine-sim-bridge] START: isomorphism suite (preset parity + round-trip serialization) ===
test-isomorphism: SKIP_HINT_MESSAGE := === [engine-sim-bridge] HINT: skip this next time by running make test-core instead of make test or make test-deep. ===
test-isomorphism: SUMMARY_PASS_MESSAGE := === [engine-sim-bridge] SUMMARY: PASS (isomorphism) ===
test-isomorphism: SUMMARY_FAIL_MESSAGE := === [engine-sim-bridge] SUMMARY: FAIL (isomorphism) ===
test-isomorphism: $(ISOMORPHISM_STAMP)
	@:

test-deep: build test-isomorphism

# test-nosonar: full test gate (core + isomorphism) WITHOUT sonar/coverage
test-nosonar: test-core test-deep
test-quick: test-core

# ============================================================================
# SonarQube scan - runs before tests as a quality gate
# ============================================================================

# LLVM coverage: build static with llvm-cov flags, export lcov
# Uses xcrun (Xcode toolchain) with Homebrew LLVM fallback
LLVM_COV := $(shell xcrun --find llvm-cov 2>/dev/null || which llvm-cov 2>/dev/null)
LLVM_PROFDATA := $(shell xcrun --find llvm-profdata 2>/dev/null || which llvm-profdata 2>/dev/null)

# Coverage build: reconfigure + build with llvm-cov instrumentation
# Does NOT run tests — tests are run separately via make coverage-run
$(COVERAGE_REPORT): $(BUILD_STAMP)
	@echo "=== [engine-sim-bridge] Configuring coverage build ==="
	@cd $(BUILD_DIR) && cmake \
		-DCMAKE_BUILD_TYPE=Debug \
		-DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping -g" \
		-DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate" \
		-DBUILD_PRESET_ENGINE_TESTS=ON \
		-DBUILD_IOS_ADAPTER_TESTS=ON \
		-DBUILD_PHASE0_SPIKES=OFF \
		..
	@echo "=== [engine-sim-bridge] Building with coverage instrumentation ==="
	@cmake --build $(BUILD_DIR) $(CMAKE_BUILD_PARALLEL_FLAG)
	@touch $@

# coverage-run: run tests on coverage-instrumented build, merge profdata, export lcov
coverage-run: $(COVERAGE_REPORT)
	@LLVM_PROFDATA="$(LLVM_PROFDATA)" LLVM_COV="$(LLVM_COV)" \
		bash scripts/run_coverage_tests.sh $(BUILD_DIR)

sonar-scan: $(SONAR_STAMP) coverage-run

SONAR_REPORT := $(BUILD_DIR)/sonar-report.json

$(SONAR_STAMP): $(COVERAGE_REPORT) $(COMPILE_DB) $(SONAR_PROJECT_PROPERTIES) $(BUILD_STAMP)
	@if [ -z "$${SONAR_TOKEN_ES}" ]; then \
		echo "ERROR: SONAR_TOKEN_ES is not set. Run: source ~/.zshrc"; \
		exit 1; \
	fi
	@echo "=== [engine-sim-bridge] Running Sonar scan ==="
	@SONAR_TOKEN="$${SONAR_TOKEN_ES}" sonar-scanner \
	-Dsonar.token="$${SONAR_TOKEN_ES}"
	@echo "=== [engine-sim-bridge] Caching SonarCloud issue report ==="
	@TOKEN="$${SONAR_TOKEN_ES}"; \
	curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/issues/search?componentKeys=danieljsinclair_engine-sim-bridge&ps=500" \
		> $(SONAR_REPORT) 2>/dev/null || true
	@touch $@

$(COMPILE_DB): $(BUILD_DIR)/CMakeCache.txt

sonar-clean:
	@rm -f $(SONAR_STAMP)
	@rm -rf .scannerwork

coverage-clean:
	@rm -f $(COVERAGE_REPORT) $(SONAR_REPORT) $(BUILD_DIR)/coverage.profdata $(BUILD_DIR)/lcov.info $(BUILD_DIR)/profraw/*.profraw
	@rm -rf $(BUILD_DIR)/profraw

check: test

# ============================================================================
# Preset compilation: es/*.mr scripts → preset/*.json
#
# Compiles .mr scripts from es/ into JSON presets using the preset_compiler.
# Wrapper scripts (e.g., es/v8_gm_ls.mr) import from engines/atg-video-2/ which
# resolves via the engine-sim root — the preset_compiler handles this.
# Standalone scripts (e.g., es/C63_M156_V2.mr) are self-contained.
#
# Single source of truth: CANDIDATE_ENGINES lists which scripts to compile.
# ============================================================================

# Sonar summary — display issues from cached SonarCloud report
sonar-summary: $(SONAR_REPORT)
	@python3 scripts/sonar_summary.py $(SONAR_REPORT)

# Add/remove engines here — this is the ONLY list. Everything here is compiled, tested, and shipped.
ENGINES := ferrari_f136 2jz C63_M156_V2 subaru_ej25 lfa_v10 v8_gm_ls 11_merlin_v12 06_subaru_ej25

# Auto-generate compilation rules: each preset/$(engine).json depends on es/$(engine).mr
# The preset_compiler takes: <script_path> <output_json> <engine_sim_root>
# Script paths must be absolute since the compiler changes CWD to engine_sim_root
define PRESET_COMPILE_RULE
$(PRESET_DIR)/$(1).json: es/$(1).mr | $(PRESET_COMPILER)
	@mkdir -p $$(dir $$@)
	$(PRESET_COMPILER) $$(abspath es/$(1).mr) $$(abspath $$@) $$(abspath $(ENGINE_SIM_ROOT))
endef
$(foreach engine,$(ENGINES),$(eval $(call PRESET_COMPILE_RULE,$(engine))))

PRESET_JSONS := $(foreach engine,$(ENGINES),$(PRESET_DIR)/$(engine).json)

ISOMORPHISM_MR_INPUTS := $(shell find es -type f -name '*.mr' -print 2>/dev/null | sed 's/ /\\ /g')
ISOMORPHISM_CODE_INPUTS := $(shell find src/common src/preset include/common include/preset include/simulator -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -print 2>/dev/null | sed 's/ /\\ /g')
ISOMORPHISM_INPUTS = \
	$(PRESET_JSONS) \
	$(ISOMORPHISM_MR_INPUTS) \
	$(ISOMORPHISM_CODE_INPUTS) \
	CMakeLists.txt \
	test/PresetIsomorphismTests.cpp \
	tools/preset_compiler.cpp \
	$(BUILD_DIR)/preset_isomorphism_tests

$(ISOMORPHISM_STAMP): $(ISOMORPHISM_INPUTS) | build presets
	@mkdir -p $(dir $@)
	@echo "$(BLOCK_START_MESSAGE)"; \
	$(call bridge_print_hint) \
	if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -R '$(BRIDGE_TEST_ISOMORPHISM_MATCH)'; then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (isomorphism) ==="; \
		$(call bridge_print_result,32,PASSED); \
		$(call bridge_print_hint) \
		touch $(abspath $@); \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (isomorphism) ==="; \
		$(call bridge_print_result,31,FAILED); \
		$(call bridge_print_hint) \
		exit 1; \
	fi
# Build the preset compiler if it doesn't exist (e.g. after scrub)
$(PRESET_COMPILER):
	+@$(MAKE) build

presets: $(PRESET_JSONS)

clean-presets:
	@rm -rf $(PRESET_DIR)

help:
	@echo "engine-sim-bridge Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build + test + presets (complete pipeline)"
	@echo "  make build    - Compile everything (no tests)"
	@echo "  make sonar-scan - Run SonarQube scan with coverage (only re-runs when build/inputs change)"
	@echo "  make sonar-summary - Show SonarCloud issues summary"
	@echo "  make test     - Run sonar scan with coverage, core tests, then deep tests"
	@echo "  make test-core - Run the always-on bridge suite"
	@echo "  make test-isomorphism - Run file-based incremental isomorphism tests when inputs are newer"
	@echo "  make test-deep - Run isomorphism suite"
	@echo "  make test-nosonar - Run the full test gate (core + isomorphism) WITHOUT sonar/coverage"
	@echo "  make presets  - Compile .mr wrappers to JSON presets"
	@echo "  make clean    - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Adding a new engine:"
	@echo "  1. Add engine name to CANDIDATE_ENGINES in this Makefile"
	@echo "  2. Place the .mr script in es/"
	@echo "  3. Run 'make presets'"

