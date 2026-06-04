.DEFAULT_GOAL := all
.PHONY: all build clean scrub help remove-orphans check test test-core test-isomorphism test-golden test-deep presets clean-presets clean-test-fixtures

BUILD_DIR ?= build
BUILD_TYPE ?= Release
CTEST_VERBOSE ?= 0
CTEST_QUIET ?= 0
CTEST_UI_FLAGS := $(if $(filter 1,$(CTEST_VERBOSE)),-V,$(if $(filter 1,$(CTEST_QUIET)),-Q,--progress))
BRIDGE_TEST_CMAKE_FLAGS := \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DBUILD_PRESET_ENGINE_TESTS=ON \
	-DBUILD_IOS_ADAPTER_TESTS=ON \
	-DBUILD_PHASE0_SPIKES=OFF
BRIDGE_TEST_EXCLUDE := (_NOT_BUILT|_spike$$)
BRIDGE_TEST_GOLDEN_MATCH := PresetGoldenFileTest
BRIDGE_TEST_ISOMORPHISM_MATCH := (IsomorphismFixtures/EndToEndPresetTest\.PresetLoadsAndProducesValidEngine|IsomorphismPresets/ParameterIsomorphismTest|AllEngines/RoundTripIsomorphismTest)
BRIDGE_TEST_CORE_EXCLUDE := $(BRIDGE_TEST_EXCLUDE)|($(BRIDGE_TEST_GOLDEN_MATCH)|$(BRIDGE_TEST_ISOMORPHISM_MATCH))
CTEST_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
BUILD_PARALLEL_LEVEL ?=
CMAKE_BUILD_PARALLEL_FLAG := $(if $(strip $(BUILD_PARALLEL_LEVEL)),--parallel $(BUILD_PARALLEL_LEVEL),)
BUILD_STAMP := $(BUILD_DIR)/.build-ready.stamp

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
	@echo "$(BLOCK_START_MESSAGE)"; \
	$(call bridge_print_hint) \
	if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) $(CTEST_SELECTOR); then \
		echo "$(SUMMARY_PASS_MESSAGE)"; \
		$(call bridge_print_result,32,PASSED); \
		$(call bridge_print_hint) \
	else \
		echo "$(SUMMARY_FAIL_MESSAGE)"; \
		$(call bridge_print_result,31,FAILED); \
		$(call bridge_print_hint) \
		exit 1; \
	fi
endef

# Preset compilation
PRESET_DIR := preset
ENGINE_SIM_ROOT := engine-sim
PRESET_COMPILER := $(BUILD_DIR)/engine-sim-preset-compiler

# Default target - build + test + presets (complete pipeline)
all: build test presets

# Compile everything (cmake configure + build)
build: $(BUILD_STAMP)

$(BUILD_STAMP): $(BUILD_INPUTS) $(BUILD_DIR)/CMakeCache.txt
	+$(call build_bridge_targets)
	@touch $@

$(BUILD_DIR)/CMakeCache.txt: CMakeLists.txt
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake $(BRIDGE_TEST_CMAKE_FLAGS) ..

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
clean: remove-orphans clean-presets clean-test-fixtures
	@if [ -d $(BUILD_DIR) ]; then cmake --build $(BUILD_DIR) --target clean >/dev/null 2>&1 || true; fi
	@rm -f $(ISOMORPHISM_STAMP)
	@rm -f $(BUILD_STAMP)
	@rm -rf tmp

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf $(BUILD_DIR) preset tmp
	@$(MAKE) remove-orphans

# Remove runtime-generated preset fixtures so next test run regenerates them.
# Needed if .mr source scripts change.
clean-test-fixtures:
	@echo "Removing cached preset fixtures from $(BUILD_DIR)/test/runtime-fixtures..."
	@rm -rf $(BUILD_DIR)/test/runtime-fixtures
	@echo "Done."

# Run the bridge suite in explicit tiers.
test: build test-core test-deep

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

test-golden: CTEST_SELECTOR := -R '$(BRIDGE_TEST_GOLDEN_MATCH)'
test-golden: BLOCK_START_MESSAGE := === [engine-sim-bridge] START: golden-audio suite (preset vs Piranha reference) ===
test-golden: SKIP_HINT_MESSAGE := === [engine-sim-bridge] HINT: skip this next time by running make test-core instead of make test or make test-deep. ===
test-golden: SUMMARY_PASS_MESSAGE := === [engine-sim-bridge] SUMMARY: PASS (golden) ===
test-golden: SUMMARY_FAIL_MESSAGE := === [engine-sim-bridge] SUMMARY: FAIL (golden) ===
test-golden: build presets
	$(run_bridge_ctest_suite)

test-deep: build test-isomorphism test-golden

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
	@echo "  make test     - Build, run core tests, then deep tests"
	@echo "  make test-core - Run the always-on bridge suite"
	@echo "  make test-isomorphism - Run file-based incremental isomorphism tests when inputs are newer"
	@echo "  make test-golden - Run preset golden-audio regressions"
	@echo "  make test-deep - Run isomorphism + golden suites"
	@echo "  make presets  - Compile .mr wrappers to JSON presets"
	@echo "  make clean    - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Adding a new engine:"
	@echo "  1. Add engine name to CANDIDATE_ENGINES in this Makefile"
	@echo "  2. Place the .mr script in es/"
	@echo "  3. Run 'make presets'"
