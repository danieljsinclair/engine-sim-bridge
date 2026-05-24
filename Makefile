.PHONY: all build-all clean scrub help remove-orphans check test test-fast test-quick testquick presets clean-presets
.PHONY: all build-all clean scrub help remove-orphans check test test-fast test-quick testquick presets clean-presets clean-test-fixtures

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
# Six expensive groups excluded by test-fast/test-quick only.
BRIDGE_TEST_SLOW_GROUPS_EXCLUDE := (PresetGoldenFileTest|ParameterIsomorphismTest\.EngineTopologyMatches|ParameterIsomorphismTest\.PortFlowFunctionsMatch|ParameterIsomorphismTest\.CamshaftLobeProfilesMatch|ParameterIsomorphismTest\.FuelParametersMatch|ParameterIsomorphismTest\.VehicleParametersMatch)
# Full quick-mode exclusion: all parameter isomorphism + golden audio regressions.
BRIDGE_TEST_QUICK_GROUPS_EXCLUDE := (PresetGoldenFileTest|ParameterIsomorphismTest)
CTEST_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
BUILD_PARALLEL_LEVEL ?=
CMAKE_BUILD_PARALLEL_FLAG := $(if $(strip $(BUILD_PARALLEL_LEVEL)),--parallel $(BUILD_PARALLEL_LEVEL),)

define build_bridge_targets
	cmake --build $(BUILD_DIR) $(CMAKE_BUILD_PARALLEL_FLAG)
endef

# Preset compilation
PRESET_DIR := preset
ENGINE_SIM_ROOT := engine-sim
PRESET_COMPILER := $(BUILD_DIR)/engine-sim-preset-compiler

# Default target - build everything (includes presets)
all: build-all presets

build-all:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..
	+$(call build_bridge_targets)

$(BUILD_DIR)/CMakeCache.txt:
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

# Clean build artifacts but keep CMake config (cascades to engine-sim)
clean: remove-orphans clean-presets clean-test-fixtures
	@if [ -d $(BUILD_DIR) ]; then cmake --build $(BUILD_DIR) --target clean >/dev/null 2>&1 || true; fi
	@if [ -d $(BUILD_DIR)/engine-sim ]; then $(MAKE) -C $(BUILD_DIR)/engine-sim clean 2>/dev/null || true; fi
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

# Run all bridge tests (build first if needed)
test: $(BUILD_DIR)/CMakeCache.txt presets
	+@$(call build_bridge_targets)
	@if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -E '$(BRIDGE_TEST_EXCLUDE)'; then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (full) ==="; \
		printf '\033[0;32m=== [engine-sim-bridge] RESULT: PASSED ===\033[0m\n'; \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (full) ==="; \
		printf '\033[0;31m=== [engine-sim-bridge] RESULT: FAILED ===\033[0m\n'; \
		exit 1; \
	fi

# Fast inner-loop run: excludes six expensive realtime/isomorphism groups.
test-fast: $(BUILD_DIR)/CMakeCache.txt presets
	+@$(call build_bridge_targets)
	@if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -E '$(BRIDGE_TEST_EXCLUDE)|$(BRIDGE_TEST_SLOW_GROUPS_EXCLUDE)'; then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (fast) ==="; \
		printf '\033[0;32m=== [engine-sim-bridge] RESULT: PASSED ===\033[0m\n'; \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (fast) ==="; \
		printf '\033[0;31m=== [engine-sim-bridge] RESULT: FAILED ===\033[0m\n'; \
		exit 1; \
	fi

# Deep tier: run the long-running preset golden-audio regressions only.
test-deep: $(BUILD_DIR)/CMakeCache.txt
	+@$(call build_bridge_targets)
	@if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -R 'PresetGoldenFileTest'; then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (deep) ==="; \
		printf '\033[0;32m=== [engine-sim-bridge] RESULT: PASSED ===\033[0m\n'; \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (deep) ==="; \
		printf '\033[0;31m=== [engine-sim-bridge] RESULT: FAILED ===\033[0m\n'; \
		exit 1; \
	fi

# Quick inner-loop run: skip all parameter isomorphism and golden-audio groups.
test-quick: presets
	+@$(MAKE) $(BUILD_DIR)/CMakeCache.txt
	+@$(call build_bridge_targets)
	@if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -E '$(BRIDGE_TEST_EXCLUDE)|$(BRIDGE_TEST_QUICK_GROUPS_EXCLUDE)'; then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (quick) ==="; \
		printf '\033[0;32m=== [engine-sim-bridge] RESULT: PASSED ===\033[0m\n'; \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (quick) ==="; \
		printf '\033[0;31m=== [engine-sim-bridge] RESULT: FAILED ===\033[0m\n'; \
		exit 1; \
	fi

testquick: test-quick

# Explicit long-running tier name; same coverage as full test.
testdeep: test-deep

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

# Add/remove engines here — this is the ONLY list to maintain
CANDIDATE_ENGINES := ferrari_f136 ferrari_412_t2 2jz C63_M156_V2 subaru_ej25 lfa_v10 radial_9 v6_60_degree v6_even_fire v6_odd_fire v8_gm_ls 11_merlin_v12

# Auto-generate compilation rules: each preset/$(engine).json depends on es/$(engine).mr
# The preset_compiler takes: <script_path> <output_json> <engine_sim_root>
# Script paths must be absolute since the compiler changes CWD to engine_sim_root
define PRESET_COMPILE_RULE
$(PRESET_DIR)/$(1).json: es/$(1).mr
	@mkdir -p $$(dir $$@)
	$(PRESET_COMPILER) $$(abspath es/$(1).mr) $$(abspath $$@) $$(abspath $(ENGINE_SIM_ROOT))
endef
$(foreach engine,$(CANDIDATE_ENGINES),$(eval $(call PRESET_COMPILE_RULE,$(engine))))

PRESET_JSONS := $(foreach engine,$(CANDIDATE_ENGINES),$(PRESET_DIR)/$(engine).json)

presets: $(PRESET_JSONS)

clean-presets:
	@rm -rf $(PRESET_DIR)

help:
	@echo "engine-sim-bridge Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build everything (creates build dir if needed)"
	@echo "  make presets  - Compile .mr wrappers to JSON presets"
	@echo "  make test      - Build and run full bridge test suite"
	@echo "  make test-deep - Run bridge preset golden-audio regressions"
	@echo "  make test-fast - Run bridge tests excluding 6 heavy groups"
	@echo "  make test-quick/testquick - Synonyms for test-fast"
	@echo "  make clean     - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Adding a new engine:"
	@echo "  1. Add engine name to CANDIDATE_ENGINES in this Makefile"
	@echo "  2. Place the .mr script in es/"
	@echo "  3. Run 'make presets'"
