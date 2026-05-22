.PHONY: all build-all clean scrub help remove-orphans check test presets clean-presets

BUILD_DIR ?= build
BUILD_TYPE ?= Release
BRIDGE_TEST_CMAKE_FLAGS := \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DBUILD_PRESET_ENGINE_TESTS=ON \
	-DBUILD_IOS_ADAPTER_TESTS=ON \
	-DBUILD_PHASE0_SPIKES=OFF
BRIDGE_TEST_EXCLUDE := (_NOT_BUILT|_spike$$)

# Default to parallel build using available CPU cores
MAKEFLAGS += -j$(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Preset compilation
PRESET_DIR := preset
ENGINE_SIM_ROOT := engine-sim
PRESET_COMPILER := ../build/engine-sim-bridge/engine-sim-preset-compiler

# Default target - build everything (includes presets)
all: build-all presets

build-all:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..
	$(MAKE) -C $(BUILD_DIR)

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
clean: remove-orphans clean-presets
	@if [ -d $(BUILD_DIR) ]; then $(MAKE) -C $(BUILD_DIR) clean 2>/dev/null || true; fi
	@if [ -d $(BUILD_DIR)/engine-sim ]; then $(MAKE) -C $(BUILD_DIR)/engine-sim clean 2>/dev/null || true; fi

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf $(BUILD_DIR) preset
	@$(MAKE) remove-orphans

# Run all bridge tests (build first if needed)
test:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake $(BRIDGE_TEST_CMAKE_FLAGS) ..
	@$(MAKE) -C $(BUILD_DIR)
	@cd $(BUILD_DIR) && ctest -V --output-on-failure -E '$(BRIDGE_TEST_EXCLUDE)'

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
CANDIDATE_ENGINES := ferrari_f136 C63_M156_V2 subaru_ej25 lfa_v10 v8_gm_ls 11_merlin_v12

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
	@echo "  make test     - Build and run all bridge tests"
	@echo "  make clean    - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Adding a new engine:"
	@echo "  1. Add engine name to CANDIDATE_ENGINES in this Makefile"
	@echo "  2. Place the .mr script in es/"
	@echo "  3. Run 'make presets'"
