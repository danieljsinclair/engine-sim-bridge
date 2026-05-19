.PHONY: all build-all clean scrub help remove-orphans test presets clean-presets

# Default to parallel build using available CPU cores
MAKEFLAGS += -j$(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Preset compilation
PRESET_DIR := preset
ENGINE_SIM_ROOT := engine-sim
PRESET_COMPILER := $(shell \
	ls build/engine-sim-preset-compiler 2>/dev/null || \
	ls ../build/engine-sim-bridge/engine-sim-preset-compiler 2>/dev/null || \
	echo "")

# Default target - build everything (includes presets)
all: build-all presets

build-all:
	@if [ ! -d build ]; then mkdir build && cd build && cmake ..; fi
	$(MAKE) -C build

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
	@if [ -d build ]; then $(MAKE) -C build clean 2>/dev/null || true; fi
	@if [ -d build/engine-sim ]; then $(MAKE) -C build/engine-sim clean 2>/dev/null || true; fi

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf build preset
	@$(MAKE) remove-orphans

# Run tests (build first if needed, log output)
test:
	$(MAKE) -C build bridge_unit_tests preset_engine_tests
	@cd build && ctest -V --output-on-failure

# ============================================================================
# Preset compilation: es/*.mr wrappers → preset/*.json
#
# Compiles the underlying engine scripts from the engine-sim submodule.
# Wrapper scripts (e.g., es/v8_gm_ls.mr) import submodule engines; we compile
# those engines and name the output JSON to match the wrapper.
# Custom engines (C63_M156, etc.) are excluded — future work.
# ============================================================================

PRESET_JSONS := $(PRESET_DIR)/v8_gm_ls.json $(PRESET_DIR)/ferrari_f136.json \
    $(PRESET_DIR)/ferrari_412_t2.json $(PRESET_DIR)/2jz.json $(PRESET_DIR)/lfa_v10.json \
    $(PRESET_DIR)/radial_9.json $(PRESET_DIR)/v6_60_degree.json \
    $(PRESET_DIR)/v6_even_fire.json $(PRESET_DIR)/v6_odd_fire.json

presets: $(PRESET_JSONS)

$(PRESET_DIR)/v8_gm_ls.json: es/v8_gm_ls.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/07_gm_ls.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/07_gm_ls.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/ferrari_f136.json: es/ferrari_f136.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/08_ferrari_f136_v8.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/08_ferrari_f136_v8.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/ferrari_412_t2.json: es/ferrari_412_t2.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/12_ferrari_412_t2.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/12_ferrari_412_t2.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/2jz.json: es/2jz.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/03_2jz.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/03_2jz.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/lfa_v10.json: es/lfa_v10.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/10_lfa_v10.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/10_lfa_v10.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/radial_9.json: es/radial_9.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/09_radial_9.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/09_radial_9.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/v6_60_degree.json: es/v6_60_degree.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/04_60_degree_v6.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/04_60_degree_v6.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/v6_even_fire.json: es/v6_even_fire.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/06_even_fire_v6.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/06_even_fire_v6.mr $@ $(ENGINE_SIM_ROOT)

$(PRESET_DIR)/v6_odd_fire.json: es/v6_odd_fire.mr $(ENGINE_SIM_ROOT)/assets/engines/atg-video-2/05_odd_fire_v6.mr
	@mkdir -p $(PRESET_DIR)
	$(PRESET_COMPILER) engines/atg-video-2/05_odd_fire_v6.mr $@ $(ENGINE_SIM_ROOT)

clean-presets:
	@rm -rf $(PRESET_DIR)

help:
	@echo "engine-sim-bridge Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build everything (creates build dir if needed)"
	@echo "  make presets  - Compile .mr wrappers to JSON presets"
	@echo "  make test     - Build and run tests (verbose output)"
	@echo "  make clean    - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
