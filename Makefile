.PHONY: all clean scrub help remove-orphans test

# Default to parallel build using available CPU cores
MAKEFLAGS += -j$(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Default target - build everything
all:
	@if [ ! -d build ]; then mkdir build && cd build && cmake ..; fi
	$(MAKE) -C build

# Remove orphaned binaries and symlinks
remove-orphans:
	@rm -f *.dylib libenginesim*.dylib 2>/dev/null || true
	@find . -name "*.dylib*" -type l -delete 2>/dev/null || true

# Clean build artifacts but keep CMake config (cascades to engine-sim)
clean: remove-orphans
	@if [ -d build ]; then $(MAKE) -C build clean 2>/dev/null || true; fi
	@if [ -d build/engine-sim ]; then $(MAKE) -C build/engine-sim clean 2>/dev/null || true; fi

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf build
	@$(MAKE) remove-orphans

# Run tests (build first if needed, log output)
test:
	$(MAKE) -C build bridge_unit_tests
	@cd build && ctest -R bridge_unit_tests -V --output-on-failure

help:
	@echo "engine-sim-bridge Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make        - Build everything (creates build dir if needed)"
	@echo "  make test   - Build and run tests (verbose output)"
	@echo "  make clean  - Clean build artifacts (fast rebuild)"
	@echo "  make scrub  - Remove entire build directory (full clean)"
	@echo "  make help   - Show this help"
