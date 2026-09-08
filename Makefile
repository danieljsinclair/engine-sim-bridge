.DEFAULT_GOAL := all
.PHONY: all build clean clean-test scrub help remove-orphans \
		check test test-core test-isomorphism test-deep test-reset \
		presets clean-presets \
		sonar-clean coverage-clean coverage-run test-nosonar \
		sonar-refresh \
		summary summary-headline summary-blocks

BUILD_DIR ?= build
BUILD_COV_DIR ?= build-cov
BUILD_TYPE ?= RelWithDebInfo
BUILD_TYPE_COV ?= RelWithDebInfo
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
BRIDGE_TEST_CORE_EXCLUDE := $(BRIDGE_TEST_EXCLUDE)|($(BRIDGE_TEST_ISOMORPHISM_MATCH)|(EnginePresets/.*|Phase4FactoryPresets/.*|twin_foundation_tests)|(GasSystemTests\..*|SynthesizerTests\..*|FunctionTests\..*))
CTEST_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
BUILD_PARALLEL_LEVEL ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
CMAKE_BUILD_PARALLEL_FLAG := $(if $(strip $(BUILD_PARALLEL_LEVEL)),--parallel $(BUILD_PARALLEL_LEVEL),)
# Real artefact dependencies replace .stamp files (KISS — Make's natural
# mtime check is sufficient; no .stamp bookkeeping required). The library
# archives are the real artefacts produced by `cmake --build`; the
# isomorphism test artefact is the test binary (ctest runs it with -R).
BUILD_STAMP := $(BUILD_DIR)/libenginesim.a
BUILD_COV_STAMP := $(BUILD_COV_DIR)/libenginesim.a
SONAR_PROJECT_PROPERTIES := sonar-project.properties
COMPILE_DB := $(BUILD_COV_DIR)/compile_commands.json
# coverage.txt is a BYPRODUCT of the lcov.info rule (kept as a var only for
# coverage-clean); lcov.info is the primary coverage artefact -- see its rule.
COVERAGE_REPORT := $(BUILD_COV_DIR)/coverage.txt
SONAR_REPORT := $(BUILD_COV_DIR)/sonar-report.json
SONAR_TOKEN ?= ${SONAR_TOKEN_ES}

# Live-cache JSON files for the sonar-summary display. These are SEPARATE
# from $(SONAR_REPORT) (the scan stamp): a summary GET must NEVER write to
# $(SONAR_REPORT), or a later `make sonar-scan` becomes a no-op (owner
# report 2026-09-06: "Sonar hasn't been run, but it SHOULD; I've tried
# make clean"). The scan OWNS sonar-report.json; the summary owns these.
SONAR_LIVE := $(BUILD_COV_DIR)/sonar-live.json
SONAR_REMOVED_FACET := $(BUILD_COV_DIR)/sonar-removed-facet.json
SONAR_MEASURES := $(BUILD_COV_DIR)/sonar-measures.json

# Cached formatted display output. The recipes write the formatted report
# here; the `sonar-summary` / `coverage-summary` convenience targets just
# `cat` this file. File-artefact semantics: re-runs only when its input
# prereqs (the cached JSON or the lcov.info) are newer.
SONAR_SUMMARY_REPORT := $(BUILD_COV_DIR)/sonar-summary.txt
COVERAGE_SUMMARY_REPORT := $(BUILD_COV_DIR)/coverage-summary.txt

# Combined ctest summary log: every ctest tier (test-core, test-deep) appends
# its "N% tests passed, M tests failed out of N" line here (via the tee in
# run_bridge_ctest_suite). reset at the start of `test` so each run is clean.
# build_summary.py aggregates ALL summary lines so the headline reflects the
# union of every tier run in this `make test`, not just the last.
TEST_SUMMARY_LOG := $(BUILD_DIR)/test-summary.log

# ---- Single source of truth for source file specs ----
# Wildcard globs over the bridge + engine-sim source tree. New files picked
# up automatically — no per-file maintenance. Reused by build, coverage,
# sonar, and the isomorphism suite (the latter filters to a subset of dirs).
SRC_HEADER_FILES := $(shell find src include test tools engine-sim -type f \
	\( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' \
	   -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' \
	   -o -name '*.cmake' \) 2>/dev/null | sort)
# (sed escapes spaces: es/ has names like "L539 V12.mr" — unescaped, Make
# would split them into phantom prereqs and die with "No rule to make
# target `es/L539'")
PRESET_SCRIPT_FILES := $(shell find es -type f -name '*.mr' 2>/dev/null | sed 's/ /\\ /g' | sort)

# Source inputs that affect the bridge build. Single source of truth — see
# $(SRC_HEADER_FILES) for the underlying globs. Includes the Makefile itself
# so a config change rebuilds everything downstream (build, coverage, sonar).
BUILD_INPUTS := Makefile CMakeLists.txt $(SRC_HEADER_FILES)

ISOMORPHISM_STAMP := $(BUILD_DIR)/bridge_unit_tests

define build_bridge_targets
	cmake --build $(BUILD_DIR) $(CMAKE_BUILD_PARALLEL_FLAG)
endef

bridge_print_result = printf '\033[0;$(1)m=== [engine-sim-bridge] RESULT: $(2) ===\033[0m\n'

define bridge_print_hint
	if [ -n "$(strip $(SKIP_HINT_MESSAGE))" ]; then printf '%s\n' "$(SKIP_HINT_MESSAGE)"; fi;
endef

define run_bridge_ctest_suite
	@set -o pipefail; \
	mkdir -p $(dir $(abspath $(TEST_SUMMARY_LOG))); \
	if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) $(CTEST_SELECTOR) \
			2>&1 | tee -a $(abspath $(TEST_SUMMARY_LOG)); then \
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

# Default target - presets + build + test + summary. `summary` is
# the LAST step so the end-of-make headline is the final build output.
# presets must precede build so the preset JSON mtimes are older than the
# binary; otherwise the isomorphism stamp (bridge_unit_tests) is always
# stale and the isomorphism suite re-runs on every invocation.
all: presets build test summary

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
	@cd $(BUILD_DIR) && cmake $(BRIDGE_TEST_CMAKE_FLAGS) -DCMAKE_BUILD_TYPE=Release .. \
		&& sed -i '' 's/^CMAKE_BUILD_TYPE:STRING=.*/CMAKE_BUILD_TYPE:STRING=Release/' CMakeCache.txt

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

# Clean build artifacts (cascades to engine-sim via cmake).
# No .stamp files to remove — the real artefact (libenginesim.a) is removed
# by `cmake --build clean` above.
clean: remove-orphans clean-presets clean-test-fixtures sonar-clean coverage-clean
	@if [ -d $(BUILD_DIR) ]; then cmake --build $(BUILD_DIR) --target clean >/dev/null 2>&1 || true; fi
	@rm -rf tmp $(BUILD_COV_DIR)/.scannerwork

# Remove any stray .stamp files left over from the pre-94f8baf Makefile.
# The current build uses real artefact mtimes (libenginesim.a, bridge_unit_tests)
# so these files are dead weight — clean them up here so a plain `make clean`
# eradicates them without needing `make scrub`.
clean-test:
	@find $(BUILD_DIR) -maxdepth 1 -name '*.stamp' -delete 2>/dev/null || true

# Full clean - remove entire build directory (superset of clean)
scrub: clean
	@echo "Scrubbing bridge build..."
	@rm -rf $(BUILD_DIR) $(BUILD_COV_DIR) preset tmp
	@$(MAKE) remove-orphans

# Remove runtime-generated preset fixtures so next test run regenerates them.
# Needed if .mr source scripts change.
clean-test-fixtures:
	@echo "Removing cached preset fixtures from $(BUILD_DIR)/test/runtime-fixtures..."
	@rm -rf $(BUILD_DIR)/test/runtime-fixtures
	@echo "Done."

# Run the bridge suite in explicit tiers. Declarative dependency list only:
# each sub-target self-labels its own output (coverage-summary passes
# --label "[engine-sim-bridge]", sonar-summary prints its own === headers), so
# no procedural echo/banner wrapper is needed under test: itself. `summary`
# (the end-of-make headline) is the LAST prereq so it is the final output.
# rc/audio keeps coverage in the default chain (owner directive 2026-09-06:
# this tree has no outer app gate, so make test must generate the coverage
# stats itself) on top of master's summary tooling (d90c1bf).
# coverage-summary is NOT listed mid-chain (before sonar-scan) on purpose:
# the scan's recipe drops the summary JSON caches, and make caches stat
# results per invocation — a mid-chain coverage-summary would make `summary`
# cat the PRE-scan cache (stale server numbers) instead of re-curling. Via
# summary (post-scan) make stat's the deleted sonar-measures.json fresh and
# re-curls it, so the final report reflects the just-settled scan.
test: test-core test-deep coverage-run sonar-scan summary

# Order-only reset of the combined ctest summary log. Both ctest tiers depend
# on this so the log is empty at the start of a `make test` regardless of
# which tier runs first (the tiers APPEND via the tee in run_bridge_ctest_suite).
test-reset:
	@mkdir -p $(BUILD_DIR)
	@: > $(TEST_SUMMARY_LOG)

test-core: CTEST_SELECTOR := -E '$(BRIDGE_TEST_CORE_EXCLUDE)'
test-core: BLOCK_START_MESSAGE := === [engine-sim-bridge] START: core bridge suite (factory + preset regression coverage) ===
test-core: SKIP_HINT_MESSAGE := === [engine-sim-bridge] HINT: this is the always-on core suite in make test; there is no core-only skip target inside the safe path. Use app run-notest or run-quick only when you intentionally want to bypass the full test gate. ===
test-core: SUMMARY_PASS_MESSAGE := === [engine-sim-bridge] SUMMARY: PASS (core) ===
test-core: SUMMARY_FAIL_MESSAGE := === [engine-sim-bridge] SUMMARY: FAIL (core) ===
test-core: build presets | test-reset
	$(run_bridge_ctest_suite)

test-isomorphism: CTEST_SELECTOR := -R '$(BRIDGE_TEST_ISOMORPHISM_MATCH)'
test-isomorphism: BLOCK_START_MESSAGE := === [engine-sim-bridge] START: isomorphism suite (preset parity + round-trip serialization) ===
test-isomorphism: SKIP_HINT_MESSAGE := === [engine-sim-bridge] HINT: skip this next time by running make test-core instead of make test or make test-deep. ===
test-isomorphism: SUMMARY_PASS_MESSAGE := === [engine-sim-bridge] SUMMARY: PASS (isomorphism) ===
test-isomorphism: SUMMARY_FAIL_MESSAGE := === [engine-sim-bridge] SUMMARY: FAIL (isomorphism) ===
test-isomorphism: $(ISOMORPHISM_STAMP) | test-reset
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

# Coverage build: separate build-cov dir with llvm-cov instrumentation.
# Uses RelWithDebInfo (NOT Debug) + -fprofile-instr-generate/-fcoverage-mapping/-g.
# Has its own CMakeCache so coverage reconfigure does NOT invalidate the test build.
# Does NOT run tests — tests are run separately via make coverage-run
# NOTE: `touch CMakeCache.txt` after configure is REQUIRED, not cosmetic:
# cmake does not rewrite an unchanged cache file, so without the touch the
# cache mtime never advances past an edited CMakeLists.txt — Make then
# reconfigures on every invocation, each configure rewrites
# compile_commands.json, and that keeps sonar-report.json permanently stale
# (the scan re-ran on every `make` in this tree until 2026-09-07). The
# build/ dir escapes this only because its sed -i rewrites the cache.
$(BUILD_COV_DIR)/CMakeCache.txt: CMakeLists.txt
	@mkdir -p $(BUILD_COV_DIR)
	@cd $(BUILD_COV_DIR) && cmake \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE_COV) \
		-DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping -g" \
		-DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate" \
		-DBUILD_PRESET_ENGINE_TESTS=ON \
		-DBUILD_IOS_ADAPTER_TESTS=ON \
		-DBUILD_PHASE0_SPIKES=OFF \
		.. && touch CMakeCache.txt

$(BUILD_COV_STAMP): $(BUILD_INPUTS) $(BUILD_COV_DIR)/CMakeCache.txt
	@echo "=== [engine-sim-bridge] Building coverage (build-cov, RelWithDebInfo+instr) ==="
	@cmake --build $(BUILD_COV_DIR) $(CMAKE_BUILD_PARALLEL_FLAG)
	@# `touch $@` mirrors $(BUILD_STAMP): `cmake --build` only advances the
	@# archive mtime when something relinks — on a no-op build the mtime
	@# stays put and Make would re-run this recipe forever. Touching the real
	@# artefact records "checked with these inputs — no change", settling the
	@# chain (same idiom, no .stamp file).
	@touch $@

# coverage-run: run tests on coverage-instrumented build, merge profdata, export lcov
# File-artefact target: re-runs only when build-cov, preset JSONs, source inputs, or
# the coverage script change. The PRIMARY artefact is lcov.info -- it is what
# coverage-summary, sonar-scan and build_summary.py consume; run_coverage_tests.sh
# writes it alongside coverage.txt (coverage.txt stays a byproduct: removed by
# coverage-clean, no rule of its own). Declaring the rule here closes the
# orphan-prereq hole: $(COVERAGE_SUMMARY_REPORT) and `summary` depend on
# $(BUILD_COV_DIR)/lcov.info, which previously had NO rule -- after `make clean`
# the summary recursion died with "No rule to make target 'build-cov/lcov.info'".
$(BUILD_COV_DIR)/lcov.info: $(BUILD_COV_STAMP) $(PRESET_JSONS) $(BUILD_INPUTS) scripts/run_coverage_tests.sh
	@LLVM_PROFDATA="$(LLVM_PROFDATA)" LLVM_COV="$(LLVM_COV)" \
		bash scripts/run_coverage_tests.sh $(BUILD_COV_DIR)
	@test -e $@ || { echo "ERROR: $@ not written by run_coverage_tests.sh"; exit 1; }

# Phony alias so callers can still `make coverage-run`.
coverage-run: $(BUILD_COV_DIR)/lcov.info

sonar-scan: $(SONAR_REPORT)

# SONAR_REPORT depends on the coverage ARTEFACT (lcov.info), so coverage is
# regenerated (tests re-run) before the scan reads the fresh coverage report.
# Re-scans only when coverage/compile-db/properties/sources change. The curl
# writes SONAR_REPORT itself. The CE-poll block guarantees the report is only
# cached after the Compute Engine has settled: under this file-stamp-gated model
# a stale report would otherwise not self-correct until the next build change.
# The recipe's final rm drops the summary JSON caches: the scan just changed
# SonarCloud's server-side state, so the next sonar-summary must re-curl and
# re-format — a summary printed from pre-scan counts would be stale, and stale
# sonar data is the failure mode this Makefile must never hide.
# Derive -Dsonar.branch.name from the current git branch (worktree-aware).
# In a worktree, git rev-parse --abbrev-ref HEAD returns the branch name of the
# worktree, NOT "HEAD". In the main checkout it returns the checked-out branch.
# If derivation fails (no git, detached HEAD with no branch, etc.), FAIL FAST —
# never publish a branchless scan that overwrites SonarCloud master.
# Master scans remain possible: on master, BRANCH derives to "master".
BRANCH := $(shell git -C $$(dirname $(abspath $(firstword $(MAKEFILE_LIST)))) rev-parse --abbrev-ref HEAD 2>/dev/null)
ifeq ($(BRANCH),)
  $(error SONAR GUARD: cannot derive sonar.branch.name from git (no branch / no git). Refusing to publish a branchless scan.)
endif
ifeq ($(BRANCH),HEAD)
  $(error SONAR GUARD: HEAD is detached — cannot derive a branch name. Check out a named branch before scanning.)
endif
SONAR_BRANCH_FLAG := -Dsonar.branch.name=$(BRANCH)

$(SONAR_REPORT): $(BUILD_COV_DIR)/lcov.info $(COMPILE_DB) $(SONAR_PROJECT_PROPERTIES) $(BUILD_INPUTS)
	@if [ -z "$${SONAR_TOKEN_ES}" ] && [ -z "$${SONAR_TOKEN}" ]; then \
		echo "ERROR: Neither SONAR_TOKEN_ES nor SONAR_TOKEN is set. Run: source ~/.zshrc"; \
		exit 1; \
	fi
	@echo "=== [engine-sim-bridge] Running Sonar scan (branch: $(BRANCH)) ==="
	@SONAR_TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}" sonar-scanner \
		$(SONAR_BRANCH_FLAG) \
		-Dsonar.coverageReportPaths=$(BUILD_COV_DIR)/coverage-sonar.xml \
		-Dsonar.cfamily.compile-commands=$(BUILD_COV_DIR)/compile_commands.json \
		> $(BUILD_COV_DIR)/sonar-scanner.log 2>&1; \
		rc=$$?; \
		if [ $$rc -ne 0 ]; then \
			echo "=== [engine-sim-bridge] sonar-scanner failed (rc=$$rc); see $(BUILD_COV_DIR)/sonar-scanner.log ==="; \
			tail -n 20 $(BUILD_COV_DIR)/sonar-scanner.log; \
			exit $$rc; \
		fi
	@echo "=== [engine-sim-bridge] Waiting for SonarCloud Compute Engine to finish ==="
	@TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}"; \
		CETASKID=$$(grep -E '^ceTaskId=' $(BUILD_COV_DIR)/.scannerwork/report-task.txt 2>/dev/null | cut -d= -f2); \
		if [ -z "$$CETASKID" ]; then \
			echo "ERROR: no ceTaskId in $(BUILD_COV_DIR)/.scannerwork/report-task.txt; cannot confirm analysis settled"; \
			exit 1; \
		fi; \
		echo "  CE task: $$CETASKID"; \
		dead=0; \
		while [ $$dead -lt 60 ]; do \
			status=$$(curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/ce/task?id=$$CETASKID" \
				| python3 -c "import json,sys; print(json.load(sys.stdin).get('task',{}).get('status',''))" 2>/dev/null); \
			if [ "$$status" = "SUCCESS" ]; then \
				echo "  CE task SUCCESS after $$(($$dead * 2))s"; \
				break; \
			fi; \
			if [ "$$status" = "FAILED" ] || [ "$$status" = "CANCELED" ]; then \
				echo "ERROR: SonarCloud CE task $$status (id=$$CETASKID); report did not settle"; \
				exit 1; \
			fi; \
			sleep 2; \
			dead=$$((dead + 1)); \
		done; \
		if [ "$$status" != "SUCCESS" ]; then \
			echo "ERROR: timed out waiting for SonarCloud CE task (last status=$$status, id=$$CETASKID)"; \
			exit 1; \
		fi
	@echo "=== [engine-sim-bridge] Caching SonarCloud issue report ==="
	@TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}"; \
	curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/issues/search?componentKeys=danieljsinclair_engine-sim-bridge&ps=500&statuses=OPEN" \
		> $(SONAR_REPORT) 2>/dev/null || true
	@rm -f $(SONAR_LIVE) $(SONAR_REMOVED_FACET) $(SONAR_MEASURES)

$(COMPILE_DB): $(BUILD_COV_DIR)/CMakeCache.txt

sonar-clean:
	@rm -f $(SONAR_REPORT) $(SONAR_LIVE) $(SONAR_REMOVED_FACET) $(SONAR_MEASURES) $(SONAR_SUMMARY_REPORT)
	@rm -rf $(BUILD_COV_DIR)/.scannerwork

coverage-clean:
	@rm -f $(COVERAGE_REPORT) $(BUILD_COV_DIR)/coverage.profdata $(BUILD_COV_DIR)/lcov.info $(COVERAGE_SUMMARY_REPORT) $(BUILD_COV_DIR)/profraw/*.profraw
	@rm -rf $(BUILD_COV_DIR)/profraw
	@rm -f $(BUILD_COV_DIR)/.build-cov-ready.stamp

check: test

# ============================================================================
# Preset compilation: es/*.mr scripts → preset/*.json
#
# Compiles .mr scripts from es/ into JSON presets using the preset_compiler.
# Wrapper scripts (e.g., es/v8_gm_ls.mr) import from engines/atg-video-2/ which
# resolves via the engine-sim root — the preset_compiler handles this.
# Standalone scripts (e.g., es/C63_M156_V3.mr) are self-contained.
#
# Single source of truth: CANDIDATE_ENGINES lists which scripts to compile.
# ============================================================================

# ============================================================================
# Coverage summary: file-artefact target. Re-runs only when lcov.info or
# coverage_summary.py is newer than the cached report file. The convenience
# target just cats the cache. NEVER triggers a scan — the only inputs are
# the local lcov export and the cached sonar-measures.json.
# ============================================================================
coverage-summary: $(COVERAGE_SUMMARY_REPORT)
	@cat $<

$(COVERAGE_SUMMARY_REPORT): $(BUILD_COV_DIR)/lcov.info scripts/coverage_summary.py $(SONAR_MEASURES)
	@mkdir -p $(BUILD_COV_DIR)
	@echo "" > $@
	@echo "=== [engine-sim-bridge] BEGIN: coverage summary ===" >> $@
	@python3 scripts/coverage_summary.py $< --label "[engine-sim-bridge]" --sonar-measures $(SONAR_MEASURES) >> $@ || true
	@echo "=== [engine-sim-bridge] END: coverage summary ===" >> $@

# ============================================================================
# Sonar summary: file-artefact target. The three JSON cache files are
# themselves file-artefact targets with $(SONAR_PROJECT_PROPERTIES) as prereq
# — a curl happens only when sonar-project.properties changes (or on a
# user-initiated `make sonar-refresh`, e.g. after someone triaged issues on
# the SonarCloud web UI). The formatted report is regenerated only when any
# of the cached JSONs OR the script changes. NEVER triggers a scan: these
# recipes only GET; the scan OWNS sonar-report.json (see vars block above).
# ============================================================================

$(SONAR_LIVE): $(SONAR_PROJECT_PROPERTIES)
	@mkdir -p $(BUILD_COV_DIR)
	@TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}"; \
	if [ -z "$$TOKEN" ]; then \
		echo "  No SONAR_TOKEN set; $(SONAR_LIVE) left as empty sentinel."; \
		printf '' > $@; \
		exit 0; \
	fi; \
	curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/issues/search?componentKeys=danieljsinclair_engine-sim-bridge&ps=500&statuses=OPEN&facets=impactSeverities" \
		> $@ 2>/dev/null || printf '' > $@

$(SONAR_REMOVED_FACET): $(SONAR_PROJECT_PROPERTIES)
	@mkdir -p $(BUILD_COV_DIR)
	@TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}"; \
	if [ -z "$$TOKEN" ]; then \
		echo "  No SONAR_TOKEN set; $(SONAR_REMOVED_FACET) left as empty sentinel."; \
		printf '' > $@; \
		exit 0; \
	fi; \
	curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/issues/search?componentKeys=danieljsinclair_engine-sim-bridge&ps=1&resolutions=REMOVED&facets=impactSeverities" \
		> $@ 2>/dev/null || printf '' > $@

$(SONAR_MEASURES): $(SONAR_PROJECT_PROPERTIES)
	@mkdir -p $(BUILD_COV_DIR)
	@TOKEN="$${SONAR_TOKEN_ES:-$${SONAR_TOKEN}}"; \
	if [ -z "$$TOKEN" ]; then \
		echo "  No SONAR_TOKEN set; $(SONAR_MEASURES) left as empty sentinel."; \
		printf '' > $@; \
		exit 0; \
	fi; \
	curl -s -u "$$TOKEN:" "https://sonarcloud.io/api/measures/component?component=danieljsinclair_engine-sim-bridge&metricKeys=coverage,lines_to_cover,uncovered_lines" \
		> $@ 2>/dev/null || printf '' > $@

# Force-regenerate the cached JSONs and the formatted report. Use after a
# SonarCloud-side change (e.g. someone triaged issues on the web UI).
sonar-refresh:
	@rm -f $(SONAR_LIVE) $(SONAR_REMOVED_FACET) $(SONAR_MEASURES) $(SONAR_SUMMARY_REPORT)
	@$(MAKE) sonar-summary

# Convenience: print the cached report. Re-generates only if one of the
# cached JSONs, the script, or sonar-project.properties is newer.
sonar-summary: $(SONAR_SUMMARY_REPORT)
	@cat $<

$(SONAR_SUMMARY_REPORT): $(SONAR_LIVE) $(SONAR_REMOVED_FACET) scripts/sonar_summary.py
	@mkdir -p $(BUILD_COV_DIR)
	@echo "" > $@
	@echo "=== [engine-sim-bridge] BEGIN: SonarCloud issues summary ===" >> $@
	@python3 scripts/sonar_summary.py $(SONAR_LIVE) --label "[engine-sim-bridge]" --removed-facet $(SONAR_REMOVED_FACET) >> $@ || true
	@echo "=== [engine-sim-bridge] END: SonarCloud issues summary ===" >> $@

# summary: the end-of-make HEADLINE line (one coloured row) for the bridge.
# Greps plain numbers from the existing report files and re-emits them
# coloured -- no live re-query, never triggers a scan/test, never crashes.
# Tests come from the combined ctest summary log (test-summary.log, aggregated
# over every tier run in this `make test`); coverage from the cached
# sonar-measures.json (the SAME headline coverage_block.py shows; falls back to
# local lcov when no token/report exists); sonar open/total from the cached
# sonar-report.json. Missing fields are omitted gracefully. This is a LEAF --
# the bridge is the deepest
# nesting doll, so its summary prints exactly one line.
# --removed-facet mirrors sonar-summary: total = open + removed (OPEN union
# REMOVED, matching the dashboard severity widget and sonar_summary.py).
BUILD_SUMMARY_SCRIPT := scripts/build_summary.py

# One shared headline command (DRY): `summary` prints the cached summary
# BLOCKS (coverage-summary / sonar-summary) and then this headline, while
# `summary-headline` prints ONLY the headline. The CLI's russian-doll
# recursion calls summary-headline so a top-level make ENDS on exactly the
# two headline rows (cli line, then bridge line) with no block rows printed
# between or after them (found 2026-09-08).
BUILD_SUMMARY_CMD = python3 $(BUILD_SUMMARY_SCRIPT) \
    --label "[engine-sim-bridge]" \
    --test-log $(TEST_SUMMARY_LOG) \
    --cov-measures $(SONAR_MEASURES) \
    --local-cov $(BUILD_COV_DIR)/lcov.info --local-type lcov \
    --sonar-report $(SONAR_LIVE) \
    --removed-facet $(SONAR_REMOVED_FACET)

summary: coverage-summary sonar-summary
	@$(BUILD_SUMMARY_CMD)

# Headline ONLY -- no coverage/sonar blocks. See BUILD_SUMMARY_CMD above.
summary-headline:
	@$(BUILD_SUMMARY_CMD)

# Blocks ONLY -- coverage-summary + sonar-summary, no headline. Display-only
# convenience for the CLI's `summary` recursion: a top-level make re-prints
# the bridge's blocks on EVERY make (cached "TESTS UP TO DATE" path included)
# while still ending on the two headline rows. `summary` and
# `summary-headline` are untouched -- a standalone make in this folder keeps
# its full blocks-then-headline behaviour.
summary-blocks: coverage-summary sonar-summary

# Add/remove engines here — this is the ONLY list. Everything here is compiled, tested, and shipped.
ENGINES := ferrari_f136 2jz C63_M156_V3 subaru_ej25 lfa_v10 v8_gm_ls 11_merlin_v12 06_subaru_ej25

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

# Derived from the single-source-of-truth globs (see SRC_HEADER_FILES /
# PRESET_SCRIPT_FILES above) — no second find spec to keep in sync.
ISOMORPHISM_MR_INPUTS := $(PRESET_SCRIPT_FILES)
ISOMORPHISM_CODE_INPUTS := $(filter src/common/% src/preset/% include/common/% include/preset/% include/simulator/%,$(SRC_HEADER_FILES))
# bridge_unit_tests is the gtest binary ctest invokes with the -R selector
# (ctest uses gtest filter matching; there is no separate preset_isomorphism_tests
# binary on disk). It IS the artefact whose mtime we check.
ISOMORPHISM_INPUTS = \
	$(PRESET_JSONS) \
	$(ISOMORPHISM_MR_INPUTS) \
	$(ISOMORPHISM_CODE_INPUTS) \
	CMakeLists.txt \
	test/PresetIsomorphismTests.cpp \
	tools/preset_compiler.cpp \
	$(BUILD_DIR)/bridge_unit_tests

# If ctest fails the recipe exits non-zero -> Make will retry on the next
# invocation. If ctest passes the recipe's exit code is success -> the binary
# is the artefact; the mtime check determines when to re-run. No .stamp
# bookkeeping required.
$(ISOMORPHISM_STAMP): $(ISOMORPHISM_INPUTS) | build presets test-reset
	@set -o pipefail; \
	echo "$(BLOCK_START_MESSAGE)"; \
	$(call bridge_print_hint) \
	if cd $(BUILD_DIR) && ctest $(CTEST_UI_FLAGS) --output-on-failure -j$(CTEST_PARALLEL_LEVEL) -R '$(BRIDGE_TEST_ISOMORPHISM_MATCH)' 2>&1 | tee -a $(abspath $(TEST_SUMMARY_LOG)); then \
		echo "=== [engine-sim-bridge] SUMMARY: PASS (isomorphism) ==="; \
		$(call bridge_print_result,32,PASSED); \
		$(call bridge_print_hint) \
	else \
		echo "=== [engine-sim-bridge] SUMMARY: FAIL (isomorphism) ==="; \
		$(call bridge_print_result,31,FAILED); \
		$(call bridge_print_hint) \
		exit 1; \
	fi
	@touch $@
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
	@echo "  make sonar-summary - Show the cached SonarCloud issues summary (re-formats only when inputs change)"
	@echo "  make sonar-refresh - Force re-fetch of the SonarCloud summary caches, then show them"
	@echo "  make test     - Run sonar scan with coverage, core tests, then deep tests"
	@echo "  make test-core - Run the always-on bridge suite"
	@echo "  make test-isomorphism - Run file-based incremental isomorphism tests when inputs are newer"
	@echo "  make test-deep - Run isomorphism suite"
	@echo "  make test-nosonar - Run the full test gate (core + isomorphism) WITHOUT sonar/coverage"
	@echo "  make summary  - Print the end-of-make headline line (tests/cov/sonar)"
	@echo "  make presets  - Compile .mr wrappers to JSON presets"
	@echo "  make clean    - Clean build artifacts (fast rebuild)"
	@echo "  make scrub    - Remove entire build directory (full clean)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Adding a new engine:"
	@echo "  1. Add engine name to CANDIDATE_ENGINES in this Makefile"
	@echo "  2. Place the .mr script in es/"
	@echo "  3. Run 'make presets'"

