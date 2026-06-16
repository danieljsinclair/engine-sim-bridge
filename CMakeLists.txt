cmake_minimum_required(VERSION 3.20)

# Prevent in-source builds that overwrite hand-written Makefiles
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR
        "In-source builds not allowed. Run 'make' from the repo root instead. "
        "Or: cmake -B build && cmake --build build")
endif()

project(engine-sim-bridge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Bridge always builds as static library
set(LIB_TYPE STATIC)

# Find engine-sim (expected as a submodule at ./engine-sim)
# Use ENGINE_SIM_PATH if set by parent project for override
if(NOT DEFINED ENGINE_SIM_PATH)
    set(ENGINE_SIM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/engine-sim")
endif()

if(NOT EXISTS "${ENGINE_SIM_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "engine-sim not found at ${ENGINE_SIM_PATH}")
endif()

# Add engine-sim as a subdirectory (it's a submodule of this repo)
# Disable the GUI app and bridge from engine-sim's own CMakeLists
# Also disable engine-sim's own tests to avoid phantom test entries
set(BUILD_ENGINE_SIM_APP OFF CACHE BOOL "Build engine-sim GUI app" FORCE)
set(BUILD_BRIDGE OFF CACHE BOOL "Build bridge from engine-sim (we are the bridge)" FORCE)
set(DISCORD_ENABLED OFF CACHE BOOL "Enable Discord Rich Presence" FORCE)
# Disable engine-sim's internal testing
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(ENGINE_SIM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
# iOS builds disable script interpreter (piranha needs Boost, not available for iOS arm64)
# Piranha's CMakeLists unconditionally calls find_package(Boost REQUIRED).
# We can't cross-compile Boost for iOS, so we must skip the entire piranha subdirectory.
# The approach: override add_subdirectory to skip piranha when building for iOS.
# ESP32: same issue — Boost unavailable, piranha Bison/Flex toolchain not cross-compilable.
if(IOS OR ESP_PLATFORM)
    set(PIRANHA_ENABLED OFF CACHE BOOL "Disable scripting for ${PLATFORM}" FORCE)
endif()

# Isolate submodule compiler flags: do not inherit our strict warnings/Werror
# This is standard practice when including 3rd party submodules
set(SAVED_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(SAVED_C_FLAGS ${CMAKE_C_FLAGS})
set(CMAKE_CXX_FLAGS "")
set(CMAKE_C_FLAGS "")

# EXCLUDE_FROM_ALL: Do not build every single target from the submodule.
# Only build targets that we actually depend on (engine-sim, not tests, not app, not tools)
# As of CMake 3.21+ this CORRECTLY still rebuilds when source files change.
# The old bug where this would permanently cache binaries was fixed in 2022.
add_subdirectory(${ENGINE_SIM_PATH} ${CMAKE_BINARY_DIR}/engine-sim EXCLUDE_FROM_ALL)

# Keep third-party piranha warnings out of normal build output.
# We keep piranha enabled, but suppress known noisy warnings from that submodule.
if(TARGET piranha)
    target_compile_options(piranha PRIVATE -Wno-unused-parameter)
endif()

# Restore our original compiler flags after including submodules
set(CMAKE_CXX_FLAGS ${SAVED_CXX_FLAGS})
set(CMAKE_C_FLAGS ${SAVED_C_FLAGS})

# Real bridge library (uses actual engine-sim with polymorphic simulator selection)
# Phase F: Audio infrastructure moved from CLI into bridge

# Bridge source files (shared between macOS and iOS)
set(BRIDGE_SOURCES
    # Simulator core
    src/simulator/SineSimulator.cpp
    src/simulator/SimulatorInitHelpers.cpp
    src/simulator/SimulatorFactory.cpp
    src/simulator/BridgeSimulator.cpp
    src/simulator/BrakeConstraint.cpp
    src/simulator/ScriptLoadHelpers.cpp
    src/simulator/ISimulator.cpp
    # Audio strategies
    src/strategy/ThreadedStrategy.cpp
    src/strategy/SyncPullStrategy.cpp
    src/strategy/IAudioBufferFactory.cpp
    # Audio hardware
    src/hardware/AudioHardwareProviderFactory.cpp
    # Telemetry
    src/telemetry/InMemoryTelemetry.cpp
    # Simulation loop
    src/simulation/CrankingController.cpp
    src/simulation/PresentationStateBuilders.cpp
    src/simulation/SimulationLoop.cpp
    # Common utilities
    src/common/ConsoleLogger.cpp
    src/common/CircularBuffer.cpp
    src/common/JsonParser.cpp
    src/common/PresetSerializer.cpp
    # Preset engine factory (JSON-based, for preset compiler output)
    src/simulator/PresetEngineFactory.cpp
    # Preset deserializers (pure JSON -> engine object graph)
    src/preset/PresetDeserializer.cpp
    src/preset/FunctionDeserializer.cpp
    src/preset/EngineDeserializer.cpp
    src/preset/CrankshaftDeserializer.cpp
    src/preset/ConnectingRodDeserializer.cpp
    src/preset/CamshaftDeserializer.cpp
    src/preset/CylinderBankDeserializer.cpp
    src/preset/CylinderHeadDeserializer.cpp
    src/preset/ExhaustSystemDeserializer.cpp
    src/preset/IntakeDeserializer.cpp
    src/preset/FuelDeserializer.cpp
    src/preset/IgnitionModuleDeserializer.cpp
    src/preset/VehicleDeserializer.cpp
    src/preset/TransmissionDeserializer.cpp
    # Hardcoded C++ engine preset factories (works on iOS without Piranha)
    src/simulator/EnginePresets.cpp
    # Twin components
    src/twin/AutomaticGearbox.cpp
    src/twin/GearboxCsvLogger.cpp
    src/twin/ThrottleSmoother.cpp
    src/twin/VirtualIceTwin.cpp
    src/twin/ManualTwin.cpp
    src/twin/GearboxCsvLogger.cpp
    # Input providers
    src/input/VirtualIceInputProvider.cpp
    src/input/ManualTwinProvider.cpp
    src/input/DemoThrottleSource.cpp
    src/input/GearSelectorInput.cpp
    src/input/IgnitionInput.cpp
    src/input/DemoInputProvider.cpp
    src/input/BrakeInput.cpp
    src/input/KeyHoldBridge.cpp
    src/input/KeyboardInputProvider.cpp
    src/input/EngineInputTarget.cpp
    src/input/DemoControlsTarget.cpp
)

# Platform-specific audio hardware and factory
if(ESP_PLATFORM)
    # ESP32: Use I2S hardware provider
    list(APPEND BRIDGE_SOURCES
        src/hardware/ESP32I2SHardwareProvider.cpp
    )
elseif(IOS)
    # iOS: Use AVAudioEngine-based hardware provider (ObjC++)
    # Factory must be .mm to include AVAudioEngine headers
    list(APPEND BRIDGE_SOURCES
        src/hardware/AVAudioEngineHardwareProvider.mm
        src/hardware/AudioHardwareProviderFactory.mm
    )
    # Remove the .cpp factory (replaced by .mm version above)
    list(REMOVE_ITEM BRIDGE_SOURCES
        src/hardware/AudioHardwareProviderFactory.cpp
    )
else()
    # macOS: Use CoreAudio AudioQueue-based hardware provider
    list(APPEND BRIDGE_SOURCES
        src/hardware/CoreAudioHardwareProvider.cpp
    )
endif()

# Bridge is always static; coverage uses llvm-cov instrumentation
option(ENABLE_COVERAGE "Enable llvm-cov coverage instrumentation" OFF)
if(ENABLE_COVERAGE)
    add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
    add_link_options(-fprofile-instr-generate)
endif()
add_library(engine-sim-bridge ${LIB_TYPE} ${BRIDGE_SOURCES})

# Platform-specific audio libraries
if(IOS)
    set(AUDIO_LIBRARY "-framework AudioToolbox -framework CoreAudio -framework CoreFoundation -framework AVFoundation")
elseif(APPLE)
    set(AUDIO_LIBRARY "-framework AudioToolbox -framework CoreAudio -framework CoreFoundation")
elseif(ESP_PLATFORM)
    set(AUDIO_LIBRARY "")
endif()

target_include_directories(engine-sim-bridge
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ENGINE_SIM_PATH}/dependencies/dr_libs
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/scripting/include
)

target_link_libraries(engine-sim-bridge
    PRIVATE
        engine-sim
        ${AUDIO_LIBRARY}
)

target_compile_options(engine-sim-bridge PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags -Wno-unused-private-field -g)

if(TARGET engine-sim-script-interpreter)
    target_link_libraries(engine-sim-bridge
        PRIVATE
            engine-sim-script-interpreter
    )
    target_compile_definitions(engine-sim-bridge PRIVATE ATG_ENGINE_SIM_PIRANHA_ENABLED)
endif()

# Set library output properties
set_target_properties(engine-sim-bridge PROPERTIES
    OUTPUT_NAME "enginesim"
    PUBLIC_HEADER include/simulator/EngineSimTypes.h
)

# Platform-specific settings (not for iOS static lib)
if(APPLE AND NOT IOS)
    set_target_properties(engine-sim-bridge PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION 1
        MACOSX_RPATH TRUE
        INSTALL_NAME_DIR "@rpath"
    )

endif()

# Critical fix for iOS cross compilation:
# Never install native binaries into system paths, never reuse binaries across architectures
# This prevents macOS arm64 binaries getting accidentally linked into iOS simulator builds
if(IOS)
    set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY ON)
    # Copy the bridge library to the parent build directory for Xcode to find
    # Xcode expects the library at ../build-$(PLATFORM_NAME)/libenginesim.a
    add_custom_command(TARGET engine-sim-bridge POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.a ${CMAKE_BINARY_DIR}/libenginesim.a
        COMMENT "Copying libenginesim.a for Xcode linking"
    )
endif()

# Installation (skip for iOS static lib builds)
if(NOT IOS)
    install(TARGETS engine-sim-bridge
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include
    )
endif()

# -----------------------------------------------------------------------------
# Preset Compiler (macOS build-time tool only)
# Compiles .mr scripts to JSON presets for iOS consumption
# Piranha/Boost required -- not available on iOS
# -----------------------------------------------------------------------------
if(NOT IOS AND TARGET engine-sim-script-interpreter)
    add_executable(engine-sim-preset-compiler
        tools/preset_compiler.cpp
        src/common/PresetSerializer.cpp
    )
    target_include_directories(engine-sim-preset-compiler
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ENGINE_SIM_PATH}/dependencies/dr_libs
            ${ENGINE_SIM_PATH}/include
            ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(engine-sim-preset-compiler
        PRIVATE
            engine-sim
            engine-sim-script-interpreter
    )
    target_compile_options(engine-sim-preset-compiler PRIVATE -Wno-unused-parameter)
    target_compile_definitions(engine-sim-preset-compiler PRIVATE ATG_ENGINE_SIM_PIRANHA_ENABLED)

    # Preset Code Generator (macOS build-time tool)
    # Compiles .mr scripts to C++ source code for baked presets
    add_executable(engine-sim-preset-codegen tools/preset_codegen.cpp)
    target_include_directories(engine-sim-preset-codegen
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ENGINE_SIM_PATH}/dependencies/dr_libs
            ${ENGINE_SIM_PATH}/include
            ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(engine-sim-preset-codegen
        PRIVATE
            engine-sim
            engine-sim-script-interpreter
    )
    target_compile_definitions(engine-sim-preset-codegen PRIVATE ATG_ENGINE_SIM_PIRANHA_ENABLED)
endif()

# -----------------------------------------------------------------------------
# Unit tests (bridge owns its own tests)
# Standalone build: `make test` works directly
# As CLI subdirectory: CLI registers bridge_unit_tests with its own CTest
# iOS builds skip tests entirely
# -----------------------------------------------------------------------------
option(BUILD_BRIDGE_TESTS "Build engine-sim-bridge unit tests" ON)
if(IOS OR ESP_PLATFORM)
    set(BUILD_BRIDGE_TESTS OFF CACHE BOOL "Skip tests for embedded build" FORCE)
endif()
if(BUILD_BRIDGE_TESTS)
    # Fetch GoogleTest if not available
    include(FetchContent)
    include(GoogleTest)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Only register with CTest when building standalone (not as a subdirectory)
    # This prevents double-registration when CLI includes us
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        enable_testing()
    endif()

    add_executable(bridge_unit_tests
        test/BridgeUnitTests.cpp
        test/CrankingControllerTests.cpp
        test/simulator/BrakeConstraintTest.cpp
    )
    target_include_directories(bridge_unit_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/dependencies/submodules/simple-2d-constraint-solver/include
    )
    target_link_libraries(bridge_unit_tests
        PRIVATE
            engine-sim-bridge
            GTest::gtest_main
    )
    target_compile_options(bridge_unit_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)

    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME bridge_unit_tests COMMAND bridge_unit_tests --gtest_color=yes)
    endif()

    # Preset engine tests - verifies C++ factory engines match Piranha scripts
    option(BUILD_PRESET_ENGINE_TESTS "Build preset engine tests" OFF)
    if(BUILD_PRESET_ENGINE_TESTS)
        # Isomorphism tests - proves .mr and JSON paths produce identical engines
        add_executable(preset_isomorphism_tests
            test/PresetIsomorphismTests.cpp
        )
        target_include_directories(preset_isomorphism_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${ENGINE_SIM_PATH}/include
        )
        target_compile_definitions(preset_isomorphism_tests PRIVATE
            TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
            TEST_ENGINE_SIM_ASSETS="${ENGINE_SIM_PATH}/assets"
            TEST_ENGINE_SIM_ROOT="${ENGINE_SIM_PATH}"
            TEST_PRESET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/preset"
            TEST_ES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/es"
        )
        if(TARGET engine-sim-script-interpreter)
            target_compile_definitions(preset_isomorphism_tests PRIVATE
                ENGINE_SIM_PIRANHA_ENABLED
                TEST_PRESET_COMPILER_PATH="$<TARGET_FILE:engine-sim-preset-compiler>"
            )
            add_dependencies(preset_isomorphism_tests engine-sim-preset-compiler)
        endif()
        target_link_libraries(preset_isomorphism_tests
            PRIVATE
                GTest::gtest_main
                engine-sim-bridge
        )
        if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
            gtest_discover_tests(
                preset_isomorphism_tests
                DISCOVERY_MODE PRE_TEST
                DISCOVERY_TIMEOUT 30
                EXTRA_ARGS --gtest_color=yes
            )
        endif()
    endif()

    # Sine wave regression tests - proves the int16/float buffer aliasing bug
    # These tests link against engine-sim-bridge (unlike bridge_unit_tests
    # which is standalone). They exercise SineWaveSimulator::renderOnDemand().
    add_executable(sine_wave_regression_tests
        test/SineWaveRegressionTests.cpp
    )
    target_include_directories(sine_wave_regression_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ENGINE_SIM_PATH}/dependencies/dr_libs
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(sine_wave_regression_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
            ${AUDIO_LIBRARY}
    )
    target_compile_options(sine_wave_regression_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME sine_wave_regression_tests COMMAND sine_wave_regression_tests --gtest_color=yes)
    endif()

    # Strategy tests - moved from engine-sim-cli/test/unit to exercise bridge
    # strategy/buffer code under coverage instrumentation (ThreadedStrategy,
    # SyncPullStrategy, IAudioBuffer, CircularBuffer telemetry).
    add_executable(strategy_tests
        test/strategy/ThreadedStrategyIntegrationTest.cpp
        test/strategy/StrategyPipelineTest.cpp
        test/strategy/BufferContextEradicationTest.cpp
        test/strategy/IAudioBufferTest.cpp
        test/strategy/CircularBufferTelemetryTest.cpp
        test/mocks/MockAudioUnit.cpp
        test/AudioTestConstants.cpp
    )
    target_include_directories(strategy_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/test
        ${CMAKE_CURRENT_SOURCE_DIR}/test/mocks
        ${ENGINE_SIM_PATH}/dependencies/dr_libs
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(strategy_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
            ${AUDIO_LIBRARY}
    )
    target_compile_options(strategy_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME strategy_tests COMMAND strategy_tests --gtest_color=yes)
    endif()

    # iOS adapter tests - only built when AudioBufferView and
    # AVAudioEngineHardwareProvider headers exist (after Phase 1 and Phase 2)
    option(BUILD_IOS_ADAPTER_TESTS "Build iOS adapter tests" OFF)
    if(BUILD_IOS_ADAPTER_TESTS)
        add_executable(ios_adapter_tests
            test/iOSAdapterTests.cpp
        )
        target_include_directories(ios_adapter_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        target_link_libraries(ios_adapter_tests
            PRIVATE
                GTest::gtest_main
                engine-sim-bridge
                ${AUDIO_LIBRARY}
        )
        if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
            add_test(NAME ios_adapter_tests COMMAND ios_adapter_tests --gtest_color=yes)
        endif()
    endif()

    # Twin foundation tests - tests for UpstreamSignal and IceVehicleProfile
    add_executable(twin_foundation_tests
        test/twin/UpstreamSignalTest.cpp
        test/twin/IceVehicleProfileTest.cpp
    )
    target_include_directories(twin_foundation_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(twin_foundation_tests
        PRIVATE
            GTest::gtest_main
    )
    target_compile_options(twin_foundation_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME twin_foundation_tests COMMAND twin_foundation_tests --gtest_color=yes)
    endif()

    # Twin gearbox tests - tests for AutomaticGearbox
    add_executable(twin_gearbox_tests
        test/twin/AutomaticGearboxTest.cpp
    )
    target_include_directories(twin_gearbox_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(twin_gearbox_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(twin_gearbox_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME twin_gearbox_tests COMMAND twin_gearbox_tests --gtest_color=yes)
    endif()

    # Twin core tests - tests for ThrottleSmoother, VirtualIceTwin, and ManualTwin
    add_executable(twin_core_tests
        test/twin/ThrottleSmootherTest.cpp
        test/twin/VirtualIceTwinTest.cpp
        test/twin/ManualTwinTest.cpp
    )
    target_include_directories(twin_core_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(twin_core_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(twin_core_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME twin_core_tests COMMAND twin_core_tests --gtest_color=yes)
    endif()

    # Twin input provider tests - tests for VirtualIceInputProvider
    add_executable(twin_input_tests
        test/input/VirtualIceInputProviderTest.cpp
    )
    target_include_directories(twin_input_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(twin_input_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(twin_input_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME twin_input_tests COMMAND twin_input_tests --gtest_color=yes)
    endif()

    # Manual twin provider tests - tests for ManualTwinProvider
    add_executable(manual_twin_provider_tests
        test/input/ManualTwinProviderTest.cpp
    )
    target_include_directories(manual_twin_provider_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(manual_twin_provider_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(manual_twin_provider_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME manual_twin_provider_tests COMMAND manual_twin_provider_tests --gtest_color=yes)
    endif()

    # Demo input tests - tests for DemoThrottleSource and DemoInputProvider
    add_executable(demo_input_tests
        test/input/DemoThrottleSourceTest.cpp
        test/input/DemoInputProviderTest.cpp
        test/input/GearSelectorInputTest.cpp
        test/input/IgnitionInputTest.cpp
        test/input/BrakeInputTest.cpp
        test/input/KeyHoldBridgeTest.cpp
        test/input/BrakeChainTest.cpp
        test/input/DemoChainIntegrationTest.cpp
    )
    target_include_directories(demo_input_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(demo_input_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(demo_input_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME demo_input_tests COMMAND demo_input_tests --gtest_color=yes)
    endif()

    # Keyboard consolidation tests - consolidated KeyboardInputProvider + EngineInputTarget
    add_executable(keyboard_consolidation_tests
        test/input/KeyboardInputProviderTest.cpp
    )
    target_include_directories(keyboard_consolidation_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(keyboard_consolidation_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(keyboard_consolidation_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME keyboard_consolidation_tests COMMAND keyboard_consolidation_tests --gtest_color=yes)
    endif()

    # Twin scenario tests - Phase 1 acceptance criteria tests
    add_executable(twin_scenario_tests
        test/scenarios/AccelerationScenarioTest.cpp
        test/scenarios/CruiseScenarioTest.cpp
        test/scenarios/DecelerationScenarioTest.cpp
        test/scenarios/KickdownScenarioTest.cpp
        test/scenarios/StandstillScenarioTest.cpp
        test/scenarios/LaunchScenarioTest.cpp
        test/scenarios/TelemetrySequenceBuilder.cpp
    )
    target_include_directories(twin_scenario_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(twin_scenario_tests
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    target_compile_options(twin_scenario_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME twin_scenario_tests COMMAND twin_scenario_tests --gtest_color=yes)
    endif()

    # Twin physics integration tests - requires engine-sim linkage
    option(BUILD_INTEGRATION_TESTS "Build integration tests that require engine-sim" OFF)
    if(BUILD_INTEGRATION_TESTS)
        add_executable(twin_integration_tests
            test/scenarios/TwinPhysicsIntegrationTest.cpp
            test/scenarios/TelemetrySequenceBuilder.cpp
        )
        target_include_directories(twin_integration_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/test
        )
        target_link_libraries(twin_integration_tests
            PRIVATE
                GTest::gtest_main
                engine-sim-bridge
        )
        target_compile_definitions(twin_integration_tests PRIVATE BUILD_INTEGRATION_TESTS=ON)
        target_compile_options(twin_integration_tests PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter -Wno-mismatched-tags)
        if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
            add_test(NAME twin_integration_tests COMMAND twin_integration_tests --gtest_color=yes)
        endif()
    endif()
endif()

# ============================================================================
# Phase 0 Evidence-Gathering Spikes
# These are GTest-based spike executables that output deterministically
# collect signal measurements for Phase 0 decisions.
# ============================================================================
option(BUILD_PHASE0_SPIKES "Build Phase 0 spike executables" OFF)
if(BUILD_PHASE0_SPIKES)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes")

    # ---- Audio Sweep Spike ----
    add_executable(audio_sweep_spike
        test/spikes/AudioSweepSpike.cpp
    )
    target_include_directories(audio_sweep_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/dependencies/dr_libs
        ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(audio_sweep_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(audio_sweep_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME audio_sweep_spike COMMAND audio_sweep_spike)
    set_tests_properties(audio_sweep_spike PROPERTIES LABELS "spike")

    # ---- Clutch Parameter Sweep Spike ----
    add_executable(clutch_sweep_spike
        test/spikes/ClutchParameterSweepSpike.cpp
    )
    target_include_directories(clutch_sweep_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
    )
    target_compile_definitions(clutch_sweep_spike PRIVATE
        TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    )
    target_link_libraries(clutch_sweep_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(clutch_sweep_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME clutch_sweep_spike COMMAND clutch_sweep_spike)
    set_tests_properties(clutch_sweep_spike PROPERTIES LABELS "spike")

    # ---- Telemetry Jitter Spike ----
    add_executable(jitter_test_spike
        test/spikes/TelemetryJitterSpike.cpp
    )
    target_include_directories(jitter_test_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/dependencies/dr_libs
        ${ENGINE_SIM_PATH}/scripting/include
    )
    target_link_libraries(jitter_test_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(jitter_test_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME jitter_test_spike COMMAND jitter_test_spike)
    set_tests_properties(jitter_test_spike PROPERTIES LABELS "spike")

    # ---- Dyno Tracking Spike ----
    add_executable(dyno_tracking_spike
        test/spikes/DynoTrackingSpike.cpp
    )
    target_include_directories(dyno_tracking_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
    )
    target_link_libraries(dyno_tracking_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(dyno_tracking_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME dyno_tracking_spike COMMAND dyno_tracking_spike)
    set_tests_properties(dyno_tracking_spike PROPERTIES LABELS "spike")

    # ---- Dyno Launch Control Spike ----
    add_executable(dyno_launch_control_spike
        test/spikes/DynoLaunchControlSpike.cpp
    )
    target_include_directories(dyno_launch_control_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
    )
    target_link_libraries(dyno_launch_control_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(dyno_launch_control_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME dyno_launch_control_spike COMMAND dyno_launch_control_spike)
    set_tests_properties(dyno_launch_control_spike PROPERTIES LABELS "spike")

    # ---- Real Engine Audio Spike (Subaru EJ25) ----
    add_executable(real_engine_audio_spike
        test/spikes/RealEngineAudioSpike.cpp
    )
    target_include_directories(real_engine_audio_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/scripting/include
        ${ENGINE_SIM_PATH}/dependencies/submodules/piranha
    )
    target_link_libraries(real_engine_audio_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
            engine-sim-script-interpreter
    )
    set_target_properties(real_engine_audio_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME real_engine_audio_spike COMMAND real_engine_audio_spike)
    set_tests_properties(real_engine_audio_spike PROPERTIES LABELS "spike")

    # ---- Gearbox Diagnostic Spike ----
    add_executable(gearbox_diagnostic_spike
        test/spikes/GearboxDiagnosticSpike.cpp
    )
    target_include_directories(gearbox_diagnostic_spike PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ENGINE_SIM_PATH}/include
    )
    target_link_libraries(gearbox_diagnostic_spike
        PRIVATE
            GTest::gtest_main
            engine-sim-bridge
    )
    set_target_properties(gearbox_diagnostic_spike PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spikes"
    )
    add_test(NAME gearbox_diagnostic_spike COMMAND gearbox_diagnostic_spike)
    set_tests_properties(gearbox_diagnostic_spike PROPERTIES LABELS "spike")
endif()

