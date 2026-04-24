cmake_minimum_required(VERSION 3.20)
project(engine-sim-bridge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
if(IOS)
    set(PIRANHA_ENABLED OFF CACHE BOOL "Disable scripting for iOS" FORCE)
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
    src/simulation/SimulationLoop.cpp
    # Common utilities
    src/common/ConsoleLogger.cpp
    src/common/CircularBuffer.cpp
)

# Platform-specific audio hardware and factory
if(IOS)
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

# iOS builds produce a static library for linking into Xcode project
# macOS builds produce a shared library (.dylib) for the CLI app
if(IOS)
    add_library(engine-sim-bridge STATIC ${BRIDGE_SOURCES})
else()
    add_library(engine-sim-bridge SHARED ${BRIDGE_SOURCES})
endif()

# Platform-specific audio libraries
if(IOS)
    set(AUDIO_LIBRARY "-framework AudioToolbox -framework CoreAudio -framework CoreFoundation -framework AVFoundation")
elseif(APPLE)
    set(AUDIO_LIBRARY "-framework AudioToolbox -framework CoreAudio -framework CoreFoundation")
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

target_compile_options(engine-sim-bridge PRIVATE -g)

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

    # Custom target that always runs to copy library files
    add_custom_target(copy-engine-sim-bridge ALL
        COMMAND ${CMAKE_COMMAND} -E remove -f
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.1.dylib
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.dylib
            ${CMAKE_BINARY_DIR}/libenginesim.dylib
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.1.0.0.dylib
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.1.dylib
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.1.0.0.dylib
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.dylib
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/libenginesim.1.0.0.dylib
            ${CMAKE_BINARY_DIR}/libenginesim.dylib
        DEPENDS engine-sim-bridge
        COMMENT "Copying libenginesim to build directories"
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
# Unit tests (bridge owns its own tests)
# Standalone build: `make test` works directly
# As CLI subdirectory: CLI registers bridge_unit_tests with its own CTest
# iOS builds skip tests entirely
# -----------------------------------------------------------------------------
option(BUILD_BRIDGE_TESTS "Build engine-sim-bridge unit tests" ON)
if(IOS)
    set(BUILD_BRIDGE_TESTS OFF CACHE BOOL "Skip tests for iOS build" FORCE)
endif()
if(BUILD_BRIDGE_TESTS)
    # Fetch GoogleTest if not available
    include(FetchContent)
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
    )
    target_include_directories(bridge_unit_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(bridge_unit_tests
        PRIVATE
            GTest::gtest_main
    )

    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME bridge_unit_tests COMMAND bridge_unit_tests --gtest_color=yes)
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
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        add_test(NAME sine_wave_regression_tests COMMAND sine_wave_regression_tests --gtest_color=yes)
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
endif()
