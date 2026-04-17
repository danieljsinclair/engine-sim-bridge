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
add_subdirectory(${ENGINE_SIM_PATH} ${CMAKE_BINARY_DIR}/engine-sim EXCLUDE_FROM_ALL)

# Real bridge library (uses actual engine-sim with polymorphic simulator selection)
# Phase F: Audio infrastructure moved from CLI into bridge
add_library(engine-sim-bridge SHARED
    # Simulator core
    src/simulator/engine_sim_bridge.cpp
    src/simulator/sine_wave_simulator.cpp
    src/simulator/BridgeSimulator.cpp
    src/simulator/EngineConfig.cpp
    # Audio strategies
    src/strategy/ThreadedStrategy.cpp
    src/strategy/SyncPullStrategy.cpp
    src/strategy/IAudioBufferFactory.cpp
    # Audio hardware
    src/hardware/CoreAudioHardwareProvider.cpp
    src/hardware/AudioHardwareProviderFactory.cpp
    # Telemetry
    src/telemetry/InMemoryTelemetry.cpp
    # Simulation loop
    src/simulation/SimulationLoop.cpp
    # Common utilities
    src/common/ConsoleLogger.cpp
    src/common/CircularBuffer.cpp
)

# Platform-specific audio libraries
if(APPLE)
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
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER include/simulator/engine_sim_bridge.h
)

# Platform-specific settings
if(APPLE)
    set_target_properties(engine-sim-bridge PROPERTIES
        MACOSX_RPATH TRUE
        INSTALL_NAME_DIR "@rpath"
    )
endif()

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

# Installation
install(TARGETS engine-sim-bridge
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# -----------------------------------------------------------------------------
# Unit tests (bridge owns its own tests)
# Standalone build: `make test` works directly
# As CLI subdirectory: CLI registers bridge_unit_tests with its own CTest
# -----------------------------------------------------------------------------
option(BUILD_BRIDGE_TESTS "Build engine-sim-bridge unit tests" ON)
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
endif()
