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
set(BUILD_ENGINE_SIM_APP OFF CACHE BOOL "Build engine-sim GUI app" FORCE)
set(BUILD_BRIDGE OFF CACHE BOOL "Build bridge from engine-sim (we are the bridge)" FORCE)
set(DISCORD_ENABLED OFF CACHE BOOL "Enable Discord Rich Presence" FORCE)
add_subdirectory(${ENGINE_SIM_PATH} ${CMAKE_BINARY_DIR}/engine-sim EXCLUDE_FROM_ALL)

# Bridge library
add_library(engine-sim-bridge SHARED
    src/engine_sim_bridge.cpp
)

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
)

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
    PUBLIC_HEADER include/engine_sim_bridge.h
)

# Platform-specific settings
if(APPLE)
    set_target_properties(engine-sim-bridge PROPERTIES
        MACOSX_RPATH TRUE
        INSTALL_NAME_DIR "@rpath"
    )
endif()

# Installation
install(TARGETS engine-sim-bridge
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)
