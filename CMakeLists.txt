cmake_minimum_required(VERSION 3.20)
project(engine-sim-bridge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find engine-sim (expected to be a sibling directory or submodule)
set(ENGINE_SIM_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../engine-sim" CACHE PATH "Path to engine-sim")

if(NOT EXISTS "${ENGINE_SIM_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "engine-sim not found at ${ENGINE_SIM_PATH}")
endif()

# Configure engine-sim
set(BUILD_ENGINE_SIM_APP OFF CACHE BOOL "Disable GUI application" FORCE)
set(PIRANHA_ENABLED ON CACHE BOOL "Enable Piranha scripting" FORCE)

# Add engine-sim subdirectory
add_subdirectory(${ENGINE_SIM_PATH} ${CMAKE_BINARY_DIR}/engine-sim EXCLUDE_FROM_ALL)

# Bridge library
add_library(engine-sim-bridge SHARED
    src/engine_sim_bridge.cpp
)

target_include_directories(engine-sim-bridge
    PUBLIC
        include
        ${ENGINE_SIM_PATH}/include
        ${ENGINE_SIM_PATH}/scripting/include
)

target_link_libraries(engine-sim-bridge
    PRIVATE
        engine-sim
)

if(PIRANHA_ENABLED)
    target_link_libraries(engine-sim-bridge
        PRIVATE
            engine-sim-script-interpreter
    )
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
