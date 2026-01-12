cmake_minimum_required(VERSION 3.20)
project(AeroSimX VERSION 0.1.0 LANGUAGES CXX)

# ============================================================================
# Project Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
option(ENABLE_CUDA "Enable CUDA support for GPU physics" OFF)
option(ENABLE_ROS2 "Enable ROS2 integration" OFF)

# ============================================================================
# Dependencies
# ============================================================================

# Thread support
find_package(Threads REQUIRED)

# Optional: CUDA for GPU physics
if(ENABLE_CUDA)
    find_package(CUDAToolkit)
    if(CUDAToolkit_FOUND)
        enable_language(CUDA)
        add_compile_definitions(AEROSIMX_CUDA_ENABLED)
        message(STATUS "CUDA support enabled")
    else()
        message(WARNING "CUDA requested but not found. Falling back to CPU physics.")
        set(ENABLE_CUDA OFF)
    endif()
endif()

# Optional: ROS2 integration
if(ENABLE_ROS2)
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    if(rclcpp_FOUND)
        add_compile_definitions(AEROSIMX_ROS2_ENABLED)
        message(STATUS "ROS2 support enabled")
    else()
        message(WARNING "ROS2 requested but not found. Disabling ROS2 support.")
        set(ENABLE_ROS2 OFF)
    endif()
endif()

# Optional: pybind11 for Python bindings
if(BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development QUIET)
    if(Python3_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.11.1
        )
        FetchContent_MakeAvailable(pybind11)
        message(STATUS "Python bindings will be built")
    else()
        message(WARNING "Python not found. Disabling Python bindings.")
        set(BUILD_PYTHON_BINDINGS OFF)
    endif()
endif()

# GoogleTest for unit tests
if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    message(STATUS "Unit tests enabled")
endif()

# ============================================================================
# Include Directories
# ============================================================================
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# ============================================================================
# Source Files
# ============================================================================

# Core library sources
set(AEROSIMX_CORE_SOURCES
    src/core/simulation.cpp
    src/core/time_manager.cpp
    src/core/entity_manager.cpp
    src/core/event_system.cpp
)

set(AEROSIMX_PHYSICS_SOURCES
    src/physics/physics_engine.cpp
    src/physics/physics_world.cpp
    src/physics/rigid_body.cpp
    src/physics/collision_system.cpp
)

set(AEROSIMX_SENSOR_SOURCES
    src/sensors/sensor_base.cpp
    src/sensors/lidar.cpp
    src/sensors/camera.cpp
    src/sensors/imu.cpp
    src/sensors/gps.cpp
    src/sensors/radar.cpp
)

set(AEROSIMX_VEHICLE_SOURCES
    src/vehicles/vehicle_base.cpp
    src/vehicles/multirotor.cpp
    src/vehicles/car.cpp
)

set(AEROSIMX_CONTROL_SOURCES
    src/control/control_api.cpp
    src/control/command_queue.cpp
)

# ============================================================================
# Libraries
# ============================================================================

# Core simulation library
add_library(aerosimx_core
    ${AEROSIMX_CORE_SOURCES}
    ${AEROSIMX_PHYSICS_SOURCES}
    ${AEROSIMX_SENSOR_SOURCES}
    ${AEROSIMX_VEHICLE_SOURCES}
    ${AEROSIMX_CONTROL_SOURCES}
)

target_include_directories(aerosimx_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(aerosimx_core
    PUBLIC
        Threads::Threads
)

if(ENABLE_CUDA)
    target_link_libraries(aerosimx_core PUBLIC CUDA::cudart)
endif()

set_target_properties(aerosimx_core PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    OUTPUT_NAME aerosimx
)

# ============================================================================
# Python Bindings
# ============================================================================
if(BUILD_PYTHON_BINDINGS)
    pybind11_add_module(_aerosimx_core
        src/bindings/python_bindings.cpp
    )
    target_link_libraries(_aerosimx_core PRIVATE aerosimx_core)
endif()

# ============================================================================
# Tests
# ============================================================================
if(BUILD_TESTS)
    add_subdirectory(tests/cpp)
endif()

# ============================================================================
# Examples
# ============================================================================
if(BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
endif()

# ============================================================================
# Installation
# ============================================================================
include(GNUInstallDirs)

install(TARGETS aerosimx_core
    EXPORT AeroSimXTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/aerosimx
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT AeroSimXTargets
    FILE AeroSimXTargets.cmake
    NAMESPACE AeroSimX::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AeroSimX
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== AeroSimX Configuration Summary ===")
message(STATUS "Version:          ${PROJECT_VERSION}")
message(STATUS "Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "Shared libs:      ${BUILD_SHARED_LIBS}")
message(STATUS "Tests:            ${BUILD_TESTS}")
message(STATUS "Examples:         ${BUILD_EXAMPLES}")
message(STATUS "Python bindings:  ${BUILD_PYTHON_BINDINGS}")
message(STATUS "CUDA support:     ${ENABLE_CUDA}")
message(STATUS "ROS2 support:     ${ENABLE_ROS2}")
message(STATUS "======================================")
message(STATUS "")
