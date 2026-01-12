#pragma once

/**
 * @file aerosimx.hpp
 * @brief Main header file for AeroSimX library
 *
 * Include this file to get access to all AeroSimX functionality.
 */

// Core
#include "aerosimx/core/entity_manager.hpp"
#include "aerosimx/core/event_system.hpp"
#include "aerosimx/core/simulation.hpp"
#include "aerosimx/core/time_manager.hpp"
#include "aerosimx/core/types.hpp"

// Physics
#include "aerosimx/physics/physics_engine.hpp"

// Sensors
#include "aerosimx/sensors/camera.hpp"
#include "aerosimx/sensors/gps.hpp"
#include "aerosimx/sensors/imu.hpp"
#include "aerosimx/sensors/lidar.hpp"
#include "aerosimx/sensors/radar.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

// Vehicles
#include "aerosimx/vehicles/car.hpp"
#include "aerosimx/vehicles/multirotor.hpp"
#include "aerosimx/vehicles/vehicle_base.hpp"

// Control
#include "aerosimx/control/control_api.hpp"

/**
 * @mainpage AeroSimX Documentation
 *
 * @section intro Introduction
 *
 * AeroSimX is a next-generation, modular simulation platform for
 * autonomous vehicles, drones, and robots.
 *
 * @section features Key Features
 *
 * - High-performance C++ simulation core
 * - Comprehensive sensor suite (Lidar, Camera, IMU, GPS, Radar)
 * - Multiple vehicle types (Multirotor, Car)
 * - Python API for easy scripting
 * - ROS2 integration
 *
 * @section quickstart Quick Start
 *
 * @code{.cpp}
 * #include <aerosimx/aerosimx.hpp>
 *
 * int main() {
 *     using namespace aerosimx;
 *
 *     core::Simulation sim;
 *     sim.initialize();
 *
 *     // Create and configure vehicles, sensors, etc.
 *
 *     sim.start();
 *     return 0;
 * }
 * @endcode
 */
