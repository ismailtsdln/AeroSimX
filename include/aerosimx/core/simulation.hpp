#pragma once

/**
 * @file simulation.hpp
 * @brief Core simulation controller for AeroSimX
 * 
 * This file contains the main Simulation class that orchestrates
 * all simulation components including physics, sensors, and vehicles.
 */

#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>

#include "aerosimx/core/types.hpp"
#include "aerosimx/core/time_manager.hpp"
#include "aerosimx/core/entity_manager.hpp"
#include "aerosimx/core/event_system.hpp"

namespace aerosimx {
namespace core {

/**
 * @brief Simulation mode enumeration
 */
enum class SimulationMode {
    RealTime,       ///< Run at real-time speed
    Accelerated,    ///< Run as fast as possible
    StepByStep      ///< Manual stepping mode for RL/ML training
};

/**
 * @brief Simulation configuration structure
 */
struct SimulationConfig {
    double time_step = 0.001;           ///< Physics time step in seconds
    double sensor_update_rate = 0.01;   ///< Sensor update interval in seconds
    SimulationMode mode = SimulationMode::RealTime;
    bool enable_physics = true;
    bool enable_rendering = true;
    int max_substeps = 10;              ///< Maximum physics substeps per frame
    bool enable_watchdog = true;        ///< Enable simulation watchdog
    double watchdog_timeout = 5.0;      ///< Watchdog timeout in seconds
};

/**
 * @brief Main simulation controller class
 * 
 * The Simulation class is the central orchestrator for the entire
 * simulation environment. It manages time stepping, entity lifecycle,
 * physics updates, and sensor data generation.
 */
class Simulation {
public:
    /**
     * @brief Construct a new Simulation object
     * @param config Simulation configuration
     */
    explicit Simulation(const SimulationConfig& config = SimulationConfig{});
    
    /**
     * @brief Destroy the Simulation object
     */
    ~Simulation();

    // Non-copyable
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // Movable
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    // ========================================================================
    // Lifecycle Management
    // ========================================================================

    /**
     * @brief Initialize the simulation
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Start the simulation
     */
    void start();

    /**
     * @brief Pause the simulation
     */
    void pause();

    /**
     * @brief Resume a paused simulation
     */
    void resume();

    /**
     * @brief Stop the simulation
     */
    void stop();

    /**
     * @brief Reset the simulation to initial state
     */
    void reset();

    /**
     * @brief Check if simulation is running
     */
    bool is_running() const;

    /**
     * @brief Check if simulation is paused
     */
    bool is_paused() const;

    // ========================================================================
    // Time Stepping
    // ========================================================================

    /**
     * @brief Advance simulation by one time step
     * @param dt Optional time step override (uses config if not specified)
     */
    void step(double dt = -1.0);

    /**
     * @brief Advance simulation by multiple steps
     * @param count Number of steps to advance
     */
    void step_multiple(int count);

    /**
     * @brief Get current simulation time
     */
    double get_time() const;

    /**
     * @brief Get current simulation tick count
     */
    uint64_t get_tick() const;

    // ========================================================================
    // Entity Management
    // ========================================================================

    /**
     * @brief Get the entity manager
     */
    EntityManager& get_entity_manager();
    const EntityManager& get_entity_manager() const;

    /**
     * @brief Get the event system
     */
    EventSystem& get_event_system();
    const EventSystem& get_event_system() const;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get current configuration
     */
    const SimulationConfig& get_config() const;

    /**
     * @brief Update simulation configuration
     * @note Some settings may require restart to take effect
     */
    void set_config(const SimulationConfig& config);

    /**
     * @brief Set simulation mode
     */
    void set_mode(SimulationMode mode);

    /**
     * @brief Get current simulation mode
     */
    SimulationMode get_mode() const;

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Register a pre-step callback
     * @param callback Function to call before each physics step
     * @return Callback ID for removal
     */
    CallbackId register_pre_step_callback(std::function<void(double)> callback);

    /**
     * @brief Register a post-step callback
     * @param callback Function to call after each physics step
     * @return Callback ID for removal
     */
    CallbackId register_post_step_callback(std::function<void(double)> callback);

    /**
     * @brief Remove a registered callback
     */
    void remove_callback(CallbackId id);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace core
} // namespace aerosimx
