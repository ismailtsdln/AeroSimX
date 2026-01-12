/**
 * @file simulation.cpp
 * @brief Core simulation controller implementation
 */

#include "aerosimx/core/simulation.hpp"
#include "aerosimx/physics/physics_engine.hpp"

#include <atomic>
#include <condition_variable>
#include <queue>
#include <thread>

namespace aerosimx {
namespace core {

struct Simulation::Impl {
  SimulationConfig config;
  TimeManager time_manager;
  EntityManager entity_manager;
  EventSystem event_system;

  std::unique_ptr<physics::PhysicsEngine> physics_engine;

  std::atomic<bool> running{false};
  std::atomic<bool> paused{false};
  std::atomic<bool> initialized{false};

  std::mutex step_mutex;
  std::condition_variable step_cv;

  // Callbacks
  std::map<CallbackId, std::function<void(double)>> pre_step_callbacks;
  std::map<CallbackId, std::function<void(double)>> post_step_callbacks;
  CallbackId next_callback_id = 1;

  // Watchdog
  std::chrono::steady_clock::time_point last_step_time;
  std::thread watchdog_thread;
  std::atomic<bool> watchdog_active{false};
};

Simulation::Simulation(const SimulationConfig &config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
  impl_->physics_engine = std::make_unique<physics::PhysicsEngine>(
      physics::PhysicsConfig{.time_step = config.time_step,
                             .max_substeps = config.max_substeps,
                             .enable_collisions = true});
}

Simulation::~Simulation() {
  stop();
  if (impl_->watchdog_thread.joinable()) {
    impl_->watchdog_active = false;
    impl_->watchdog_thread.join();
  }
}

Simulation::Simulation(Simulation &&) noexcept = default;
Simulation &Simulation::operator=(Simulation &&) noexcept = default;

bool Simulation::initialize() {
  if (impl_->initialized) {
    return true;
  }

  // Initialize physics engine
  if (!impl_->physics_engine->initialize()) {
    return false;
  }

  // Initialize time manager
  impl_->time_manager.reset();

  // Start watchdog if enabled
  if (impl_->config.enable_watchdog) {
    impl_->watchdog_active = true;
    impl_->last_step_time = std::chrono::steady_clock::now();

    impl_->watchdog_thread = std::thread([this]() {
      while (impl_->watchdog_active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (impl_->running && !impl_->paused) {
          auto now = std::chrono::steady_clock::now();
          auto elapsed =
              std::chrono::duration<double>(now - impl_->last_step_time)
                  .count();

          if (elapsed > impl_->config.watchdog_timeout) {
            // Watchdog triggered - simulation might be frozen
            // In production, this would trigger recovery
          }
        }
      }
    });
  }

  impl_->initialized = true;
  return true;
}

void Simulation::start() {
  if (!impl_->initialized) {
    initialize();
  }
  impl_->running = true;
  impl_->paused = false;
}

void Simulation::pause() { impl_->paused = true; }

void Simulation::resume() {
  impl_->paused = false;
  impl_->step_cv.notify_all();
}

void Simulation::stop() {
  impl_->running = false;
  impl_->paused = false;
  impl_->step_cv.notify_all();
}

void Simulation::reset() {
  bool was_running = impl_->running;
  stop();

  impl_->time_manager.reset();
  impl_->physics_engine->reset();
  impl_->entity_manager.clear();
  impl_->event_system.clear_queue();

  if (was_running) {
    start();
  }
}

bool Simulation::is_running() const { return impl_->running; }

bool Simulation::is_paused() const { return impl_->paused; }

void Simulation::step(double dt) {
  if (!impl_->initialized || impl_->paused) {
    return;
  }

  std::lock_guard<std::mutex> lock(impl_->step_mutex);

  double actual_dt = (dt > 0) ? dt : impl_->config.time_step;

  // Update watchdog timestamp
  impl_->last_step_time = std::chrono::steady_clock::now();

  // Pre-step callbacks
  for (const auto &[id, callback] : impl_->pre_step_callbacks) {
    callback(actual_dt);
  }

  // Physics step
  if (impl_->config.enable_physics) {
    impl_->physics_engine->step(actual_dt);
  }

  // Update vehicles
  for (auto &vehicle : impl_->entity_manager.get_all_vehicles()) {
    if (vehicle) {
      vehicle->update(actual_dt);
    }
  }

  // Update time
  impl_->time_manager.advance(actual_dt);

  // Process queued events
  impl_->event_system.process_queue();

  // Post-step callbacks
  for (const auto &[id, callback] : impl_->post_step_callbacks) {
    callback(actual_dt);
  }

  // Sync to real time if needed
  if (impl_->config.mode == SimulationMode::RealTime) {
    impl_->time_manager.sync_to_realtime();
  }
}

void Simulation::step_multiple(int count) {
  for (int i = 0; i < count; ++i) {
    step();
  }
}

double Simulation::get_time() const { return impl_->time_manager.get_time(); }

uint64_t Simulation::get_tick() const { return impl_->time_manager.get_tick(); }

EntityManager &Simulation::get_entity_manager() {
  return impl_->entity_manager;
}

const EntityManager &Simulation::get_entity_manager() const {
  return impl_->entity_manager;
}

EventSystem &Simulation::get_event_system() { return impl_->event_system; }

const EventSystem &Simulation::get_event_system() const {
  return impl_->event_system;
}

const SimulationConfig &Simulation::get_config() const { return impl_->config; }

void Simulation::set_config(const SimulationConfig &config) {
  impl_->config = config;
}

void Simulation::set_mode(SimulationMode mode) {
  impl_->config.mode = mode;
  if (mode == SimulationMode::RealTime) {
    impl_->time_manager.set_target_time_ratio(1.0);
  }
}

SimulationMode Simulation::get_mode() const { return impl_->config.mode; }

CallbackId
Simulation::register_pre_step_callback(std::function<void(double)> callback) {
  CallbackId id = impl_->next_callback_id++;
  impl_->pre_step_callbacks[id] = std::move(callback);
  return id;
}

CallbackId
Simulation::register_post_step_callback(std::function<void(double)> callback) {
  CallbackId id = impl_->next_callback_id++;
  impl_->post_step_callbacks[id] = std::move(callback);
  return id;
}

void Simulation::remove_callback(CallbackId id) {
  impl_->pre_step_callbacks.erase(id);
  impl_->post_step_callbacks.erase(id);
}

} // namespace core
} // namespace aerosimx
