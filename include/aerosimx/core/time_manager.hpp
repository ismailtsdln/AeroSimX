#pragma once

/**
 * @file time_manager.hpp
 * @brief Time management for simulation
 */

#include <atomic>
#include <chrono>
#include <functional>

namespace aerosimx {
namespace core {

/**
 * @brief Manages simulation time and synchronization
 */
class TimeManager {
public:
  TimeManager();
  ~TimeManager();

  /**
   * @brief Reset time to zero
   */
  void reset();

  /**
   * @brief Advance simulation time
   * @param dt Time step in seconds
   */
  void advance(double dt);

  /**
   * @brief Get current simulation time in seconds
   */
  double get_time() const;

  /**
   * @brief Get current tick count
   */
  uint64_t get_tick() const;

  /**
   * @brief Get time since last reset in wall-clock time
   */
  double get_wall_time() const;

  /**
   * @brief Get simulation-to-real time ratio
   */
  double get_time_ratio() const;

  /**
   * @brief Set target time ratio for real-time sync
   * @param ratio 1.0 = real-time, 2.0 = 2x speed, etc.
   */
  void set_target_time_ratio(double ratio);

  /**
   * @brief Synchronize with real time (for real-time mode)
   */
  void sync_to_realtime();

  /**
   * @brief Check if we're behind real time
   */
  bool is_behind_realtime() const;

private:
  std::atomic<double> sim_time_{0.0};
  std::atomic<uint64_t> tick_count_{0};
  std::chrono::steady_clock::time_point start_wall_time_;
  std::chrono::steady_clock::time_point last_sync_time_;
  double target_time_ratio_{1.0};
};

} // namespace core
} // namespace aerosimx
