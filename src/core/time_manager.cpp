/**
 * @file time_manager.cpp
 * @brief Time management implementation
 */

#include "aerosimx/core/time_manager.hpp"

#include <thread>

namespace aerosimx {
namespace core {

TimeManager::TimeManager() { reset(); }

TimeManager::~TimeManager() = default;

void TimeManager::reset() {
  sim_time_ = 0.0;
  tick_count_ = 0;
  start_wall_time_ = std::chrono::steady_clock::now();
  last_sync_time_ = start_wall_time_;
}

void TimeManager::advance(double dt) {
  sim_time_ += dt;
  tick_count_++;
}

double TimeManager::get_time() const { return sim_time_; }

uint64_t TimeManager::get_tick() const { return tick_count_; }

double TimeManager::get_wall_time() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start_wall_time_).count();
}

double TimeManager::get_time_ratio() const {
  double wall_time = get_wall_time();
  if (wall_time <= 0) {
    return 1.0;
  }
  return sim_time_ / wall_time;
}

void TimeManager::set_target_time_ratio(double ratio) {
  target_time_ratio_ = ratio;
}

void TimeManager::sync_to_realtime() {
  double target_wall_time = sim_time_ / target_time_ratio_;
  double actual_wall_time = get_wall_time();

  double sleep_time = target_wall_time - actual_wall_time;

  if (sleep_time > 0.0001) { // Sleep if more than 0.1ms ahead
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
  }

  last_sync_time_ = std::chrono::steady_clock::now();
}

bool TimeManager::is_behind_realtime() const {
  double target_wall_time = sim_time_ / target_time_ratio_;
  double actual_wall_time = get_wall_time();
  return actual_wall_time > target_wall_time;
}

} // namespace core
} // namespace aerosimx
