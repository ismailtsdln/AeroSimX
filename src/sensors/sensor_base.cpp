/**
 * @file sensor_base.cpp
 * @brief Base sensor implementation
 */

#include "aerosimx/sensors/sensor_base.hpp"

#include <cmath>
#include <random>

namespace aerosimx {
namespace sensors {

namespace {
// Thread-local random generator
thread_local std::mt19937 rng{std::random_device{}()};
} // namespace

SensorBase::SensorBase(const SensorConfig &config) : config_(config) {}

SensorBase::~SensorBase() = default;

void SensorBase::shutdown() { config_.enabled = false; }

void SensorBase::reset() {
  time_since_last_update_ = 0.0;
  last_update_time_ = std::chrono::steady_clock::now();
}

void SensorBase::register_callback(GenericSensorCallback callback) {
  callbacks_.push_back(std::move(callback));
}

void SensorBase::clear_callbacks() { callbacks_.clear(); }

void SensorBase::notify_callbacks(const std::any &data) {
  for (const auto &callback : callbacks_) {
    try {
      callback(data);
    } catch (...) {
      // Swallow exceptions from callbacks
    }
  }
}

double SensorBase::apply_noise(double value) const {
  if (config_.noise.model == NoiseModel::None) {
    return value;
  }

  switch (config_.noise.model) {
  case NoiseModel::Gaussian: {
    std::normal_distribution<double> dist(config_.noise.mean,
                                          config_.noise.stddev);
    return value + dist(rng);
  }
  case NoiseModel::Uniform: {
    std::uniform_real_distribution<double> dist(config_.noise.min_value,
                                                config_.noise.max_value);
    return value + dist(rng);
  }
  case NoiseModel::SaltAndPepper: {
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    if (prob_dist(rng) < config_.noise.probability) {
      return (prob_dist(rng) < 0.5) ? config_.noise.min_value
                                    : config_.noise.max_value;
    }
    return value;
  }
  default:
    return value;
  }
}

Vector3 SensorBase::apply_noise(const Vector3 &value) const {
  return {apply_noise(value.x), apply_noise(value.y), apply_noise(value.z)};
}

void SensorBase::update_world_pose(const Pose &parent_pose) {
  // Combine parent pose with relative pose
  world_pose_.position =
      parent_pose.position +
      parent_pose.orientation.rotate(config_.relative_pose.position);
  world_pose_.orientation =
      parent_pose.orientation * config_.relative_pose.orientation;
}

} // namespace sensors
} // namespace aerosimx
