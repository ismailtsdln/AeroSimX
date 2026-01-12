#pragma once

/**
 * @file sensor_base.hpp
 * @brief Base class for all sensors
 */

#include <any>
#include <functional>
#include <memory>
#include <string>

#include "aerosimx/core/types.hpp"

namespace aerosimx {
namespace sensors {

/**
 * @brief Sensor type enumeration
 */
enum class SensorType {
  Lidar,
  Camera,
  DepthCamera,
  SegmentationCamera,
  Imu,
  Gps,
  Radar,
  Ultrasonic,
  Magnetometer,
  Barometer,
  Custom
};

/**
 * @brief Noise model types
 */
enum class NoiseModel { None, Gaussian, Uniform, SaltAndPepper, Custom };

/**
 * @brief Noise configuration
 */
struct NoiseConfig {
  NoiseModel model = NoiseModel::Gaussian;
  double mean = 0.0;
  double stddev = 0.01;
  double min_value = 0.0;
  double max_value = 1.0;
  double probability = 0.05; // For salt-and-pepper noise
};

/**
 * @brief Base sensor configuration
 */
struct SensorConfig {
  std::string name = "sensor";
  double update_rate = 10.0; // Hz
  Pose relative_pose;        // Pose relative to parent entity
  bool enabled = true;
  NoiseConfig noise;
};

/**
 * @brief Sensor data callback type
 */
template <typename DataType>
using SensorCallback = std::function<void(const DataType &)>;

/**
 * @brief Generic sensor data callback
 */
using GenericSensorCallback = std::function<void(const std::any &)>;

/**
 * @brief Base class for all sensors
 */
class SensorBase {
public:
  explicit SensorBase(const SensorConfig &config);
  virtual ~SensorBase();

  // Non-copyable
  SensorBase(const SensorBase &) = delete;
  SensorBase &operator=(const SensorBase &) = delete;

  // ========================================================================
  // Lifecycle
  // ========================================================================

  /**
   * @brief Initialize the sensor
   */
  virtual bool initialize() = 0;

  /**
   * @brief Shutdown the sensor
   */
  virtual void shutdown();

  /**
   * @brief Update the sensor (called by simulation)
   * @param dt Time since last update
   * @param parent_pose Current pose of parent entity
   */
  virtual void update(double dt, const Pose &parent_pose) = 0;

  /**
   * @brief Reset sensor state
   */
  virtual void reset();

  // ========================================================================
  // Properties
  // ========================================================================

  /**
   * @brief Get sensor type
   */
  virtual SensorType get_type() const = 0;

  /**
   * @brief Get sensor type as string
   */
  virtual std::string get_type_name() const = 0;

  /**
   * @brief Get sensor ID
   */
  EntityId get_id() const { return id_; }

  /**
   * @brief Set sensor ID
   */
  void set_id(EntityId id) { id_ = id; }

  /**
   * @brief Get sensor name
   */
  const std::string &get_name() const { return config_.name; }

  /**
   * @brief Get sensor configuration
   */
  const SensorConfig &get_config() const { return config_; }

  /**
   * @brief Update sensor configuration
   */
  void set_config(const SensorConfig &config) { config_ = config; }

  /**
   * @brief Get current world pose
   */
  const Pose &get_world_pose() const { return world_pose_; }

  /**
   * @brief Check if sensor is enabled
   */
  bool is_enabled() const { return config_.enabled; }

  /**
   * @brief Enable/disable sensor
   */
  void set_enabled(bool enabled) { config_.enabled = enabled; }

  // ========================================================================
  // Data Access
  // ========================================================================

  /**
   * @brief Register a data callback
   */
  void register_callback(GenericSensorCallback callback);

  /**
   * @brief Clear all callbacks
   */
  void clear_callbacks();

  /**
   * @brief Get last data timestamp
   */
  Timestamp get_last_update_time() const { return last_update_time_; }

protected:
  /**
   * @brief Notify registered callbacks with new data
   */
  void notify_callbacks(const std::any &data);

  /**
   * @brief Apply noise to a value
   */
  double apply_noise(double value) const;

  /**
   * @brief Apply noise to a vector
   */
  Vector3 apply_noise(const Vector3 &value) const;

  /**
   * @brief Update world pose based on parent pose
   */
  void update_world_pose(const Pose &parent_pose);

  EntityId id_ = INVALID_ENTITY_ID;
  SensorConfig config_;
  Pose world_pose_;
  Timestamp last_update_time_;
  double time_since_last_update_ = 0.0;
  std::vector<GenericSensorCallback> callbacks_;
};

} // namespace sensors
} // namespace aerosimx
