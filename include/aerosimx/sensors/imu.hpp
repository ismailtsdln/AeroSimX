#pragma once

/**
 * @file imu.hpp
 * @brief Inertial Measurement Unit sensor implementation
 */

#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

namespace aerosimx {
namespace sensors {

/**
 * @brief IMU noise and bias model
 */
struct ImuNoiseModel {
  // Accelerometer noise
  double accel_noise_density = 0.01;     ///< m/s^2/sqrt(Hz)
  double accel_bias_random_walk = 0.001; ///< m/s^3/sqrt(Hz)
  Vector3 accel_bias = {0, 0, 0};        ///< Current bias state

  // Gyroscope noise
  double gyro_noise_density = 0.001;     ///< rad/s/sqrt(Hz)
  double gyro_bias_random_walk = 0.0001; ///< rad/s^2/sqrt(Hz)
  Vector3 gyro_bias = {0, 0, 0};         ///< Current bias state

  // Temperature effects
  double temp_coefficient = 0.0; ///< Bias change per degree C
  double reference_temp = 25.0;  ///< Reference temperature
};

/**
 * @brief IMU configuration
 */
struct ImuConfig : public SensorConfig {
  ImuNoiseModel noise_model;

  // Sensor ranges
  double accel_max_range = 160.0; ///< Max acceleration in m/s^2
  double gyro_max_range = 34.9;   ///< Max angular velocity in rad/s

  // Sensor resolution
  double accel_resolution = 0.0001; ///< m/s^2 per LSB
  double gyro_resolution = 0.00001; ///< rad/s per LSB

  // Orientation estimation
  bool estimate_orientation = true; ///< Enable onboard orientation filter
  double orientation_noise = 0.01;  ///< Orientation uncertainty

  // Gravity compensation
  Vector3 gravity = {0, 0, -9.81};
};

/**
 * @brief IMU sensor implementation
 *
 * Simulates a 6-DOF IMU with accelerometer and gyroscope,
 * including realistic noise models, bias drift, and saturation.
 */
class Imu : public SensorBase {
public:
  explicit Imu(const ImuConfig &config);
  ~Imu() override;

  // ========================================================================
  // SensorBase interface
  // ========================================================================

  bool initialize() override;
  void update(double dt, const Pose &parent_pose) override;
  void reset() override;
  SensorType get_type() const override { return SensorType::Imu; }
  std::string get_type_name() const override { return "IMU"; }

  // ========================================================================
  // IMU-specific interface
  // ========================================================================

  /**
   * @brief Set the true vehicle state for computing IMU readings
   */
  void set_vehicle_state(const VehicleState &state);

  /**
   * @brief Get the latest IMU data
   */
  const ImuData &get_imu_data() const { return imu_data_; }

  /**
   * @brief Get IMU configuration
   */
  const ImuConfig &get_imu_config() const { return imu_config_; }

  /**
   * @brief Update IMU configuration
   */
  void set_imu_config(const ImuConfig &config);

  /**
   * @brief Get current accelerometer bias estimate
   */
  Vector3 get_accel_bias() const;

  /**
   * @brief Get current gyroscope bias estimate
   */
  Vector3 get_gyro_bias() const;

  /**
   * @brief Set temperature for temperature-dependent bias
   */
  void set_temperature(double temp_celsius);

  /**
   * @brief Register IMU data callback
   */
  void register_imu_callback(SensorCallback<ImuData> callback);

private:
  /**
   * @brief Compute accelerometer reading from true state
   */
  Vector3 compute_acceleration(const VehicleState &state);

  /**
   * @brief Compute gyroscope reading from true state
   */
  Vector3 compute_angular_velocity(const VehicleState &state);

  /**
   * @brief Apply noise and bias to measurement
   */
  void apply_sensor_noise(double dt);

  /**
   * @brief Update bias random walk
   */
  void update_bias_drift(double dt);

  /**
   * @brief Apply sensor saturation
   */
  void apply_saturation();

  ImuConfig imu_config_;
  ImuData imu_data_;
  VehicleState current_state_;
  double current_temperature_ = 25.0;

  // Internal bias state (evolves over time)
  Vector3 accel_bias_state_;
  Vector3 gyro_bias_state_;

  // Previous state for differentiation
  Pose previous_pose_;
  Twist previous_velocity_;

  std::vector<SensorCallback<ImuData>> imu_callbacks_;
};

} // namespace sensors
} // namespace aerosimx
