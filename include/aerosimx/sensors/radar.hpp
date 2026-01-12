#pragma once

/**
 * @file radar.hpp
 * @brief Radar sensor implementation
 */

#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

#include <vector>

namespace aerosimx {
namespace sensors {

/**
 * @brief Radar configuration
 */
struct RadarConfig : public SensorConfig {
  // Range settings
  double range_min = 0.5;        ///< Minimum detection range (meters)
  double range_max = 200.0;      ///< Maximum detection range (meters)
  double range_resolution = 0.5; ///< Range resolution (meters)

  // Angular settings
  double horizontal_fov = 60.0;    ///< Horizontal FOV (degrees)
  double vertical_fov = 20.0;      ///< Vertical FOV (degrees)
  double angular_resolution = 1.0; ///< Angular resolution (degrees)

  // Velocity measurement
  double velocity_max = 100.0;      ///< Max measurable velocity (m/s)
  double velocity_resolution = 0.1; ///< Velocity resolution (m/s)

  // Detection settings
  int max_detections = 256;            ///< Maximum detected objects
  double detection_probability = 0.95; ///< Probability of detection
  double false_alarm_rate = 0.001;     ///< False alarm probability

  // Noise settings
  double range_noise_stddev = 0.2;    ///< Range measurement noise
  double velocity_noise_stddev = 0.5; ///< Velocity measurement noise
  double angle_noise_stddev = 0.5;    ///< Angle measurement noise (degrees)
};

/**
 * @brief Object query callback for radar
 */
using RadarQueryCallback = std::function<void(
    const Pose &radar_pose, double range_max, double horizontal_fov,
    double vertical_fov, std::vector<RadarDetection> &detections)>;

/**
 * @brief Radar sensor implementation
 *
 * Simulates automotive radar with range, velocity, and angle
 * measurements. Supports FMCW radar characteristics.
 */
class Radar : public SensorBase {
public:
  explicit Radar(const RadarConfig &config);
  ~Radar() override;

  // ========================================================================
  // SensorBase interface
  // ========================================================================

  bool initialize() override;
  void update(double dt, const Pose &parent_pose) override;
  void reset() override;
  SensorType get_type() const override { return SensorType::Radar; }
  std::string get_type_name() const override { return "Radar"; }

  // ========================================================================
  // Radar-specific interface
  // ========================================================================

  /**
   * @brief Set the query callback for object detection
   */
  void set_query_callback(RadarQueryCallback callback);

  /**
   * @brief Get the latest radar data
   */
  const RadarData &get_radar_data() const { return radar_data_; }

  /**
   * @brief Get radar configuration
   */
  const RadarConfig &get_radar_config() const { return radar_config_; }

  /**
   * @brief Update radar configuration
   */
  void set_radar_config(const RadarConfig &config);

  /**
   * @brief Get number of detections in last scan
   */
  size_t get_detection_count() const { return radar_data_.detections.size(); }

  /**
   * @brief Register radar data callback
   */
  void register_radar_callback(SensorCallback<RadarData> callback);

private:
  /**
   * @brief Apply detection probability filter
   */
  void filter_detections();

  /**
   * @brief Add false alarms
   */
  void add_false_alarms();

  /**
   * @brief Apply measurement noise
   */
  void apply_measurement_noise();

  RadarConfig radar_config_;
  RadarData radar_data_;
  RadarQueryCallback query_callback_;
  std::vector<SensorCallback<RadarData>> radar_callbacks_;
};

} // namespace sensors
} // namespace aerosimx
