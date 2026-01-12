#pragma once

/**
 * @file gps.hpp
 * @brief GPS/GNSS sensor implementation
 */

#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

namespace aerosimx {
namespace sensors {

/**
 * @brief GPS fix type
 */
enum class GpsFixType {
  NoFix,
  Fix2D,
  Fix3D,
  DGPS,      ///< Differential GPS
  RTK_Float, ///< RTK floating
  RTK_Fixed  ///< RTK fixed
};

/**
 * @brief GPS signal degradation scenarios
 */
struct GpsDegradation {
  bool enabled = false;
  double multipath_error = 0.0; ///< Additional error from multipath (meters)
  double signal_blockage = 0.0; ///< Probability of signal loss (0-1)
  double jamming_level = 0.0;   ///< Jamming interference level (0-1)
  double spoofing_offset_lat = 0.0; ///< Spoofed position offset
  double spoofing_offset_lon = 0.0;
  bool urban_canyon = false; ///< Enable urban canyon effects
  bool indoor = false;       ///< Indoor (degraded/no signal)
};

/**
 * @brief GPS configuration
 */
struct GpsConfig : public SensorConfig {
  // Reference origin (for local to GPS coordinate conversion)
  double origin_latitude = 0.0;  ///< Origin latitude in degrees
  double origin_longitude = 0.0; ///< Origin longitude in degrees
  double origin_altitude = 0.0;  ///< Origin altitude in meters (ASL)

  // Accuracy settings
  double horizontal_accuracy = 2.5; ///< Horizontal CEP in meters
  double vertical_accuracy = 5.0;   ///< Vertical accuracy in meters
  double velocity_accuracy = 0.1;   ///< Velocity accuracy in m/s

  // Timing
  double fix_rate = 1.0;         ///< Fixes per second (1-20 Hz typical)
  double cold_start_time = 30.0; ///< Time to first fix (cold start)
  double warm_start_time = 5.0;  ///< Time to first fix (warm start)

  // Constellation
  int min_satellites = 4;  ///< Minimum satellites for fix
  int max_satellites = 12; ///< Maximum visible satellites

  // Degradation
  GpsDegradation degradation;
};

/**
 * @brief GPS sensor implementation
 *
 * Simulates GPS/GNSS receiver with realistic accuracy models,
 * satellite visibility, and signal degradation scenarios.
 */
class Gps : public SensorBase {
public:
  explicit Gps(const GpsConfig &config);
  ~Gps() override;

  // ========================================================================
  // SensorBase interface
  // ========================================================================

  bool initialize() override;
  void update(double dt, const Pose &parent_pose) override;
  void reset() override;
  SensorType get_type() const override { return SensorType::Gps; }
  std::string get_type_name() const override { return "GPS"; }

  // ========================================================================
  // GPS-specific interface
  // ========================================================================

  /**
   * @brief Get the latest GPS data
   */
  const GpsData &get_gps_data() const { return gps_data_; }

  /**
   * @brief Get GPS configuration
   */
  const GpsConfig &get_gps_config() const { return gps_config_; }

  /**
   * @brief Update GPS configuration
   */
  void set_gps_config(const GpsConfig &config);

  /**
   * @brief Get current fix type
   */
  GpsFixType get_fix_type() const { return fix_type_; }

  /**
   * @brief Check if GPS has valid fix
   */
  bool has_fix() const { return gps_data_.fix_valid; }

  /**
   * @brief Get time since last fix
   */
  double get_time_since_fix() const;

  /**
   * @brief Set degradation scenario
   */
  void set_degradation(const GpsDegradation &degradation);

  /**
   * @brief Convert local position to GPS coordinates
   */
  static void local_to_gps(const Vector3 &local_pos, double origin_lat,
                           double origin_lon, double origin_alt, double &lat,
                           double &lon, double &alt);

  /**
   * @brief Convert GPS coordinates to local position
   */
  static void gps_to_local(double lat, double lon, double alt,
                           double origin_lat, double origin_lon,
                           double origin_alt, Vector3 &local_pos);

  /**
   * @brief Register GPS data callback
   */
  void register_gps_callback(SensorCallback<GpsData> callback);

private:
  /**
   * @brief Simulate satellite visibility
   */
  int compute_visible_satellites();

  /**
   * @brief Compute DOP values
   */
  void compute_dop();

  /**
   * @brief Apply GPS error model
   */
  void apply_gps_error();

  /**
   * @brief Apply degradation effects
   */
  void apply_degradation();

  GpsConfig gps_config_;
  GpsData gps_data_;
  GpsFixType fix_type_ = GpsFixType::NoFix;

  // Internal state
  double time_since_start_ = 0.0;
  double time_since_last_fix_ = 0.0;
  bool first_fix_acquired_ = false;

  std::vector<SensorCallback<GpsData>> gps_callbacks_;
};

} // namespace sensors
} // namespace aerosimx
