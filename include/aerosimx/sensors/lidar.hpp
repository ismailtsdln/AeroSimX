#pragma once

/**
 * @file lidar.hpp
 * @brief Lidar sensor implementation
 */

#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

#include <functional>
#include <vector>

namespace aerosimx {
namespace sensors {

/**
 * @brief Lidar configuration
 */
struct LidarConfig : public SensorConfig {
  int channels = 16; ///< Number of vertical channels (16, 32, 64, 128)
  int points_per_channel = 1024;     ///< Horizontal resolution
  double range_min = 0.5;            ///< Minimum range in meters
  double range_max = 100.0;          ///< Maximum range in meters
  double horizontal_fov = 360.0;     ///< Horizontal FOV in degrees
  double vertical_fov_upper = 15.0;  ///< Upper vertical FOV in degrees
  double vertical_fov_lower = -15.0; ///< Lower vertical FOV in degrees
  double rotation_frequency = 10.0;  ///< Rotation frequency in Hz

  // Noise settings
  double range_noise_stddev = 0.02; ///< Range noise standard deviation (meters)
  double intensity_noise_stddev = 0.1;
  double dropout_rate = 0.0; ///< Probability of point dropout

  // Advanced settings
  bool enable_intensity = true;
  bool enable_ring_index = true;
  bool return_mode_dual = false; ///< Dual return mode
};

/**
 * @brief Raycast callback for environment queries
 */
using RaycastCallback = std::function<bool(
    const Vector3 &origin, const Vector3 &direction, double max_range,
    double &hit_distance, double &intensity, int &object_id)>;

/**
 * @brief Lidar sensor implementation
 *
 * Simulates various lidar configurations from 16 to 128 channels.
 * Supports configurable noise models, multiple return modes, and
 * intensity simulation.
 */
class Lidar : public SensorBase {
public:
  explicit Lidar(const LidarConfig &config);
  ~Lidar() override;

  // ========================================================================
  // SensorBase interface
  // ========================================================================

  bool initialize() override;
  void update(double dt, const Pose &parent_pose) override;
  void reset() override;
  SensorType get_type() const override { return SensorType::Lidar; }
  std::string get_type_name() const override { return "Lidar"; }

  // ========================================================================
  // Lidar-specific interface
  // ========================================================================

  /**
   * @brief Set the raycast callback for environment queries
   *
   * This callback is used to query the environment for ray-object
   * intersections. Must be set before the sensor can generate data.
   */
  void set_raycast_callback(RaycastCallback callback);

  /**
   * @brief Get the latest point cloud data
   */
  const PointCloud &get_point_cloud() const { return point_cloud_; }

  /**
   * @brief Get lidar configuration
   */
  const LidarConfig &get_lidar_config() const { return lidar_config_; }

  /**
   * @brief Update lidar configuration
   */
  void set_lidar_config(const LidarConfig &config);

  /**
   * @brief Get number of points in last scan
   */
  size_t get_point_count() const { return point_cloud_.num_points(); }

  /**
   * @brief Get theoretical max points per scan
   */
  size_t get_max_points() const;

  /**
   * @brief Register point cloud callback
   */
  void register_point_cloud_callback(SensorCallback<PointCloud> callback);

private:
  /**
   * @brief Generate a complete scan
   */
  void generate_scan();

  /**
   * @brief Calculate ray directions for all channels
   */
  void compute_ray_directions();

  /**
   * @brief Apply sensor-specific noise
   */
  double apply_range_noise(double range) const;

  LidarConfig lidar_config_;
  PointCloud point_cloud_;
  RaycastCallback raycast_callback_;
  std::vector<SensorCallback<PointCloud>> point_cloud_callbacks_;

  // Pre-computed ray directions
  std::vector<Vector3> ray_directions_;
  std::vector<int> ray_ring_indices_;
};

} // namespace sensors
} // namespace aerosimx
