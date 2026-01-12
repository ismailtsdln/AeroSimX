/**
 * @file lidar.cpp
 * @brief Lidar sensor implementation
 */

#include "aerosimx/sensors/lidar.hpp"

#include <cmath>
#include <random>

namespace aerosimx {
namespace sensors {

namespace {
thread_local std::mt19937 rng{std::random_device{}()};
}

Lidar::Lidar(const LidarConfig &config)
    : SensorBase(config), lidar_config_(config) {
  config_.name = config.name.empty() ? "lidar" : config.name;
}

Lidar::~Lidar() = default;

bool Lidar::initialize() {
  compute_ray_directions();
  reset();
  return true;
}

void Lidar::update(double dt, const Pose &parent_pose) {
  if (!config_.enabled) {
    return;
  }

  time_since_last_update_ += dt;
  double update_interval = 1.0 / config_.update_rate;

  if (time_since_last_update_ >= update_interval) {
    time_since_last_update_ = 0.0;
    update_world_pose(parent_pose);
    generate_scan();
    last_update_time_ = std::chrono::steady_clock::now();

    // Notify callbacks
    for (const auto &callback : point_cloud_callbacks_) {
      callback(point_cloud_);
    }
    notify_callbacks(std::any(point_cloud_));
  }
}

void Lidar::reset() {
  SensorBase::reset();
  point_cloud_.points.clear();
  point_cloud_.intensities.clear();
  point_cloud_.ring.clear();
}

void Lidar::set_raycast_callback(RaycastCallback callback) {
  raycast_callback_ = std::move(callback);
}

void Lidar::set_lidar_config(const LidarConfig &config) {
  lidar_config_ = config;
  compute_ray_directions();
}

size_t Lidar::get_max_points() const {
  return static_cast<size_t>(lidar_config_.channels) *
         static_cast<size_t>(lidar_config_.points_per_channel);
}

void Lidar::register_point_cloud_callback(SensorCallback<PointCloud> callback) {
  point_cloud_callbacks_.push_back(std::move(callback));
}

void Lidar::compute_ray_directions() {
  ray_directions_.clear();
  ray_ring_indices_.clear();

  int num_channels = lidar_config_.channels;
  int points_per_channel = lidar_config_.points_per_channel;

  double h_fov_rad = lidar_config_.horizontal_fov * M_PI / 180.0;
  double v_fov_upper_rad = lidar_config_.vertical_fov_upper * M_PI / 180.0;
  double v_fov_lower_rad = lidar_config_.vertical_fov_lower * M_PI / 180.0;
  double v_fov_total = v_fov_upper_rad - v_fov_lower_rad;

  ray_directions_.reserve(num_channels * points_per_channel);
  ray_ring_indices_.reserve(num_channels * points_per_channel);

  for (int channel = 0; channel < num_channels; ++channel) {
    // Vertical angle for this channel
    double vertical_angle;
    if (num_channels > 1) {
      vertical_angle =
          v_fov_upper_rad -
          (static_cast<double>(channel) / (num_channels - 1)) * v_fov_total;
    } else {
      vertical_angle = 0.0;
    }

    for (int point = 0; point < points_per_channel; ++point) {
      // Horizontal angle for this point
      double horizontal_angle =
          -h_fov_rad / 2.0 +
          (static_cast<double>(point) / points_per_channel) * h_fov_rad;

      // Convert spherical to Cartesian
      double cos_v = std::cos(vertical_angle);
      double sin_v = std::sin(vertical_angle);
      double cos_h = std::cos(horizontal_angle);
      double sin_h = std::sin(horizontal_angle);

      Vector3 direction = {cos_v * cos_h, cos_v * sin_h, sin_v};

      ray_directions_.push_back(direction);
      ray_ring_indices_.push_back(channel);
    }
  }
}

void Lidar::generate_scan() {
  if (!raycast_callback_) {
    // No raycast callback - generate empty point cloud
    point_cloud_.points.clear();
    point_cloud_.timestamp = std::chrono::steady_clock::now();
    return;
  }

  point_cloud_.points.clear();
  point_cloud_.intensities.clear();
  point_cloud_.ring.clear();
  point_cloud_.timestamp = std::chrono::steady_clock::now();
  point_cloud_.frame_id = config_.name;

  std::uniform_real_distribution<double> dropout_dist(0.0, 1.0);

  for (size_t i = 0; i < ray_directions_.size(); ++i) {
    // Point dropout
    if (lidar_config_.dropout_rate > 0 &&
        dropout_dist(rng) < lidar_config_.dropout_rate) {
      continue;
    }

    // Transform ray direction to world frame
    Vector3 world_direction =
        world_pose_.orientation.rotate(ray_directions_[i]);

    double hit_distance;
    double intensity;
    int object_id;

    bool hit = raycast_callback_(world_pose_.position, world_direction,
                                 lidar_config_.range_max, hit_distance,
                                 intensity, object_id);

    if (hit && hit_distance >= lidar_config_.range_min) {
      // Apply range noise
      double noisy_distance = apply_range_noise(hit_distance);

      // Clamp to valid range
      if (noisy_distance >= lidar_config_.range_min &&
          noisy_distance <= lidar_config_.range_max) {

        // Calculate point position (in sensor frame)
        Vector3 point = ray_directions_[i] * noisy_distance;

        point_cloud_.points.push_back(static_cast<float>(point.x));
        point_cloud_.points.push_back(static_cast<float>(point.y));
        point_cloud_.points.push_back(static_cast<float>(point.z));

        if (lidar_config_.enable_intensity) {
          std::normal_distribution<double> intensity_noise(
              0.0, lidar_config_.intensity_noise_stddev);
          double noisy_intensity =
              std::clamp(intensity + intensity_noise(rng), 0.0, 1.0);
          point_cloud_.intensities.push_back(
              static_cast<float>(noisy_intensity));
        }

        if (lidar_config_.enable_ring_index) {
          point_cloud_.ring.push_back(
              static_cast<uint8_t>(ray_ring_indices_[i]));
        }
      }
    }
  }
}

double Lidar::apply_range_noise(double range) const {
  std::normal_distribution<double> noise(0.0, lidar_config_.range_noise_stddev);
  return range + noise(rng);
}

} // namespace sensors
} // namespace aerosimx
