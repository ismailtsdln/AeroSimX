#pragma once

/**
 * @file types.hpp
 * @brief Common type definitions for AeroSimX
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace aerosimx {

// ============================================================================
// Fundamental Types
// ============================================================================

/// Entity identifier type
using EntityId = uint64_t;

/// Callback identifier type
using CallbackId = uint64_t;

/// Timestamp type
using Timestamp = std::chrono::steady_clock::time_point;

/// Invalid entity ID constant
constexpr EntityId INVALID_ENTITY_ID = 0;

/// Invalid callback ID constant
constexpr CallbackId INVALID_CALLBACK_ID = 0;

// ============================================================================
// Mathematical Types
// ============================================================================

/**
 * @brief 3D Vector type
 */
struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vector3() = default;
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vector3 operator+(const Vector3 &other) const {
    return {x + other.x, y + other.y, z + other.z};
  }

  Vector3 operator-(const Vector3 &other) const {
    return {x - other.x, y - other.y, z - other.z};
  }

  Vector3 operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  Vector3 operator/(double scalar) const {
    return {x / scalar, y / scalar, z / scalar};
  }

  Vector3 &operator+=(const Vector3 &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Vector3 &operator-=(const Vector3 &other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  double dot(const Vector3 &other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  Vector3 cross(const Vector3 &other) const {
    return {y * other.z - z * other.y, z * other.x - x * other.z,
            x * other.y - y * other.x};
  }

  double magnitude() const;
  double magnitude_squared() const { return x * x + y * y + z * z; }
  Vector3 normalized() const;

  static Vector3 zero() { return {0, 0, 0}; }
  static Vector3 one() { return {1, 1, 1}; }
  static Vector3 up() { return {0, 0, 1}; }
  static Vector3 forward() { return {1, 0, 0}; }
  static Vector3 right() { return {0, 1, 0}; }
};

/**
 * @brief Quaternion for rotation representation
 */
struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Quaternion() = default;
  Quaternion(double w_, double x_, double y_, double z_)
      : w(w_), x(x_), y(y_), z(z_) {}

  Quaternion operator*(const Quaternion &other) const;
  Vector3 rotate(const Vector3 &v) const;
  Quaternion conjugate() const { return {w, -x, -y, -z}; }
  Quaternion normalized() const;

  /// Convert to Euler angles (roll, pitch, yaw in radians)
  Vector3 to_euler() const;

  /// Create from Euler angles (roll, pitch, yaw in radians)
  static Quaternion from_euler(double roll, double pitch, double yaw);

  /// Create from axis-angle representation
  static Quaternion from_axis_angle(const Vector3 &axis, double angle);

  static Quaternion identity() { return {1, 0, 0, 0}; }
};

/**
 * @brief Pose (position + orientation)
 */
struct Pose {
  Vector3 position;
  Quaternion orientation;

  Pose() = default;
  Pose(const Vector3 &pos, const Quaternion &orient)
      : position(pos), orientation(orient) {}
};

/**
 * @brief Twist (linear + angular velocity)
 */
struct Twist {
  Vector3 linear;
  Vector3 angular;

  Twist() = default;
  Twist(const Vector3 &lin, const Vector3 &ang) : linear(lin), angular(ang) {}
};

/**
 * @brief 4x4 Transformation matrix
 */
struct Transform {
  std::array<double, 16> data;

  Transform();

  static Transform from_pose(const Pose &pose);
  Pose to_pose() const;

  Transform operator*(const Transform &other) const;
  Vector3 transform_point(const Vector3 &point) const;
  Vector3 transform_vector(const Vector3 &vector) const;

  static Transform identity();
};

// ============================================================================
// Sensor Data Types
// ============================================================================

/**
 * @brief Point cloud data structure
 */
struct PointCloud {
  std::vector<float> points;      ///< XYZ points (size = 3 * num_points)
  std::vector<float> intensities; ///< Intensity per point (optional)
  std::vector<uint8_t> ring;      ///< Ring index per point (for lidar)
  Timestamp timestamp;
  std::string frame_id;

  size_t num_points() const { return points.size() / 3; }
};

/**
 * @brief Image data structure
 */
struct ImageData {
  std::vector<uint8_t> data;
  uint32_t width = 0;
  uint32_t height = 0;
  uint8_t channels = 3; ///< 1=grayscale, 3=RGB, 4=RGBA
  Timestamp timestamp;
  std::string frame_id;

  size_t size_bytes() const { return data.size(); }
};

/**
 * @brief Depth image data
 */
struct DepthImage {
  std::vector<float> data; ///< Depth values in meters
  uint32_t width = 0;
  uint32_t height = 0;
  float min_depth = 0.1f;
  float max_depth = 100.0f;
  Timestamp timestamp;
  std::string frame_id;
};

/**
 * @brief IMU measurement
 */
struct ImuData {
  Vector3 linear_acceleration; ///< m/s^2
  Vector3 angular_velocity;    ///< rad/s
  Quaternion orientation;      ///< Optional orientation estimate
  Timestamp timestamp;
  std::string frame_id;
};

/**
 * @brief GPS/GNSS measurement
 */
struct GpsData {
  double latitude = 0.0;  ///< degrees
  double longitude = 0.0; ///< degrees
  double altitude = 0.0;  ///< meters above sea level
  double hdop = 1.0;      ///< Horizontal dilution of precision
  double vdop = 1.0;      ///< Vertical dilution of precision
  int num_satellites = 0;
  bool fix_valid = false;
  Timestamp timestamp;
};

/**
 * @brief Radar detection
 */
struct RadarDetection {
  float range = 0.0f;     ///< Distance in meters
  float azimuth = 0.0f;   ///< Horizontal angle in radians
  float elevation = 0.0f; ///< Vertical angle in radians
  float velocity = 0.0f;  ///< Radial velocity in m/s
  float rcs = 0.0f;       ///< Radar cross-section in dBsm
};

/**
 * @brief Radar data frame
 */
struct RadarData {
  std::vector<RadarDetection> detections;
  Timestamp timestamp;
  std::string frame_id;
};

// ============================================================================
// Vehicle State Types
// ============================================================================

/**
 * @brief Complete vehicle state
 */
struct VehicleState {
  Pose pose;
  Twist velocity;
  Vector3 acceleration;
  Timestamp timestamp;
  bool collision_detected = false;
};

/**
 * @brief Multirotor (drone) specific state
 */
struct MultirotorState {
  VehicleState base_state;
  std::vector<double> motor_speeds; ///< RPM per motor
  double battery_percentage = 100.0;
  bool is_armed = false;
  bool is_landed = true;
};

/**
 * @brief Ground vehicle specific state
 */
struct CarState {
  VehicleState base_state;
  double steering_angle = 0.0; ///< radians
  double throttle = 0.0;       ///< 0-1
  double brake = 0.0;          ///< 0-1
  double speed = 0.0;          ///< m/s
  int gear = 0;                ///< -1=reverse, 0=neutral, 1+=forward
};

// ============================================================================
// Control Input Types
// ============================================================================

/**
 * @brief Multirotor control input
 */
struct MultirotorControls {
  double roll = 0.0;     ///< Target roll angle or rate
  double pitch = 0.0;    ///< Target pitch angle or rate
  double yaw_rate = 0.0; ///< Target yaw rate
  double throttle = 0.0; ///< Collective throttle (0-1)
  bool is_rate_mode = false;
};

/**
 * @brief Ground vehicle control input
 */
struct CarControls {
  double steering = 0.0; ///< -1 (left) to 1 (right)
  double throttle = 0.0; ///< 0 to 1
  double brake = 0.0;    ///< 0 to 1
  bool handbrake = false;
  int gear = 0; ///< -1=reverse, 0=neutral, 1+=forward
};

} // namespace aerosimx
