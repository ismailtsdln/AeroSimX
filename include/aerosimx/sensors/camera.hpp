#pragma once

/**
 * @file camera.hpp
 * @brief Camera sensor implementation
 */

#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

#include <vector>

namespace aerosimx {
namespace sensors {

/**
 * @brief Camera image type
 */
enum class CameraImageType {
  RGB,          ///< Standard RGB image
  RGBA,         ///< RGB with alpha channel
  Grayscale,    ///< Single channel grayscale
  Depth,        ///< Depth image (float meters)
  Segmentation, ///< Semantic segmentation
  Normals,      ///< Surface normals
  OpticalFlow   ///< Optical flow
};

/**
 * @brief Lens distortion model
 */
enum class DistortionModel {
  None,
  BrownConrady, ///< Standard radial-tangential model
  Fisheye,      ///< Fisheye (equidistant) model
  Kannala       ///< Kannala-Brandt model
};

/**
 * @brief Camera configuration
 */
struct CameraConfig : public SensorConfig {
  uint32_t width = 640;  ///< Image width in pixels
  uint32_t height = 480; ///< Image height in pixels
  double fov = 90.0;     ///< Horizontal field of view in degrees
  CameraImageType image_type = CameraImageType::RGB;

  // Intrinsic parameters
  double fx = 0.0; ///< Focal length x (auto-computed if 0)
  double fy = 0.0; ///< Focal length y (auto-computed if 0)
  double cx = 0.0; ///< Principal point x (auto-computed if 0)
  double cy = 0.0; ///< Principal point y (auto-computed if 0)

  // Distortion parameters
  DistortionModel distortion_model = DistortionModel::None;
  std::array<double, 5> distortion_coeffs = {0, 0, 0, 0,
                                             0}; // k1, k2, p1, p2, k3

  // Depth camera settings
  double depth_min = 0.1;   ///< Minimum depth in meters
  double depth_max = 100.0; ///< Maximum depth in meters

  // Post-processing
  bool auto_exposure = false;
  double exposure_time = 0.0; ///< Exposure time in seconds (0 = auto)
  double gamma = 1.0;
  bool motion_blur = false;
  double motion_blur_amount = 0.0;

  // Noise settings
  double gaussian_noise_stddev = 0.0;
  double chromatic_aberration = 0.0;
  double vignette_intensity = 0.0;
};

/**
 * @brief Render callback type for image generation
 */
using RenderCallback = std::function<void(
    const Pose &camera_pose, const CameraConfig &config, ImageData &out_image)>;

/**
 * @brief Camera sensor implementation
 *
 * Supports RGB, depth, and segmentation imaging with configurable
 * resolution, FOV, and lens distortion models.
 */
class Camera : public SensorBase {
public:
  explicit Camera(const CameraConfig &config);
  ~Camera() override;

  // ========================================================================
  // SensorBase interface
  // ========================================================================

  bool initialize() override;
  void update(double dt, const Pose &parent_pose) override;
  void reset() override;
  SensorType get_type() const override;
  std::string get_type_name() const override;

  // ========================================================================
  // Camera-specific interface
  // ========================================================================

  /**
   * @brief Set the render callback for image generation
   */
  void set_render_callback(RenderCallback callback);

  /**
   * @brief Get the latest image
   */
  const ImageData &get_image() const { return image_; }

  /**
   * @brief Get depth image (if depth camera)
   */
  const DepthImage &get_depth_image() const { return depth_image_; }

  /**
   * @brief Get camera configuration
   */
  const CameraConfig &get_camera_config() const { return camera_config_; }

  /**
   * @brief Update camera configuration
   */
  void set_camera_config(const CameraConfig &config);

  /**
   * @brief Get camera intrinsic matrix (3x3, row-major)
   */
  std::array<double, 9> get_intrinsic_matrix() const;

  /**
   * @brief Get projection matrix (4x4, row-major)
   */
  std::array<double, 16> get_projection_matrix() const;

  /**
   * @brief Project a 3D point to 2D image coordinates
   */
  bool project_point(const Vector3 &world_point, double &u, double &v) const;

  /**
   * @brief Unproject 2D image coordinates to 3D ray
   */
  Vector3 unproject_point(double u, double v) const;

  /**
   * @brief Register image callback
   */
  void register_image_callback(SensorCallback<ImageData> callback);

  /**
   * @brief Register depth image callback
   */
  void register_depth_callback(SensorCallback<DepthImage> callback);

private:
  /**
   * @brief Compute intrinsic parameters from FOV
   */
  void compute_intrinsics();

  /**
   * @brief Apply lens distortion to a point
   */
  void apply_distortion(double &x, double &y) const;

  /**
   * @brief Remove lens distortion from a point
   */
  void remove_distortion(double &x, double &y) const;

  CameraConfig camera_config_;
  ImageData image_;
  DepthImage depth_image_;
  RenderCallback render_callback_;
  std::vector<SensorCallback<ImageData>> image_callbacks_;
  std::vector<SensorCallback<DepthImage>> depth_callbacks_;

  // Computed intrinsics
  double fx_, fy_, cx_, cy_;
};

} // namespace sensors
} // namespace aerosimx
