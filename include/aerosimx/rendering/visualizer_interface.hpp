#pragma once

/**
 * @file visualizer_interface.hpp
 * @brief Abstraction layer for external rendering engines (Unreal, Unity, etc.)
 */

#include "aerosimx/core/types.hpp"
#include <memory>
#include <string>
#include <vector>

namespace aerosimx {
namespace rendering {

/**
 * @brief Rendering engine types
 */
enum class RenderingEngineType {
  None,
  UnrealEngine,
  Unity,
  Godot,
  OpenGL,
  Vulkan
};

/**
 * @brief Rendering state for an entity
 */
struct RenderState {
  std::string mesh_path;
  std::string material_path;
  Pose pose;
  Vector3 scale = {1.0, 1.0, 1.0};
  bool visible = true;
  bool casts_shadows = true;
};

/**
 * @brief Viewport configuration
 */
struct ViewportConfig {
  int width = 1280;
  int height = 720;
  double fov = 90.0;
  std::string window_title = "AeroSimX Visualizer";
};

/**
 * @brief Visualizer Interface
 *
 * This interface allows the AeroSimX core to communicate with
 * external rendering engines without being coupled to their APIs.
 */
class IVisualizer {
public:
  virtual ~IVisualizer() = default;

  /**
   * @brief Initialize the rendering engine
   */
  virtual bool initialize(const ViewportConfig &config) = 0;

  /**
   * @brief Shutdown the rendering engine
   */
  virtual void shutdown() = 0;

  /**
   * @brief Update the rendering engine (main loop step)
   */
  virtual void update(double dt) = 0;

  /**
   * @brief Create a renderable entity
   */
  virtual bool create_entity(const std::string &id,
                             const RenderState &initial_state) = 0;

  /**
   * @brief Remove a renderable entity
   */
  virtual bool remove_entity(const std::string &id) = 0;

  /**
   * @brief Update entity pose
   */
  virtual void set_entity_pose(const std::string &id, const Pose &pose) = 0;

  /**
   * @brief Set camera pose
   */
  virtual void set_camera_pose(const Pose &pose) = 0;

  /**
   * @brief Capture image from a virtual camera
   */
  virtual std::vector<uint8_t> capture_image(const std::string &camera_id) = 0;

  /**
   * @brief Get rendering engine type
   */
  virtual RenderingEngineType get_type() const = 0;
};

/**
 * @brief Null Visualizer (for headless mode)
 */
class NullVisualizer : public IVisualizer {
public:
  bool initialize(const ViewportConfig &) override { return true; }
  void shutdown() override {}
  void update(double) override {}
  bool create_entity(const std::string &, const RenderState &) override {
    return true;
  }
  bool remove_entity(const std::string &) override { return true; }
  void set_entity_pose(const std::string &, const Pose &) override {}
  void set_camera_pose(const Pose &) override {}
  std::vector<uint8_t> capture_image(const std::string &) override {
    return {};
  }
  RenderingEngineType get_type() const override {
    return RenderingEngineType::None;
  }
};

} // namespace rendering
} // namespace aerosimx
