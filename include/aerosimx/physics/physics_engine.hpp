#pragma once

/**
 * @file physics_engine.hpp
 * @brief Physics engine interface and implementation
 */

#include <functional>
#include <memory>
#include <vector>

#include "aerosimx/core/types.hpp"

namespace aerosimx {
namespace physics {

/**
 * @brief Physics configuration
 */
struct PhysicsConfig {
  Vector3 gravity = {0, 0, -9.81};
  double time_step = 0.001;
  int max_substeps = 10;
  bool enable_collisions = true;
  bool enable_friction = true;
  double default_friction = 0.5;
  double default_restitution = 0.3;

  // GPU acceleration settings (for future CUDA support)
  bool use_gpu = false;
  int batch_size = 64;
};

/**
 * @brief Rigid body properties
 */
struct RigidBodyProperties {
  double mass = 1.0;
  Vector3 inertia = {1, 1, 1};
  double friction = 0.5;
  double restitution = 0.3;
  bool is_static = false;
  bool is_kinematic = false;
  bool enable_gravity = true;
};

/**
 * @brief Rigid body handle
 */
using BodyId = uint64_t;
constexpr BodyId INVALID_BODY_ID = 0;

/**
 * @brief Collision shape types
 */
enum class ShapeType { Sphere, Box, Capsule, Cylinder, Mesh, Compound };

/**
 * @brief Collision shape definition
 */
struct CollisionShape {
  ShapeType type = ShapeType::Box;
  Vector3 dimensions = {1, 1, 1}; // Interpretation depends on type
  Pose local_pose;                // Offset from body center

  static CollisionShape sphere(double radius) {
    CollisionShape shape;
    shape.type = ShapeType::Sphere;
    shape.dimensions = {radius, 0, 0};
    return shape;
  }

  static CollisionShape box(double x, double y, double z) {
    CollisionShape shape;
    shape.type = ShapeType::Box;
    shape.dimensions = {x, y, z};
    return shape;
  }

  static CollisionShape capsule(double radius, double height) {
    CollisionShape shape;
    shape.type = ShapeType::Capsule;
    shape.dimensions = {radius, height, 0};
    return shape;
  }
};

/**
 * @brief Contact point information
 */
struct ContactPoint {
  Vector3 position;
  Vector3 normal;
  double penetration_depth;
  BodyId body_a;
  BodyId body_b;
};

/**
 * @brief Collision callback function type
 */
using CollisionCallback =
    std::function<void(BodyId, BodyId, const std::vector<ContactPoint> &)>;

/**
 * @brief Physics engine interface
 *
 * Provides physics simulation capabilities with GPU-ready architecture.
 * Initial implementation uses CPU-based physics, with hooks for
 * future CUDA acceleration.
 */
class PhysicsEngine {
public:
  PhysicsEngine();
  explicit PhysicsEngine(const PhysicsConfig &config);
  ~PhysicsEngine();

  // Non-copyable
  PhysicsEngine(const PhysicsEngine &) = delete;
  PhysicsEngine &operator=(const PhysicsEngine &) = delete;

  // Movable
  PhysicsEngine(PhysicsEngine &&) noexcept;
  PhysicsEngine &operator=(PhysicsEngine &&) noexcept;

  // ========================================================================
  // Lifecycle
  // ========================================================================

  /**
   * @brief Initialize the physics engine
   */
  bool initialize();

  /**
   * @brief Reset to initial state
   */
  void reset();

  /**
   * @brief Step the simulation forward
   * @param dt Time step in seconds
   */
  void step(double dt);

  /**
   * @brief Batch step for multiple bodies (GPU-optimized)
   */
  void step_batch(double dt, int num_steps);

  // ========================================================================
  // Body Management
  // ========================================================================

  /**
   * @brief Create a rigid body
   */
  BodyId create_body(const RigidBodyProperties &props, const Pose &initial_pose,
                     const CollisionShape &shape);

  /**
   * @brief Destroy a rigid body
   */
  void destroy_body(BodyId id);

  /**
   * @brief Check if body exists
   */
  bool has_body(BodyId id) const;

  /**
   * @brief Get body pose
   */
  Pose get_pose(BodyId id) const;

  /**
   * @brief Set body pose (teleport)
   */
  void set_pose(BodyId id, const Pose &pose);

  /**
   * @brief Get body velocity
   */
  Twist get_velocity(BodyId id) const;

  /**
   * @brief Set body velocity
   */
  void set_velocity(BodyId id, const Twist &velocity);

  // ========================================================================
  // Forces and Torques
  // ========================================================================

  /**
   * @brief Apply force at center of mass
   */
  void apply_force(BodyId id, const Vector3 &force);

  /**
   * @brief Apply force at a point in world space
   */
  void apply_force_at_point(BodyId id, const Vector3 &force,
                            const Vector3 &point);

  /**
   * @brief Apply torque
   */
  void apply_torque(BodyId id, const Vector3 &torque);

  /**
   * @brief Apply impulse at center of mass
   */
  void apply_impulse(BodyId id, const Vector3 &impulse);

  // ========================================================================
  // Collision Detection
  // ========================================================================

  /**
   * @brief Set collision callback
   */
  void set_collision_callback(CollisionCallback callback);

  /**
   * @brief Ray cast into the physics world
   * @return True if hit, false otherwise
   */
  bool raycast(const Vector3 &origin, const Vector3 &direction,
               double max_distance, BodyId &hit_body, Vector3 &hit_point,
               Vector3 &hit_normal) const;

  /**
   * @brief Sphere cast
   */
  bool sphere_cast(const Vector3 &origin, const Vector3 &direction,
                   double radius, double max_distance, BodyId &hit_body,
                   Vector3 &hit_point) const;

  // ========================================================================
  // Configuration
  // ========================================================================

  /**
   * @brief Get current configuration
   */
  const PhysicsConfig &get_config() const;

  /**
   * @brief Update configuration
   */
  void set_config(const PhysicsConfig &config);

  /**
   * @brief Set gravity
   */
  void set_gravity(const Vector3 &gravity);

  // ========================================================================
  // Statistics
  // ========================================================================

  /**
   * @brief Get number of active bodies
   */
  size_t get_body_count() const;

  /**
   * @brief Get last step computation time in milliseconds
   */
  double get_step_time_ms() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace physics
} // namespace aerosimx
