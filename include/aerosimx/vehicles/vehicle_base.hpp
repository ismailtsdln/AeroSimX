#pragma once

/**
 * @file vehicle_base.hpp
 * @brief Base class for all vehicles
 */

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "aerosimx/core/types.hpp"
#include "aerosimx/physics/physics_engine.hpp"

namespace aerosimx {

namespace sensors {
class SensorBase;
}

namespace vehicles {

/**
 * @brief Vehicle type enumeration
 */
enum class VehicleType {
  Multirotor, ///< Quadcopter, hexacopter, etc.
  FixedWing,  ///< Fixed-wing aircraft
  Car,        ///< Ground vehicle
  Truck,      ///< Large ground vehicle
  Boat,       ///< Watercraft
  Robot,      ///< Generic robot
  Custom      ///< Custom vehicle type
};

/**
 * @brief Collision response types
 */
enum class CollisionResponse {
  Stop,        ///< Stop simulation on collision
  Bounce,      ///< Physical bounce response
  PassThrough, ///< Ignore collision
  Damage       ///< Apply damage model
};

/**
 * @brief Base vehicle configuration
 */
struct VehicleConfig {
  std::string name = "vehicle";
  VehicleType type = VehicleType::Multirotor;
  Pose initial_pose;

  // Physics properties
  double mass = 1.0;                  ///< Mass in kg
  Vector3 inertia = {0.1, 0.1, 0.1};  ///< Diagonal inertia tensor
  Vector3 center_of_mass = {0, 0, 0}; ///< CoM offset from origin
  double drag_coefficient = 0.3;

  // Collision settings
  CollisionResponse collision_response = CollisionResponse::Bounce;
  physics::CollisionShape collision_shape;

  // State limits
  double max_speed = 50.0;        ///< Maximum speed in m/s
  double max_acceleration = 20.0; ///< Maximum acceleration in m/s^2
  double max_angular_rate = 3.14; ///< Maximum angular rate in rad/s
};

/**
 * @brief Base class for all vehicles
 *
 * Provides core vehicle functionality including state management,
 * physics integration, sensor attachment, and control input handling.
 */
class VehicleBase {
public:
  explicit VehicleBase(const VehicleConfig &config);
  virtual ~VehicleBase();

  // Non-copyable
  VehicleBase(const VehicleBase &) = delete;
  VehicleBase &operator=(const VehicleBase &) = delete;

  // ========================================================================
  // Lifecycle
  // ========================================================================

  /**
   * @brief Initialize the vehicle
   */
  virtual bool initialize();

  /**
   * @brief Reset to initial state
   */
  virtual void reset();

  /**
   * @brief Update vehicle state
   * @param dt Time step in seconds
   */
  virtual void update(double dt) = 0;

  /**
   * @brief Called when physics step completes
   */
  virtual void on_physics_update();

  // ========================================================================
  // State Access
  // ========================================================================

  /**
   * @brief Get current vehicle state
   */
  const VehicleState &get_state() const { return state_; }

  /**
   * @brief Get current pose
   */
  const Pose &get_pose() const { return state_.pose; }

  /**
   * @brief Get current velocity
   */
  const Twist &get_velocity() const { return state_.velocity; }

  /**
   * @brief Get current acceleration
   */
  const Vector3 &get_acceleration() const { return state_.acceleration; }

  /**
   * @brief Check if collision detected
   */
  bool has_collision() const { return state_.collision_detected; }

  // ========================================================================
  // State Modification
  // ========================================================================

  /**
   * @brief Teleport vehicle to a new pose
   */
  void set_pose(const Pose &pose);

  /**
   * @brief Set velocity directly
   */
  void set_velocity(const Twist &velocity);

  /**
   * @brief Apply force at center of mass
   */
  void apply_force(const Vector3 &force);

  /**
   * @brief Apply force at a point
   */
  void apply_force_at_point(const Vector3 &force, const Vector3 &point);

  /**
   * @brief Apply torque
   */
  void apply_torque(const Vector3 &torque);

  // ========================================================================
  // Properties
  // ========================================================================

  /**
   * @brief Get entity ID
   */
  EntityId get_id() const { return id_; }

  /**
   * @brief Set entity ID
   */
  void set_id(EntityId id) { id_ = id; }

  /**
   * @brief Get vehicle name
   */
  const std::string &get_name() const { return config_.name; }

  /**
   * @brief Get vehicle type
   */
  VehicleType get_type() const { return config_.type; }

  /**
   * @brief Get vehicle type as string
   */
  virtual std::string get_type_name() const = 0;

  /**
   * @brief Get vehicle configuration
   */
  const VehicleConfig &get_config() const { return config_; }

  // ========================================================================
  // Sensor Management
  // ========================================================================

  /**
   * @brief Attach a sensor to the vehicle
   */
  bool attach_sensor(std::shared_ptr<sensors::SensorBase> sensor,
                     const Pose &relative_pose = Pose{});

  /**
   * @brief Detach a sensor
   */
  bool detach_sensor(EntityId sensor_id);

  /**
   * @brief Get all attached sensors
   */
  const std::vector<std::shared_ptr<sensors::SensorBase>> &get_sensors() const;

  /**
   * @brief Get sensor by name
   */
  std::shared_ptr<sensors::SensorBase>
  get_sensor(const std::string &name) const;

  /**
   * @brief Update all attached sensors
   */
  void update_sensors(double dt);

  // ========================================================================
  // Physics Integration
  // ========================================================================

  /**
   * @brief Set physics engine reference
   */
  void set_physics_engine(physics::PhysicsEngine *engine);

  /**
   * @brief Get physics body ID
   */
  physics::BodyId get_physics_body() const { return physics_body_; }

  // ========================================================================
  // Callbacks
  // ========================================================================

  /**
   * @brief Register collision callback
   */
  void set_collision_callback(
      std::function<void(const Vector3 &, EntityId)> callback);

  /**
   * @brief Register state update callback
   */
  void set_state_callback(std::function<void(const VehicleState &)> callback);

protected:
  /**
   * @brief Update state from physics engine
   */
  void sync_from_physics();

  /**
   * @brief Push state to physics engine
   */
  void sync_to_physics();

  /**
   * @brief Apply control inputs (vehicle-specific)
   */
  virtual void apply_controls(double dt) = 0;

  /**
   * @brief Compute aerodynamic forces
   */
  virtual void compute_aerodynamics(double dt);

  /**
   * @brief Handle collision event
   */
  virtual void on_collision(const Vector3 &contact_point, EntityId other_id);

  EntityId id_ = INVALID_ENTITY_ID;
  VehicleConfig config_;
  VehicleState state_;

  // Physics
  physics::PhysicsEngine *physics_engine_ = nullptr;
  physics::BodyId physics_body_ = physics::INVALID_BODY_ID;

  // Sensors
  std::vector<std::shared_ptr<sensors::SensorBase>> sensors_;
  std::map<std::string, size_t> sensor_name_map_;
  std::vector<Pose> sensor_relative_poses_;

  // Callbacks
  std::function<void(const Vector3 &, EntityId)> collision_callback_;
  std::function<void(const VehicleState &)> state_callback_;

  // Forces accumulated this frame
  Vector3 accumulated_force_;
  Vector3 accumulated_torque_;
};

} // namespace vehicles
} // namespace aerosimx
