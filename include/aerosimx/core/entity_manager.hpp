#pragma once

/**
 * @file entity_manager.hpp
 * @brief Entity lifecycle and management
 */

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "aerosimx/core/types.hpp"

namespace aerosimx {

// Forward declarations
namespace vehicles {
class VehicleBase;
}
namespace sensors {
class SensorBase;
}

namespace core {

/**
 * @brief Entity type enumeration
 */
enum class EntityType { Vehicle, Sensor, Environment, Obstacle, Other };

/**
 * @brief Entity information structure
 */
struct EntityInfo {
  EntityId id;
  EntityType type;
  std::string name;
  Pose pose;
  bool active = true;
};

/**
 * @brief Manages all simulation entities
 */
class EntityManager {
public:
  EntityManager();
  ~EntityManager();

  // ========================================================================
  // Entity Lifecycle
  // ========================================================================

  /**
   * @brief Generate a new unique entity ID
   */
  EntityId generate_id();

  /**
   * @brief Register an entity
   */
  void register_entity(EntityId id, EntityType type, const std::string &name);

  /**
   * @brief Unregister an entity
   */
  void unregister_entity(EntityId id);

  /**
   * @brief Check if entity exists
   */
  bool has_entity(EntityId id) const;

  /**
   * @brief Get entity info
   */
  std::optional<EntityInfo> get_entity_info(EntityId id) const;

  /**
   * @brief Get all entities of a specific type
   */
  std::vector<EntityId> get_entities_by_type(EntityType type) const;

  /**
   * @brief Get all entity IDs
   */
  std::vector<EntityId> get_all_entities() const;

  /**
   * @brief Clear all entities
   */
  void clear();

  /**
   * @brief Get entity count
   */
  size_t count() const;

  // ========================================================================
  // Vehicle Management
  // ========================================================================

  /**
   * @brief Add a vehicle to the simulation
   */
  EntityId add_vehicle(std::shared_ptr<vehicles::VehicleBase> vehicle,
                       const std::string &name);

  /**
   * @brief Get a vehicle by ID
   */
  std::shared_ptr<vehicles::VehicleBase> get_vehicle(EntityId id) const;

  /**
   * @brief Get all vehicles
   */
  std::vector<std::shared_ptr<vehicles::VehicleBase>> get_all_vehicles() const;

  /**
   * @brief Remove a vehicle
   */
  bool remove_vehicle(EntityId id);

  // ========================================================================
  // Sensor Management
  // ========================================================================

  /**
   * @brief Attach a sensor to an entity
   */
  bool attach_sensor(EntityId entity_id,
                     std::shared_ptr<sensors::SensorBase> sensor,
                     const Pose &relative_pose = Pose{});

  /**
   * @brief Get sensors attached to an entity
   */
  std::vector<std::shared_ptr<sensors::SensorBase>>
  get_sensors(EntityId entity_id) const;

  /**
   * @brief Detach a sensor from an entity
   */
  bool detach_sensor(EntityId entity_id, EntityId sensor_id);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace core
} // namespace aerosimx
