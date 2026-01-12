/**
 * @file entity_manager.cpp
 * @brief Entity lifecycle management implementation
 */

#include "aerosimx/core/entity_manager.hpp"
#include "aerosimx/sensors/sensor_base.hpp"
#include "aerosimx/vehicles/vehicle_base.hpp"

#include <algorithm>

namespace aerosimx {
namespace core {

struct EntityManager::Impl {
  std::unordered_map<EntityId, EntityInfo> entities;
  std::unordered_map<EntityId, std::shared_ptr<vehicles::VehicleBase>> vehicles;
  std::unordered_map<EntityId,
                     std::vector<std::shared_ptr<sensors::SensorBase>>>
      entity_sensors;
  std::unordered_map<EntityId, std::vector<Pose>> sensor_relative_poses;

  std::atomic<EntityId> next_entity_id{1};
  mutable std::mutex mutex;
};

EntityManager::EntityManager() : impl_(std::make_unique<Impl>()) {}

EntityManager::~EntityManager() = default;

EntityId EntityManager::generate_id() { return impl_->next_entity_id++; }

void EntityManager::register_entity(EntityId id, EntityType type,
                                    const std::string &name) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  EntityInfo info;
  info.id = id;
  info.type = type;
  info.name = name;
  info.active = true;

  impl_->entities[id] = info;
}

void EntityManager::unregister_entity(EntityId id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  impl_->entities.erase(id);
  impl_->vehicles.erase(id);
  impl_->entity_sensors.erase(id);
  impl_->sensor_relative_poses.erase(id);
}

bool EntityManager::has_entity(EntityId id) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->entities.find(id) != impl_->entities.end();
}

std::optional<EntityInfo> EntityManager::get_entity_info(EntityId id) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  auto it = impl_->entities.find(id);
  if (it == impl_->entities.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<EntityId>
EntityManager::get_entities_by_type(EntityType type) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  std::vector<EntityId> result;
  for (const auto &[id, info] : impl_->entities) {
    if (info.type == type) {
      result.push_back(id);
    }
  }
  return result;
}

std::vector<EntityId> EntityManager::get_all_entities() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  std::vector<EntityId> result;
  result.reserve(impl_->entities.size());
  for (const auto &[id, _] : impl_->entities) {
    result.push_back(id);
  }
  return result;
}

void EntityManager::clear() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  impl_->entities.clear();
  impl_->vehicles.clear();
  impl_->entity_sensors.clear();
  impl_->sensor_relative_poses.clear();
}

size_t EntityManager::count() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->entities.size();
}

EntityId
EntityManager::add_vehicle(std::shared_ptr<vehicles::VehicleBase> vehicle,
                           const std::string &name) {
  EntityId id = generate_id();
  vehicle->set_id(id);

  std::lock_guard<std::mutex> lock(impl_->mutex);

  EntityInfo info;
  info.id = id;
  info.type = EntityType::Vehicle;
  info.name = name;
  info.active = true;

  impl_->entities[id] = info;
  impl_->vehicles[id] = vehicle;

  return id;
}

std::shared_ptr<vehicles::VehicleBase>
EntityManager::get_vehicle(EntityId id) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  auto it = impl_->vehicles.find(id);
  if (it == impl_->vehicles.end()) {
    return nullptr;
  }
  return it->second;
}

std::vector<std::shared_ptr<vehicles::VehicleBase>>
EntityManager::get_all_vehicles() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  std::vector<std::shared_ptr<vehicles::VehicleBase>> result;
  result.reserve(impl_->vehicles.size());
  for (const auto &[_, vehicle] : impl_->vehicles) {
    result.push_back(vehicle);
  }
  return result;
}

bool EntityManager::remove_vehicle(EntityId id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  auto it = impl_->vehicles.find(id);
  if (it == impl_->vehicles.end()) {
    return false;
  }

  impl_->vehicles.erase(it);
  impl_->entities.erase(id);
  impl_->entity_sensors.erase(id);

  return true;
}

bool EntityManager::attach_sensor(EntityId entity_id,
                                  std::shared_ptr<sensors::SensorBase> sensor,
                                  const Pose &relative_pose) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  if (impl_->entities.find(entity_id) == impl_->entities.end()) {
    return false;
  }

  EntityId sensor_id = impl_->next_entity_id++;
  sensor->set_id(sensor_id);

  EntityInfo sensor_info;
  sensor_info.id = sensor_id;
  sensor_info.type = EntityType::Sensor;
  sensor_info.name = sensor->get_name();
  sensor_info.active = true;

  impl_->entities[sensor_id] = sensor_info;
  impl_->entity_sensors[entity_id].push_back(sensor);
  impl_->sensor_relative_poses[entity_id].push_back(relative_pose);

  return true;
}

std::vector<std::shared_ptr<sensors::SensorBase>>
EntityManager::get_sensors(EntityId entity_id) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  auto it = impl_->entity_sensors.find(entity_id);
  if (it == impl_->entity_sensors.end()) {
    return {};
  }
  return it->second;
}

bool EntityManager::detach_sensor(EntityId entity_id, EntityId sensor_id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  auto it = impl_->entity_sensors.find(entity_id);
  if (it == impl_->entity_sensors.end()) {
    return false;
  }

  auto &sensors = it->second;
  auto &poses = impl_->sensor_relative_poses[entity_id];

  for (size_t i = 0; i < sensors.size(); ++i) {
    if (sensors[i]->get_id() == sensor_id) {
      sensors.erase(sensors.begin() + i);
      poses.erase(poses.begin() + i);
      impl_->entities.erase(sensor_id);
      return true;
    }
  }

  return false;
}

} // namespace core
} // namespace aerosimx
