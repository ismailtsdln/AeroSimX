/**
 * @file vehicle_base.cpp
 * @brief Base vehicle implementation
 */

#include "aerosimx/vehicles/vehicle_base.hpp"
#include "aerosimx/sensors/sensor_base.hpp"

namespace aerosimx {
namespace vehicles {

VehicleBase::VehicleBase(const VehicleConfig &config) : config_(config) {
  state_.pose = config.initial_pose;
}

VehicleBase::~VehicleBase() = default;

bool VehicleBase::initialize() {
  // Create physics body if engine is available
  if (physics_engine_) {
    physics::RigidBodyProperties props;
    props.mass = config_.mass;
    props.inertia = config_.inertia;
    props.is_static = false;

    physics_body_ = physics_engine_->create_body(props, config_.initial_pose,
                                                 config_.collision_shape);
  }

  // Initialize sensors
  for (auto &sensor : sensors_) {
    if (sensor) {
      sensor->initialize();
    }
  }

  return true;
}

void VehicleBase::reset() {
  state_.pose = config_.initial_pose;
  state_.velocity = {};
  state_.acceleration = Vector3::zero();
  state_.collision_detected = false;

  accumulated_force_ = Vector3::zero();
  accumulated_torque_ = Vector3::zero();

  if (physics_engine_ && physics_body_ != physics::INVALID_BODY_ID) {
    physics_engine_->set_pose(physics_body_, config_.initial_pose);
    physics_engine_->set_velocity(physics_body_, {});
  }

  for (auto &sensor : sensors_) {
    if (sensor) {
      sensor->reset();
    }
  }
}

void VehicleBase::on_physics_update() { sync_from_physics(); }

void VehicleBase::set_pose(const Pose &pose) {
  state_.pose = pose;
  sync_to_physics();
}

void VehicleBase::set_velocity(const Twist &velocity) {
  state_.velocity = velocity;
  if (physics_engine_ && physics_body_ != physics::INVALID_BODY_ID) {
    physics_engine_->set_velocity(physics_body_, velocity);
  }
}

void VehicleBase::apply_force(const Vector3 &force) {
  accumulated_force_ += force;
}

void VehicleBase::apply_force_at_point(const Vector3 &force,
                                       const Vector3 &point) {
  accumulated_force_ += force;
  Vector3 r = point - state_.pose.position;
  accumulated_torque_ += r.cross(force);
}

void VehicleBase::apply_torque(const Vector3 &torque) {
  accumulated_torque_ += torque;
}

bool VehicleBase::attach_sensor(std::shared_ptr<sensors::SensorBase> sensor,
                                const Pose &relative_pose) {
  if (!sensor) {
    return false;
  }

  sensors_.push_back(sensor);
  sensor_relative_poses_.push_back(relative_pose);
  sensor_name_map_[sensor->get_name()] = sensors_.size() - 1;

  return true;
}

bool VehicleBase::detach_sensor(EntityId sensor_id) {
  for (size_t i = 0; i < sensors_.size(); ++i) {
    if (sensors_[i]->get_id() == sensor_id) {
      sensor_name_map_.erase(sensors_[i]->get_name());
      sensors_.erase(sensors_.begin() + i);
      sensor_relative_poses_.erase(sensor_relative_poses_.begin() + i);
      return true;
    }
  }
  return false;
}

const std::vector<std::shared_ptr<sensors::SensorBase>> &
VehicleBase::get_sensors() const {
  return sensors_;
}

std::shared_ptr<sensors::SensorBase>
VehicleBase::get_sensor(const std::string &name) const {
  auto it = sensor_name_map_.find(name);
  if (it != sensor_name_map_.end() && it->second < sensors_.size()) {
    return sensors_[it->second];
  }
  return nullptr;
}

void VehicleBase::update_sensors(double dt) {
  for (size_t i = 0; i < sensors_.size(); ++i) {
    if (sensors_[i] && sensors_[i]->is_enabled()) {
      // Compute sensor world pose
      Pose sensor_world_pose;
      sensor_world_pose.position =
          state_.pose.position +
          state_.pose.orientation.rotate(sensor_relative_poses_[i].position);
      sensor_world_pose.orientation =
          state_.pose.orientation * sensor_relative_poses_[i].orientation;

      sensors_[i]->update(dt, state_.pose);
    }
  }
}

void VehicleBase::set_physics_engine(physics::PhysicsEngine *engine) {
  physics_engine_ = engine;
}

void VehicleBase::set_collision_callback(
    std::function<void(const Vector3 &, EntityId)> callback) {
  collision_callback_ = std::move(callback);
}

void VehicleBase::set_state_callback(
    std::function<void(const VehicleState &)> callback) {
  state_callback_ = std::move(callback);
}

void VehicleBase::sync_from_physics() {
  if (physics_engine_ && physics_body_ != physics::INVALID_BODY_ID) {
    state_.pose = physics_engine_->get_pose(physics_body_);
    state_.velocity = physics_engine_->get_velocity(physics_body_);
  }
}

void VehicleBase::sync_to_physics() {
  if (physics_engine_ && physics_body_ != physics::INVALID_BODY_ID) {
    physics_engine_->set_pose(physics_body_, state_.pose);
  }
}

void VehicleBase::compute_aerodynamics(double dt) {
  // Default drag model
  double speed_sq = state_.velocity.linear.magnitude_squared();
  if (speed_sq > 0.001) {
    Vector3 velocity_dir = state_.velocity.linear.normalized();
    double drag_magnitude = 0.5 * 1.225 * config_.drag_coefficient * speed_sq;
    Vector3 drag_force = velocity_dir * (-drag_magnitude);
    accumulated_force_ += drag_force;
  }
}

void VehicleBase::on_collision(const Vector3 &contact_point,
                               EntityId other_id) {
  state_.collision_detected = true;
  if (collision_callback_) {
    collision_callback_(contact_point, other_id);
  }
}

} // namespace vehicles
} // namespace aerosimx
