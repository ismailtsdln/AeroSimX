/**
 * @file physics_engine.cpp
 * @brief Physics engine implementation
 */

#include "aerosimx/physics/physics_engine.hpp"

#include <chrono>
#include <cmath>
#include <unordered_map>

namespace aerosimx {
namespace physics {

struct BodyState {
  RigidBodyProperties properties;
  CollisionShape shape;
  Pose pose;
  Twist velocity;
  Vector3 force_accumulator;
  Vector3 torque_accumulator;
};

struct PhysicsEngine::Impl {
  PhysicsConfig config;
  std::unordered_map<BodyId, BodyState> bodies;
  std::atomic<BodyId> next_body_id{1};
  std::mutex bodies_mutex;

  CollisionCallback collision_callback;

  double last_step_time_ms = 0.0;
  bool initialized = false;
};

PhysicsEngine::PhysicsEngine() : impl_(std::make_unique<Impl>()) {}

PhysicsEngine::PhysicsEngine(const PhysicsConfig &config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
}

PhysicsEngine::~PhysicsEngine() = default;

PhysicsEngine::PhysicsEngine(PhysicsEngine &&) noexcept = default;
PhysicsEngine &PhysicsEngine::operator=(PhysicsEngine &&) noexcept = default;

bool PhysicsEngine::initialize() {
  impl_->initialized = true;
  return true;
}

void PhysicsEngine::reset() {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  impl_->bodies.clear();
  impl_->next_body_id = 1;
}

void PhysicsEngine::step(double dt) {
  auto start_time = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);

  // Integration step for each body
  for (auto &[id, body] : impl_->bodies) {
    if (body.properties.is_static || body.properties.is_kinematic) {
      continue;
    }

    // Apply gravity
    if (body.properties.enable_gravity) {
      body.force_accumulator += impl_->config.gravity * body.properties.mass;
    }

    // Calculate acceleration
    Vector3 linear_acceleration = body.force_accumulator / body.properties.mass;

    // Calculate angular acceleration (simplified - assumes diagonal inertia)
    Vector3 angular_acceleration = {
        body.torque_accumulator.x / body.properties.inertia.x,
        body.torque_accumulator.y / body.properties.inertia.y,
        body.torque_accumulator.z / body.properties.inertia.z};

    // Euler integration for velocity
    body.velocity.linear += linear_acceleration * dt;
    body.velocity.angular += angular_acceleration * dt;

    // Euler integration for position
    body.pose.position += body.velocity.linear * dt;

    // Quaternion integration for orientation
    double omega_mag =
        std::sqrt(body.velocity.angular.x * body.velocity.angular.x +
                  body.velocity.angular.y * body.velocity.angular.y +
                  body.velocity.angular.z * body.velocity.angular.z);

    if (omega_mag > 1e-10) {
      double half_angle = omega_mag * dt * 0.5;
      double s = std::sin(half_angle) / omega_mag;
      Quaternion dq = {std::cos(half_angle), body.velocity.angular.x * s,
                       body.velocity.angular.y * s,
                       body.velocity.angular.z * s};
      body.pose.orientation = body.pose.orientation * dq;
      body.pose.orientation = body.pose.orientation.normalized();
    }

    // Clear accumulators
    body.force_accumulator = Vector3::zero();
    body.torque_accumulator = Vector3::zero();
  }

  // TODO: Collision detection and response

  auto end_time = std::chrono::high_resolution_clock::now();
  impl_->last_step_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

void PhysicsEngine::step_batch(double dt, int num_steps) {
  for (int i = 0; i < num_steps; ++i) {
    step(dt);
  }
}

BodyId PhysicsEngine::create_body(const RigidBodyProperties &props,
                                  const Pose &initial_pose,
                                  const CollisionShape &shape) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);

  BodyId id = impl_->next_body_id++;

  BodyState state;
  state.properties = props;
  state.shape = shape;
  state.pose = initial_pose;
  state.velocity = {};
  state.force_accumulator = Vector3::zero();
  state.torque_accumulator = Vector3::zero();

  impl_->bodies[id] = state;

  return id;
}

void PhysicsEngine::destroy_body(BodyId id) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  impl_->bodies.erase(id);
}

bool PhysicsEngine::has_body(BodyId id) const {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  return impl_->bodies.find(id) != impl_->bodies.end();
}

Pose PhysicsEngine::get_pose(BodyId id) const {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it == impl_->bodies.end()) {
    return {};
  }
  return it->second.pose;
}

void PhysicsEngine::set_pose(BodyId id, const Pose &pose) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end()) {
    it->second.pose = pose;
  }
}

Twist PhysicsEngine::get_velocity(BodyId id) const {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it == impl_->bodies.end()) {
    return {};
  }
  return it->second.velocity;
}

void PhysicsEngine::set_velocity(BodyId id, const Twist &velocity) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end()) {
    it->second.velocity = velocity;
  }
}

void PhysicsEngine::apply_force(BodyId id, const Vector3 &force) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end()) {
    it->second.force_accumulator += force;
  }
}

void PhysicsEngine::apply_force_at_point(BodyId id, const Vector3 &force,
                                         const Vector3 &point) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end()) {
    it->second.force_accumulator += force;

    // Calculate torque from off-center force
    Vector3 r = point - it->second.pose.position;
    it->second.torque_accumulator += r.cross(force);
  }
}

void PhysicsEngine::apply_torque(BodyId id, const Vector3 &torque) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end()) {
    it->second.torque_accumulator += torque;
  }
}

void PhysicsEngine::apply_impulse(BodyId id, const Vector3 &impulse) {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  auto it = impl_->bodies.find(id);
  if (it != impl_->bodies.end() && !it->second.properties.is_static) {
    it->second.velocity.linear += impulse / it->second.properties.mass;
  }
}

void PhysicsEngine::set_collision_callback(CollisionCallback callback) {
  impl_->collision_callback = std::move(callback);
}

bool PhysicsEngine::raycast(const Vector3 &origin, const Vector3 &direction,
                            double max_distance, BodyId &hit_body,
                            Vector3 &hit_point, Vector3 &hit_normal) const {
  // Simple sphere/AABB raycast implementation
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);

  double closest_distance = max_distance;
  bool hit = false;

  Vector3 dir_normalized = direction.normalized();

  for (const auto &[id, body] : impl_->bodies) {
    if (body.shape.type == ShapeType::Sphere) {
      double radius = body.shape.dimensions.x;
      Vector3 oc = origin - body.pose.position;

      double a = dir_normalized.dot(dir_normalized);
      double b = 2.0 * oc.dot(dir_normalized);
      double c = oc.dot(oc) - radius * radius;
      double discriminant = b * b - 4 * a * c;

      if (discriminant > 0) {
        double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
        if (t > 0 && t < closest_distance) {
          closest_distance = t;
          hit_body = id;
          hit_point = origin + dir_normalized * t;
          hit_normal = (hit_point - body.pose.position).normalized();
          hit = true;
        }
      }
    }
    // TODO: Box and other shapes
  }

  return hit;
}

bool PhysicsEngine::sphere_cast(const Vector3 &origin, const Vector3 &direction,
                                double radius, double max_distance,
                                BodyId &hit_body, Vector3 &hit_point) const {
  // Simple implementation - cast along center and expand by radius
  Vector3 hit_normal;
  return raycast(origin, direction, max_distance + radius, hit_body, hit_point,
                 hit_normal);
}

const PhysicsConfig &PhysicsEngine::get_config() const { return impl_->config; }

void PhysicsEngine::set_config(const PhysicsConfig &config) {
  impl_->config = config;
}

void PhysicsEngine::set_gravity(const Vector3 &gravity) {
  impl_->config.gravity = gravity;
}

size_t PhysicsEngine::get_body_count() const {
  std::lock_guard<std::mutex> lock(impl_->bodies_mutex);
  return impl_->bodies.size();
}

double PhysicsEngine::get_step_time_ms() const {
  return impl_->last_step_time_ms;
}

} // namespace physics
} // namespace aerosimx
