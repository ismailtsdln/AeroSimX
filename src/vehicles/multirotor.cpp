/**
 * @file multirotor.cpp
 * @brief Multirotor vehicle implementation
 */

#include "aerosimx/vehicles/multirotor.hpp"

#include <algorithm>
#include <cmath>

namespace aerosimx {
namespace vehicles {

Multirotor::Multirotor(const MultirotorConfig &config)
    : VehicleBase(config), multirotor_config_(config) {
  config_.type = VehicleType::Multirotor;
  config_.name = config.name.empty() ? "multirotor" : config.name;
}

Multirotor::~Multirotor() = default;

bool Multirotor::initialize() {
  if (!VehicleBase::initialize()) {
    return false;
  }

  // Generate motor layout if not provided
  if (multirotor_config_.motors.empty()) {
    generate_motor_layout();
  }

  // Initialize motor state
  size_t num_motors = multirotor_config_.motors.size();
  motor_speeds_.resize(num_motors, 0.0);
  motor_speed_commands_.resize(num_motors, 0.0);
  motor_thrusts_.resize(num_motors, 0.0);
  motor_torques_.resize(num_motors, 0.0);
  motor_commands_.resize(num_motors, 0.0);

  // Initialize state
  multirotor_state_.base_state = state_;
  multirotor_state_.motor_speeds.resize(num_motors, 0.0);
  multirotor_state_.is_armed = false;
  multirotor_state_.is_landed = true;
  multirotor_state_.battery_percentage = 100.0;

  battery_capacity_remaining_ = multirotor_config_.battery_capacity;

  // Store home position
  home_position_ = state_.pose.position;
  home_yaw_ = state_.pose.orientation.to_euler().z;

  return true;
}

void Multirotor::reset() {
  VehicleBase::reset();

  std::fill(motor_speeds_.begin(), motor_speeds_.end(), 0.0);
  std::fill(motor_speed_commands_.begin(), motor_speed_commands_.end(), 0.0);
  std::fill(motor_thrusts_.begin(), motor_thrusts_.end(), 0.0);
  std::fill(motor_commands_.begin(), motor_commands_.end(), 0.0);

  multirotor_state_.is_armed = false;
  multirotor_state_.is_landed = true;
  multirotor_state_.battery_percentage = 100.0;
  battery_capacity_remaining_ = multirotor_config_.battery_capacity;

  flight_mode_ = FlightMode::Idle;

  position_integral_ = Vector3::zero();
  velocity_integral_ = Vector3::zero();
  attitude_integral_ = Vector3::zero();
  rate_integral_ = Vector3::zero();
}

void Multirotor::update(double dt) {
  // Update controller based on flight mode
  update_controller(dt);

  // Apply control inputs
  apply_controls(dt);

  // Update motor dynamics
  update_motors(dt);

  // Compute motor forces
  compute_motor_forces();

  // Compute aerodynamic forces
  compute_aerodynamics(dt);

  // Apply accumulated forces to physics
  if (physics_engine_ && physics_body_ != physics::INVALID_BODY_ID) {
    physics_engine_->apply_force(physics_body_, accumulated_force_);
    physics_engine_->apply_torque(physics_body_, accumulated_torque_);
  } else {
    // Simple Euler integration if no physics engine
    Vector3 acceleration = accumulated_force_ / config_.mass;
    acceleration.z -= 9.81; // Gravity

    state_.velocity.linear += acceleration * dt;
    state_.pose.position += state_.velocity.linear * dt;

    // Simple attitude integration
    double omega_mag = state_.velocity.angular.magnitude();
    if (omega_mag > 1e-10) {
      Quaternion dq = Quaternion::from_axis_angle(
          state_.velocity.angular / omega_mag, omega_mag * dt);
      state_.pose.orientation = state_.pose.orientation * dq;
    }
  }

  // Clear force/torque accumulators
  accumulated_force_ = Vector3::zero();
  accumulated_torque_ = Vector3::zero();

  // Update battery
  update_battery(dt);

  // Detect landing
  detect_landing();

  // Update sensors
  update_sensors(dt);

  // Update state
  multirotor_state_.base_state = state_;
  multirotor_state_.motor_speeds = motor_speeds_;
  multirotor_state_.battery_percentage =
      battery_capacity_remaining_ / multirotor_config_.battery_capacity * 100.0;

  // Notify state callback
  if (state_callback_) {
    state_callback_(state_);
  }
}

void Multirotor::set_controls(const MultirotorControls &controls) {
  current_controls_ = controls;
  flight_mode_ = FlightMode::Manual;
}

void Multirotor::move_by_velocity(double vx, double vy, double vz,
                                  double yaw_rate) {
  if (!is_armed())
    return;

  velocity_setpoint_ = {vx, vy, vz};
  rate_setpoint_.z = yaw_rate;
  flight_mode_ = FlightMode::Velocity;
}

void Multirotor::move_to_position(double x, double y, double z,
                                  double velocity) {
  if (!is_armed())
    return;

  position_setpoint_ = {x, y, z};
  flight_mode_ = FlightMode::Position;
}

void Multirotor::move_to_position(double x, double y, double z, double yaw,
                                  double velocity) {
  move_to_position(x, y, z, velocity);
  yaw_setpoint_ = yaw;
}

void Multirotor::hover() {
  if (!is_armed())
    return;

  position_setpoint_ = state_.pose.position;
  yaw_setpoint_ = state_.pose.orientation.to_euler().z;
  flight_mode_ = FlightMode::Hover;
}

void Multirotor::arm() {
  if (multirotor_state_.is_landed) {
    multirotor_state_.is_armed = true;
    flight_mode_ = FlightMode::Armed;
  }
}

void Multirotor::disarm() {
  multirotor_state_.is_armed = false;
  flight_mode_ = FlightMode::Idle;
  std::fill(motor_commands_.begin(), motor_commands_.end(), 0.0);
}

void Multirotor::takeoff(double altitude, double velocity) {
  if (!is_armed()) {
    arm();
  }

  takeoff_altitude_ = altitude;
  position_setpoint_ = {state_.pose.position.x, state_.pose.position.y,
                        altitude};
  yaw_setpoint_ = state_.pose.orientation.to_euler().z;
  flight_mode_ = FlightMode::Takeoff;
  multirotor_state_.is_landed = false;
}

void Multirotor::land(double velocity) {
  landing_velocity_ = velocity;
  flight_mode_ = FlightMode::Landing;
}

void Multirotor::return_to_home(double altitude, double velocity) {
  return_altitude_ = altitude;
  return_velocity_ = velocity;
  flight_mode_ = FlightMode::Return;
}

void Multirotor::set_motor_speeds(const std::vector<double> &speeds) {
  for (size_t i = 0; i < std::min(speeds.size(), motor_commands_.size()); ++i) {
    motor_commands_[i] = std::clamp(speeds[i], 0.0, 1.0);
  }
}

void Multirotor::set_position_setpoint(const Vector3 &position) {
  position_setpoint_ = position;
}

void Multirotor::set_position_setpoint(const Vector3 &position, double yaw) {
  position_setpoint_ = position;
  yaw_setpoint_ = yaw;
}

bool Multirotor::at_position_setpoint(double tolerance) const {
  Vector3 error = position_setpoint_ - state_.pose.position;
  return error.magnitude() < tolerance;
}

void Multirotor::apply_controls(double dt) {
  if (!is_armed()) {
    std::fill(motor_commands_.begin(), motor_commands_.end(), 0.0);
    return;
  }

  // Mix controls to motors
  control_mixer();
}

void Multirotor::compute_aerodynamics(double dt) {
  VehicleBase::compute_aerodynamics(dt);

  // Add rotor drag
  double speed = state_.velocity.linear.magnitude();
  if (speed > 0.1) {
    Vector3 drag_dir = state_.velocity.linear.normalized();
    double rotor_drag = 0.5 * 1.225 * 0.1 * speed * speed;
    accumulated_force_ -= drag_dir * rotor_drag;
  }
}

void Multirotor::generate_motor_layout() {
  int n = multirotor_config_.num_motors;
  double arm = multirotor_config_.arm_length;

  multirotor_config_.motors.resize(n);

  for (int i = 0; i < n; ++i) {
    double angle = 2.0 * M_PI * i / n + M_PI / 4.0;

    MotorConfig &motor = multirotor_config_.motors[i];
    motor.position = {arm * std::cos(angle), arm * std::sin(angle), 0.0};
    motor.direction = (i % 2 == 0) ? 1.0 : -1.0;
    motor.max_thrust = config_.mass * 9.81 * 2.0 / n;
    motor.max_rpm = 15000.0;
    motor.time_constant = 0.05;
    motor.moment_constant = 0.01;
  }
}

void Multirotor::update_motors(double dt) {
  for (size_t i = 0; i < motor_speeds_.size(); ++i) {
    double target_rpm =
        motor_commands_[i] * multirotor_config_.motors[i].max_rpm;
    double tau = multirotor_config_.motors[i].time_constant;

    // First-order motor dynamics
    motor_speeds_[i] += (target_rpm - motor_speeds_[i]) * dt / tau;
    motor_speeds_[i] = std::max(0.0, motor_speeds_[i]);
  }
}

void Multirotor::compute_motor_forces() {
  Vector3 total_force = Vector3::zero();
  Vector3 total_torque = Vector3::zero();

  for (size_t i = 0; i < motor_speeds_.size(); ++i) {
    const auto &motor = multirotor_config_.motors[i];
    double rpm_normalized = motor_speeds_[i] / motor.max_rpm;

    // Thrust (proportional to RPM squared)
    double thrust = motor.max_thrust * rpm_normalized * rpm_normalized;
    motor_thrusts_[i] = thrust;

    // Thrust force (in body frame, pointing up)
    Vector3 thrust_vec = state_.pose.orientation.rotate({0, 0, thrust});
    total_force += thrust_vec;

    // Torque from motor position
    Vector3 motor_pos_world = state_.pose.orientation.rotate(motor.position);
    total_torque += motor_pos_world.cross({0, 0, thrust});

    // Reaction torque (yaw torque from motor rotation)
    double reaction_torque = motor.moment_constant * thrust * motor.direction;
    motor_torques_[i] = reaction_torque;
    total_torque.z += reaction_torque;
  }

  accumulated_force_ += total_force;
  accumulated_torque_ += total_torque;
}

void Multirotor::update_controller(double dt) {
  switch (flight_mode_) {
  case FlightMode::Hover:
  case FlightMode::Position:
  case FlightMode::Takeoff:
    position_controller(dt);
    velocity_controller(dt);
    attitude_controller(dt);
    rate_controller(dt);
    break;

  case FlightMode::Velocity:
    velocity_controller(dt);
    attitude_controller(dt);
    rate_controller(dt);
    break;

  case FlightMode::Manual:
    attitude_controller(dt);
    rate_controller(dt);
    break;

  case FlightMode::Landing:
    position_setpoint_.x = state_.pose.position.x;
    position_setpoint_.y = state_.pose.position.y;
    velocity_setpoint_.z = -landing_velocity_;
    velocity_controller(dt);
    attitude_controller(dt);
    rate_controller(dt);
    break;

  case FlightMode::Return:
    position_setpoint_ = home_position_;
    position_setpoint_.z = return_altitude_;
    yaw_setpoint_ = home_yaw_;
    position_controller(dt);
    velocity_controller(dt);
    attitude_controller(dt);
    rate_controller(dt);
    break;

  default:
    break;
  }
}

void Multirotor::position_controller(double dt) {
  Vector3 error = position_setpoint_ - state_.pose.position;
  position_integral_ += error * dt;

  velocity_setpoint_ =
      error * pos_p_gain_.x + position_integral_ * pos_i_gain_.x;

  // Clamp velocity setpoint
  double max_vel = multirotor_config_.max_horizontal_velocity;
  velocity_setpoint_.x = std::clamp(velocity_setpoint_.x, -max_vel, max_vel);
  velocity_setpoint_.y = std::clamp(velocity_setpoint_.y, -max_vel, max_vel);
  velocity_setpoint_.z = std::clamp(velocity_setpoint_.z,
                                    -multirotor_config_.max_vertical_velocity,
                                    multirotor_config_.max_vertical_velocity);
}

void Multirotor::velocity_controller(double dt) {
  Vector3 error = velocity_setpoint_ - state_.velocity.linear;
  velocity_integral_ += error * dt;

  // Compute desired acceleration
  Vector3 accel_cmd =
      error * vel_p_gain_.x + velocity_integral_ * vel_i_gain_.x;

  // Convert to attitude setpoint
  double max_tilt = multirotor_config_.max_tilt_angle * M_PI / 180.0;

  attitude_setpoint_.x =
      std::clamp(-accel_cmd.y / 9.81, -max_tilt, max_tilt); // Roll
  attitude_setpoint_.y =
      std::clamp(accel_cmd.x / 9.81, -max_tilt, max_tilt); // Pitch
  attitude_setpoint_.z = yaw_setpoint_;

  // Throttle from z velocity error
  current_controls_.throttle = 0.5 + accel_cmd.z * 0.1;
  current_controls_.throttle = std::clamp(current_controls_.throttle, 0.0, 1.0);
}

void Multirotor::attitude_controller(double dt) {
  Vector3 euler = state_.pose.orientation.to_euler();
  Vector3 error = attitude_setpoint_ - euler;

  // Wrap yaw error
  while (error.z > M_PI)
    error.z -= 2 * M_PI;
  while (error.z < -M_PI)
    error.z += 2 * M_PI;

  attitude_integral_ += error * dt;

  rate_setpoint_ = error * att_p_gain_.x + attitude_integral_ * 0.0;
}

void Multirotor::rate_controller(double dt) {
  Vector3 error = rate_setpoint_ - state_.velocity.angular;
  rate_integral_ += error * dt;

  // Compute control outputs
  current_controls_.roll =
      error.x * rate_p_gain_.x + rate_integral_.x * rate_i_gain_.x;
  current_controls_.pitch =
      error.y * rate_p_gain_.y + rate_integral_.y * rate_i_gain_.y;
  current_controls_.yaw_rate =
      error.z * rate_p_gain_.z + rate_integral_.z * rate_i_gain_.z;
}

void Multirotor::control_mixer() {
  // Quadrotor X configuration mixer
  // Motor layout:
  //   1   2
  //    \ /
  //    / \
    //   3   4

  if (motor_commands_.size() < 4)
    return;

  double throttle = current_controls_.throttle;
  double roll = current_controls_.roll;
  double pitch = current_controls_.pitch;
  double yaw = current_controls_.yaw_rate;

  motor_commands_[0] = throttle - roll + pitch + yaw; // Front-right
  motor_commands_[1] = throttle + roll + pitch - yaw; // Front-left
  motor_commands_[2] = throttle + roll - pitch + yaw; // Rear-left
  motor_commands_[3] = throttle - roll - pitch - yaw; // Rear-right

  // Clamp and normalize
  for (auto &cmd : motor_commands_) {
    cmd = std::clamp(cmd, 0.0, 1.0);
  }
}

void Multirotor::update_battery(double dt) {
  // Calculate current draw based on motor speeds
  double total_current = 0.0;
  for (size_t i = 0; i < motor_speeds_.size(); ++i) {
    double rpm_ratio = motor_speeds_[i] / multirotor_config_.motors[i].max_rpm;
    total_current += multirotor_config_.hover_current * rpm_ratio * rpm_ratio;
  }
  current_draw_ = total_current;

  // Discharge battery (mAh used = current * time_hours * 1000)
  double mah_used = total_current * (dt / 3600.0) * 1000.0;
  battery_capacity_remaining_ -= mah_used;
  battery_capacity_remaining_ = std::max(0.0, battery_capacity_remaining_);
}

void Multirotor::detect_landing() {
  // Simple landing detection based on altitude and velocity
  if (state_.pose.position.z < 0.1 &&
      std::abs(state_.velocity.linear.z) < 0.1 &&
      flight_mode_ == FlightMode::Landing) {
    multirotor_state_.is_landed = true;
    flight_mode_ = FlightMode::Armed;
  }
}

} // namespace vehicles
} // namespace aerosimx
