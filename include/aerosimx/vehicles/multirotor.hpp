#pragma once

/**
 * @file multirotor.hpp
 * @brief Multirotor (drone) vehicle implementation
 */

#include "aerosimx/vehicles/vehicle_base.hpp"

#include <vector>

namespace aerosimx {
namespace vehicles {

/**
 * @brief Motor configuration
 */
struct MotorConfig {
  Vector3 position;              ///< Motor position relative to CoM
  double direction = 1.0;        ///< +1 = CW, -1 = CCW
  double max_thrust = 10.0;      ///< Maximum thrust in Newtons
  double max_rpm = 15000.0;      ///< Maximum RPM
  double time_constant = 0.05;   ///< Motor response time constant
  double moment_constant = 0.01; ///< Torque to thrust ratio
};

/**
 * @brief Multirotor configuration
 */
struct MultirotorConfig : public VehicleConfig {
  // Motor configuration
  std::vector<MotorConfig> motors;

  // Default quadcopter layout (if motors not specified)
  double arm_length = 0.25; ///< Arm length in meters
  int num_motors = 4;       ///< Number of motors

  // Aerodynamics
  double prop_diameter = 0.254;     ///< Propeller diameter in meters
  double thrust_coefficient = 1.0;  ///< Thrust coefficient
  double torque_coefficient = 0.05; ///< Torque coefficient
  double air_density = 1.225;       ///< Air density in kg/m^3

  // Limits
  double max_tilt_angle = 45.0;          ///< Maximum tilt angle in degrees
  double max_vertical_velocity = 10.0;   ///< Max vertical velocity in m/s
  double max_horizontal_velocity = 20.0; ///< Max horizontal velocity in m/s

  // Battery
  double battery_capacity = 5000.0; ///< Battery capacity in mAh
  double battery_voltage = 22.2;    ///< Battery voltage
  double hover_current = 15.0;      ///< Current draw at hover in A

  // Control mode
  bool angle_mode = true; ///< true=angle mode, false=rate mode
};

/**
 * @brief Multirotor flight modes
 */
enum class FlightMode {
  Idle,     ///< Motors off
  Armed,    ///< Motors armed but not flying
  Takeoff,  ///< Autonomous takeoff
  Hover,    ///< Position hold
  Manual,   ///< Manual control
  Velocity, ///< Velocity control
  Position, ///< Position control
  Landing,  ///< Autonomous landing
  Return,   ///< Return to home
  Emergency ///< Emergency stop
};

/**
 * @brief Multirotor vehicle implementation
 *
 * Simulates multirotors including quadcopters, hexacopters, and octocopters.
 * Includes motor dynamics, aerodynamics, and battery simulation.
 */
class Multirotor : public VehicleBase {
public:
  explicit Multirotor(const MultirotorConfig &config);
  ~Multirotor() override;

  // ========================================================================
  // VehicleBase interface
  // ========================================================================

  bool initialize() override;
  void reset() override;
  void update(double dt) override;
  std::string get_type_name() const override { return "Multirotor"; }

  // ========================================================================
  // Control Interface
  // ========================================================================

  /**
   * @brief Set control input (attitude/rate mode)
   */
  void set_controls(const MultirotorControls &controls);

  /**
   * @brief Move with velocity command
   */
  void move_by_velocity(double vx, double vy, double vz, double yaw_rate);

  /**
   * @brief Move to position
   */
  void move_to_position(double x, double y, double z, double velocity);

  /**
   * @brief Move to position with heading
   */
  void move_to_position(double x, double y, double z, double yaw,
                        double velocity);

  /**
   * @brief Command hover at current position
   */
  void hover();

  /**
   * @brief Arm motors
   */
  void arm();

  /**
   * @brief Disarm motors
   */
  void disarm();

  /**
   * @brief Takeoff to specified altitude
   */
  void takeoff(double altitude, double velocity = 1.0);

  /**
   * @brief Land at current position
   */
  void land(double velocity = 0.5);

  /**
   * @brief Return to home position
   */
  void return_to_home(double altitude = 10.0, double velocity = 5.0);

  /**
   * @brief Set individual motor speeds (normalized 0-1)
   */
  void set_motor_speeds(const std::vector<double> &speeds);

  // ========================================================================
  // State Access
  // ========================================================================

  /**
   * @brief Get multirotor-specific state
   */
  const MultirotorState &get_multirotor_state() const {
    return multirotor_state_;
  }

  /**
   * @brief Get multirotor configuration
   */
  const MultirotorConfig &get_multirotor_config() const {
    return multirotor_config_;
  }

  /**
   * @brief Get current motor speeds (RPM)
   */
  const std::vector<double> &get_motor_speeds() const { return motor_speeds_; }

  /**
   * @brief Get motor thrusts (Newtons)
   */
  const std::vector<double> &get_motor_thrusts() const {
    return motor_thrusts_;
  }

  /**
   * @brief Get current flight mode
   */
  FlightMode get_flight_mode() const { return flight_mode_; }

  /**
   * @brief Check if armed
   */
  bool is_armed() const { return multirotor_state_.is_armed; }

  /**
   * @brief Check if landed
   */
  bool is_landed() const { return multirotor_state_.is_landed; }

  /**
   * @brief Get battery percentage
   */
  double get_battery_percentage() const {
    return multirotor_state_.battery_percentage;
  }

  /**
   * @brief Get current altitude (z position)
   */
  double get_altitude() const { return state_.pose.position.z; }

  // ========================================================================
  // Position Control Interface
  // ========================================================================

  /**
   * @brief Set position setpoint
   */
  void set_position_setpoint(const Vector3 &position);

  /**
   * @brief Set position with yaw setpoint
   */
  void set_position_setpoint(const Vector3 &position, double yaw);

  /**
   * @brief Get current position setpoint
   */
  Vector3 get_position_setpoint() const { return position_setpoint_; }

  /**
   * @brief Check if at position setpoint
   */
  bool at_position_setpoint(double tolerance = 0.5) const;

protected:
  void apply_controls(double dt) override;
  void compute_aerodynamics(double dt) override;

private:
  /**
   * @brief Generate default motor layout
   */
  void generate_motor_layout();

  /**
   * @brief Update motor dynamics
   */
  void update_motors(double dt);

  /**
   * @brief Compute motor forces and torques
   */
  void compute_motor_forces();

  /**
   * @brief Update flight controller
   */
  void update_controller(double dt);

  /**
   * @brief Position controller (outer loop)
   */
  void position_controller(double dt);

  /**
   * @brief Velocity controller (middle loop)
   */
  void velocity_controller(double dt);

  /**
   * @brief Attitude controller (inner loop)
   */
  void attitude_controller(double dt);

  /**
   * @brief Rate controller (innermost loop)
   */
  void rate_controller(double dt);

  /**
   * @brief Mix control outputs to motor commands
   */
  void control_mixer();

  /**
   * @brief Update battery state
   */
  void update_battery(double dt);

  /**
   * @brief Detect landing
   */
  void detect_landing();

  MultirotorConfig multirotor_config_;
  MultirotorState multirotor_state_;
  MultirotorControls current_controls_;
  FlightMode flight_mode_ = FlightMode::Idle;

  // Motor state
  std::vector<double> motor_speeds_;         ///< Current motor speeds (RPM)
  std::vector<double> motor_speed_commands_; ///< Commanded motor speeds
  std::vector<double> motor_thrusts_;        ///< Current motor thrusts (N)
  std::vector<double> motor_torques_;        ///< Current motor torques (Nm)

  // Controller state
  Vector3 position_setpoint_;
  Vector3 velocity_setpoint_;
  Vector3 attitude_setpoint_; ///< Roll, pitch, yaw in radians
  Vector3 rate_setpoint_;
  double yaw_setpoint_ = 0.0;

  // Controller outputs
  Vector3 velocity_command_;
  Vector3 attitude_command_;
  Vector3 rate_command_;
  std::vector<double> motor_commands_;

  // Controller integral terms
  Vector3 position_integral_;
  Vector3 velocity_integral_;
  Vector3 attitude_integral_;
  Vector3 rate_integral_;

  // Controller gains (PID)
  Vector3 pos_p_gain_ = {1.0, 1.0, 1.0};
  Vector3 pos_i_gain_ = {0.0, 0.0, 0.0};
  Vector3 pos_d_gain_ = {0.0, 0.0, 0.0};
  Vector3 vel_p_gain_ = {2.0, 2.0, 2.0};
  Vector3 vel_i_gain_ = {0.1, 0.1, 0.1};
  Vector3 vel_d_gain_ = {0.0, 0.0, 0.0};
  Vector3 att_p_gain_ = {4.5, 4.5, 2.0};
  Vector3 rate_p_gain_ = {0.1, 0.1, 0.05};
  Vector3 rate_i_gain_ = {0.01, 0.01, 0.01};
  Vector3 rate_d_gain_ = {0.001, 0.001, 0.0};

  // Home position
  Vector3 home_position_;
  double home_yaw_ = 0.0;

  // Autonomous state
  double takeoff_altitude_ = 0.0;
  double landing_velocity_ = 0.5;
  double return_altitude_ = 10.0;
  double return_velocity_ = 5.0;

  // Battery state
  double battery_capacity_remaining_; ///< Current capacity in mAh
  double current_draw_ = 0.0;
};

} // namespace vehicles
} // namespace aerosimx
