#pragma once

/**
 * @file car.hpp
 * @brief Ground vehicle implementation
 */

#include "aerosimx/vehicles/vehicle_base.hpp"

namespace aerosimx {
namespace vehicles {

/**
 * @brief Wheel configuration
 */
struct WheelConfig {
  Vector3 position;                ///< Wheel position relative to CoM
  double radius = 0.35;            ///< Wheel radius in meters
  double width = 0.25;             ///< Wheel width in meters
  double mass = 20.0;              ///< Wheel mass in kg
  double max_steer_angle = 0.0;    ///< Max steering angle (0 for non-steering)
  double suspension_travel = 0.15; ///< Suspension travel in meters
  double suspension_stiffness = 30000.0; ///< Spring rate in N/m
  double damping_coefficient = 3000.0;   ///< Damper rate in Ns/m
};

/**
 * @brief Tire model parameters
 */
struct TireModel {
  double friction_coefficient = 1.0;
  double cornering_stiffness = 80000.0; ///< N/rad
  double longitudinal_stiffness = 100000.0;
  double peak_slip_angle = 0.15; ///< rad
  double peak_slip_ratio = 0.1;
  bool use_pacejka = false; ///< Use Pacejka tire model
  std::array<double, 10> pacejka_params = {1.3,   -40,   1600, 2600,  8.7,
                                           0.014, -0.24, 1.0,  -0.03, 0.0};
};

/**
 * @brief Car configuration
 */
struct CarConfig : public VehicleConfig {
  // Dimensions
  double wheelbase = 2.5;   ///< Distance between axles
  double track_width = 1.5; ///< Distance between wheels on same axle
  double height = 1.5;      ///< Vehicle height

  // Wheels (auto-generated if empty)
  std::vector<WheelConfig> wheels;

  // Tire model
  TireModel tire_model;

  // Drivetrain
  enum class DriveType { FWD, RWD, AWD };
  DriveType drive_type = DriveType::RWD;

  // Engine/Motor
  double max_engine_torque = 400.0;   ///< Maximum torque in Nm
  double max_engine_power = 150000.0; ///< Maximum power in Watts
  double max_rpm = 7000.0;
  double idle_rpm = 800.0;
  std::vector<double> gear_ratios = {3.5, 2.5, 1.8, 1.3, 1.0, 0.8};
  double final_drive_ratio = 3.5;

  // Brakes
  double max_brake_torque = 3000.0; ///< Maximum brake torque in Nm
  double brake_bias = 0.6;          ///< Front brake bias (0-1)

  // Limits
  double max_steer_angle = 35.0; ///< Maximum steering angle in degrees
  double max_speed = 60.0;       ///< Maximum speed in m/s

  // Electric vehicle
  bool is_electric = false;
  double battery_capacity = 75.0; ///< Battery capacity in kWh
  double motor_efficiency = 0.95;
};

/**
 * @brief Car driving modes
 */
enum class DrivingMode {
  Manual,   ///< Full manual control
  Cruise,   ///< Cruise control (speed control)
  ACC,      ///< Adaptive cruise control
  Lane,     ///< Lane keeping
  AutoPilot ///< Full autopilot
};

/**
 * @brief Ground vehicle implementation
 *
 * Simulates cars and trucks with detailed tire/suspension physics,
 * drivetrain simulation, and ADAS features.
 */
class Car : public VehicleBase {
public:
  explicit Car(const CarConfig &config);
  ~Car() override;

  // ========================================================================
  // VehicleBase interface
  // ========================================================================

  bool initialize() override;
  void reset() override;
  void update(double dt) override;
  std::string get_type_name() const override { return "Car"; }

  // ========================================================================
  // Control Interface
  // ========================================================================

  /**
   * @brief Set control inputs
   */
  void set_controls(const CarControls &controls);

  /**
   * @brief Set steering angle directly (-1 to 1)
   */
  void set_steering(double steering);

  /**
   * @brief Set throttle (0 to 1)
   */
  void set_throttle(double throttle);

  /**
   * @brief Set brake (0 to 1)
   */
  void set_brake(double brake);

  /**
   * @brief Set handbrake
   */
  void set_handbrake(bool engaged);

  /**
   * @brief Set gear (-1=reverse, 0=neutral, 1+=forward)
   */
  void set_gear(int gear);

  /**
   * @brief Shift gear up
   */
  void shift_up();

  /**
   * @brief Shift gear down
   */
  void shift_down();

  // ========================================================================
  // State Access
  // ========================================================================

  /**
   * @brief Get car-specific state
   */
  const CarState &get_car_state() const { return car_state_; }

  /**
   * @brief Get car configuration
   */
  const CarConfig &get_car_config() const { return car_config_; }

  /**
   * @brief Get current speed in m/s
   */
  double get_speed() const { return car_state_.speed; }

  /**
   * @brief Get current steering angle in radians
   */
  double get_steering_angle() const { return car_state_.steering_angle; }

  /**
   * @brief Get current gear
   */
  int get_gear() const { return car_state_.gear; }

  /**
   * @brief Get current engine RPM
   */
  double get_engine_rpm() const { return engine_rpm_; }

  /**
   * @brief Get wheel speeds (m/s)
   */
  const std::vector<double> &get_wheel_speeds() const { return wheel_speeds_; }

  /**
   * @brief Get wheel slip ratios
   */
  const std::vector<double> &get_wheel_slip_ratios() const {
    return wheel_slip_ratios_;
  }

  /**
   * @brief Get wheel slip angles
   */
  const std::vector<double> &get_wheel_slip_angles() const {
    return wheel_slip_angles_;
  }

  /**
   * @brief Get suspension compressions
   */
  const std::vector<double> &get_suspension_compressions() const {
    return suspension_compressions_;
  }

  // ========================================================================
  // ADAS Features
  // ========================================================================

  /**
   * @brief Set driving mode
   */
  void set_driving_mode(DrivingMode mode);

  /**
   * @brief Get current driving mode
   */
  DrivingMode get_driving_mode() const { return driving_mode_; }

  /**
   * @brief Set cruise control speed
   */
  void set_cruise_speed(double speed);

  /**
   * @brief Enable/disable ABS
   */
  void set_abs_enabled(bool enabled) { abs_enabled_ = enabled; }

  /**
   * @brief Enable/disable traction control
   */
  void set_tcs_enabled(bool enabled) { tcs_enabled_ = enabled; }

  /**
   * @brief Enable/disable stability control
   */
  void set_esc_enabled(bool enabled) { esc_enabled_ = enabled; }

protected:
  void apply_controls(double dt) override;
  void compute_aerodynamics(double dt) override;

private:
  /**
   * @brief Generate default wheel layout
   */
  void generate_wheel_layout();

  /**
   * @brief Update drivetrain simulation
   */
  void update_drivetrain(double dt);

  /**
   * @brief Update wheel dynamics
   */
  void update_wheels(double dt);

  /**
   * @brief Compute tire forces using tire model
   */
  Vector3 compute_tire_force(int wheel_idx, double normal_force);

  /**
   * @brief Update suspension
   */
  void update_suspension(double dt);

  /**
   * @brief Apply ABS intervention
   */
  void apply_abs(int wheel_idx, double &brake_torque);

  /**
   * @brief Apply traction control
   */
  void apply_tcs(int wheel_idx, double &drive_torque);

  /**
   * @brief Apply stability control
   */
  void apply_esc();

  /**
   * @brief Update ADAS systems
   */
  void update_adas(double dt);

  /**
   * @brief Compute engine torque at given RPM and throttle
   */
  double compute_engine_torque(double rpm, double throttle);

  CarConfig car_config_;
  CarState car_state_;
  CarControls current_controls_;
  DrivingMode driving_mode_ = DrivingMode::Manual;

  // Wheel state
  std::vector<double> wheel_speeds_; ///< Angular velocities
  std::vector<double> wheel_slip_ratios_;
  std::vector<double> wheel_slip_angles_;
  std::vector<double> wheel_normal_forces_;
  std::vector<Vector3> wheel_forces_;
  std::vector<double> suspension_compressions_;
  std::vector<double> suspension_velocities_;

  // Drivetrain state
  double engine_rpm_ = 0.0;
  double engine_torque_ = 0.0;
  double transmission_output_torque_ = 0.0;
  int current_gear_ = 0;
  bool clutch_engaged_ = true;

  // ADAS state
  bool abs_enabled_ = true;
  bool tcs_enabled_ = true;
  bool esc_enabled_ = true;
  double cruise_speed_ = 0.0;
  bool cruise_active_ = false;

  // Controller state for ADAS
  double speed_controller_integral_ = 0.0;
  double yaw_rate_setpoint_ = 0.0;
};

} // namespace vehicles
} // namespace aerosimx
