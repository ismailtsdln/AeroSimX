#pragma once

/**
 * @file control_api.hpp
 * @brief External control API for simulation interaction
 */

#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aerosimx/core/simulation.hpp"
#include "aerosimx/core/types.hpp"
#include "aerosimx/sensors/sensor_base.hpp"
#include "aerosimx/vehicles/vehicle_base.hpp"

namespace aerosimx {
namespace control {

/**
 * @brief API connection configuration
 */
struct ApiConfig {
  std::string host = "localhost";
  int port = 41451;
  bool enable_server = true;
  int max_clients = 10;
  double timeout_seconds = 10.0;
  bool async_mode = false;
};

/**
 * @brief Command result structure
 */
struct CommandResult {
  bool success = false;
  std::string message;
  std::optional<std::any> data;
};

/**
 * @brief Async command handle
 */
using AsyncCommandHandle = std::future<CommandResult>;

/**
 * @brief Control API for external access to simulation
 *
 * Provides a unified interface for controlling the simulation,
 * spawning vehicles, collecting sensor data, and more. Can be
 * accessed remotely via network protocol.
 */
class ControlApi {
public:
  explicit ControlApi(core::Simulation &simulation,
                      const ApiConfig &config = ApiConfig{});
  ~ControlApi();

  // Non-copyable
  ControlApi(const ControlApi &) = delete;
  ControlApi &operator=(const ControlApi &) = delete;

  // ========================================================================
  // Lifecycle
  // ========================================================================

  /**
   * @brief Initialize the API
   */
  bool initialize();

  /**
   * @brief Start API server (for remote access)
   */
  bool start_server();

  /**
   * @brief Stop API server
   */
  void stop_server();

  /**
   * @brief Check if server is running
   */
  bool is_server_running() const;

  // ========================================================================
  // Simulation Control
  // ========================================================================

  /**
   * @brief Reset simulation
   */
  CommandResult reset();

  /**
   * @brief Start simulation
   */
  CommandResult start();

  /**
   * @brief Pause simulation
   */
  CommandResult pause();

  /**
   * @brief Resume simulation
   */
  CommandResult resume();

  /**
   * @brief Step simulation by N steps
   */
  CommandResult step(int count = 1);

  /**
   * @brief Set simulation speed
   */
  CommandResult set_speed(double speed_multiplier);

  /**
   * @brief Get simulation time
   */
  double get_simulation_time() const;

  /**
   * @brief Check if simulation is running
   */
  bool is_simulation_running() const;

  // ========================================================================
  // Vehicle Management
  // ========================================================================

  /**
   * @brief Spawn a multirotor vehicle
   */
  EntityId spawn_multirotor(const std::string &name, const Pose &pose,
                            const vehicles::MultirotorConfig &config = {});

  /**
   * @brief Spawn a car vehicle
   */
  EntityId spawn_car(const std::string &name, const Pose &pose,
                     const vehicles::CarConfig &config = {});

  /**
   * @brief Remove a vehicle
   */
  bool remove_vehicle(EntityId id);

  /**
   * @brief Get vehicle by name
   */
  std::shared_ptr<vehicles::VehicleBase>
  get_vehicle(const std::string &name) const;

  /**
   * @brief Get vehicle by ID
   */
  std::shared_ptr<vehicles::VehicleBase> get_vehicle(EntityId id) const;

  /**
   * @brief Get all vehicles
   */
  std::vector<std::shared_ptr<vehicles::VehicleBase>> get_all_vehicles() const;

  /**
   * @brief Get vehicle state
   */
  std::optional<VehicleState> get_vehicle_state(EntityId id) const;

  /**
   * @brief Teleport vehicle to position
   */
  CommandResult teleport_vehicle(EntityId id, const Pose &pose);

  // ========================================================================
  // Multirotor Control
  // ========================================================================

  /**
   * @brief Arm multirotor
   */
  CommandResult arm_multirotor(EntityId id);

  /**
   * @brief Disarm multirotor
   */
  CommandResult disarm_multirotor(EntityId id);

  /**
   * @brief Takeoff multirotor
   */
  CommandResult takeoff(EntityId id, double altitude, double velocity = 1.0);

  /**
   * @brief Land multirotor
   */
  CommandResult land(EntityId id, double velocity = 0.5);

  /**
   * @brief Move multirotor by velocity
   */
  CommandResult move_by_velocity(EntityId id, double vx, double vy, double vz,
                                 double yaw_rate = 0.0);

  /**
   * @brief Move multirotor to position
   */
  CommandResult move_to_position(EntityId id, double x, double y, double z,
                                 double velocity = 5.0);

  /**
   * @brief Hover multirotor at current position
   */
  CommandResult hover(EntityId id);

  /**
   * @brief Set multirotor controls directly
   */
  CommandResult set_multirotor_controls(EntityId id,
                                        const MultirotorControls &controls);

  // ========================================================================
  // Car Control
  // ========================================================================

  /**
   * @brief Set car controls
   */
  CommandResult set_car_controls(EntityId id, const CarControls &controls);

  /**
   * @brief Set car throttle
   */
  CommandResult set_car_throttle(EntityId id, double throttle);

  /**
   * @brief Set car steering
   */
  CommandResult set_car_steering(EntityId id, double steering);

  /**
   * @brief Set car brake
   */
  CommandResult set_car_brake(EntityId id, double brake);

  // ========================================================================
  // Sensor Management
  // ========================================================================

  /**
   * @brief Attach a lidar sensor to a vehicle
   */
  EntityId attach_lidar(EntityId vehicle_id, const std::string &name,
                        const Pose &relative_pose,
                        const sensors::LidarConfig &config = {});

  /**
   * @brief Attach a camera to a vehicle
   */
  EntityId attach_camera(EntityId vehicle_id, const std::string &name,
                         const Pose &relative_pose,
                         const sensors::CameraConfig &config = {});

  /**
   * @brief Attach an IMU to a vehicle
   */
  EntityId attach_imu(EntityId vehicle_id, const std::string &name,
                      const Pose &relative_pose,
                      const sensors::ImuConfig &config = {});

  /**
   * @brief Attach a GPS to a vehicle
   */
  EntityId attach_gps(EntityId vehicle_id, const std::string &name,
                      const Pose &relative_pose,
                      const sensors::GpsConfig &config = {});

  /**
   * @brief Get latest point cloud from lidar
   */
  std::optional<PointCloud> get_lidar_data(EntityId sensor_id) const;

  /**
   * @brief Get latest image from camera
   */
  std::optional<ImageData> get_camera_image(EntityId sensor_id) const;

  /**
   * @brief Get latest depth image
   */
  std::optional<DepthImage> get_depth_image(EntityId sensor_id) const;

  /**
   * @brief Get latest IMU data
   */
  std::optional<ImuData> get_imu_data(EntityId sensor_id) const;

  /**
   * @brief Get latest GPS data
   */
  std::optional<GpsData> get_gps_data(EntityId sensor_id) const;

  // ========================================================================
  // Environment Control
  // ========================================================================

  /**
   * @brief Set weather conditions
   */
  CommandResult set_weather(double rain, double fog, double snow = 0.0);

  /**
   * @brief Set time of day
   */
  CommandResult set_time_of_day(int hour, int minute);

  /**
   * @brief Set sun position
   */
  CommandResult set_sun_position(double azimuth, double elevation);

  /**
   * @brief Set wind
   */
  CommandResult set_wind(const Vector3 &velocity);

  // ========================================================================
  // Data Recording
  // ========================================================================

  /**
   * @brief Start recording sensor data
   */
  CommandResult start_recording(const std::string &output_path);

  /**
   * @brief Stop recording
   */
  CommandResult stop_recording();

  /**
   * @brief Check if recording
   */
  bool is_recording() const;

  // ========================================================================
  // Async Operations
  // ========================================================================

  /**
   * @brief Execute command asynchronously
   */
  template <typename Func> AsyncCommandHandle async_execute(Func &&func);

  /**
   * @brief Wait for async command to complete
   */
  CommandResult wait_for(AsyncCommandHandle &handle,
                         double timeout_seconds = -1.0);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  core::Simulation &simulation_;
  ApiConfig config_;
};

} // namespace control
} // namespace aerosimx
