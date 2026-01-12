#pragma once

/**
 * @file scenario_manager.hpp
 * @brief Scenario management for automated testing and simulation
 */

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "aerosimx/core/types.hpp"

namespace aerosimx {
namespace scenario {

/**
 * @brief Scenario event types
 */
enum class EventType {
  SpawnVehicle,
  RemoveVehicle,
  SetPose,
  SetVelocity,
  SetWeather,
  SetTimeOfDay,
  TriggerSensor,
  WaitTime,
  WaitCondition,
  Custom
};

/**
 * @brief Weather conditions
 */
struct WeatherConditions {
  double rain = 0.0;        ///< Rain intensity (0-1)
  double fog = 0.0;         ///< Fog density (0-1)
  double snow = 0.0;        ///< Snow intensity (0-1)
  double cloudiness = 0.0;  ///< Cloud coverage (0-1)
  Vector3 wind = {0, 0, 0}; ///< Wind velocity (m/s)
  double dust = 0.0;        ///< Dust/sand (0-1)
};

/**
 * @brief Time of day settings
 */
struct TimeOfDay {
  int hour = 12;
  int minute = 0;
  double sun_azimuth = 0.0;    ///< Sun azimuth angle (degrees)
  double sun_elevation = 60.0; ///< Sun elevation angle (degrees)
  bool auto_advance = false;   ///< Automatically advance time
  double time_scale = 1.0;     ///< Time advancement speed
};

/**
 * @brief GPS degradation scenario
 */
struct GpsDegradationScenario {
  bool enabled = false;
  double start_time = 0.0;          ///< When degradation starts
  double duration = 10.0;           ///< Duration of degradation
  double accuracy_multiplier = 5.0; ///< Accuracy reduction factor
  double dropout_probability = 0.3; ///< Probability of signal loss
  bool jamming = false;
  bool spoofing = false;
  Vector3 spoofing_offset = {0, 0, 0};
};

/**
 * @brief IMU degradation scenario
 */
struct ImuDegradationScenario {
  bool enabled = false;
  double start_time = 0.0;
  double duration = 10.0;
  double bias_drift_rate = 0.01;
  double noise_multiplier = 5.0;
  bool complete_failure = false;
};

/**
 * @brief Scenario event data
 */
struct ScenarioEvent {
  EventType type;
  double trigger_time = 0.0; ///< Time to trigger (seconds)
  std::string target_entity; ///< Target entity name
  std::map<std::string, std::variant<double, std::string, Vector3, bool>>
      params;
  std::function<bool()> condition; ///< Optional condition
  bool triggered = false;
};

/**
 * @brief Scenario definition
 */
struct ScenarioDefinition {
  std::string name;
  std::string description;
  double duration = -1.0; ///< Max duration (-1 = infinite)
  bool loop = false;      ///< Loop scenario

  // Environment settings
  WeatherConditions initial_weather;
  TimeOfDay initial_time;

  // Degradation scenarios
  std::vector<GpsDegradationScenario> gps_degradations;
  std::vector<ImuDegradationScenario> imu_degradations;

  // Events
  std::vector<ScenarioEvent> events;

  // Success/failure conditions
  std::function<bool()> success_condition;
  std::function<bool()> failure_condition;
};

/**
 * @brief Scenario execution state
 */
enum class ScenarioState { Idle, Running, Paused, Completed, Failed, Timeout };

/**
 * @brief Scenario execution result
 */
struct ScenarioResult {
  ScenarioState final_state;
  double execution_time;
  int events_triggered;
  std::string message;
  std::map<std::string, double> metrics;
};

/**
 * @brief Callback types
 */
using ScenarioCallback =
    std::function<void(const std::string &, ScenarioState)>;
using EventCallback = std::function<void(const ScenarioEvent &)>;

/**
 * @brief Scenario Manager
 *
 * Manages scenario loading, execution, and automated testing.
 * Supports event sequencing, condition-based triggers, and
 * comprehensive environment control.
 */
class ScenarioManager {
public:
  ScenarioManager();
  ~ScenarioManager();

  // Non-copyable
  ScenarioManager(const ScenarioManager &) = delete;
  ScenarioManager &operator=(const ScenarioManager &) = delete;

  // ========================================================================
  // Scenario Loading
  // ========================================================================

  /**
   * @brief Load scenario from file
   * @param path Path to scenario file (JSON/YAML)
   * @return Scenario ID, or empty string on failure
   */
  std::string load_scenario(const std::string &path);

  /**
   * @brief Load scenario from definition
   * @return Scenario ID
   */
  std::string load_scenario(const ScenarioDefinition &definition);

  /**
   * @brief Unload a scenario
   */
  bool unload_scenario(const std::string &scenario_id);

  /**
   * @brief Get loaded scenario
   */
  const ScenarioDefinition *get_scenario(const std::string &scenario_id) const;

  /**
   * @brief List all loaded scenarios
   */
  std::vector<std::string> list_scenarios() const;

  // ========================================================================
  // Scenario Execution
  // ========================================================================

  /**
   * @brief Start executing a scenario
   */
  bool start(const std::string &scenario_id);

  /**
   * @brief Pause scenario execution
   */
  void pause();

  /**
   * @brief Resume paused scenario
   */
  void resume();

  /**
   * @brief Stop scenario execution
   */
  void stop();

  /**
   * @brief Reset and restart current scenario
   */
  void restart();

  /**
   * @brief Update scenario (called each simulation step)
   * @param dt Time step
   * @param sim_time Current simulation time
   */
  void update(double dt, double sim_time);

  /**
   * @brief Get current scenario state
   */
  ScenarioState get_state() const;

  /**
   * @brief Get current scenario ID
   */
  std::string get_current_scenario() const;

  /**
   * @brief Get execution result (after completion)
   */
  ScenarioResult get_result() const;

  // ========================================================================
  // Environment Control
  // ========================================================================

  /**
   * @brief Set weather conditions
   */
  void set_weather(const WeatherConditions &weather);

  /**
   * @brief Get current weather
   */
  WeatherConditions get_weather() const;

  /**
   * @brief Set time of day
   */
  void set_time_of_day(const TimeOfDay &time);

  /**
   * @brief Get current time of day
   */
  TimeOfDay get_time_of_day() const;

  /**
   * @brief Apply GPS degradation
   */
  void apply_gps_degradation(const GpsDegradationScenario &scenario);

  /**
   * @brief Apply IMU degradation
   */
  void apply_imu_degradation(const ImuDegradationScenario &scenario);

  /**
   * @brief Clear all degradations
   */
  void clear_degradations();

  // ========================================================================
  // Event Management
  // ========================================================================

  /**
   * @brief Add event to current scenario
   */
  void add_event(const ScenarioEvent &event);

  /**
   * @brief Trigger event manually
   */
  void trigger_event(const std::string &event_name);

  /**
   * @brief Cancel pending event
   */
  bool cancel_event(const std::string &event_name);

  // ========================================================================
  // Callbacks
  // ========================================================================

  /**
   * @brief Register scenario state change callback
   */
  void set_state_callback(ScenarioCallback callback);

  /**
   * @brief Register event trigger callback
   */
  void set_event_callback(EventCallback callback);

  // ========================================================================
  // Metrics & Logging
  // ========================================================================

  /**
   * @brief Record a metric value
   */
  void record_metric(const std::string &name, double value);

  /**
   * @brief Get recorded metrics
   */
  std::map<std::string, std::vector<double>> get_metrics() const;

  /**
   * @brief Clear metrics
   */
  void clear_metrics();

  /**
   * @brief Export scenario log
   */
  bool export_log(const std::string &path);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace scenario
} // namespace aerosimx
