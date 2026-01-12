/**
 * @file scenario_manager.cpp
 * @brief Scenario manager implementation
 */

#include "aerosimx/scenario/scenario_manager.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>

namespace aerosimx {
namespace scenario {

struct ScenarioManager::Impl {
  // Loaded scenarios
  std::map<std::string, ScenarioDefinition> scenarios;
  int next_scenario_id = 1;

  // Current execution state
  std::string current_scenario_id;
  ScenarioState state = ScenarioState::Idle;
  double scenario_start_time = 0.0;
  double scenario_elapsed_time = 0.0;
  int events_triggered = 0;

  // Environment state
  WeatherConditions current_weather;
  TimeOfDay current_time;
  std::vector<GpsDegradationScenario> active_gps_degradations;
  std::vector<ImuDegradationScenario> active_imu_degradations;

  // Callbacks
  ScenarioCallback state_callback;
  EventCallback event_callback;

  // Metrics
  std::map<std::string, std::vector<double>> metrics;

  // Result
  ScenarioResult result;
};

ScenarioManager::ScenarioManager() : impl_(std::make_unique<Impl>()) {}

ScenarioManager::~ScenarioManager() = default;

std::string ScenarioManager::load_scenario(const std::string &path) {
  // Load scenario from file
  // In production, would parse JSON/YAML

  std::ifstream file(path);
  if (!file.is_open()) {
    return "";
  }

  ScenarioDefinition def;
  def.name = "Loaded Scenario";

  // Parse file contents...
  // Placeholder implementation

  return load_scenario(def);
}

std::string
ScenarioManager::load_scenario(const ScenarioDefinition &definition) {
  std::string id = "scenario_" + std::to_string(impl_->next_scenario_id++);
  impl_->scenarios[id] = definition;
  return id;
}

bool ScenarioManager::unload_scenario(const std::string &scenario_id) {
  if (impl_->current_scenario_id == scenario_id) {
    stop();
  }
  return impl_->scenarios.erase(scenario_id) > 0;
}

const ScenarioDefinition *
ScenarioManager::get_scenario(const std::string &scenario_id) const {
  auto it = impl_->scenarios.find(scenario_id);
  if (it != impl_->scenarios.end()) {
    return &it->second;
  }
  return nullptr;
}

std::vector<std::string> ScenarioManager::list_scenarios() const {
  std::vector<std::string> result;
  for (const auto &[id, _] : impl_->scenarios) {
    result.push_back(id);
  }
  return result;
}

bool ScenarioManager::start(const std::string &scenario_id) {
  auto it = impl_->scenarios.find(scenario_id);
  if (it == impl_->scenarios.end()) {
    return false;
  }

  impl_->current_scenario_id = scenario_id;
  impl_->state = ScenarioState::Running;
  impl_->scenario_elapsed_time = 0.0;
  impl_->events_triggered = 0;

  // Apply initial conditions
  set_weather(it->second.initial_weather);
  set_time_of_day(it->second.initial_time);

  // Apply degradation scenarios
  for (const auto &gps_deg : it->second.gps_degradations) {
    apply_gps_degradation(gps_deg);
  }
  for (const auto &imu_deg : it->second.imu_degradations) {
    apply_imu_degradation(imu_deg);
  }

  // Reset event states
  for (auto &event : impl_->scenarios[scenario_id].events) {
    event.triggered = false;
  }

  if (impl_->state_callback) {
    impl_->state_callback(scenario_id, impl_->state);
  }

  return true;
}

void ScenarioManager::pause() {
  if (impl_->state == ScenarioState::Running) {
    impl_->state = ScenarioState::Paused;
    if (impl_->state_callback) {
      impl_->state_callback(impl_->current_scenario_id, impl_->state);
    }
  }
}

void ScenarioManager::resume() {
  if (impl_->state == ScenarioState::Paused) {
    impl_->state = ScenarioState::Running;
    if (impl_->state_callback) {
      impl_->state_callback(impl_->current_scenario_id, impl_->state);
    }
  }
}

void ScenarioManager::stop() {
  impl_->state = ScenarioState::Idle;
  impl_->current_scenario_id.clear();
  clear_degradations();

  if (impl_->state_callback) {
    impl_->state_callback("", impl_->state);
  }
}

void ScenarioManager::restart() {
  if (!impl_->current_scenario_id.empty()) {
    std::string scenario_id = impl_->current_scenario_id;
    stop();
    start(scenario_id);
  }
}

void ScenarioManager::update(double dt, double sim_time) {
  if (impl_->state != ScenarioState::Running) {
    return;
  }

  impl_->scenario_elapsed_time += dt;

  auto it = impl_->scenarios.find(impl_->current_scenario_id);
  if (it == impl_->scenarios.end()) {
    return;
  }

  ScenarioDefinition &scenario = it->second;

  // Check timeout
  if (scenario.duration > 0 &&
      impl_->scenario_elapsed_time >= scenario.duration) {
    impl_->state = ScenarioState::Timeout;
    impl_->result.final_state = impl_->state;
    impl_->result.execution_time = impl_->scenario_elapsed_time;
    impl_->result.events_triggered = impl_->events_triggered;
    impl_->result.message = "Scenario timed out";

    if (impl_->state_callback) {
      impl_->state_callback(impl_->current_scenario_id, impl_->state);
    }
    return;
  }

  // Process events
  for (auto &event : scenario.events) {
    if (!event.triggered &&
        impl_->scenario_elapsed_time >= event.trigger_time) {
      // Check condition if present
      if (event.condition && !event.condition()) {
        continue;
      }

      // Trigger event
      event.triggered = true;
      impl_->events_triggered++;

      if (impl_->event_callback) {
        impl_->event_callback(event);
      }

      // Process event based on type
      switch (event.type) {
      case EventType::SetWeather:
        // Would parse params and set weather
        break;
      case EventType::SetTimeOfDay:
        // Would parse params and set time
        break;
      // Handle other event types...
      default:
        break;
      }
    }
  }

  // Update active degradations
  for (auto &gps_deg : impl_->active_gps_degradations) {
    if (gps_deg.enabled) {
      double deg_elapsed = impl_->scenario_elapsed_time - gps_deg.start_time;
      if (deg_elapsed >= 0 && deg_elapsed <= gps_deg.duration) {
        // Apply degradation...
      } else if (deg_elapsed > gps_deg.duration) {
        gps_deg.enabled = false;
      }
    }
  }

  for (auto &imu_deg : impl_->active_imu_degradations) {
    if (imu_deg.enabled) {
      double deg_elapsed = impl_->scenario_elapsed_time - imu_deg.start_time;
      if (deg_elapsed >= 0 && deg_elapsed <= imu_deg.duration) {
        // Apply degradation...
      } else if (deg_elapsed > imu_deg.duration) {
        imu_deg.enabled = false;
      }
    }
  }

  // Check success/failure conditions
  if (scenario.success_condition && scenario.success_condition()) {
    impl_->state = ScenarioState::Completed;
    impl_->result.final_state = impl_->state;
    impl_->result.execution_time = impl_->scenario_elapsed_time;
    impl_->result.events_triggered = impl_->events_triggered;
    impl_->result.message = "Scenario completed successfully";

    if (impl_->state_callback) {
      impl_->state_callback(impl_->current_scenario_id, impl_->state);
    }
    return;
  }

  if (scenario.failure_condition && scenario.failure_condition()) {
    impl_->state = ScenarioState::Failed;
    impl_->result.final_state = impl_->state;
    impl_->result.execution_time = impl_->scenario_elapsed_time;
    impl_->result.events_triggered = impl_->events_triggered;
    impl_->result.message = "Scenario failed";

    if (impl_->state_callback) {
      impl_->state_callback(impl_->current_scenario_id, impl_->state);
    }
    return;
  }

  // Handle looping
  bool all_events_triggered =
      std::all_of(scenario.events.begin(), scenario.events.end(),
                  [](const ScenarioEvent &e) { return e.triggered; });

  if (all_events_triggered && scenario.loop) {
    restart();
  }
}

ScenarioState ScenarioManager::get_state() const { return impl_->state; }

std::string ScenarioManager::get_current_scenario() const {
  return impl_->current_scenario_id;
}

ScenarioResult ScenarioManager::get_result() const { return impl_->result; }

void ScenarioManager::set_weather(const WeatherConditions &weather) {
  impl_->current_weather = weather;
}

WeatherConditions ScenarioManager::get_weather() const {
  return impl_->current_weather;
}

void ScenarioManager::set_time_of_day(const TimeOfDay &time) {
  impl_->current_time = time;
}

TimeOfDay ScenarioManager::get_time_of_day() const {
  return impl_->current_time;
}

void ScenarioManager::apply_gps_degradation(
    const GpsDegradationScenario &scenario) {
  impl_->active_gps_degradations.push_back(scenario);
}

void ScenarioManager::apply_imu_degradation(
    const ImuDegradationScenario &scenario) {
  impl_->active_imu_degradations.push_back(scenario);
}

void ScenarioManager::clear_degradations() {
  impl_->active_gps_degradations.clear();
  impl_->active_imu_degradations.clear();
}

void ScenarioManager::add_event(const ScenarioEvent &event) {
  if (!impl_->current_scenario_id.empty()) {
    impl_->scenarios[impl_->current_scenario_id].events.push_back(event);
  }
}

void ScenarioManager::trigger_event(const std::string &event_name) {
  // Manual event triggering would be implemented here
}

bool ScenarioManager::cancel_event(const std::string &event_name) {
  // Event cancellation would be implemented here
  return false;
}

void ScenarioManager::set_state_callback(ScenarioCallback callback) {
  impl_->state_callback = std::move(callback);
}

void ScenarioManager::set_event_callback(EventCallback callback) {
  impl_->event_callback = std::move(callback);
}

void ScenarioManager::record_metric(const std::string &name, double value) {
  impl_->metrics[name].push_back(value);
}

std::map<std::string, std::vector<double>>
ScenarioManager::get_metrics() const {
  return impl_->metrics;
}

void ScenarioManager::clear_metrics() { impl_->metrics.clear(); }

bool ScenarioManager::export_log(const std::string &path) {
  // Export scenario execution log to file
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }

  file << "{\n";
  file << "  \"scenario\": \"" << impl_->current_scenario_id << "\",\n";
  file << "  \"state\": " << static_cast<int>(impl_->state) << ",\n";
  file << "  \"elapsed_time\": " << impl_->scenario_elapsed_time << ",\n";
  file << "  \"events_triggered\": " << impl_->events_triggered << ",\n";
  file << "  \"metrics\": {\n";

  bool first = true;
  for (const auto &[name, values] : impl_->metrics) {
    if (!first)
      file << ",\n";
    file << "    \"" << name << "\": [";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0)
        file << ", ";
      file << values[i];
    }
    file << "]";
    first = false;
  }

  file << "\n  }\n";
  file << "}\n";

  return true;
}

} // namespace scenario
} // namespace aerosimx
