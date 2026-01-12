/**
 * @file test_integration.cpp
 * @brief Integration tests for AeroSimX core
 */

#include "aerosimx/aerosimx.hpp"
#include <gtest/gtest.h>

using namespace aerosimx;

class IntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    core::SimulationConfig config;
    config.time_step = 0.01;
    config.mode = core::SimulationMode::StepByStep;
    sim = std::make_unique<core::Simulation>(config);
    sim->initialize();
  }

  std::unique_ptr<core::Simulation> sim;
};

TEST_F(IntegrationTest, VehicleSensorLifecycle) {
  // 1. Create a multirotor
  vehicles::MultirotorConfig drone_config;
  auto drone = std::make_shared<vehicles::Multirotor>(drone_config);
  drone->set_physics_engine(nullptr); // Use internal kinematics for this test
  drone->initialize();

  // 2. Attach a lidar
  sensors::LidarConfig lidar_config;
  lidar_config.name = "lidar_main";
  auto lidar = std::make_shared<sensors::Lidar>(lidar_config);
  drone->attach_sensor(lidar);

  // 3. Add to simulation
  EntityId drone_id =
      sim->get_entity_manager().add_vehicle(drone, "test_drone");
  ASSERT_NE(drone_id, core::INVALID_ENTITY_ID);

  // 4. Arm and Takeoff
  drone->arm();
  drone->takeoff(10.0);

  // 5. Step simulation and verify movement
  for (int i = 0; i < 100; ++i) {
    sim->step();
  }

  auto pos = drone->get_state().pose.position;
  EXPECT_GT(pos.z, 0.5); // Should have moved up

  // 6. Verify sensor data
  auto sensor = drone->get_sensor("lidar_main");
  ASSERT_NE(sensor, nullptr);
  EXPECT_TRUE(sensor->is_enabled());
}

TEST_F(IntegrationTest, ScenarioExecution) {
  // Setup a simple scenario
  scenario::ScenarioManager scenario_mgr;

  scenario::ScenarioDefinition def;
  def.name = "Test Mission";
  def.duration = 5.0;

  scenario::ScenarioEvent event;
  event.type = scenario::EventType::Custom;
  event.trigger_time = 1.0;
  def.events.push_back(event);

  std::string scenario_id = scenario_mgr.load_scenario(def);
  EXPECT_FALSE(scenario_id.empty());

  scenario_mgr.start(scenario_id);
  EXPECT_EQ(scenario_mgr.get_state(), scenario::ScenarioState::Running);

  // Step and verify
  scenario_mgr.update(1.0, 1.0);
  EXPECT_EQ(scenario_mgr.get_state(), scenario::ScenarioState::Running);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
