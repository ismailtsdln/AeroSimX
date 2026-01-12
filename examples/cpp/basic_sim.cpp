#include "aerosimx/aerosimx.hpp"
#include <iostream>

using namespace aerosimx;

int main() {
  std::cout << "AeroSimX - Basic Simulation Example" << std::endl;

  // 1. Configure simulation
  core::SimulationConfig config;
  config.time_step = 0.01;
  config.mode = core::SimulationMode::RealTime;

  // 2. Initialize simulation
  core::Simulation sim(config);
  sim.initialize();

  // 3. Create a drone
  vehicles::MultirotorConfig drone_config;
  auto drone = std::make_shared<vehicles::Multirotor>(drone_config);
  drone->initialize();

  // 4. Register to simulation
  sim.get_entity_manager().add_vehicle(drone, "drone1");

  // 5. Start simulation
  std::cout << "Starting simulation..." << std::endl;
  sim.start();

  // 6. Run for a few steps (in this example we'll just run 100 steps)
  for (int i = 0; i < 100; ++i) {
    sim.step();
  }

  std::cout << "Simulation completed." << std::endl;
  sim.stop();

  return 0;
}
