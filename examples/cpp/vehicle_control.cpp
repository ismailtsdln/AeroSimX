#include "aerosimx/aerosimx.hpp"
#include <iostream>

using namespace aerosimx;

int main() {
  std::cout << "AeroSimX - Vehicle Control Example" << std::endl;

  core::Simulation sim;
  sim.initialize();

  vehicles::MultirotorConfig drone_config;
  auto drone = std::make_shared<vehicles::Multirotor>(drone_config);
  drone->initialize();
  sim.get_entity_manager().add_vehicle(drone, "drone1");

  // Arm and Takeoff
  drone->arm();
  drone->takeoff(10.0);

  // Run simulation loop
  for (int i = 0; i < 500; ++i) {
    sim.step();
    auto state = drone->get_state();
    if (i % 50 == 0) {
      std::cout << "Altitude: " << state.pose.position.z << "m" << std::endl;
    }
  }

  // Move to position
  std::cout << "Moving to (10, 10, 10)..." << std::endl;
  drone->move_to_position(10, 10, 10);

  for (int i = 0; i < 1000; ++i) {
    sim.step();
  }

  // Land
  std::cout << "Landing..." << std::endl;
  drone->land();

  for (int i = 0; i < 500; ++i) {
    sim.step();
  }

  std::cout << "Mission accomplished." << std::endl;
  sim.stop();

  return 0;
}
