/**
 * @file benchmark_physics.cpp
 * @brief Performance benchmarks for the physics engine
 */

#include "aerosimx/physics/physics_engine.hpp"
#include <chrono>
#include <iostream>
#include <vector>

using namespace aerosimx;

void benchmark_physics_load(int num_bodies, int steps) {
  physics::PhysicsConfig config;
  physics::PhysicsEngine engine(config);
  engine.initialize();

  std::vector<physics::BodyId> body_ids;
  for (int i = 0; i < num_bodies; ++i) {
    physics::RigidBodyProperties props;
    props.mass = 1.0;
    Pose pose;
    pose.position = {static_cast<double>(i), 0, 10};

    body_ids.push_back(engine.create_body(props, pose));
  }

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < steps; ++i) {
    engine.step(0.01);
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;

  std::cout << "Physics Benchmark (" << num_bodies << " bodies, " << steps
            << " steps): " << diff.count() << " seconds ("
            << (diff.count() / steps) * 1000.0 << " ms/step)" << std::endl;
}

int main() {
  std::cout << "Running AeroSimX Benchmarks..." << std::endl;

  benchmark_physics_load(10, 1000);
  benchmark_physics_load(100, 1000);
  benchmark_physics_load(1000, 1000);

  return 0;
}
