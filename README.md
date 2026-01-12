<p align="center">
  <img src="docs/assets/logo.png" alt="AeroSimX Logo" width="400">
</p>

<h1 align="center">AeroSimX</h1>

<p align="center">
  <strong>Advanced Autonomous Robotics Simulation Platform</strong>
</p>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#installation">Installation</a> •
  <a href="#quick-start">Quick Start</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#examples">Examples</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/version-0.1.0-blue.svg" alt="Version">
  <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="License">
  <img src="https://img.shields.io/badge/C++-20-orange.svg" alt="C++20">
  <img src="https://img.shields.io/badge/Python-3.9+-yellow.svg" alt="Python">
</p>

---

## 🚀 Overview

**AeroSimX** is a next-generation, modular simulation platform for autonomous vehicles, drones, and robots. Built with a high-performance C++ core and intuitive Python APIs, it provides everything you need for:

- 🤖 **Autonomous System Development** — Test control algorithms in realistic environments
- 🧠 **Machine Learning & RL** — Generate training data and run reinforcement learning experiments
- 📷 **Computer Vision** — Synthetic data generation with photorealistic sensors
- 🔬 **Research** — Rapid prototyping and experimentation platform

## ✨ Features

### 🎮 Simulation Core
- **High-performance C++ engine** with GPU-ready architecture
- **Physics simulation** with rigid body dynamics and collision detection
- **Real-time and accelerated modes** for training efficiency
- **Multi-vehicle support** with batched simulation

### 🚁 Vehicle Models
- **Multirotors** — Quadcopters, hexacopters, octocopters with motor dynamics
- **Ground Vehicles** — Cars and trucks with tire/suspension physics
- **Extensible architecture** — Easy to add custom vehicle types

### 📡 Sensor Suite
| Sensor | Features |
|--------|----------|
| **Lidar** | 16-128 channels, configurable FOV, noise models |
| **Camera** | RGB, Depth, Segmentation with lens distortion |
| **IMU** | Accelerometer + Gyroscope with bias drift |
| **GPS** | GNSS simulation with HDOP/VDOP and degradation |
| **Radar** | Range-velocity detection with false alarm modeling |

### 🐍 Python API
```python
from pyaerosimx import AeroSimXClient

with AeroSimXClient() as client:
    # Spawn a drone
    drone = client.spawn_multirotor("drone1", position=(0, 0, 1))
    
    # Attach sensors
    drone.attach_sensor(Lidar("lidar", channels=16))
    drone.attach_sensor(Camera("camera", width=640, height=480))
    
    # Control the drone
    drone.takeoff(altitude=10)
    client.step(1000)
    
    # Get sensor data
    point_cloud = drone.get_sensor("lidar").get_point_cloud()
    image = drone.get_sensor("camera").get_image()
```

### 📊 Data & Training
- **Dataset export** in COCO, KITTI, and ROS bag formats
- **Domain randomization** for robust perception training
- **RL-ready** with step-by-step mode and gym-compatible interface

### 🔌 Integration
- **ROS2 Bridge** — Native ROS2 node with standard message types
- **Plugin System** — Extend with custom vehicles, sensors, and environments

## 📦 Installation

### Prerequisites

- **C++ Compiler**: GCC 10+ / Clang 12+ / MSVC 2019+
- **CMake**: 3.20+
- **Python**: 3.9+
- **Optional**: CUDA 11+ (for GPU physics)

### From Source (C++ Library)

```bash
git clone https://github.com/ismailtasdelen/AeroSimX.git
cd AeroSimX

# Create build directory
mkdir build && cd build

# Configure (adjust options as needed)
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_PYTHON_BINDINGS=ON

# Build
cmake --build . -j$(nproc)

# Install (optional)
sudo cmake --install .
```

### Python Package

```bash
# Install from source
pip install .

# Or install in development mode
pip install -e ".[dev]"
```

### Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_SHARED_LIBS` | ON | Build shared libraries |
| `BUILD_TESTS` | ON | Build unit tests |
| `BUILD_EXAMPLES` | ON | Build example applications |
| `BUILD_PYTHON_BINDINGS` | ON | Build Python bindings |
| `ENABLE_CUDA` | OFF | Enable CUDA for GPU physics |
| `ENABLE_ROS2` | OFF | Enable ROS2 integration |

## 🎯 Quick Start

### C++ Example

```cpp
#include <aerosimx/aerosimx.hpp>

int main() {
    using namespace aerosimx;
    
    // Create simulation
    core::SimulationConfig config;
    config.time_step = 0.001;
    config.mode = core::SimulationMode::StepByStep;
    
    core::Simulation sim(config);
    sim.initialize();
    
    // Create a multirotor
    vehicles::MultirotorConfig drone_config;
    drone_config.mass = 1.5;
    drone_config.arm_length = 0.25;
    
    auto drone = std::make_shared<vehicles::Multirotor>(drone_config);
    drone->initialize();
    
    // Set initial position
    Pose initial_pose;
    initial_pose.position = {0, 0, 1};
    drone->set_pose(initial_pose);
    
    // Add to simulation
    sim.get_entity_manager().add_vehicle(drone, "drone1");
    
    // Attach lidar
    sensors::LidarConfig lidar_config;
    lidar_config.channels = 16;
    lidar_config.range_max = 100.0;
    
    auto lidar = std::make_shared<sensors::Lidar>(lidar_config);
    drone->attach_sensor(lidar);
    
    // Arm and takeoff
    drone->arm();
    drone->takeoff(10.0);
    
    // Run simulation
    sim.start();
    for (int i = 0; i < 10000; ++i) {
        sim.step();
    }
    
    return 0;
}
```

### Python Example

```python
from pyaerosimx import AeroSimXClient, Lidar, Camera, Imu

# Connect to simulation
client = AeroSimXClient()
client.connect()

# Spawn vehicles
drone = client.spawn_multirotor(
    name="drone1",
    position=(0, 0, 1),
    mass=1.5
)

car = client.spawn_car(
    name="car1",
    position=(10, 0, 0)
)

# Attach sensors to drone
drone.attach_sensor(Lidar("lidar_front", channels=32, range_max=100))
drone.attach_sensor(Camera("camera_front", width=1280, height=720))
drone.attach_sensor(Imu("imu"))

# Control the drone
drone.arm()
drone.takeoff(altitude=10)

# Wait for takeoff
client.step(5000)

# Move to waypoint
drone.move_to_position(20, 10, 15)
client.step(10000)

# Get sensor data
lidar = drone.get_sensor("lidar_front")
point_cloud = lidar.get_point_cloud()
print(f"Lidar points: {point_cloud.num_points}")

camera = drone.get_sensor("camera_front")
image = camera.get_image()
print(f"Image shape: {image.data.shape}")

# Land and cleanup
drone.land()
client.step(5000)
drone.disarm()

client.disconnect()
```

## 📚 Documentation

### API Reference

- [C++ API Documentation](docs/api/cpp/index.md)
- [Python API Documentation](docs/api/python/index.md)
- [Sensor Configuration Guide](docs/sensors.md)
- [Vehicle Dynamics Reference](docs/vehicles.md)

### Tutorials

1. [Getting Started](docs/tutorials/01_getting_started.md)
2. [Creating Custom Vehicles](docs/tutorials/02_custom_vehicles.md)
3. [Training RL Agents](docs/tutorials/03_rl_training.md)
4. [Computer Vision Pipelines](docs/tutorials/04_cv_pipelines.md)
5. [ROS2 Integration](docs/tutorials/05_ros2_integration.md)

## 🔧 Examples

| Example | Description |
|---------|-------------|
| [basic_flight](examples/python/basic_flight.py) | Simple drone takeoff and landing |
| [waypoint_mission](examples/python/waypoint_mission.py) | Multi-waypoint navigation |
| [sensor_fusion](examples/python/sensor_fusion.py) | Combining lidar and camera data |
| [rl_hover](examples/python/rl_hover.py) | Reinforcement learning for hovering |
| [dataset_generation](examples/python/dataset_generation.py) | Generating training datasets |
| [car_simulation](examples/python/car_simulation.py) | Ground vehicle simulation |

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Python API Layer                        │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐            │
│  │ Client  │ │Vehicles │ │ Sensors │ │Datasets │            │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘            │
└───────┼───────────┼───────────┼───────────┼─────────────────┘
        │           │           │           │
        └───────────┴───────────┴───────────┘
                         │
                    ┌────┴────┐
                    │pybind11 │
                    └────┬────┘
                         │
┌────────────────────────┼────────────────────────────────────┐
│                   C++ Core Engine                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                  Control API                         │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────┐  │
│  │Simulation│ │ Physics │ │ Sensors │ │Vehicles │ │Events│  │
│  │   Core   │ │ Engine  │ │  Suite  │ │  Models │ │System│  │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └──────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 🧪 Testing

```bash
# C++ tests
cd build
ctest --output-on-failure

# Python tests
pytest tests/python/ -v

# Code coverage
cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
cmake --build .
ctest
gcov -r src/**/*.cpp
```

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Inspired by [AirSim](https://github.com/microsoft/AirSim) by Microsoft Research
- Physics based on modern rigid body dynamics research
- Sensor models derived from real-world specifications

## 📬 Contact

- **Issues**: [GitHub Issues](https://github.com/ismailtasdelen/AeroSimX/issues)
- **Discussions**: [GitHub Discussions](https://github.com/ismailtasdelen/AeroSimX/discussions)

---

<p align="center">
  Made with ❤️ for the robotics community
</p>
