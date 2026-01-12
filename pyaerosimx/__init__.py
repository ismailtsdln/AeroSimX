"""
AeroSimX - Advanced Autonomous Robotics Simulation Platform

A next-generation, modular simulation platform for autonomous vehicles,
drones, and robots with high-performance C++ core and Python APIs.
"""

__version__ = "0.1.0"
__author__ = "AeroSimX Team"

from pyaerosimx.client import AeroSimXClient
from pyaerosimx.vehicles import Vehicle, Multirotor, Car
from pyaerosimx.sensors import Sensor, Lidar, Camera, Imu, Gps, Radar

__all__ = [
    "AeroSimXClient",
    "Vehicle",
    "Multirotor", 
    "Car",
    "Sensor",
    "Lidar",
    "Camera",
    "Imu",
    "Gps",
    "Radar",
]
