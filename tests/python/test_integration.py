"""
Integration tests for AeroSimX Python API
"""

import pytest
import time
from pyaerosimx import AeroSimXClient
from pyaerosimx.vehicles import FlightMode


def test_full_mission_flow():
    """Test a full takeoff, move, and land flow."""
    client = AeroSimXClient()
    client.connect()
    
    try:
        drone = client.spawn_multirotor("mission_drone", position=(0, 0, 0))
        
        # 1. Arm
        drone.arm()
        assert drone.is_armed
        
        # 2. Takeoff
        drone.takeoff(altitude=5.0)
        
        # Step simulation to reach altitude
        for _ in range(100):
            client.step(1)
            if drone.altitude > 4.5:
                break
                
        assert drone.altitude > 4.0
        
        # 3. Move
        drone.move_to_position(x=5.0, y=5.0, z=5.0)
        for _ in range(200):
            client.step(1)
            pos = drone.get_position()
            if abs(pos[0] - 5.0) < 0.5:
                break
                
        assert abs(drone.get_position()[0] - 5.0) < 1.0
        
        # 4. Land
        drone.land()
        for _ in range(200):
            client.step(1)
            if drone.is_landed:
                break
                
        assert drone.is_landed
        
    finally:
        client.disconnect()


def test_sensor_data_retrieval():
    """Test retrieving sensor data through the client."""
    client = AeroSimXClient()
    client.connect()
    
    try:
        drone = client.spawn_multirotor("sensor_drone", position=(0, 0, 5))
        from pyaerosimx.sensors import Lidar, Camera
        
        drone.attach_sensor(Lidar("lidar_test"))
        drone.attach_sensor(Camera("camera_test"))
        
        client.step(10)
        
        lidar = drone.get_sensor("lidar_test")
        camera = drone.get_sensor("camera_test")
        
        assert lidar.get_point_cloud() is not None
        assert camera.get_image() is not None
        
    finally:
        client.disconnect()
