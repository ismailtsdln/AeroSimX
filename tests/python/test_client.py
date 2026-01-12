"""
Tests for AeroSimX Python client
"""

import pytest
import numpy as np
from pyaerosimx import AeroSimXClient
from pyaerosimx.client import Vector3, Quaternion, Pose, SimulationMode


class TestVector3:
    """Tests for Vector3 class."""
    
    def test_default_constructor(self):
        v = Vector3()
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0
    
    def test_constructor_with_values(self):
        v = Vector3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0
    
    def test_addition(self):
        v1 = Vector3(1, 2, 3)
        v2 = Vector3(4, 5, 6)
        result = v1 + v2
        assert result.x == 5
        assert result.y == 7
        assert result.z == 9
    
    def test_subtraction(self):
        v1 = Vector3(4, 5, 6)
        v2 = Vector3(1, 2, 3)
        result = v1 - v2
        assert result.x == 3
        assert result.y == 3
        assert result.z == 3
    
    def test_scalar_multiplication(self):
        v = Vector3(1, 2, 3)
        result = v * 2
        assert result.x == 2
        assert result.y == 4
        assert result.z == 6
    
    def test_magnitude(self):
        v = Vector3(3, 4, 0)
        assert v.magnitude() == pytest.approx(5.0)
    
    def test_normalized(self):
        v = Vector3(3, 4, 0)
        n = v.normalized()
        assert n.magnitude() == pytest.approx(1.0)
    
    def test_to_numpy(self):
        v = Vector3(1, 2, 3)
        arr = v.to_numpy()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (3,)
        np.testing.assert_array_equal(arr, [1, 2, 3])
    
    def test_from_numpy(self):
        arr = np.array([1.0, 2.0, 3.0])
        v = Vector3.from_numpy(arr)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0


class TestQuaternion:
    """Tests for Quaternion class."""
    
    def test_default_constructor(self):
        q = Quaternion()
        assert q.w == 1.0
        assert q.x == 0.0
        assert q.y == 0.0
        assert q.z == 0.0
    
    def test_from_euler_identity(self):
        q = Quaternion.from_euler(0, 0, 0)
        assert q.w == pytest.approx(1.0)
        assert q.x == pytest.approx(0.0)
        assert q.y == pytest.approx(0.0)
        assert q.z == pytest.approx(0.0)
    
    def test_to_euler_identity(self):
        q = Quaternion()
        euler = q.to_euler()
        assert euler.x == pytest.approx(0.0, abs=1e-6)
        assert euler.y == pytest.approx(0.0, abs=1e-6)
        assert euler.z == pytest.approx(0.0, abs=1e-6)
    
    def test_euler_roundtrip(self):
        import math
        roll, pitch, yaw = 0.1, 0.2, 0.3
        q = Quaternion.from_euler(roll, pitch, yaw)
        euler = q.to_euler()
        assert euler.x == pytest.approx(roll, abs=1e-6)
        assert euler.y == pytest.approx(pitch, abs=1e-6)
        assert euler.z == pytest.approx(yaw, abs=1e-6)


class TestAeroSimXClient:
    """Tests for AeroSimXClient class."""
    
    def test_connect_disconnect(self):
        client = AeroSimXClient()
        assert not client.is_connected()
        
        result = client.connect()
        assert result is True
        assert client.is_connected()
        
        client.disconnect()
        assert not client.is_connected()
    
    def test_context_manager(self):
        with AeroSimXClient() as client:
            assert client.is_connected()
        # After context, should be disconnected
    
    def test_spawn_multirotor(self):
        with AeroSimXClient() as client:
            drone = client.spawn_multirotor(
                name="test_drone",
                position=(1, 2, 3)
            )
            assert drone is not None
            assert drone.name == "test_drone"
            assert drone.get_position() == pytest.approx((1, 2, 3), abs=0.1)
    
    def test_spawn_car(self):
        with AeroSimXClient() as client:
            car = client.spawn_car(
                name="test_car",
                position=(5, 0, 0)
            )
            assert car is not None
            assert car.name == "test_car"
    
    def test_simulation_time(self):
        with AeroSimXClient() as client:
            initial_time = client.get_simulation_time()
            assert initial_time == 0.0
            
            client.step(100)
            
            new_time = client.get_simulation_time()
            assert new_time > initial_time
    
    def test_reset(self):
        with AeroSimXClient() as client:
            client.step(100)
            assert client.get_simulation_time() > 0
            
            client.reset()
            assert client.get_simulation_time() == 0.0
    
    def test_get_vehicle(self):
        with AeroSimXClient() as client:
            client.spawn_multirotor("drone1", position=(0, 0, 1))
            
            vehicle = client.get_vehicle("drone1")
            assert vehicle is not None
            assert vehicle.name == "drone1"
            
            missing = client.get_vehicle("nonexistent")
            assert missing is None
    
    def test_remove_vehicle(self):
        with AeroSimXClient() as client:
            client.spawn_multirotor("drone1", position=(0, 0, 1))
            
            assert client.get_vehicle("drone1") is not None
            
            result = client.remove_vehicle("drone1")
            assert result is True
            
            assert client.get_vehicle("drone1") is None
    
    def test_set_mode(self):
        client = AeroSimXClient()
        client.set_mode(SimulationMode.ACCELERATED)
        # Should not raise


class TestMultirotor:
    """Tests for Multirotor vehicle."""
    
    @pytest.fixture
    def client_with_drone(self):
        client = AeroSimXClient()
        client.connect()
        drone = client.spawn_multirotor("drone", position=(0, 0, 0.5))
        yield client, drone
        client.disconnect()
    
    def test_arm_disarm(self, client_with_drone):
        client, drone = client_with_drone
        
        assert not drone.is_armed
        
        drone.arm()
        assert drone.is_armed
        
        drone.disarm()
        assert not drone.is_armed
    
    def test_takeoff(self, client_with_drone):
        client, drone = client_with_drone
        
        drone.takeoff(altitude=10)
        
        # Simulate for a bit
        for _ in range(1000):
            client.step(1)
        
        assert drone.altitude > 5  # Should be climbing
    
    def test_hover(self, client_with_drone):
        client, drone = client_with_drone
        
        drone.arm()
        drone.hover()
        
        from pyaerosimx.vehicles import FlightMode
        assert drone.flight_mode == FlightMode.HOVER
    
    def test_move_to_position(self, client_with_drone):
        client, drone = client_with_drone
        
        drone.arm()
        result = drone.move_to_position(10, 5, 10)
        
        assert result is True
    
    def test_battery_drains(self, client_with_drone):
        client, drone = client_with_drone
        
        initial_battery = drone.battery_percentage
        
        drone.arm()
        drone.takeoff(5)
        
        for _ in range(1000):
            client.step(1)
        
        assert drone.battery_percentage < initial_battery


class TestCar:
    """Tests for Car vehicle."""
    
    @pytest.fixture
    def client_with_car(self):
        client = AeroSimXClient()
        client.connect()
        car = client.spawn_car("car", position=(0, 0, 0))
        yield client, car
        client.disconnect()
    
    def test_set_controls(self, client_with_car):
        client, car = client_with_car
        
        car.set_controls(throttle=0.5, steering=0.1)
        
        # Simulate
        for _ in range(100):
            client.step(1)
        
        assert car.speed > 0
    
    def test_get_speed(self, client_with_car):
        client, car = client_with_car
        
        assert car.get_speed() == 0
        
        car.set_throttle(1.0)
        for _ in range(500):
            client.step(1)
        
        assert car.get_speed() > 0
        assert car.get_speed_kmh() == pytest.approx(car.get_speed() * 3.6)
    
    def test_shift_gears(self, client_with_car):
        client, car = client_with_car
        
        initial_gear = car.gear
        
        car.shift_up()
        assert car.gear == initial_gear + 1
        
        car.shift_down()
        assert car.gear == initial_gear
