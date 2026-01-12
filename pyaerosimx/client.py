"""
AeroSimX Client Module

Main client class for connecting to and controlling AeroSimX simulations.
"""

from __future__ import annotations

import time
import logging
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

logger = logging.getLogger(__name__)


class SimulationMode(Enum):
    """Simulation execution mode."""
    REAL_TIME = auto()
    ACCELERATED = auto()
    STEP_BY_STEP = auto()


@dataclass
class Vector3:
    """3D vector representation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def __add__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Vector3':
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> 'Vector3':
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))
    
    def magnitude(self) -> float:
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalized(self) -> 'Vector3':
        mag = self.magnitude()
        if mag < 1e-10:
            return Vector3()
        return self * (1.0 / mag)


@dataclass
class Quaternion:
    """Quaternion for rotation representation."""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def to_euler(self) -> Vector3:
        """Convert to Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x**2 + self.y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y**2 + self.z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return Vector3(roll, pitch, yaw)
    
    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> 'Quaternion':
        """Create from Euler angles (roll, pitch, yaw) in radians."""
        cr, sr = np.cos(roll/2), np.sin(roll/2)
        cp, sp = np.cos(pitch/2), np.sin(pitch/2)
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        
        return cls(
            w=cr * cp * cy + sr * sp * sy,
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy
        )


@dataclass
class Pose:
    """Position and orientation."""
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass  
class Twist:
    """Linear and angular velocity."""
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class VehicleState:
    """Complete vehicle state."""
    pose: Pose = field(default_factory=Pose)
    velocity: Twist = field(default_factory=Twist)
    acceleration: Vector3 = field(default_factory=Vector3)
    timestamp: float = 0.0
    collision_detected: bool = False


@dataclass
class MultirotorControls:
    """Multirotor control inputs."""
    roll: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0
    is_rate_mode: bool = False


@dataclass
class CarControls:
    """Ground vehicle control inputs."""
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    handbrake: bool = False
    gear: int = 0


class AeroSimXClient:
    """
    Main client for AeroSimX simulation control.
    
    Provides high-level API for:
    - Simulation lifecycle management
    - Vehicle spawning and control
    - Sensor data acquisition
    - Environment configuration
    
    Example:
        >>> client = AeroSimXClient()
        >>> client.connect()
        >>> drone = client.spawn_multirotor("drone1", position=(0, 0, 1))
        >>> drone.takeoff(altitude=10)
        >>> client.step(100)
        >>> state = drone.get_state()
    """
    
    def __init__(self, host: str = "localhost", port: int = 41451):
        """
        Initialize client.
        
        Args:
            host: Simulation server host
            port: Simulation server port
        """
        self.host = host
        self.port = port
        self._connected = False
        self._simulation_time = 0.0
        self._vehicles: Dict[str, Any] = {}
        self._sensors: Dict[str, Any] = {}
        self._mode = SimulationMode.STEP_BY_STEP
        
    def connect(self, timeout: float = 10.0) -> bool:
        """
        Connect to simulation.
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful
        """
        logger.info(f"Connecting to AeroSimX at {self.host}:{self.port}")
        # In production, this would establish network connection
        # For now, simulate local mode
        self._connected = True
        logger.info("Connected to AeroSimX simulation")
        return True
    
    def disconnect(self) -> None:
        """Disconnect from simulation."""
        self._connected = False
        self._vehicles.clear()
        self._sensors.clear()
        logger.info("Disconnected from AeroSimX")
    
    def is_connected(self) -> bool:
        """Check if connected to simulation."""
        return self._connected
    
    # =========================================================================
    # Simulation Control
    # =========================================================================
    
    def reset(self) -> bool:
        """Reset simulation to initial state."""
        self._check_connected()
        self._simulation_time = 0.0
        for vehicle in self._vehicles.values():
            vehicle.reset()
        logger.info("Simulation reset")
        return True
    
    def start(self) -> bool:
        """Start continuous simulation."""
        self._check_connected()
        logger.info("Simulation started")
        return True
    
    def pause(self) -> bool:
        """Pause simulation."""
        self._check_connected()
        logger.info("Simulation paused")
        return True
    
    def resume(self) -> bool:
        """Resume paused simulation."""
        self._check_connected()
        logger.info("Simulation resumed")
        return True
    
    def step(self, count: int = 1) -> None:
        """
        Step simulation forward.
        
        Args:
            count: Number of steps to advance
        """
        self._check_connected()
        dt = 0.001  # 1ms time step
        for _ in range(count):
            self._simulation_time += dt
            for vehicle in self._vehicles.values():
                vehicle._update(dt)
    
    def get_simulation_time(self) -> float:
        """Get current simulation time in seconds."""
        return self._simulation_time
    
    def set_mode(self, mode: SimulationMode) -> None:
        """Set simulation mode."""
        self._mode = mode
    
    # =========================================================================
    # Vehicle Management
    # =========================================================================
    
    def spawn_multirotor(
        self,
        name: str,
        position: Tuple[float, float, float] = (0, 0, 0),
        orientation: Tuple[float, float, float] = (0, 0, 0),
        **config
    ) -> 'Multirotor':
        """
        Spawn a multirotor (drone) vehicle.
        
        Args:
            name: Unique vehicle name
            position: Initial position (x, y, z) in meters
            orientation: Initial orientation (roll, pitch, yaw) in radians
            **config: Additional configuration options
            
        Returns:
            Multirotor vehicle instance
        """
        self._check_connected()
        
        from pyaerosimx.vehicles import Multirotor
        
        pose = Pose(
            position=Vector3(*position),
            orientation=Quaternion.from_euler(*orientation)
        )
        
        vehicle = Multirotor(name=name, initial_pose=pose, **config)
        self._vehicles[name] = vehicle
        
        logger.info(f"Spawned multirotor '{name}' at {position}")
        return vehicle
    
    def spawn_car(
        self,
        name: str,
        position: Tuple[float, float, float] = (0, 0, 0),
        orientation: Tuple[float, float, float] = (0, 0, 0),
        **config
    ) -> 'Car':
        """
        Spawn a ground vehicle.
        
        Args:
            name: Unique vehicle name
            position: Initial position (x, y, z) in meters
            orientation: Initial orientation (roll, pitch, yaw) in radians
            **config: Additional configuration options
            
        Returns:
            Car vehicle instance
        """
        self._check_connected()
        
        from pyaerosimx.vehicles import Car
        
        pose = Pose(
            position=Vector3(*position),
            orientation=Quaternion.from_euler(*orientation)
        )
        
        vehicle = Car(name=name, initial_pose=pose, **config)
        self._vehicles[name] = vehicle
        
        logger.info(f"Spawned car '{name}' at {position}")
        return vehicle
    
    def get_vehicle(self, name: str) -> Optional[Any]:
        """Get vehicle by name."""
        return self._vehicles.get(name)
    
    def get_all_vehicles(self) -> List[Any]:
        """Get all vehicles."""
        return list(self._vehicles.values())
    
    def remove_vehicle(self, name: str) -> bool:
        """Remove a vehicle."""
        if name in self._vehicles:
            del self._vehicles[name]
            logger.info(f"Removed vehicle '{name}'")
            return True
        return False
    
    # =========================================================================
    # Environment Control
    # =========================================================================
    
    def set_weather(
        self,
        rain: float = 0.0,
        fog: float = 0.0,
        snow: float = 0.0
    ) -> bool:
        """
        Set weather conditions.
        
        Args:
            rain: Rain intensity (0-1)
            fog: Fog density (0-1)
            snow: Snow intensity (0-1)
        """
        self._check_connected()
        logger.info(f"Weather set: rain={rain}, fog={fog}, snow={snow}")
        return True
    
    def set_time_of_day(self, hour: int, minute: int = 0) -> bool:
        """Set time of day (affects lighting)."""
        self._check_connected()
        logger.info(f"Time of day set to {hour:02d}:{minute:02d}")
        return True
    
    def set_wind(self, velocity: Tuple[float, float, float]) -> bool:
        """Set wind velocity in m/s."""
        self._check_connected()
        logger.info(f"Wind set to {velocity} m/s")
        return True
    
    # =========================================================================
    # Data Recording
    # =========================================================================
    
    def start_recording(self, output_path: str) -> bool:
        """Start recording sensor data."""
        self._check_connected()
        logger.info(f"Started recording to {output_path}")
        return True
    
    def stop_recording(self) -> bool:
        """Stop recording."""
        self._check_connected()
        logger.info("Stopped recording")
        return True
    
    # =========================================================================
    # Internal
    # =========================================================================
    
    def _check_connected(self) -> None:
        """Raise exception if not connected."""
        if not self._connected:
            raise RuntimeError("Not connected to simulation. Call connect() first.")
    
    def __enter__(self) -> 'AeroSimXClient':
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.disconnect()
