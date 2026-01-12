"""
AeroSimX Vehicle Classes

Vehicle implementations for different platform types.
"""

from __future__ import annotations

import logging
from typing import Optional, List, Dict, Any, Tuple, TYPE_CHECKING
from dataclasses import dataclass, field
from enum import Enum, auto
from abc import ABC, abstractmethod

import numpy as np

if TYPE_CHECKING:
    from pyaerosimx.sensors import Sensor

from pyaerosimx.client import (
    Vector3, Quaternion, Pose, Twist, VehicleState,
    MultirotorControls, CarControls
)

logger = logging.getLogger(__name__)


class FlightMode(Enum):
    """Multirotor flight modes."""
    IDLE = auto()
    ARMED = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MANUAL = auto()
    VELOCITY = auto()
    POSITION = auto()
    LANDING = auto()
    RETURN = auto()
    EMERGENCY = auto()


class Vehicle(ABC):
    """Base class for all vehicles."""
    
    def __init__(
        self,
        name: str,
        initial_pose: Pose,
        mass: float = 1.0,
        **kwargs
    ):
        self.name = name
        self.initial_pose = initial_pose
        self.mass = mass
        
        self._state = VehicleState(pose=initial_pose)
        self._sensors: Dict[str, 'Sensor'] = {}
        
    @property
    def state(self) -> VehicleState:
        """Get current vehicle state."""
        return self._state
    
    @property
    def position(self) -> Vector3:
        """Get current position."""
        return self._state.pose.position
    
    @property
    def velocity(self) -> Vector3:
        """Get current linear velocity."""
        return self._state.velocity.linear
    
    def get_state(self) -> VehicleState:
        """Get full vehicle state."""
        return self._state
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get position as tuple."""
        p = self._state.pose.position
        return (p.x, p.y, p.z)
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """Get orientation as Euler angles (roll, pitch, yaw)."""
        euler = self._state.pose.orientation.to_euler()
        return (euler.x, euler.y, euler.z)
    
    def get_velocity(self) -> Tuple[float, float, float]:
        """Get velocity as tuple."""
        v = self._state.velocity.linear
        return (v.x, v.y, v.z)
    
    def teleport(
        self,
        position: Tuple[float, float, float],
        orientation: Optional[Tuple[float, float, float]] = None
    ) -> None:
        """
        Teleport vehicle to new pose.
        
        Args:
            position: Target position (x, y, z)
            orientation: Target orientation (roll, pitch, yaw) in radians
        """
        self._state.pose.position = Vector3(*position)
        if orientation is not None:
            self._state.pose.orientation = Quaternion.from_euler(*orientation)
        self._state.velocity = Twist()
    
    def attach_sensor(self, sensor: 'Sensor', relative_pose: Optional[Pose] = None) -> None:
        """Attach a sensor to the vehicle."""
        self._sensors[sensor.name] = sensor
        logger.info(f"Attached sensor '{sensor.name}' to vehicle '{self.name}'")
    
    def get_sensor(self, name: str) -> Optional['Sensor']:
        """Get attached sensor by name."""
        return self._sensors.get(name)
    
    def get_sensors(self) -> List['Sensor']:
        """Get all attached sensors."""
        return list(self._sensors.values())
    
    def reset(self) -> None:
        """Reset vehicle to initial state."""
        self._state = VehicleState(pose=self.initial_pose)
    
    @abstractmethod
    def _update(self, dt: float) -> None:
        """Update vehicle state (called by simulation)."""
        pass


class Multirotor(Vehicle):
    """
    Multirotor (drone) vehicle.
    
    Supports various control modes including manual control,
    velocity control, and position control.
    
    Example:
        >>> drone = client.spawn_multirotor("drone1", position=(0, 0, 1))
        >>> drone.arm()
        >>> drone.takeoff(altitude=10)
        >>> drone.move_to_position(10, 0, 10)
        >>> drone.hover()
        >>> drone.land()
    """
    
    def __init__(
        self,
        name: str,
        initial_pose: Pose,
        mass: float = 1.5,
        num_motors: int = 4,
        arm_length: float = 0.25,
        **kwargs
    ):
        super().__init__(name, initial_pose, mass, **kwargs)
        
        self.num_motors = num_motors
        self.arm_length = arm_length
        
        self._flight_mode = FlightMode.IDLE
        self._is_armed = False
        self._is_landed = True
        self._battery_percentage = 100.0
        
        self._motor_speeds = [0.0] * num_motors
        self._controls = MultirotorControls()
        
        self._position_setpoint = Vector3()
        self._velocity_setpoint = Vector3()
        self._yaw_setpoint = 0.0
    
    @property
    def is_armed(self) -> bool:
        """Check if motors are armed."""
        return self._is_armed
    
    @property
    def is_landed(self) -> bool:
        """Check if vehicle is on ground."""
        return self._is_landed
    
    @property
    def flight_mode(self) -> FlightMode:
        """Get current flight mode."""
        return self._flight_mode
    
    @property
    def battery_percentage(self) -> float:
        """Get remaining battery percentage."""
        return self._battery_percentage
    
    @property
    def altitude(self) -> float:
        """Get current altitude (z coordinate)."""
        return self._state.pose.position.z
    
    def arm(self) -> bool:
        """Arm the motors."""
        if self._is_landed:
            self._is_armed = True
            self._flight_mode = FlightMode.ARMED
            logger.info(f"Multirotor '{self.name}' armed")
            return True
        return False
    
    def disarm(self) -> bool:
        """Disarm the motors."""
        self._is_armed = False
        self._flight_mode = FlightMode.IDLE
        self._motor_speeds = [0.0] * self.num_motors
        logger.info(f"Multirotor '{self.name}' disarmed")
        return True
    
    def takeoff(self, altitude: float, velocity: float = 1.0) -> bool:
        """
        Take off to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            velocity: Ascent velocity in m/s
        """
        if not self._is_armed:
            self.arm()
        
        self._is_landed = False
        self._position_setpoint = Vector3(
            self._state.pose.position.x,
            self._state.pose.position.y,
            altitude
        )
        self._flight_mode = FlightMode.TAKEOFF
        logger.info(f"Multirotor '{self.name}' taking off to {altitude}m")
        return True
    
    def land(self, velocity: float = 0.5) -> bool:
        """
        Land at current position.
        
        Args:
            velocity: Descent velocity in m/s
        """
        self._flight_mode = FlightMode.LANDING
        logger.info(f"Multirotor '{self.name}' landing")
        return True
    
    def hover(self) -> bool:
        """Hover at current position."""
        if not self._is_armed:
            return False
        
        self._position_setpoint = self._state.pose.position
        self._yaw_setpoint = self._state.pose.orientation.to_euler().z
        self._flight_mode = FlightMode.HOVER
        return True
    
    def move_by_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0
    ) -> bool:
        """
        Move with velocity command.
        
        Args:
            vx: Forward velocity in m/s
            vy: Right velocity in m/s
            vz: Up velocity in m/s
            yaw_rate: Yaw rate in rad/s
        """
        if not self._is_armed:
            return False
        
        self._velocity_setpoint = Vector3(vx, vy, vz)
        self._flight_mode = FlightMode.VELOCITY
        return True
    
    def move_to_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        velocity: float = 5.0
    ) -> bool:
        """
        Move to target position.
        
        Args:
            x: Target X position in meters
            y: Target Y position in meters
            z: Target Z position in meters
            yaw: Target heading in radians (None = maintain current)
            velocity: Maximum velocity in m/s
        """
        if not self._is_armed:
            return False
        
        self._position_setpoint = Vector3(x, y, z)
        if yaw is not None:
            self._yaw_setpoint = yaw
        self._flight_mode = FlightMode.POSITION
        return True
    
    def set_controls(self, controls: MultirotorControls) -> None:
        """Set direct control inputs."""
        self._controls = controls
        self._flight_mode = FlightMode.MANUAL
    
    def set_motor_speeds(self, speeds: List[float]) -> None:
        """Set motor speeds directly (normalized 0-1)."""
        self._motor_speeds = [max(0, min(1, s)) for s in speeds[:self.num_motors]]
    
    def get_motor_speeds(self) -> List[float]:
        """Get current motor speeds."""
        return self._motor_speeds.copy()
    
    def reset(self) -> None:
        """Reset to initial state."""
        super().reset()
        self._is_armed = False
        self._is_landed = True
        self._battery_percentage = 100.0
        self._flight_mode = FlightMode.IDLE
        self._motor_speeds = [0.0] * self.num_motors
    
    def _update(self, dt: float) -> None:
        """Update vehicle state."""
        if not self._is_armed:
            return
        
        # Simplified dynamics for Python simulation
        # In production, this calls the C++ core
        
        if self._flight_mode == FlightMode.TAKEOFF:
            error_z = self._position_setpoint.z - self._state.pose.position.z
            vel_z = min(2.0, max(-2.0, error_z * 2.0))
            self._state.velocity.linear.z = vel_z
            
            if abs(error_z) < 0.1:
                self._flight_mode = FlightMode.HOVER
        
        elif self._flight_mode == FlightMode.HOVER:
            # Hold position with simple P controller
            error = self._position_setpoint - self._state.pose.position
            self._state.velocity.linear = error * 2.0
        
        elif self._flight_mode == FlightMode.POSITION:
            error = self._position_setpoint - self._state.pose.position
            dist = error.magnitude()
            if dist > 0.1:
                direction = error.normalized()
                speed = min(5.0, dist * 2.0)
                self._state.velocity.linear = direction * speed
            else:
                self._state.velocity.linear = Vector3()
                self._flight_mode = FlightMode.HOVER
        
        elif self._flight_mode == FlightMode.VELOCITY:
            self._state.velocity.linear = self._velocity_setpoint
        
        elif self._flight_mode == FlightMode.LANDING:
            self._state.velocity.linear = Vector3(0, 0, -0.5)
            if self._state.pose.position.z <= 0.1:
                self._state.pose.position.z = 0
                self._state.velocity.linear = Vector3()
                self._is_landed = True
                self._flight_mode = FlightMode.ARMED
        
        # Integrate position
        self._state.pose.position = self._state.pose.position + self._state.velocity.linear * dt
        
        # Simple motor speed simulation
        thrust_level = 0.5 if not self._is_landed else 0.0
        self._motor_speeds = [thrust_level] * self.num_motors
        
        # Battery drain
        self._battery_percentage -= 0.001


class Car(Vehicle):
    """
    Ground vehicle (car).
    
    Supports driving with throttle, steering, and braking.
    
    Example:
        >>> car = client.spawn_car("car1", position=(0, 0, 0))
        >>> car.set_controls(throttle=0.5, steering=0.1)
        >>> car.get_speed()
    """
    
    def __init__(
        self,
        name: str,
        initial_pose: Pose,
        mass: float = 1500.0,
        max_speed: float = 50.0,
        **kwargs
    ):
        super().__init__(name, initial_pose, mass, **kwargs)
        
        self.max_speed = max_speed
        
        self._controls = CarControls()
        self._speed = 0.0
        self._steering_angle = 0.0
        self._gear = 0
        self._engine_rpm = 800.0
    
    @property
    def speed(self) -> float:
        """Get current speed in m/s."""
        return self._speed
    
    @property
    def steering_angle(self) -> float:
        """Get current steering angle in radians."""
        return self._steering_angle
    
    @property
    def gear(self) -> int:
        """Get current gear."""
        return self._gear
    
    def get_speed(self) -> float:
        """Get speed in m/s."""
        return self._speed
    
    def get_speed_kmh(self) -> float:
        """Get speed in km/h."""
        return self._speed * 3.6
    
    def set_controls(
        self,
        throttle: float = 0.0,
        steering: float = 0.0,
        brake: float = 0.0,
        handbrake: bool = False
    ) -> None:
        """
        Set driving controls.
        
        Args:
            throttle: Throttle input (0 to 1)
            steering: Steering input (-1=left to 1=right)
            brake: Brake input (0 to 1)
            handbrake: Handbrake engaged
        """
        self._controls = CarControls(
            throttle=max(0, min(1, throttle)),
            steering=max(-1, min(1, steering)),
            brake=max(0, min(1, brake)),
            handbrake=handbrake,
            gear=self._gear
        )
    
    def set_throttle(self, throttle: float) -> None:
        """Set throttle (0 to 1)."""
        self._controls.throttle = max(0, min(1, throttle))
    
    def set_steering(self, steering: float) -> None:
        """Set steering (-1 to 1)."""
        self._controls.steering = max(-1, min(1, steering))
    
    def set_brake(self, brake: float) -> None:
        """Set brake (0 to 1)."""
        self._controls.brake = max(0, min(1, brake))
    
    def shift_up(self) -> None:
        """Shift gear up."""
        self._gear = min(6, self._gear + 1)
    
    def shift_down(self) -> None:
        """Shift gear down."""
        self._gear = max(-1, self._gear - 1)
    
    def reset(self) -> None:
        """Reset to initial state."""
        super().reset()
        self._speed = 0.0
        self._steering_angle = 0.0
        self._gear = 0
        self._controls = CarControls()
    
    def _update(self, dt: float) -> None:
        """Update vehicle state."""
        # Simplified car dynamics
        throttle = self._controls.throttle
        brake = self._controls.brake
        steering = self._controls.steering
        
        # Acceleration
        max_accel = 10.0  # m/s^2
        accel = throttle * max_accel - brake * 20.0
        
        # Speed update
        self._speed += accel * dt
        self._speed = max(0, min(self.max_speed, self._speed))
        
        if self._controls.handbrake:
            self._speed *= 0.9
        
        # Steering
        max_steer = 0.6  # radians
        self._steering_angle = steering * max_steer
        
        # Update pose
        euler = self._state.pose.orientation.to_euler()
        yaw = euler.z
        
        # Simple bicycle model
        wheelbase = 2.5
        if abs(self._speed) > 0.1:
            angular_vel = (self._speed / wheelbase) * np.tan(self._steering_angle)
            yaw += angular_vel * dt
        
        # Update position
        self._state.pose.position.x += self._speed * np.cos(yaw) * dt
        self._state.pose.position.y += self._speed * np.sin(yaw) * dt
        self._state.pose.orientation = Quaternion.from_euler(0, 0, yaw)
        
        # Update velocity
        self._state.velocity.linear.x = self._speed * np.cos(yaw)
        self._state.velocity.linear.y = self._speed * np.sin(yaw)
        self._state.velocity.angular.z = (self._speed / wheelbase) * np.tan(self._steering_angle)
