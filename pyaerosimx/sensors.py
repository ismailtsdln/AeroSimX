"""
AeroSimX Sensor Classes

Sensor implementations for various modalities.
"""

from __future__ import annotations

import logging
from typing import Optional, List, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum, auto
from abc import ABC, abstractmethod

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class PointCloud:
    """Point cloud data from lidar."""
    points: np.ndarray  # Nx3 array of XYZ points
    intensities: Optional[np.ndarray] = None  # N array of intensities
    ring: Optional[np.ndarray] = None  # N array of ring indices
    timestamp: float = 0.0
    frame_id: str = ""
    
    @property
    def num_points(self) -> int:
        return len(self.points) if self.points is not None else 0


@dataclass
class ImageData:
    """Image data from camera."""
    data: np.ndarray  # HxWxC array
    width: int = 0
    height: int = 0
    channels: int = 3
    timestamp: float = 0.0
    frame_id: str = ""
    
    @property
    def shape(self) -> tuple:
        return self.data.shape if self.data is not None else (0, 0, 0)


@dataclass
class DepthImage:
    """Depth image data."""
    data: np.ndarray  # HxW array of depth values in meters
    width: int = 0
    height: int = 0
    min_depth: float = 0.1
    max_depth: float = 100.0
    timestamp: float = 0.0
    frame_id: str = ""


@dataclass
class ImuData:
    """IMU measurement data."""
    linear_acceleration: np.ndarray  # 3-element array (x, y, z) in m/s^2
    angular_velocity: np.ndarray  # 3-element array (x, y, z) in rad/s
    orientation: Optional[np.ndarray] = None  # 4-element quaternion (w, x, y, z)
    timestamp: float = 0.0
    frame_id: str = ""


@dataclass
class GpsData:
    """GPS/GNSS measurement data."""
    latitude: float = 0.0  # degrees
    longitude: float = 0.0  # degrees
    altitude: float = 0.0  # meters above sea level
    hdop: float = 1.0
    vdop: float = 1.0
    num_satellites: int = 0
    fix_valid: bool = False
    timestamp: float = 0.0


@dataclass
class RadarDetection:
    """Single radar detection."""
    range: float = 0.0  # meters
    azimuth: float = 0.0  # radians
    elevation: float = 0.0  # radians
    velocity: float = 0.0  # m/s (radial)
    rcs: float = 0.0  # dBsm


@dataclass
class RadarData:
    """Radar detection data."""
    detections: List[RadarDetection] = field(default_factory=list)
    timestamp: float = 0.0
    frame_id: str = ""


class Sensor(ABC):
    """Base class for all sensors."""
    
    def __init__(
        self,
        name: str,
        update_rate: float = 10.0,
        enabled: bool = True
    ):
        self.name = name
        self.update_rate = update_rate
        self.enabled = enabled
        self._callbacks: List[Callable] = []
        self._last_update_time = 0.0
    
    @abstractmethod
    def get_data(self) -> Any:
        """Get latest sensor data."""
        pass
    
    def register_callback(self, callback: Callable) -> None:
        """Register callback for new data."""
        self._callbacks.append(callback)
    
    def clear_callbacks(self) -> None:
        """Clear all callbacks."""
        self._callbacks.clear()
    
    def _notify_callbacks(self, data: Any) -> None:
        """Notify all registered callbacks."""
        for callback in self._callbacks:
            try:
                callback(data)
            except Exception as e:
                logger.warning(f"Sensor callback error: {e}")


class Lidar(Sensor):
    """
    Lidar sensor.
    
    Simulates scanning lidar with configurable channels and resolution.
    
    Example:
        >>> lidar = Lidar("lidar_front", channels=16, range_max=100.0)
        >>> drone.attach_sensor(lidar)
        >>> pc = lidar.get_point_cloud()
    """
    
    def __init__(
        self,
        name: str,
        channels: int = 16,
        points_per_channel: int = 1024,
        range_min: float = 0.5,
        range_max: float = 100.0,
        horizontal_fov: float = 360.0,
        vertical_fov_upper: float = 15.0,
        vertical_fov_lower: float = -15.0,
        update_rate: float = 10.0,
        **kwargs
    ):
        super().__init__(name, update_rate)
        
        self.channels = channels
        self.points_per_channel = points_per_channel
        self.range_min = range_min
        self.range_max = range_max
        self.horizontal_fov = horizontal_fov
        self.vertical_fov_upper = vertical_fov_upper
        self.vertical_fov_lower = vertical_fov_lower
        
        self._point_cloud = PointCloud(points=np.zeros((0, 3)))
    
    def get_data(self) -> PointCloud:
        """Get latest point cloud."""
        return self._point_cloud
    
    def get_point_cloud(self) -> PointCloud:
        """Get latest point cloud."""
        return self._point_cloud
    
    def get_points_numpy(self) -> np.ndarray:
        """Get points as Nx3 numpy array."""
        return self._point_cloud.points


class Camera(Sensor):
    """
    Camera sensor.
    
    Supports RGB, depth, and segmentation imaging.
    
    Example:
        >>> camera = Camera("camera_front", width=640, height=480)
        >>> drone.attach_sensor(camera)
        >>> image = camera.get_image()
    """
    
    def __init__(
        self,
        name: str,
        width: int = 640,
        height: int = 480,
        fov: float = 90.0,
        image_type: str = "rgb",
        update_rate: float = 30.0,
        **kwargs
    ):
        super().__init__(name, update_rate)
        
        self.width = width
        self.height = height
        self.fov = fov
        self.image_type = image_type
        
        channels = 3 if image_type in ["rgb", "rgba", "segmentation"] else 1
        self._image = ImageData(
            data=np.zeros((height, width, channels), dtype=np.uint8),
            width=width,
            height=height,
            channels=channels
        )
        self._depth_image = DepthImage(
            data=np.zeros((height, width), dtype=np.float32),
            width=width,
            height=height
        )
    
    def get_data(self) -> ImageData:
        """Get latest image."""
        return self._image
    
    def get_image(self) -> ImageData:
        """Get latest RGB image."""
        return self._image
    
    def get_image_numpy(self) -> np.ndarray:
        """Get image as numpy array."""
        return self._image.data
    
    def get_depth_image(self) -> DepthImage:
        """Get depth image."""
        return self._depth_image
    
    def get_depth_numpy(self) -> np.ndarray:
        """Get depth as numpy array."""
        return self._depth_image.data


class Imu(Sensor):
    """
    Inertial Measurement Unit sensor.
    
    Provides accelerometer and gyroscope measurements.
    
    Example:
        >>> imu = Imu("imu")
        >>> drone.attach_sensor(imu)
        >>> data = imu.get_imu_data()
    """
    
    def __init__(
        self,
        name: str,
        accel_noise_stddev: float = 0.01,
        gyro_noise_stddev: float = 0.001,
        update_rate: float = 100.0,
        **kwargs
    ):
        super().__init__(name, update_rate)
        
        self.accel_noise_stddev = accel_noise_stddev
        self.gyro_noise_stddev = gyro_noise_stddev
        
        self._imu_data = ImuData(
            linear_acceleration=np.array([0.0, 0.0, 9.81]),
            angular_velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )
    
    def get_data(self) -> ImuData:
        """Get latest IMU data."""
        return self._imu_data
    
    def get_imu_data(self) -> ImuData:
        """Get latest IMU measurement."""
        return self._imu_data
    
    def get_acceleration(self) -> np.ndarray:
        """Get linear acceleration."""
        return self._imu_data.linear_acceleration
    
    def get_angular_velocity(self) -> np.ndarray:
        """Get angular velocity."""
        return self._imu_data.angular_velocity


class Gps(Sensor):
    """
    GPS/GNSS sensor.
    
    Provides position in GPS coordinates.
    
    Example:
        >>> gps = Gps("gps", origin_latitude=37.7749)
        >>> drone.attach_sensor(gps)
        >>> data = gps.get_gps_data()
    """
    
    def __init__(
        self,
        name: str,
        origin_latitude: float = 0.0,
        origin_longitude: float = 0.0,
        origin_altitude: float = 0.0,
        horizontal_accuracy: float = 2.5,
        update_rate: float = 1.0,
        **kwargs
    ):
        super().__init__(name, update_rate)
        
        self.origin_latitude = origin_latitude
        self.origin_longitude = origin_longitude
        self.origin_altitude = origin_altitude
        self.horizontal_accuracy = horizontal_accuracy
        
        self._gps_data = GpsData(
            latitude=origin_latitude,
            longitude=origin_longitude,
            altitude=origin_altitude,
            fix_valid=True,
            num_satellites=10
        )
    
    def get_data(self) -> GpsData:
        """Get latest GPS data."""
        return self._gps_data
    
    def get_gps_data(self) -> GpsData:
        """Get latest GPS measurement."""
        return self._gps_data
    
    def get_position(self) -> tuple:
        """Get position as (lat, lon, alt)."""
        return (
            self._gps_data.latitude,
            self._gps_data.longitude,
            self._gps_data.altitude
        )


class Radar(Sensor):
    """
    Radar sensor.
    
    Provides object detections with range and velocity.
    
    Example:
        >>> radar = Radar("radar_front", range_max=200.0)
        >>> car.attach_sensor(radar)
        >>> detections = radar.get_detections()
    """
    
    def __init__(
        self,
        name: str,
        range_min: float = 0.5,
        range_max: float = 200.0,
        horizontal_fov: float = 60.0,
        vertical_fov: float = 20.0,
        update_rate: float = 20.0,
        **kwargs
    ):
        super().__init__(name, update_rate)
        
        self.range_min = range_min
        self.range_max = range_max
        self.horizontal_fov = horizontal_fov
        self.vertical_fov = vertical_fov
        
        self._radar_data = RadarData()
    
    def get_data(self) -> RadarData:
        """Get latest radar data."""
        return self._radar_data
    
    def get_detections(self) -> List[RadarDetection]:
        """Get list of radar detections."""
        return self._radar_data.detections
    
    def get_num_detections(self) -> int:
        """Get number of detections."""
        return len(self._radar_data.detections)
