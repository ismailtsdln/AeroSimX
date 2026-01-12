"""
Tests for AeroSimX sensors
"""

import pytest
import numpy as np

from pyaerosimx.sensors import (
    Lidar, Camera, Imu, Gps, Radar,
    PointCloud, ImageData, ImuData, GpsData, RadarData
)


class TestLidar:
    """Tests for Lidar sensor."""
    
    def test_default_config(self):
        lidar = Lidar("test_lidar")
        assert lidar.name == "test_lidar"
        assert lidar.channels == 16
        assert lidar.range_max == 100.0
    
    def test_custom_config(self):
        lidar = Lidar(
            "custom_lidar",
            channels=32,
            points_per_channel=2048,
            range_max=150.0
        )
        assert lidar.channels == 32
        assert lidar.points_per_channel == 2048
        assert lidar.range_max == 150.0
    
    def test_get_point_cloud(self):
        lidar = Lidar("test_lidar")
        pc = lidar.get_point_cloud()
        
        assert isinstance(pc, PointCloud)
        assert hasattr(pc, 'points')
        assert hasattr(pc, 'intensities')
    
    def test_get_points_numpy(self):
        lidar = Lidar("test_lidar")
        points = lidar.get_points_numpy()
        
        assert isinstance(points, np.ndarray)


class TestCamera:
    """Tests for Camera sensor."""
    
    def test_default_config(self):
        camera = Camera("test_camera")
        assert camera.name == "test_camera"
        assert camera.width == 640
        assert camera.height == 480
    
    def test_custom_config(self):
        camera = Camera(
            "hd_camera",
            width=1920,
            height=1080,
            fov=120.0
        )
        assert camera.width == 1920
        assert camera.height == 1080
        assert camera.fov == 120.0
    
    def test_get_image(self):
        camera = Camera("test_camera", width=640, height=480)
        img = camera.get_image()
        
        assert isinstance(img, ImageData)
        assert img.width == 640
        assert img.height == 480
    
    def test_get_image_numpy(self):
        camera = Camera("test_camera", width=640, height=480)
        arr = camera.get_image_numpy()
        
        assert isinstance(arr, np.ndarray)
        assert arr.shape[0] == 480
        assert arr.shape[1] == 640


class TestImu:
    """Tests for IMU sensor."""
    
    def test_default_config(self):
        imu = Imu("test_imu")
        assert imu.name == "test_imu"
        assert imu.update_rate == 100.0
    
    def test_get_imu_data(self):
        imu = Imu("test_imu")
        data = imu.get_imu_data()
        
        assert isinstance(data, ImuData)
        assert isinstance(data.linear_acceleration, np.ndarray)
        assert isinstance(data.angular_velocity, np.ndarray)
        assert data.linear_acceleration.shape == (3,)
        assert data.angular_velocity.shape == (3,)
    
    def test_get_acceleration(self):
        imu = Imu("test_imu")
        accel = imu.get_acceleration()
        
        assert isinstance(accel, np.ndarray)
        assert accel.shape == (3,)


class TestGps:
    """Tests for GPS sensor."""
    
    def test_default_config(self):
        gps = Gps("test_gps")
        assert gps.name == "test_gps"
        assert gps.update_rate == 1.0
    
    def test_custom_origin(self):
        gps = Gps(
            "test_gps",
            origin_latitude=37.7749,
            origin_longitude=-122.4194
        )
        assert gps.origin_latitude == 37.7749
        assert gps.origin_longitude == -122.4194
    
    def test_get_gps_data(self):
        gps = Gps("test_gps")
        data = gps.get_gps_data()
        
        assert isinstance(data, GpsData)
        assert hasattr(data, 'latitude')
        assert hasattr(data, 'longitude')
        assert hasattr(data, 'altitude')
    
    def test_get_position(self):
        gps = Gps("test_gps", origin_latitude=40.0, origin_longitude=-74.0)
        lat, lon, alt = gps.get_position()
        
        assert lat == 40.0
        assert lon == -74.0


class TestRadar:
    """Tests for Radar sensor."""
    
    def test_default_config(self):
        radar = Radar("test_radar")
        assert radar.name == "test_radar"
        assert radar.range_max == 200.0
    
    def test_get_detections(self):
        radar = Radar("test_radar")
        detections = radar.get_detections()
        
        assert isinstance(detections, list)
    
    def test_get_num_detections(self):
        radar = Radar("test_radar")
        count = radar.get_num_detections()
        
        assert isinstance(count, int)
        assert count >= 0


class TestSensorCallbacks:
    """Tests for sensor callback functionality."""
    
    def test_register_callback(self):
        lidar = Lidar("test_lidar")
        
        callback_data = []
        
        def callback(data):
            callback_data.append(data)
        
        lidar.register_callback(callback)
        
        # Trigger callback manually
        lidar._notify_callbacks(lidar.get_point_cloud())
        
        assert len(callback_data) == 1
    
    def test_clear_callbacks(self):
        camera = Camera("test_camera")
        
        camera.register_callback(lambda x: None)
        camera.register_callback(lambda x: None)
        
        camera.clear_callbacks()
        
        # Should not raise
        camera._notify_callbacks(camera.get_image())
