"""
AeroSimX Dataset Utilities

Tools for recording, exporting, and managing simulation datasets.
"""

from __future__ import annotations

import os
import json
import logging
from typing import Optional, List, Dict, Any, Union
from dataclasses import dataclass, asdict
from pathlib import Path
from datetime import datetime
import struct

import numpy as np

logger = logging.getLogger(__name__)


class DatasetFormat:
    """Supported dataset export formats."""
    COCO = "coco"
    KITTI = "kitti"
    ROSBAG = "rosbag"
    CSV = "csv"
    JSON = "json"
    HDF5 = "hdf5"


@dataclass
class RecordingConfig:
    """Configuration for data recording."""
    output_path: str
    format: str = DatasetFormat.JSON
    record_images: bool = True
    record_depth: bool = True
    record_lidar: bool = True
    record_imu: bool = True
    record_gps: bool = True
    record_vehicle_state: bool = True
    image_format: str = "png"
    compression: bool = True
    max_frames: int = -1  # -1 = unlimited


class DataRecorder:
    """
    Records simulation data for training datasets.
    
    Example:
        >>> recorder = DataRecorder(config)
        >>> recorder.start()
        >>> # ... run simulation ...
        >>> recorder.stop()
        >>> recorder.export('output.json')
    """
    
    def __init__(self, config: RecordingConfig):
        self.config = config
        self._recording = False
        self._frame_count = 0
        self._data_buffer: List[Dict[str, Any]] = []
        self._start_time = 0.0
        
        # Create output directory
        Path(config.output_path).mkdir(parents=True, exist_ok=True)
    
    def start(self) -> None:
        """Start recording."""
        self._recording = True
        self._frame_count = 0
        self._data_buffer.clear()
        self._start_time = datetime.now().timestamp()
        logger.info(f"Started recording to {self.config.output_path}")
    
    def stop(self) -> None:
        """Stop recording."""
        self._recording = False
        logger.info(f"Stopped recording. {self._frame_count} frames captured.")
    
    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._recording
    
    def record_frame(
        self,
        timestamp: float,
        vehicle_state: Optional[Dict] = None,
        images: Optional[Dict[str, np.ndarray]] = None,
        depth: Optional[np.ndarray] = None,
        lidar: Optional[np.ndarray] = None,
        imu: Optional[Dict] = None,
        gps: Optional[Dict] = None,
        **extras
    ) -> None:
        """
        Record a single frame of data.
        
        Args:
            timestamp: Simulation timestamp
            vehicle_state: Vehicle pose and velocity
            images: Dict of camera name -> image array
            depth: Depth image array
            lidar: Point cloud array (Nx3)
            imu: IMU measurements dict
            gps: GPS data dict
            **extras: Additional data to record
        """
        if not self._recording:
            return
        
        if self.config.max_frames > 0 and self._frame_count >= self.config.max_frames:
            return
        
        frame_data = {
            "frame_id": self._frame_count,
            "timestamp": timestamp,
        }
        
        if vehicle_state and self.config.record_vehicle_state:
            frame_data["vehicle_state"] = vehicle_state
        
        if images and self.config.record_images:
            # Save images to files, store paths in frame data
            frame_data["images"] = {}
            for name, img in images.items():
                img_path = self._save_image(img, name, self._frame_count)
                frame_data["images"][name] = img_path
        
        if depth is not None and self.config.record_depth:
            depth_path = self._save_depth(depth, self._frame_count)
            frame_data["depth"] = depth_path
        
        if lidar is not None and self.config.record_lidar:
            lidar_path = self._save_lidar(lidar, self._frame_count)
            frame_data["lidar"] = lidar_path
        
        if imu and self.config.record_imu:
            frame_data["imu"] = imu
        
        if gps and self.config.record_gps:
            frame_data["gps"] = gps
        
        # Add extras
        frame_data.update(extras)
        
        self._data_buffer.append(frame_data)
        self._frame_count += 1
    
    def export(self, output_file: Optional[str] = None) -> str:
        """
        Export recorded data to file.
        
        Args:
            output_file: Output file path (uses config path if None)
            
        Returns:
            Path to exported file
        """
        if output_file is None:
            output_file = os.path.join(
                self.config.output_path,
                f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
        
        export_data = {
            "metadata": {
                "format_version": "1.0",
                "created": datetime.now().isoformat(),
                "frame_count": len(self._data_buffer),
                "config": asdict(self.config)
            },
            "frames": self._data_buffer
        }
        
        with open(output_file, 'w') as f:
            json.dump(export_data, f, indent=2, default=str)
        
        logger.info(f"Exported {len(self._data_buffer)} frames to {output_file}")
        return output_file
    
    def export_coco(self, output_file: str) -> str:
        """Export in COCO format."""
        coco_data = {
            "info": {
                "description": "AeroSimX Simulation Dataset",
                "url": "",
                "version": "1.0",
                "year": datetime.now().year,
                "contributor": "AeroSimX",
                "date_created": datetime.now().isoformat()
            },
            "licenses": [],
            "images": [],
            "annotations": [],
            "categories": []
        }
        
        for i, frame in enumerate(self._data_buffer):
            if "images" in frame:
                for name, path in frame["images"].items():
                    coco_data["images"].append({
                        "id": i,
                        "file_name": path,
                        "width": 640,  # Would be read from actual image
                        "height": 480
                    })
        
        with open(output_file, 'w') as f:
            json.dump(coco_data, f, indent=2)
        
        return output_file
    
    def export_kitti(self, output_dir: str) -> str:
        """Export in KITTI format."""
        os.makedirs(output_dir, exist_ok=True)
        
        # Create KITTI directory structure
        for subdir in ['image_2', 'velodyne', 'calib', 'label_2']:
            os.makedirs(os.path.join(output_dir, subdir), exist_ok=True)
        
        for i, frame in enumerate(self._data_buffer):
            # Copy/move lidar data
            if "lidar" in frame:
                src = frame["lidar"]
                dst = os.path.join(output_dir, 'velodyne', f'{i:06d}.bin')
                # In real implementation, would convert format
            
            # Copy/move images
            if "images" in frame:
                for name, path in frame["images"].items():
                    if name == "front" or "rgb" in name.lower():
                        dst = os.path.join(output_dir, 'image_2', f'{i:06d}.png')
                        # Would copy/convert image
        
        logger.info(f"Exported KITTI format to {output_dir}")
        return output_dir
    
    def _save_image(self, image: np.ndarray, name: str, frame_id: int) -> str:
        """Save image to file and return path."""
        filename = f"images/{name}_{frame_id:06d}.{self.config.image_format}"
        filepath = os.path.join(self.config.output_path, filename)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        # Would use cv2.imwrite or PIL in real implementation
        # For now, just return the path
        return filename
    
    def _save_depth(self, depth: np.ndarray, frame_id: int) -> str:
        """Save depth image to file and return path."""
        filename = f"depth/depth_{frame_id:06d}.npz"
        filepath = os.path.join(self.config.output_path, filename)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        if self.config.compression:
            np.savez_compressed(filepath, depth=depth)
        else:
            np.save(filepath.replace('.npz', '.npy'), depth)
        
        return filename
    
    def _save_lidar(self, points: np.ndarray, frame_id: int) -> str:
        """Save lidar point cloud to file and return path."""
        filename = f"lidar/lidar_{frame_id:06d}.bin"
        filepath = os.path.join(self.config.output_path, filename)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        # Save as binary float32 (KITTI format compatible)
        points.astype(np.float32).tofile(filepath)
        
        return filename


class DatasetLoader:
    """
    Loads simulation datasets for training.
    
    Example:
        >>> loader = DatasetLoader('dataset.json')
        >>> for frame in loader:
        ...     image = frame['images']['front']
        ...     state = frame['vehicle_state']
    """
    
    def __init__(self, dataset_path: str):
        self.dataset_path = dataset_path
        self._data = None
        self._load()
    
    def _load(self) -> None:
        """Load dataset from file."""
        with open(self.dataset_path, 'r') as f:
            self._data = json.load(f)
        
        logger.info(f"Loaded dataset with {len(self._data.get('frames', []))} frames")
    
    @property
    def metadata(self) -> Dict:
        """Get dataset metadata."""
        return self._data.get("metadata", {})
    
    @property
    def frames(self) -> List[Dict]:
        """Get all frames."""
        return self._data.get("frames", [])
    
    def __len__(self) -> int:
        return len(self.frames)
    
    def __getitem__(self, idx: int) -> Dict:
        return self.frames[idx]
    
    def __iter__(self):
        return iter(self.frames)
    
    def get_frame(self, idx: int) -> Dict:
        """Get frame by index."""
        return self.frames[idx]
    
    def get_vehicle_states(self) -> List[Dict]:
        """Get all vehicle states."""
        return [f.get("vehicle_state") for f in self.frames if "vehicle_state" in f]
    
    def get_timestamps(self) -> List[float]:
        """Get all timestamps."""
        return [f.get("timestamp", 0) for f in self.frames]
