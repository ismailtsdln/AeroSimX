#!/usr/bin/env python3
"""
AeroSimX - Dataset Generation Example

Demonstrates generating training datasets for computer vision and RL.
"""

from pathlib import Path
from pyaerosimx import AeroSimXClient
from pyaerosimx.sensors import Lidar, Camera
from pyaerosimx.datasets import DataRecorder, RecordingConfig


def main():
    print("=" * 60)
    print("AeroSimX - Dataset Generation Example")
    print("=" * 60)
    
    # Create output directory
    output_dir = Path("./dataset_output")
    output_dir.mkdir(exist_ok=True)
    
    # Configure recorder
    recording_config = RecordingConfig(
        output_path=str(output_dir),
        format="json",
        record_images=True,
        record_depth=True,
        record_lidar=True,
        record_imu=True,
        record_gps=True,
        record_vehicle_state=True,
        max_frames=500
    )
    
    recorder = DataRecorder(recording_config)
    
    # Connect to simulation
    client = AeroSimXClient()
    client.connect()
    print("✓ Connected to simulation")
    
    # Spawn drone with sensors
    drone = client.spawn_multirotor("drone1", position=(0, 0, 5))
    drone.attach_sensor(Lidar("lidar", channels=32))
    drone.attach_sensor(Camera("camera_rgb", width=1280, height=720))
    drone.attach_sensor(Camera("camera_depth", width=640, height=480, image_type="depth"))
    print("✓ Spawned drone with sensors")
    
    # Arm and hover
    drone.arm()
    drone.hover()
    
    # Start recording
    recorder.start()
    print("✓ Started recording")
    
    # Fly a pattern while recording
    waypoints = [
        (10, 0, 5),
        (10, 10, 8),
        (0, 10, 6),
        (-10, 10, 10),
        (-10, 0, 5),
        (-10, -10, 8),
        (0, -10, 5),
        (10, -10, 6),
        (0, 0, 5),
    ]
    
    frame_count = 0
    
    for i, (x, y, z) in enumerate(waypoints):
        print(f"\n▶ Flying to waypoint {i+1}/{len(waypoints)}: ({x}, {y}, {z})")
        drone.move_to_position(x, y, z)
        
        # Simulate flight with data capture
        for step in range(500):
            client.step(1)
            
            # Record every 10th step
            if step % 10 == 0 and frame_count < recording_config.max_frames:
                # Get sensor data
                lidar = drone.get_sensor("lidar")
                camera_rgb = drone.get_sensor("camera_rgb")
                camera_depth = drone.get_sensor("camera_depth")
                
                # Record frame
                recorder.record_frame(
                    timestamp=client.get_simulation_time(),
                    vehicle_state={
                        "position": drone.get_position(),
                        "orientation": drone.get_orientation(),
                        "velocity": drone.get_velocity(),
                    },
                    images={
                        "rgb": camera_rgb.get_image_numpy(),
                    },
                    depth=camera_depth.get_depth_numpy(),
                    lidar=lidar.get_points_numpy(),
                )
                frame_count += 1
                
                if frame_count % 50 == 0:
                    print(f"    Recorded {frame_count} frames...")
    
    # Stop recording
    recorder.stop()
    print(f"\n✓ Recording complete: {frame_count} frames")
    
    # Export dataset
    print("\n▶ Exporting dataset...")
    
    # JSON format
    json_path = recorder.export()
    print(f"  ✓ JSON: {json_path}")
    
    # COCO format
    coco_path = str(output_dir / "coco_annotations.json")
    recorder.export_coco(coco_path)
    print(f"  ✓ COCO: {coco_path}")
    
    # KITTI format
    kitti_dir = str(output_dir / "kitti")
    recorder.export_kitti(kitti_dir)
    print(f"  ✓ KITTI: {kitti_dir}")
    
    # Land and cleanup
    drone.land()
    client.step(3000)
    drone.disarm()
    client.disconnect()
    
    print("\n" + "=" * 60)
    print("Dataset generation complete!")
    print(f"Output directory: {output_dir.absolute()}")
    print("=" * 60)


if __name__ == "__main__":
    main()
