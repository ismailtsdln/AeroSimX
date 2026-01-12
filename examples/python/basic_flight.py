#!/usr/bin/env python3
"""
AeroSimX - Basic Flight Example

Demonstrates basic drone takeoff, waypoint navigation, and landing.
"""

from pyaerosimx import AeroSimXClient
from pyaerosimx.sensors import Lidar, Camera, Imu, Gps


def main():
    print("=" * 60)
    print("AeroSimX - Basic Flight Example")
    print("=" * 60)
    
    # Create client and connect
    client = AeroSimXClient()
    client.connect()
    print("✓ Connected to simulation")
    
    # Spawn a multirotor drone
    drone = client.spawn_multirotor(
        name="drone1",
        position=(0, 0, 0.5),  # Start just above ground
        mass=1.5
    )
    print(f"✓ Spawned drone '{drone.name}'")
    
    # Attach sensors
    drone.attach_sensor(Lidar("lidar_front", channels=16, range_max=100))
    drone.attach_sensor(Camera("camera_front", width=640, height=480))
    drone.attach_sensor(Imu("imu"))
    drone.attach_sensor(Gps("gps"))
    print("✓ Attached sensors")
    
    # Arm the drone
    drone.arm()
    print("✓ Motors armed")
    
    # Takeoff to 10 meters
    print("\n▶ Taking off to 10m altitude...")
    drone.takeoff(altitude=10.0)
    
    # Run simulation until at altitude
    for _ in range(5000):
        client.step(1)
        if drone.altitude >= 9.5:
            break
    
    print(f"  Current altitude: {drone.altitude:.2f}m")
    
    # Navigate to waypoints
    waypoints = [
        (10, 0, 10),
        (10, 10, 12),
        (0, 10, 10),
        (0, 0, 10),  # Return to start
    ]
    
    print("\n▶ Navigating waypoints...")
    for i, (x, y, z) in enumerate(waypoints):
        print(f"  → Waypoint {i+1}: ({x}, {y}, {z})")
        drone.move_to_position(x, y, z)
        
        # Fly to waypoint
        for _ in range(3000):
            client.step(1)
            pos = drone.get_position()
            dist = ((pos[0]-x)**2 + (pos[1]-y)**2 + (pos[2]-z)**2) ** 0.5
            if dist < 0.5:
                break
        
        print(f"    ✓ Reached waypoint {i+1}")
    
    # Get sensor data
    print("\n▶ Reading sensors...")
    
    lidar = drone.get_sensor("lidar_front")
    pc = lidar.get_point_cloud()
    print(f"  Lidar: {pc.num_points} points")
    
    camera = drone.get_sensor("camera_front")
    img = camera.get_image()
    print(f"  Camera: {img.width}x{img.height}")
    
    imu = drone.get_sensor("imu")
    imu_data = imu.get_imu_data()
    print(f"  IMU: accel={imu_data.linear_acceleration}")
    
    gps = drone.get_sensor("gps")
    gps_data = gps.get_gps_data()
    print(f"  GPS: lat={gps_data.latitude:.6f}, lon={gps_data.longitude:.6f}")
    
    # Land
    print("\n▶ Landing...")
    drone.land()
    
    for _ in range(5000):
        client.step(1)
        if drone.is_landed:
            break
    
    print("  ✓ Landed")
    
    # Disarm
    drone.disarm()
    print("✓ Motors disarmed")
    
    # Disconnect
    client.disconnect()
    print("\n✓ Simulation complete!")
    
    # Print final state
    state = drone.get_state()
    print(f"\nFinal state:")
    print(f"  Position: {drone.get_position()}")
    print(f"  Battery: {drone.battery_percentage:.1f}%")


if __name__ == "__main__":
    main()
