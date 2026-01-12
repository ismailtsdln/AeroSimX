"""
ROS2 Launch file for AeroSimX Bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for AeroSimX bridge."""
    
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='AeroSimX server host'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='41451',
        description='AeroSimX server port'
    )
    
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='drone1',
        description='Name of the vehicle in simulation'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='100.0',
        description='Update rate in Hz'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )
    
    # AeroSimX Bridge Node
    bridge_node = Node(
        package='aerosimx_bridge',
        executable='aerosimx_bridge_node',
        name='aerosimx_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'update_rate': LaunchConfiguration('update_rate'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
        remappings=[
            ('aerosimx/cmd_vel', 'cmd_vel'),
            ('aerosimx/odom', 'odom'),
        ]
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        vehicle_name_arg,
        update_rate_arg,
        publish_tf_arg,
        bridge_node,
    ])
