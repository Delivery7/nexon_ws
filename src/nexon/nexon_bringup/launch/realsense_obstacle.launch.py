from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pointcloud → LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan',
            remappings=[
                ('cloud_in', '/camera/depth/color/points'),
                ('scan', '/camera/scan')
            ],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': 0.05,
                'max_height': 1.2,
                'range_min': 0.2,
                'range_max': 3.0,
                'use_inf': True,
                'output_frame_id': 'camera_link'
            }]
        )
    ])
