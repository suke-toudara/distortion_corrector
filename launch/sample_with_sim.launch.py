"""
Sample launch file showing how to integrate distortion corrector
with other nodes (simulation example)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('distortion_corrector')

    # Declare arguments
    use_data_source_arg = DeclareLaunchArgument(
        'use_data_source',
        default_value='imu',
        description='Data source for correction: imu or odom'
    )

    # Parameters
    params_file = os.path.join(pkg_dir, 'config', 'distortion_corrector.yaml')

    # Distortion Corrector Node
    distortion_corrector_node = Node(
        package='distortion_corrector',
        executable='distortion_corrector_main',
        name='distortion_corrector',
        parameters=[
            params_file,
            {
                'use_data_source': LaunchConfiguration('use_data_source'),
                'base_frame': 'base_link',
            }
        ],
        remappings=[
            ('~/input/pointcloud', '/velodyne_points'),
            ('~/input/imu', '/imu/data'),
            ('~/input/odom', '/odom'),
            ('~/output/pointcloud', '/corrected_points'),
        ],
        output='screen'
    )

    # RViz2 for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'default.rviz')],
        condition=lambda: False  # Set to True to enable RViz
    )

    return LaunchDescription([
        use_data_source_arg,
        distortion_corrector_node,
        # rviz_node,  # Uncomment to launch RViz
    ])
