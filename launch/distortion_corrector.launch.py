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

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame for transformation'
    )

    # Parameters
    params_file = os.path.join(pkg_dir, 'config', 'distortion_corrector.yaml')

    # Node
    distortion_corrector_node = Node(
        package='distortion_corrector',
        executable='distortion_corrector_main',
        name='distortion_corrector',
        parameters=[
            params_file,
            {
                'use_data_source': LaunchConfiguration('use_data_source'),
                'base_frame': LaunchConfiguration('base_frame'),
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

    return LaunchDescription([
        use_data_source_arg,
        base_frame_arg,
        distortion_corrector_node,
    ])
