import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('distortion_corrector')

    # Declare arguments
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU for rotation correction'
    )

    use_odom_arg = DeclareLaunchArgument(
        'use_odom',
        default_value='true',
        description='Use Odometry for translation correction'
    )

    scan_duration_arg = DeclareLaunchArgument(
        'scan_duration',
        default_value='0.1',
        description='LiDAR scan duration in seconds'
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
                'use_imu': LaunchConfiguration('use_imu'),
                'use_odom': LaunchConfiguration('use_odom'),
                'scan_duration': LaunchConfiguration('scan_duration'),
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
        use_imu_arg,
        use_odom_arg,
        scan_duration_arg,
        distortion_corrector_node,
    ])
