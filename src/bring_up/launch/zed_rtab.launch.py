import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ─── Launch Arguments ─────────────────────────────────────────────────────

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )

    # ─── Config file paths ────────────────────────────────────────────────────
       
    pkg_share = FindPackageShare('bring_up')

    rtabmap_params = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.yaml'])
    zed_params     = PathJoinSubstitution([pkg_share, 'config', 'zed2.yaml'])

    # ─── ZED2 Camera Node ─────────────────────────────────────────────────────

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name':  'zed2',
            'config_path':  zed_params,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ─── RTAB-Map Stereo SLAM Node ────────────────────────────────────────────

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Stereo image topics from ZED2 wrapper
            ('rgb/image',        '/zed2/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info',  '/zed2/zed_node/rgb/color/rect/camera_info'),
            # Depth image
            ('depth/image',      '/zed2/zed_node/depth/depth_registered'),
            # Odometry from ZED2 positional tracking
            ('odom',             '/zed2/zed_node/odom'),
            # IMU data from ZED2
            ('imu',              '/zed2/zed_node/imu/data'),
        ],
        arguments=['--delete_db_on_start'],  # remove this to persist the map between runs
    )

    # ─────────────────────────────────────────────────────────────────────────

    return LaunchDescription([
        declare_use_sim_time,
        zed_wrapper_launch,
        rtabmap_node,
    ])
