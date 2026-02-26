from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('urdf_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'urdf.xacro')
    world_file = os.path.join(share_dir, 'worlds', 'sea_coast.sdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()


      #nav2
    bringup_dir = get_package_share_directory('bring-up')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': False,
            'qos_overrides./joint_states.subscription.reliability': 'reliable'
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_depth_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   'zed_camera_link',
                   'rover/base_link/zed2_depth'],
        parameters=[{'use_sim_time': True}]
    )

    # Launch Gazebo Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge between Gazebo topics and ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rover/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/zed2/depth/image_raw/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2/depth/image_raw/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed2/depth/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/zed2/depth/image_raw/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2/rgb/image_rect_color@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        remappings=[
            ('/model/rover/tf', '/gz_tf'),
            ('/zed2/depth/image_raw', '/zed2/depth/depth_registered'),
            ('/model/rover/joint_state', '/joint_states'),
            ('/zed2/depth/image_raw/points', '/zed2/point_cloud/cloud_registered'),
        ],
        parameters=[{
            'use_sim_time': True
            }],
        output='screen'
    )
    
    tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_relay',
        arguments=['/gz_tf', '/tf'],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rtab = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': False,        # ← add this
            'subscribe_stereo': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'queue_size': 30,
            'Grid/RayTracing': 'true',
            'Grid/3D': 'true',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.1',
            'GridGlobal/MinSize': '20',
            'Mem/IncrementalMemory': 'true',
            'ICP/PointToPlaneNormalNeighbors': '20',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Grid/NoiseFilteringRadius': '0.2',
            'Grid/NoiseFilteringMinNeighbors': '8',
        }],
        remappings=[
            ('/scan_cloud', '/zed2/point_cloud/cloud_registered'),
            ('/odom', '/odom'),
        ],
        arguments=['--delete_db_on_start'],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items()
    )
    

    return LaunchDescription([
        gazebo,
        TimerAction(period=5.0, actions=[robot_state_publisher_node]),
        TimerAction(period=5.0, actions=[joint_state_publisher_node]),
        TimerAction(period=8.0, actions=[spawn_robot]),
        TimerAction(period=10.0, actions=[bridge]),
        tf_relay,
        static_tf,
        TimerAction(period=12.0, actions=[rtab]),
    ])