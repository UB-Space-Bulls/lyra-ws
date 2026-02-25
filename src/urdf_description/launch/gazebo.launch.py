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

    # Launch Gazebo Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
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
            '/model/rover/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rover/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/zed2/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed2/point_cloud/cloud_registered@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/zed2/rgb/image_rect_color@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        remappings=[
            ('/model/rover/odometry', '/odom'),
            ('/model/rover/tf', '/gz_tf'),
            ('/zed2/depth/image_raw', '/zed2/depth/depth_registered'),
            ('/model/rover/joint_state', '/joint_states'),
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


    # ZED2 + RTAB-Map SLAM
    zed_rtab = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'zed_rtab.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
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
        TimerAction(period=10.0, actions=[zed_rtab]),
        TimerAction(period=12.0, actions=[nav2]),
        tf_relay,
    ])