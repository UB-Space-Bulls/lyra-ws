import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    # ─── Launch Arguments ─────────────────────────────────────────────────────

    use_sim_time   = LaunchConfiguration('use_sim_time')
    autostart      = LaunchConfiguration('autostart')
    use_rviz       = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 for visualization'
    )

    # ─── Config / URDF paths ──────────────────────────────────────────────────

    pkg_share = FindPackageShare('bringup')
    urdf_pkg = get_package_share_directory('urdf_description')

    nav2_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'nav2bringup.rviz'])
    urdf_path        = os.path.join(urdf_pkg, 'urdf', 'urdf.xacro')  # adjust filenames
    apriltag_params_file = PathJoinSubstitution([pkg_share, 'config', 'apriltag.yaml'])

    # ─── ZED + RTAB-Map (your existing launch) ────────────────────────────────

    zed_rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'zed_rtab.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    apriltag_node = Node(       #change 5
    package='apriltag_ros',
    executable='apriltag_node',
    name='apriltag',
    remappings=[
        ('image_rect', '/zed2/zed_node/rgb/image_rect_color'),
        ('camera_info', '/zed2/zed_node/rgb/camera_info'),
    ],
    parameters=[apriltag_params_file],
    extra_arguments=[{'use_intra_process_comms': True}],
    ros_arguments=[
        '--ros-args',
        '--param', 'image_transport:=raw',
        '--qos-profile-overrides-path', PathJoinSubstitution([pkg_share, 'config', 'apriltag_qos.yaml'])
    ]
)

    # ─── Robot State Publisher ────────────────────────────────────────────────
    # Publishes URDF-derived transforms (base_link → sensor frames).
    # Required so RTAB-Map and Nav2 can look up camera/IMU positions.

    with open('/mnt/user-data/uploads/', 'r') if False else open('/dev/null') as f:
        robot_description = ''

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_path).read()
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    #Deleted odom tf publisher ( change 1 )

    # ─── Nav2 Bringup ─────────────────────────────────────────────────────────

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time':    use_sim_time,
            'autostart':       autostart,
            'params_file':     nav2_params_file,
            # We do NOT pass map_subscribe_transient_local here —
            # it's handled per-layer in nav2_params.yaml
            'use_lifecycle_mgr': 'true',
        }.items()
    )

    nav2tag_pub = Node(
        package='bringup',
        executable='nav2tag',
        name='nav2tag'
    )

    # ─── Waypoint Follower ────────────────────────────────────────────────────
    # nav2_bringup's navigation_launch.py includes this, but listed explicitly
    # here for clarity. Remove if you get duplicate node warnings.

    # ─── RViz2 (optional) ─────────────────────────────────────────────────────

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ─────────────────────────────────────────────────────────────────────────

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,
        
        
        TimerAction(period=2.0,actions=[robot_state_publisher,joint_state_publisher]),

        # 2. start zed rtab stack
        TimerAction(period=6.0,actions=[zed_rtabmap_launch]),
        TimerAction(period=7.0,actions=[apriltag_node]),

        # 3. Start Nav2 stack
        TimerAction(period=10.0,actions=[nav2_bringup_launch]),
        TimerAction(period=14.0,actions=[nav2tag_pub]),

        # 4. Optional RViz
        rviz_node,
    ])