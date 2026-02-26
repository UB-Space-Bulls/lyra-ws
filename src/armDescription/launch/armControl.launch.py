import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "armDescription"
XACRO_FILE   = "arm.xacro"


def generate_launch_description():

    pkg_share  = get_package_share_directory(PACKAGE_NAME)
    xacro_path = os.path.join(pkg_share, "urdf", XACRO_FILE)
    ctrl_yaml  = os.path.join(pkg_share, "config", "armcontroller.yaml")

    # Process xacro then force-inject the yaml path (xacro args don't
    # propagate into included files so $(arg controller_params) comes out empty)
    robot_description = xacro.process_file(
        xacro_path,
        mappings={"controller_params": ctrl_yaml}
    ).toxml().replace(
        "<parameters></parameters>",
        f"<parameters>{ctrl_yaml}</parameters>"
    )

    assert ctrl_yaml in robot_description, \
        f"FATAL: controller yaml not in URDF — check arm.gazebo includes <parameters> tag"
    print(f"\n[arm_sim] yaml confirmed in URDF: {ctrl_yaml}\n")

    set_ign_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.dirname(pkg_share) + ":"
              + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    # ------------------------------------------------------------------ #
    # 1. Robot State Publisher — starts first so RSP is up before Ignition
    #    tries to read robot_description
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ------------------------------------------------------------------ #
    # 2. Gazebo — delayed by 2s to ensure RSP is advertising robot_description
    #    before Ignition's plugin tries to read it
    # ------------------------------------------------------------------ #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    gz_sim_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                TimerAction(period=2.0, actions=[gz_sim])
            ],
        )
    )

    # ------------------------------------------------------------------ #
    # 3. Spawn robot — after Gazebo has had time to start
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_arm",
        output="screen",
        arguments=[
            "-name",  "arm",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
    )

    # ------------------------------------------------------------------ #
    # 4. Bridge
    # ------------------------------------------------------------------ #
    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # ------------------------------------------------------------------ #
    # 5. Controllers — wait 5s after spawn for controller_manager to appear
    # ------------------------------------------------------------------ #
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "velocity_controller",
        ],
        output="screen",
    )

    activate_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=5.0,   # increased from 2s — gives controller_manager time to start
                    actions=[load_joint_state_broadcaster],
                )
            ],
        )
    )

    activate_vel_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_velocity_controller],
        )
    )

    return LaunchDescription([
        set_ign_path,
        robot_state_publisher,
        gz_sim_delayed,         # gz starts 2s after RSP is up
        gz_ros_bridge,
        spawn_robot,
        activate_jsb_after_spawn,
        activate_vel_after_jsb,
    ])