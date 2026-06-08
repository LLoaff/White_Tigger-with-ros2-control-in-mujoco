from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    control_pkg_path = os.path.join(get_package_share_directory('control_pkg'))
    # mujoco_xml = os.path.join(get_package_share_directory('descrip_pkg'))
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("descrip_pkg"),
                    "urdf",
                    "total.urdf.xacro"
                ]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("control_pkg"), "rviz_config", "rviz.rviz"]
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("control_pkg"),
            "controller_config",
            "config.yaml",
        ]
    )
    mujoco_plugins_file = PathJoinSubstitution([control_pkg_path, "controller_config", "mujoco_ros2_control_plugins.yaml"])

    control_node = Node(
        # Specify the control node from this package!
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            robot_controllers,
            mujoco_plugins_file
        ],
        remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file",robot_controllers],
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--param-file",robot_controllers],
    )
    mit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mit_controller","--param-file",robot_controllers],
    )

    start_node = Node(
        package="control_pkg",
        executable="start",
    )

    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        mit_controller_spawner,
        # start_node
        # rviz_node,

    ]

    return LaunchDescription(nodes)
