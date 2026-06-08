# from launch import LaunchDescription
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# import os

# def generate_launch_description():
#     control_pkg_path = os.path.join(get_package_share_directory('control_pkg'))
#     mujoco_xml = os.path.join(get_package_share_directory('descrip_pino'))

#     paramter_config = os.path.join(control_pkg_path, 'param_config', 'config.yaml')
#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [
#                     FindPackageShare("descrip_pino"),
#                     "urdf",
#                     "unitree.urdf.xacro"
#                 ]
#             )
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}
#     rviz_config_file = PathJoinSubstitution(
#         [FindPackageShare("control_pkg"), "rviz_config", "rviz.rviz"]
#     )
#     robot_controllers = PathJoinSubstitution(
#         [
#             FindPackageShare("control_pkg"),
#             "controller_config",
#             "config.yaml",
#         ]
#     )

#     control_node = Node(
#         # Specify the control node from this package!
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_controllers],
#         output="both",
#         remappings=[
#             ("~/robot_description", "/robot_description"),
#         ],
#     )

#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description,{"use_sim_time": False}],
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--param-file",robot_controllers],
#     )

#     mit_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["mit_controller","--param-file",robot_controllers],
#     )

#     start_node = Node(
#         package="control_pkg",
#         executable="start",
#         parameters=[paramter_config]
#     )
#     imu = Node(
#         package="imu_pkg",
#         executable="imu",
#     )

#     nodes = [
#         robot_state_pub_node,
#         control_node,
#         joint_state_broadcaster_spawner,
#         mit_controller_spawner,
#         imu,
#         start_node,
#         # rviz_node,

#     ]

#     return LaunchDescription(nodes)
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
def generate_launch_description():
    control_pkg_path = os.path.join(get_package_share_directory('control_pkg'))
    mujoco_xml = os.path.join(get_package_share_directory('descrip_pino'))
    robot_controllers = os.path.join(control_pkg_path, 'controller_config', 'config.yaml')

    paramter_config = os.path.join(control_pkg_path, 'param_config', 'config.yaml')
    xacro_file_path = os.path.join(mujoco_xml, 'urdf', 'unitree.urdf.xacro')

    robot_description_content = xacro.process_file(xacro_file_path).toxml()
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        # Specify the control node from this package!
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": False}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file",robot_controllers],
    )

    mit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mit_controller","--param-file",robot_controllers],
    )

    start_node = Node(
        package="control_pkg",
        executable="start",
        parameters=[paramter_config],
        output="screen",
        emulate_tty=True,
    )
    imu = Node(
        package="imu_pkg",
        executable="imu",
    )

    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        mit_controller_spawner,
        imu,
        start_node,

    ]

    return LaunchDescription(nodes)
