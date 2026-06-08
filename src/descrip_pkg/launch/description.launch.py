import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_package_path = get_package_share_directory('descrip_pkg')
    urdf_path = os.path.join(urdf_package_path,'urdf','small_dog_copy.urdf')
    rviz_path = os.path.join(urdf_package_path,'config','rviz.rviz')

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(urdf_path),description='填入加载的模型文件路径'
    )

    substitutions_command_result = launch.substitutions.Command(['cat ',
    launch.substitutions.LaunchConfiguration('model')])

    robot_desciption_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_desciption_value}]
        
    )

    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", rviz_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])