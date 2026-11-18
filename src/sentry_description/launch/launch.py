import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Params
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_share = get_package_share_directory('sentry_description');
    sentry_urdf = os.path.join(package_share, 'urdf/sentry.urdf.xacro')
    print(sentry_urdf)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', sentry_urdf]), value_type=str)
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
                             robot_state_publisher,
                             joint_state_publisher_gui,
                             rviz2,
                         ])
