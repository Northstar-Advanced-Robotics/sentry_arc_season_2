import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():

    package_dir = get_package_share_directory('sentry_sim')
    package_share = get_package_share_directory('sentry_description');
    sentry_urdf = os.path.join(package_share, 'urdf/sentry.urdf.xacro')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        ros2_supervisor=True,
    )

    robot_driver = WebotsController(
        robot_name='sentry',
        parameters=[
            {
                'robot_description': sentry_urdf,
            },
        ]
    )

    keyboard_node = Node(
        package="sentry_sim",
        executable="keyboard_node",
        output="both",
        name="keyboard",
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_driver,
        keyboard_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
