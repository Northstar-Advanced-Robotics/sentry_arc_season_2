import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_launch")
    _repo_root = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..")
    )

    composable_nodes = []
    tf_nodes = []

    uart_bridge = Node(
        package="uart_bridge",
        executable="uart_bridge_node",
        output="both",
        name="uart_bridge",
    )

    path_planning = Node(
        package="path_planning",
        executable="path_planning_node",
        output="both",
        name="path_planning",
    )

    # Only EKF (map -> base_link)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        remappings=[
            ("odometry/filtered", "odometry/global")
        ],  # Prevents topic collision with local EKF
    )

    arducam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="arducam_node",
        output="screen",
        remappings=[
            ("image_raw", f"/front/camera/color/image_raw"),
            ("camera_info", f"/front/camera/color/camera_info"),
        ],
    )

    return launch.LaunchDescription(
        [
            uart_bridge,
            ekf_node,
            arducam,
            path_planning,
        ]
    )

