"""Launch the vision stack in a component container."""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

camera_names_ports = ["left", "right", "back", "zoom"]


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("cv_camera"), "launch", "usb_camera_params.yaml"
    )

    return LaunchDescription(
        [
            # -------------- COMPOSITION -------------------------------
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use node composition",
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration("use_composition")),
                actions=[
                    Node(
                        name="vision_kronos",
                        package="rclcpp_components",
                        executable="component_container",
                        output="both",
                    ),
                    LoadComposableNodes(
                        target_container="vision_kronos",
                        composable_node_descriptions=[
                            ComposableNode(
                                parameters=[config],
                                package="cv_camera",
                                plugin="cv_camera::Driver",
                                namespace=camera_name,
                                name=f"cv_camera_{camera_name}",
                            )
                            for camera_name in camera_names_ports
                        ],
                    ),
                ],
            ),
            # -------------- NO COMPOSITION ----------------------------
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration("use_composition")),
                actions=[
                    Node(
                        parameters=[config],
                        package="cv_camera",
                        executable="cv_camera_node",
                        name=f"cv_camera_{camera_name}",
                        output="screen",
                        namespace=camera_name,
                    )
                    for camera_name in camera_names_ports
                ],
            ),
        ]
    )
