"""Launch the vision stack in a component container."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

from python_tools.launch_utils import find_cameras

# Check current running device to determine which camera ports to use
running_device = os.getenv(key="BOARD_VERSION", default="local")

# List of cameras to be used and their properties
camera_handlers = find_cameras(
    running_device=running_device,
    ports_file=os.path.join(
        get_package_share_directory("cv_camera"), "launch", "vision_ports.yaml"
    ),
)


def generate_launch_description():

    vision_config = os.path.join(
        get_package_share_directory("vision_bringup"), "launch", "vision_params.yaml"
    )

    return LaunchDescription(
        [
            # -------------- COMPOSITION -------------------------------
            GroupAction(
                condition=IfCondition(LaunchConfiguration("use_composition")),
                actions=[
                    # Node(
                    #     name="vision_kronos",
                    #     package="rclcpp_components",
                    #     executable="component_container",
                    #     output="both",
                    # ),
                    LoadComposableNodes(
                        target_container="vision_kronos",
                        composable_node_descriptions=[
                            ComposableNode(
                                parameters=[
                                    vision_config,
                                    {
                                        "device_id": camera.device_id,
                                        "port": camera.port,
                                    },
                                ],
                                package="cv_camera",
                                plugin="cv_camera::Driver",
                                name=f"{camera.label}",
                                extra_arguments=[{"use_intra_process_comms": True}],
                            )
                            for camera in camera_handlers
                            if camera.device_id is not None
                        ],
                    ),
                ],
            ),
            # -------------- NO COMPOSITION ----------------------------
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration("use_composition")),
                actions=[
                    Node(
                        parameters=[
                            vision_config,
                            {
                                "device_id": camera.device_id,
                                "port": camera.port,
                            },
                        ],
                        package="cv_camera",
                        executable="cv_camera_node",
                        name=f"{camera.label}",
                        output="screen",
                        respawn=LaunchConfiguration("use_respawn"),
                        respawn_delay=2.0,
                    )
                    for camera in camera_handlers
                    if camera.device_id is not None
                ],
            ),
        ]
    )
