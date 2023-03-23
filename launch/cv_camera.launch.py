"""Launch the vision stack in a component container."""
import os
import cv2

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch.actions import (
    DeclareLaunchArgument,
    # SetEnvironmentVariable,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

from nav2_common.launch import RewrittenYaml
from python_tools.launch_utils import find_cameras, read_cams_ports

# Check current running device to determine which camera ports to use
running_device = os.getenv(key="BOARD_VERSION", default="local")

# List of cameras to be used and their properties
camera_handlers = find_cameras(
    running_device=running_device,
    ports_file=os.path.join(
        get_package_share_directory("video_mapping"), "launch", "vision_ports.yaml"
    ),
)

def generate_launch_description():

    use_respawn = LaunchConfiguration("use_respawn")

    vision_config = os.path.join(
        get_package_share_directory("video_mapping"), "launch", "vision_params.yaml"
    )
    vision_param_yaml = read_cams_ports(vision_config)

    # Encode the fourcc string into VideoWriter_fourcc format
    param_substitutions = {
        f"{camera.label}.ros__parameters.fourcc": str(
            float(cv2.VideoWriter_fourcc(*prop["ros__parameters"]["fourcc"]))
        )
        for node, prop in vision_param_yaml.items()
        for camera in camera_handlers
        if node == camera.label
    }

    # Use the RewrittenYaml class (from nav2_common) to do the param substitutions
    configured_params = RewrittenYaml(
        source_file=vision_config,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    return LaunchDescription(
        [
            # -------------- COMPOSITION -------------------------------
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use node composition",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="True",
                description="Whether to respawn if a node crashes. Applied when composition is disabled.",
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
                                parameters=[
                                    configured_params,
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
                            configured_params,
                            {
                                "device_id": camera.device_id,
                                "port": camera.port,
                            },
                        ],
                        package="cv_camera",
                        executable="cv_camera_node",
                        name=f"{camera.label}",
                        output="screen",
                        respawn=use_respawn,
                        respawn_delay=2.0,
                    )
                    for camera in camera_handlers
                    if camera.device_id is not None
                ],
            ),
        ]
    )
