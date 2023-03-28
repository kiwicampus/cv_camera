"""Launch the vision stack in a component container."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import GroupAction, DeclareLaunchArgument
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

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_composition = LaunchConfiguration("use_composition")

    if not camera_handlers:
        return LaunchDescription()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if True",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("vision_bringup"),
                    "params",
                    "vision_params.yaml",
                ),
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composition or not",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="True",
                description="Whether to respawn nodes or not",
            ),
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
                                    params_file,
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
                            params_file,
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
