from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def parse_bool2string(boolean: bool) -> str:
    """! Convert boolean to string
    @param bool: boolean to be converted
    @return: string
    """
    try:
        if int(boolean):
            return "True"
        else:
            return "False"
    except:
        return boolean


try:
    base_path = get_package_share_directory("vision_bringup")
except Exception:
    print("Error: No shared vision_bringup directory found. It is necessary for params and intrisic files!")
    exit(1)

use_composition = parse_bool2string(int(os.getenv("VISION_USE_COMPOSITION", True)))
container_name = os.getenv("VISION_CONTAINER_NAME", "vision_kronos")


def generate_launch_description():
    cam_name = LaunchConfiguration("cam_name")
    declare_cam_name_cmd = DeclareLaunchArgument(
        "cam_name",
        default_value="unknown",
        description="camera name to plug",
    )
    cam_port = LaunchConfiguration("cam_port")
    declare_cam_port_cmd = DeclareLaunchArgument(
        "cam_port",
        default_value="unknown",
        description="camera port to plug",
    )
    intrinsic_file = LaunchConfiguration("intrinsic_file")
    declare_intrinsic_file_cmd = DeclareLaunchArgument(
        "intrinsic_file",
        default_value="unknown",
        description="camera intrinsic file",
    )
    params_file = LaunchConfiguration("params_file")
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value="vision_params.yaml",
        description="path to the params file",
    )

    nodes_group = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                parameters=[
                    PathJoinSubstitution([base_path, "params", params_file]),
                    {
                        "port": cam_port,
                        "intrinsic_file": PathJoinSubstitution(
                            [base_path, "configs", "calibration", intrinsic_file]
                        ),
                    },
                ],
                package="cv_camera",
                executable="cv_camera_node",
                name=cam_name,
                output="screen",
            )
        ],
    )
    composition_group = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        parameters=[
                            PathJoinSubstitution([base_path, "params", params_file]),
                            {
                                "port": cam_port,
                                "intrinsic_file": PathJoinSubstitution(
                                    [
                                        base_path,
                                        "configs",
                                        "calibration",
                                        intrinsic_file,
                                    ]
                                ),
                            },
                        ],
                        package="cv_camera",
                        plugin="cv_camera::Driver",
                        name=cam_name,
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_cam_name_cmd,
            declare_cam_port_cmd,
            declare_intrinsic_file_cmd,
            declare_params_file_cmd,
            nodes_group,
            composition_group,
        ]
    )
