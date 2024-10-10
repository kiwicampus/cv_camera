from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration
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
    print("Error: No shared vision_bringup directory found.")
    exit(1)

plug_cam = os.getenv("PLUG_CAM_NAME", "")
plug_port = os.getenv("PLUG_CAM_PORT", "")
intrinsic_file = os.path.join(
    base_path, "configs", "calibration", os.getenv("PLUG_CAM_INTRINSIC_FILE", "")
)
use_composition = parse_bool2string(int(os.getenv("VISION_USE_COMPOSITION", True)))
container_name = os.getenv("VISION_CONTAINER_NAME", "vision_kronos")

if not plug_cam or not plug_port:
    print("No camera to plug")
    exit(1)

params_path = os.path.join(base_path, "params", "vision_params.yaml")


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=params_path,
        description="path to the params file",
    )

    nodes_group = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                parameters=[
                    params_file,
                    {
                        "port": plug_port,
                        "intrinsic_file": intrinsic_file,
                    },
                ],
                package="cv_camera",
                executable="cv_camera_node",
                name=plug_cam,
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
                            params_file,
                            {
                                "port": plug_port,
                                "intrinsic_file": intrinsic_file,
                            },
                        ],
                        package="cv_camera",
                        plugin="cv_camera::Driver",
                        name=plug_cam,
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
            declare_params_file_cmd,
            nodes_group,
            composition_group,
        ]
    )
