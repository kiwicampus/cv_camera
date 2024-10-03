from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml

use_composition = os.getenv("VISION_USE_COMPOSITION", 1)
container_name = os.getenv("VISION_CONTAINER_NAME", "vision_kronos")
board_version = os.getenv("BOARD_VERSION", "local")

vision_bringup_path = get_package_share_directory("vision_bringup")
cv_camera_path = get_package_share_directory("cv_camera")
intrinsic_file = "file:///" + os.path.join(cv_camera_path, "launch", "camera_info.yaml")

try:
    params_path = os.path.join(vision_bringup_path,"params","vision_params.yaml")
    usb_cams_ports_path = os.path.join(vision_bringup_path,"params","usb_cams_ports.yaml")
    with open(usb_cams_ports_path, 'r') as file:
        usb_cams_ports = yaml.safe_load(file)
except Exception as e:
    print(f"Error: {e}")
    params_path = os.path.join(cv_camera_path, "config", "params.yaml")

    # Default ports for cm_plus
    usb_cams_ports = {
        "cm_plus": {
            "left": "1-3.3:1.0",
            "right": "1-3.4:1.0",
            "rear": "1-3.1:1.0",
            "zoom": "1-3.5:1.0",
            "inner": "1-2:1.0",
        }
    }

def generate_launch_description():
    plug_cam = LaunchConfiguration("plug_cam")
    declare_plug_cam_cmd = DeclareLaunchArgument(
        "plug_cam",
        default_value="left",
        description="Camera to plug"
    )
    camera_info_url = LaunchConfiguration("camera_info_url")
    declare_camera_info_url_cmd = DeclareLaunchArgument(
        "camera_info_url",
        default_value=intrinsic_file,
        description="path to the camera info file",
    )
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
                        "port": usb_cams_ports[board_version]["left"],
                        "intrinsic_file": camera_info_url,
                    },
                ],
                package="cv_camera",
                executable="cv_camera_node",
                name=plug_cam,
                output="screen",
            )
        ]
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
                                "port": usb_cams_ports[board_version]["left"],
                                "intrinsic_file": camera_info_url,
                            },
                        ],
                        package="cv_camera",
                        plugin="cv_camera::Driver",
                        name=plug_cam,
                        extra_arguments=[
                            {"use_intra_process_comms": True}
                        ],
                    )
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_plug_cam_cmd,
            declare_camera_info_url_cmd,
            declare_params_file_cmd,
            nodes_group,
            composition_group,
        ]
    )
