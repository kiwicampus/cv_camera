from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

camera_names_ports = {
    "left": 0,
    # "right": 0,
    # "back": 0,
    # "zoom": 0,
}

base_path = get_package_share_directory("cv_camera")
config_path = "file:///" + os.path.join(base_path, "launch", "camera_info.yaml")


def generate_launch_description():

    camera_info_url = LaunchConfiguration("camera_info_url")
    declare_camera_info_url_cmd = DeclareLaunchArgument(
        "camera_info_url",
        default_value=config_path,
        description="path to the camera info file",
    )
    params_file = LaunchConfiguration("params_file")
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(base_path, "config", "params.yaml"),
        description="path to the params file",
    )

    remappings = [
        ("/left/image_raw", "/video_mapping/left"),
        ("/right/image_raw", "/video_mapping/right"),
        ("/back/image_raw", "/video_mapping/back"),
        ("/zoom/image_raw", "/video_mapping/zoom"),
    ]

    nodes = []
    for camera_name, port in camera_names_ports.items():
        node = Node(
            parameters=[
                params_file,
                {"device_id": port},
                {"intrinsic_file": camera_info_url},
            ],
            package="cv_camera",
            executable="cv_camera_node",
            name=f"{camera_name}",
            output="screen",
            remappings=remappings,
        )

        nodes.append(node)

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_camera_info_url_cmd,
            declare_params_file_cmd,
            *nodes,
        ]
    )