from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

camera_names_device_id = {
    "left": 0,
    # "right": 2,
    # "rear": 4,
    # "zoom": 6,
}
vision_bringup_path = get_package_share_directory("vision_bringup")
cv_camera_path = get_package_share_directory("cv_camera")
intrinsic_file = "file:///" + os.path.join(cv_camera_path, "launch", "camera_info.yaml")
try:
    params_path = os.path.join(vision_bringup_path,"params","vision_params.yaml")
except:
    params_path = os.path.join(cv_camera_path, "config", "params.yaml")


def generate_launch_description():

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

    nodes = []
    for camera_name, device_id in camera_names_device_id.items():
        node = Node(
            parameters=[
                params_file,
                {"device_id": device_id},
                {"intrinsic_file": camera_info_url},
            ],
            package="cv_camera",
            executable="cv_camera_node",
            name=f"{camera_name}",
            output="screen",
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