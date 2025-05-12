from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Dictionary mapping camera names to their device IDs
# The device ID is the number assigned by the OS to the camera device
# For example, /dev/video3 would have device_id = 3
camera_names_device_id = {
    "left": 3,
    # "right": 2,
    # "rear": 4,
    # "zoom": 6,
}

vision_bringup_path = get_package_share_directory("vision_bringup")

# Default paths
default_intrinsic_file = PathJoinSubstitution([vision_bringup_path, "configs", "calibration", "intrinsic_calibration_params_2_1mm.yaml"])
default_params_file = PathJoinSubstitution([vision_bringup_path, "params", "vision_params.yaml"])

def generate_launch_description():
    # Declare launch arguments for camera info and params files
    declare_camera_info_url_cmd = DeclareLaunchArgument(
        "camera_info_url",
        default_value=default_intrinsic_file,
        description="Path to the camera info file",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the params file",
    )

    # Use LaunchConfiguration to get the values, falling back to defaults if not provided
    camera_info_url = LaunchConfiguration("camera_info_url")
    params_file = LaunchConfiguration("params_file")

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
