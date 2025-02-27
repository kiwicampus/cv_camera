from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

camera_names_device_id = {
    "left": {
        'device_id': 1,
        'intrinsic_filename': 'intrinsic_calibration_params_2_1mm.yaml'
        },
    "econ_wide_len": {
        'device_id': 0,
        'intrinsic_filename': 'intrinsic_calibration_params_econ.yaml'
        },
    # "right": 2,
    # "rear": 4,
    # "zoom": 6,
}

vision_bringup_path = get_package_share_directory("vision_bringup")

try:
    params_path = os.path.join(vision_bringup_path, "params", "vision_params.yaml")
    params_file = 'vision_params.yaml'
except:
    print("Unable to load vision config params", flush=True)


def generate_launch_description():

    params_fileparams_file = LaunchConfiguration("params_file")
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value="vision_params.yaml",
        description="path to the vision params file",
    )

    nodes = []
    for camera_name, items in camera_names_device_id.items():
        device_id = items['device_id']
        intrinsic_filename = items['intrinsic_filename']

        node = Node(
            parameters=[
                PathJoinSubstitution([vision_bringup_path, "params", params_file]),
                {
                    "port": str(device_id),
                    "intrinsic_file": PathJoinSubstitution(
                        [vision_bringup_path, "configs", "calibration", intrinsic_filename]
                    ),
                },
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
            declare_params_file_cmd,
            *nodes,
        ]
    )
