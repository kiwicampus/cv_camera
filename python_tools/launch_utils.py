import re
import os
import yaml
import subprocess
from typing import Dict, List, Union, Tuple

from python_utils.vision_utils import printlog


# cam_labels remapping
remap_label = {
    "stereo": "S",
    "zoom": "Z",
    "left": "LL",
    "right": "RR",
    "rear": "B",
    "inner": "I",
}

class Camera:
    def __init__(
        self,
        label: str,
        device_id: int = -1,
        port: str = "",
        cam_format: str = "",
        intrinsic_file: str = "",
        video_path: str = "",
    ):
        self.label = label
        self.device_id = device_id
        self.port = port
        self.cam_format = cam_format
        self.intrinsic_file = intrinsic_file
        self.video_path = video_path


def find_cameras(running_device: str, ports_file: str, bot_id: str, params_file: str, 
                 balena_app_name: str, params_file_dict: dict) -> Dict[str, Union[str, int]]:
    """!
    Finds the camera numbers and ports that correspond to real cameras
    Note: (devices may be repeated)
    @return cam_ports `dictionary` with video devices ports, numbers and formats
    """
    # Finds and lists video devices numbers
    cams = find_video_devices()
    # Finds and lists video devices ports
    avail_ports, cam_devices = find_usb_ports(cams)
    # Finds available camera format types
    cam_formats = get_camera_format()
    # Reads de cameras configurations file.
    vision_ports = read_cams_ports(CONF_PATH=ports_file)
    # Get cameras intrinsic parameters file
    fleet_id = "NAV2" if "Nav2" in balena_app_name else bot_id
    calibration_params_dict = read_fleet_config_file(
            read_cams_configuration(
                params_file,
                "fleet_cams_calibration_params.yaml",
            ),
            fleet_id
        )

    # print("cams:", cams, flush=True)
    # print("avail_ports:", avail_ports, flush=True)
    # print("cam_devices:", cam_devices, flush=True)
    # print("cam_formats:", cam_formats, flush=True)

    # list with video devices ports and numbers
    cam_ports = dict()
    for idx, cam_port in enumerate(cam_devices):

        if not cam_formats:
            if cam_port not in cam_ports:
                cam_ports[cam_port] = [cams[idx], "Unknown"]
            continue

        for type in cam_formats:
            # If the port is not in the list and the camera type matches the device id then add it
            if cam_port not in cam_ports and cams[idx] in type["id"]:
                cam_ports[cam_port] = [cams[idx], type["name"]]

    if not len(cam_ports):
        printlog("No video devices detected", msg_type="ERROR")

    # Get camera ports for usb devices
    cams_ports = [
        vision_ports[run_device][cam_dic]
        for run_device, cam_dev in vision_ports.items()
        for cam_dic, port in cam_dev.items()
        if run_device == running_device and cam_dic != "S"
    ]

    video_numbers = [
        cam_ports[str(port)][0] if port in cam_ports else None for port in cams_ports
    ]

    cams_formats = [
        cam_ports[str(port)][1] if port in cam_ports else None for port in cams_ports
    ]

    non_stereo_labels = [
        cam_label
        for cam_label in vision_ports[running_device].keys()
        if cam_label != "S" or vision_ports[running_device][cam_label] != "None"
    ]

    calibration_params_dict = {cam_label: os.getenv(f"{cam_label}_CALIBRATION_PARAMS_FILE") or params
                           for cam_label, params in calibration_params_dict.items()}

    # Initializes camera objects
    camera_handlers = [
        Camera(camera_label, device_number, cam_port, camera_format, 
               os.path.join(params_file, calibration_params_dict[remap_label[camera_label]]) 
               if remap_label[camera_label] in calibration_params_dict.keys()
               else "")
        for device_number, camera_label, camera_format, cam_port in zip(
            video_numbers, non_stereo_labels, cams_formats, cams_ports
        )
        if device_number is not None
    ]

    # if there is video file create a cam object for it
    for cam_label, param in params_file_dict.items():
        if "video_path" in param["ros__parameters"].keys() and param["ros__parameters"]["video_path"]:
            camera_handlers.append(
                Camera(
                    label = cam_label,
                    intrinsic_file = os.path.join(params_file, calibration_params_dict[remap_label[cam_label]]),
                    video_path = param["ros__parameters"]["video_path"],
                )
            )

    if not camera_handlers:
        printlog("No USB cameras found", msg_type="ERROR")

    return camera_handlers


def find_video_devices() -> List[int]:

    """!
    Finds and lists video devices numbers
    @return cams_list `list` with video camera devices numbers
    """

    # Check for video devices
    p = re.compile(r".+video(?P<video>\d+)$", re.I)
    devices = subprocess.check_output("ls -l /dev", shell=True).decode("utf-8")
    avail_cameras = []

    for device in devices.split("\n"):
        if device:
            info = p.match(device)
            if info:
                dinfo = info.groupdict()
                avail_cameras.append(dinfo["video"])
    cams_list = list(sorted(map(int, avail_cameras)))

    return cams_list


def find_usb_ports(cameras: List[int]) -> Tuple[List[str], List[str]]:

    """!
    Finds and lists video devices ports
    @param cameras: `list` with video camera devices numbers
    @return avail_ports `list` with video camera devices ports
    """

    # find usb port given a video number device
    avail_ports = []
    p = re.compile(r"\d-(?P<video>[0-9.]+).+", re.I)
    cam_devices = []

    for cam in cameras:

        try:
            # List of physical ports used in /dev/video# (some devices may represent same thing)
            path = subprocess.check_output(
                "udevadm info --query=path --name=/dev/video" + str(cam), shell=True
            ).decode("utf-8")

        except Exception as e:
            printlog(
                f"----- ERROR READING VIDEO DEVICE ----- (Error: {e})", msg_type="ERROR"
            )
            avail_ports.append("None-{}".format(cam))
            continue

        cam_device = path.strip("\n").split("/")[-3]
        cam_device = cam_device + str(cam) if cam_device == "virtual" else cam_device
        cam_devices.append(cam_device)

        info = p.match(path)  # get actual address
        if info:
            dinfo = info.groupdict()
            avail_ports.append(dinfo["video"])
        elif ".vi" in path:
            avail_ports.append(path)
        elif "virtual" in path:
            avail_ports.append(cam_device)

    if len(cam_devices):
        printlog(
            "Video devices ports: " + ", ".join(str(e) for e in list(set(cam_devices))),
            msg_type="OKPURPLE",
        )

    return avail_ports, cam_devices


def get_camera_format() -> List[Dict[str, Union[str, int]]]:

    """!
    # Retrieve camera type from v4l2 message
    @return cam_formats `list` with video camera devices formats
    """
    try:
        df = subprocess.check_output("v4l2-ctl --list-devices --verbose", shell=True)
    except subprocess.CalledProcessError:
        return []
    except Exception as ex:
        printlog(
            f"----- ERROR RETRIEVING VIDEO FORMAT ----- (Error: {ex})", msg_type="ERROR"
        )
        return []

    v4l2_re = re.compile(r"(?P<name>[\w\s]*)[:|\(]", re.I)
    dev_re = re.compile(r"^\s*/dev/video(?P<video_port>\d+)", re.I)

    cam_formats = []
    current_cam = None

    # Split message by line to get each camera format. Decode to utf-8 to avoid errors
    formats_dev_video_list = df.decode("utf-8").split("\n")

    for i in formats_dev_video_list:
        if not i or "media" in i:
            continue

        info = v4l2_re.match(i)
        dev_info = dev_re.match(i)

        if info:
            dinfo = info.groupdict()
            cam_formats.append(dinfo)
            current_cam = dinfo["name"]
            cam_formats[-1]["id"] = []

        elif dev_info and current_cam:
            dinfo = dev_info.groupdict()
            cam_formats[-1]["id"].append(int(dinfo["video_port"]))

        else:
            printlog(f"Skipping unexpected video format. Got: {i}", msg_type="WARN")

    return cam_formats


def read_cams_ports(CONF_PATH: str) -> Dict[str, Dict[str, Union[str, int]]]:

    """!
    Function for seting the process name
    @param CONF_PATH `string` absolute path to configuration of cameras
    @param FILE_NAME `string` name of cameras configuration file
    @return data_loaded `dictionary` key: camera labels, values: dictionary with camera
            properties and settings, see yaml file for more details
    """

    if not os.path.isfile(CONF_PATH):
        printlog("No configuration file found", msg_type="ERROR")
        return {}

    with open(CONF_PATH, "r") as stream:
        data_loaded = yaml.safe_load(stream)
        return data_loaded

def read_cams_configuration(CONF_PATH: str, FILE_NAME: str) -> dict:

    """!
    Function for seting the process name
    @param CONF_PATH `string` absolute path to configuration of cameras
    @param FILE_NAME `string` name of cameras configuration file
    @return data_loaded `dictionary` key: camera labels, values: dictionary with camera
            properties and settings, see yaml file for more details
    """

    abs_path = os.path.join(CONF_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, "r") as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        return {}

def read_fleet_config_file(fleet_params_file: dict, bot_id: str) -> dict:
    """!
    Reads the fleet configuration file and returns a dictionary with the current bot cam configuration
    @param params_directory: 'dict' camera configuration file
    @param bot_id: 'str' bot id
    @return: 'dict' dictionary with configuration parameters
    """

    # Several bot versions are in the yaml file
    # but only the params of the current one are the ones we need.
    for current_fleet_model in fleet_params_file.keys():
        if current_fleet_model in bot_id:
            return fleet_params_file[current_fleet_model]
    return {}