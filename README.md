ROS OpenCV camera driver
========================

It is very easy to capture video device if we use `cv::VideoCapture` of OpenCV.

cv_camera_node
------------------

This node uses [camera_info_manager](http://wiki.ros.org/camera_info_manager) for dealing with camera_info.
If no calibration data is set, it has dummy values except for width and height.

### Publish

* `~image_raw` (*sensor_msgs/Image*)
* `~camera_info` (*sensor_msgs/CameraInfo*)

### Service

* `~set_camera_info` (*sensor_msgs/SetCameraInfo*)

### Parameters

* `~rate` (*double*, default: 30.0) – publish rate [Hz].
* `~device_id` (*int*, default: 0) – capture device id.
* `~device_path` (*string*, default: "") – path to camera device file, e. g. `/dev/video0`.
* `~frame_id` (*string*, default: "camera") – `frame_id` of message header.
* `~image_width` (*int*) – try to set capture image width.
* `~image_height` (*int*) – try to set capture image height.
* `~camera_info_url` (*string*) – url of camera info yaml.
* `~file` (*string*, default: "") – if not "" then use movie file instead of device.
* `~capture_delay` (*double*, default: 0) – estimated duration of capturing and receiving the image.
* `~rescale_camera_info` (*bool*, default: false) – rescale camera calibration info automatically.

Supports CV_CAP_PROP_*, by below params.

* `~cv_cap_prop_pos_msec` (*double*)
* `~cv_cap_prop_pos_avi_ratio` (*double*)
* `~cv_cap_prop_frame_width` (*double*)
* `~cv_cap_prop_frame_height` (*double*)
* `~cv_cap_prop_fps` (*double*)
* `~cv_cap_prop_fourcc` (*double*)
* `~cv_cap_prop_frame_count` (*double*)
* `~cv_cap_prop_format` (*double*)
* `~cv_cap_prop_mode` (*double*)
* `~cv_cap_prop_brightness` (*double*)
* `~cv_cap_prop_contrast` (*double*)
* `~cv_cap_prop_saturation` (*double*)
* `~cv_cap_prop_hue` (*double*)
* `~cv_cap_prop_gain` (*double*)
* `~cv_cap_prop_exposure` (*double*)
* `~cv_cap_prop_convert_rgb` (*double*)
* `~cv_cap_prop_rectification` (*double*)
* `~cv_cap_prop_iso_speed` (*double*)

And supports any props. Thanks to Hernan Badino!

* `~property_$(i)_code` (*int*) – set this code property using `~property_$(i)_value`, $(i) must start from 0.
* `~property_$(i)_value` (*double*) – the value to be set to `~property_$(i)_code`

If you want to set the property which code is 404 as 1,

```bash
rosrun cv_camera cv_camera_node _property_0_code:=404 _property_0_code:=1
```

If you want to set more, use `~property_1_code` and `~property_1_code`.

Nodelet
-------------------

This node works as nodelet (`cv_camera/CvCameraNodelet`).

### Using the Launch File

The `kiwi_runtime_load.launch.py` file is used to launch the camera node with specific configurations. It supports both standalone node execution and composition in a container.

#### Launch Arguments

- `cam_name`: The name of the camera to be used. Default is `"unknown"`.
- `cam_port`: The port to which the camera is connected. Default is `"unknown"`.
- `intrinsic_file`: The path to the camera's intrinsic calibration file. Default is `"unknown"`.
- `params_file`: The path to the parameters file. Default is `"vision_params.yaml"`.

##### cam_port Format

The `cam_port` parameter uses a specific format to represent the USB port path:

- The format is `"bus-port_path:interface"`.
- To determine the correct value for your system:
  1. Use the `lsusb` command to list all connected USB devices.
  2. Identify the Bus and Device numbers for your camera.
  3. Use `udevadm` to find the detailed path of the device.
     ```bash
     udevadm info --query=path --name=/dev/bus/usb/<Bus>/<Device>
     ```
     Replace `<Bus>` and `<Device>` with the numbers from your `lsusb` output.
  4. The output will include a path like `/devices/.../usb3/3-4/3-4.4/3-4.4.4`.
  5. Use the bus number and the hierarchical path to construct the `cam_port` entry.
     Example: If the path is `3-4.4.4` and the bus is `3`, the entry is `"3-4.4.4:1.0"`.

#### How to Launch

To launch the camera node, use the following command:

```bash
ros2 launch cv_camera kiwi_runtime_load.launch.py \
    cam_name:=<camera_name> \
    cam_port:=<camera_port> \
    intrinsic_file:=<path_to_intrinsic_file> \
    params_file:=<path_to_params_file>
```

Replace `<camera_name>`, `<cam_port>`, `<path_to_intrinsic_file>`, and `<path_to_params_file>` with the appropriate values for your setup.

#### Example

Here is an example command with real values:

```bash
ros2 launch cv_camera kiwi_runtime_load.launch.py \
    cam_name:=front_camera \
    cam_port:=3-4.4.4:1.0 \
    intrinsic_file:=front_camera_intrinsics.yaml \
    params_file:=front_camera_params.yaml
```

### Using the Kiwi Launch File

The `kiwi.launch.py` file is designed to launch multiple camera nodes using predefined device IDs and configuration files.

#### Launch Arguments

- `camera_info_url`: Path to the camera info file. Defaults to the intrinsic calibration file in the `vision_bringup` package.
- `params_file`: Path to the parameters file. Defaults to the `vision_params.yaml` in the `vision_bringup` package.

#### How to Launch

To launch the camera nodes, use the following command:

```bash
ros2 launch cv_camera kiwi.launch.py \
    camera_info_url:=<path_to_camera_info_file> \
    params_file:=<path_to_params_file>
```

Replace `<path_to_camera_info_file>` and `<path_to_params_file>` with the appropriate paths if you wish to override the defaults.

About
--------------------

This package is a fork of the [original cv_camera package](https://github.com/OTL/cv_camera) created by Takashi Ogura (OTL).

