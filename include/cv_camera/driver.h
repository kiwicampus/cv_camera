// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_DRIVER_H
#define CV_CAMERA_DRIVER_H

#include "cv_camera/capture.h"

namespace cv_camera
{

/**
 * @brief ROS cv camera driver.
 *
 * This wraps getting parameters and publish in specified rate.
 */
class Driver : public rclcpp::Node
{
 public:
  /**
   * @brief construct with ROS node handles.
  */
  explicit Driver(const rclcpp::NodeOptions& options);
  ~Driver();


  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cam_status_;

  /**
   * @brief Setup camera device and ROS parameters.
   *
   * @throw cv_camera::DeviceError device open failed.
   */
  bool setup();
  /**
   * @brief Grab image from camera.
  */
  void read();
  /**
   * @brief Retrieve, publish and sleep
  */
  void proceed();
 private:
  /**
   * @brief ROS private timer for publishing images.
   */
  rclcpp::TimerBase::SharedPtr read_tmr_;
  /**
   * @brief ROS private timer for publishing images.
   */
  rclcpp::TimerBase::SharedPtr publish_tmr_;
  /**
   * @brief wrapper of cv::VideoCapture.
   */
  std::shared_ptr<Capture> camera_;

  /**
   * @brief publishing rate.
   */
  std::shared_ptr<rclcpp::Rate> rate_;

  /**
   * @brief camera port.
   */
  std::string port_;

  /**
   * @brief camera device id /dev/videoX.
   */
  int device_id_;

  /**
   * @brief Topic name.
   */
  std::string topic_name_;

  /**
   * @brief Camera name.
   */
  std::string name_;
  /**
   * @brief Publish rate
   */
  float publish_rate_;
  /**
   * @brief read rate
  */
  float read_rate_;
  /**
   * @brief Camera frame_id.
   */
  std::string frame_id_;

  /**
   * @brief Flip image.
  */
  bool flip_;
  /**
   * @brief Video path
  */
  std::string video_path_;

  /**
   * @brief Camera info topic
  */
  std::string cam_info_topic_;
  /**
   * @brief Camera info period
  */
  int cam_info_period_;
  /**
   * @brief Camera intrinsic parameters file
  */
  std::string intrinsic_file_;

  /**
   * @brief Fourcc vector to be used in cv::VideoCapture::set
   */
  std::vector<std::string> fourcc_;

  /**
   * @brief camera status
     Subscription to get cameras status from video_mapping
     -1 -> Unknown
     0 -> Unrecognized
     1 -> Online
     2 -> Disconnected
     3 -> Offline/Lost  [No for stereo]
     4 -> Lecture Error [No for stereo]
  */
  std::shared_ptr<std_msgs::msg::UInt8> cam_status_;

  /**
   * Status of the cameras for easier handling
  */
  enum Status
  {
      UNRECOGNIZED = 0,
      ONLINE = 1,
      DISCONNECTED = 2,
      LOST = 3,
      LECTURE_ERROR = 4
  };

  /**
   * @brief Environment variables
   */
  int video_stream_recovery_time_;
  int video_stream_recovery_tries_;

  /**
   * @brief Reconnection attempts to open a camera port
   */
  int reconnection_attempts_ = 0;

  // Parameters Handling
  NodeParamManager param_manager_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
