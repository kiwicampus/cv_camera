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
   * @brief Capture, publish and sleep
  */
  void proceed();
 private:
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
  int32_t device_id_;

  /**
   * @brief Topic name.
   */
  std::string topic_name_;

  /**
   * @brief Camera name.
   */
  std::string name_;

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
   * @brief Environment variables
   */
  int video_stream_recovery_time_;
  int video_stream_recovery_tries_;

  /**
   * @brief Reconnection attempts to open a camera port
   */
  int reconnection_attempts_ = 0;
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
