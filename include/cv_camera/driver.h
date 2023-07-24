// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_DRIVER_H
#define CV_CAMERA_DRIVER_H

#include "cv_camera/capture.h"

namespace cv_camera
{
  typedef std::shared_ptr<rmw_request_id_t> shared_ptr_request_id;
  typedef std::shared_ptr<std_srvs::srv::Trigger::Request> shared_ptr_trigger_request;
  typedef std::shared_ptr<std_srvs::srv::Trigger::Response> shared_ptr_trigger_response;

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
   * @brief parameters setup
   */
   void parameters_setup();
  /**
   * @brief Grab image from camera.
  */
  void read();
  /**
   * @brief Retrieve, publish and sleep
  */
  void proceed();
  /**
   * @brief Callback for restart node. Run setup() again.
  */
  void RestartNodeCb(shared_ptr_request_id const request_header, shared_ptr_trigger_request const request,
                     shared_ptr_trigger_response response);
 private:
   /**
   * @brief ROS subscription for undistort request.
   */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr undistort_req_sub_;
  /**
   * @brief ROS private timer for publishing images.
   */
  rclcpp::TimerBase::SharedPtr read_tmr_;
  /**
   * @brief ROS private timer for publishing images.
   */
  rclcpp::TimerBase::SharedPtr publish_tmr_;
  /**
   * @brief ROS Service for triggering re setup of the node.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_srv_;
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
   * @brief Undistort image request from console
  */
  bool undistort_img_req_bool_;
  /**
   * @brief Rectify image.
  */
  bool rectify_;
  /**
   * @brief Always publish rectified image.
  */
  bool always_rectify_;
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

  // video capture parameters
  /**
   * @brief Camera cv_cap_prop_brightness.
   */
  float cv_cap_prop_brightness_;
  /**
   * @brief Camera cv_cap_prop_contrast.
   */
  float cv_cap_prop_contrast_;
  /**
   * @brief Camera cv_cap_prop_saturation.
   */
  float cv_cap_prop_saturation_;
  /**
   * @brief Camera cv_cap_prop_hue.
   */
  float cv_cap_prop_hue_;
  /**
   * @brief Camera cv_cap_prop_gain.
   */
  float cv_cap_prop_gain_;
  /**
   * @brief Camera cv_cap_prop_exposure.
   */
  float cv_cap_prop_exposure_;
  /**
   * @brief Camera cv_cap_prop_auto_exposure.
   */
  float cv_cap_prop_auto_exposure_;

  /**
   * Status of the cameras for easier handling
  */
  enum Status
  {
      UNRECOGNIZED = 0,
      ONLINE = 1,
      DISCONNECTED = 2,
      LOST = 3,
      READING_ERROR = 4
  };

  std::map<int, std::string> status_map_ = {
        {0, "UNRECOGNIZED"}, {1, "ONLINE"}, {2, "RECONNECTING"}, {3, "LOST"}, {4, "READING_ERROR"}};

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
