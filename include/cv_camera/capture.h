// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include "cv_camera/exception.h"
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "opencv2/opencv.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "usr_srvs/srv/grab_frame.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "utils/parameters.hpp"
#include "utils/string_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/**
 * @brief namespace of this package
 */
namespace cv_camera
{

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class Capture
{
public:
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param img_topic_name name of topic to publish (this may be image_raw).
   * @param cam_info_topic_name name of topic to publish cam info (this may be camera_info).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */
  Capture(rclcpp::Node::SharedPtr node,
          const std::string &img_topic_name,
          const std::string &cam_info_topic_name,
          const std::string &rect_image_topic_name,
          const std::string &frame_id,
          const bool roi_exposure,
          uint32_t buffer_size);

  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  bool open(int32_t device_id);

  /**
   * @brief Open capture device with device name.
   *
   * @param port path of the camera device
   * @throw cv_camera::DeviceError device open failed
   */
  bool open(const std::string &port);

  /**
   * @brief Finds the equivalent camera device associated to
   *        one port.
   * @param command Terminal command to be executed
   * @return command_output::Command output delivered by the terminal
   */
  std::string execute_command(const char* command);
  
  /**
   * @brief Finds the equivalent camera device associated to
   *        one port.
   * @param port Port number of interest
   * @return port::DeviceError device open failed
   */
  std::string det_device_path(const char* port);
  
  /**
   * @brief Load camera info from file.
   *
   * This loads the camera info from the file specified in the camera_info_url parameter.
   */
  void loadCameraInfo();

  /**
   * @brief Rectify image using camera info.
   *
   * This uses the camera info loaded by loadCameraInfo() and creates a rectified image.
   */
  void rectify();

  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /**
   * @brief open video file instead of capture device.
   */
  bool openFile(const std::string &file_path);

  /**
   * @brief Close capture device.
   * Uses release OpenCV function.
  */
  void close();

  /**
   * @brief Checks if capture device is opened
   * Uses release OpenCV function.
  */
  bool is_opened();

  /**
   * @brief capture an image and store.
   * to publish the captured image, call publish();
   * @param flip flip the image around vertical axis if true
   * @return true if success to capture, false if not captured.
   */
  bool capture(bool flip);

  /**
   * @brief pull an image from the camera but dont decode it
   *
   * @return true if success to pull, false if not pulled.
   */
  bool grab();

  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish(sensor_msgs::msg::Image::UniquePtr msg);

  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  inline const sensor_msgs::msg::CameraInfo &getInfo() const
  {
    return info_;
  }

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvImage() const
  {
    return bridge_.image;
  }

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::msg::Image::SharedPtr getImageMsgPtr() const
  {
    return bridge_.toImageMsg();
  }

  /**
   * @brief try capture image width
   * @return true if success
   */
  inline bool setWidth(int32_t width)
  {
    return cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  }

  /**
   * @brief try capture image height
   * @return true if success
   */
  inline bool setHeight(int32_t height)
  {
    return cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  }

  /**
   * @brief set CV_PROP_*
   * @return true if success
   */
  bool setPropertyFromParam(int property_id, const std::string &param_name);

  /**
   * @brief get CV_PROP_*
   * @return value of property
   */
  double getProperty(int property_id);

  /**
   * @brief Set black error image in case of error.
   */
  void set_error_image(const std::string& error_msg, int width = 640, int height = 360);

private:
  /**
   * @brief rescale camera calibration to another resolution
   */
  void rescaleCameraInfo(uint width, uint height);

  /**
   * @brief Select appropiate encoding for the image
   */
  std::string mat_type2encoding(int mat_type);

  /**
   * @brief set current time to message header
   */
  void set_now(builtin_interfaces::msg::Time& time);

  /**
   * @brief Sets the exposure of the camera based on the histogram of the image
   * @param frame to set exposure
   */
  void custom_roi_exposure(cv::Mat& frame);

  /**
   * @brief node handle for advertise.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string img_topic_name_;

  /**
   * @brief name of topic without namespace (usually "camera_info").
   */
  std::string cam_info_topic_name_;

  /**
   * @brief name of rectified topic without namespace (usually "image_rect").
   */
  std::string rect_img_topic_name_;
  /**
   * @brief header.frame_id for publishing images.
   */
  std::string frame_id_;
  /**
   * @brief Enables/Disables out custom exposure controller depending on histogram
   */
  bool roi_exposure_;
  /**
   * @brief timestamp of capture image
   */
  rclcpp::Time timestamp_;
  /**
   * @brief Rectified image
   */
  cv::Mat rect_image_;
  /**
   * @brief Rectification maps
   */
  cv::Mat map1_, map2_;
  /**
   * @brief size of publisher buffer
   */
  uint32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher pub_;

  /**
   * @brief video path to be streamed
   */
  std::string video_path_ = "";

  /**
   * @brief capture device.
   */
  cv::VideoCapture cap_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage bridge_;

  /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::msg::CameraInfo info_;

  /**
   * @brief camera info manager
   */
  camera_info_manager::CameraInfoManager info_manager_;

  /**
   * @brief rescale_camera_info param value
   */
  bool rescale_camera_info_;

  /**
   * @brief capture_delay param value
   */
  rclcpp::Duration capture_delay_;

  /**
   * @brief Final publisher for image messages
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_image_ptr;
  /**
   * @brief Final publisher for rectified image messages
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_rect_image_ptr;
  /**
   * @brief Final publisher for camera info messages
   */
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_pub_camera_info_ptr;
};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
