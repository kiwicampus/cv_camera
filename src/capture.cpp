// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(rclcpp::Node::SharedPtr node, const std::string &img_topic_name, const std::string &cam_info_topic_name, 
                 const std::string &rect_img_topic_name, const std::string &frame_id, const bool &flip, uint32_t buffer_size)
    : node_(node),
      it_(node_),
      img_topic_name_(img_topic_name),
      cam_info_topic_name_(cam_info_topic_name),
      rect_img_topic_name_(rect_img_topic_name),
      frame_id_(frame_id),
      flip_(flip),
      buffer_size_(buffer_size),
      info_manager_(node_.get(), frame_id),
      capture_delay_(rclcpp::Duration(0, 0.0))
{
    int dur = 0;
    m_pub_image_ptr = node->create_publisher<sensor_msgs::msg::Image>(img_topic_name_, 1);
    m_pub_rect_image_ptr = node->create_publisher<sensor_msgs::msg::Image>(rect_img_topic_name_, 1);
    m_pub_camera_info_ptr = node->create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic_name_, 1);
    node_->get_parameter_or("capture_delay", dur, dur);
    this->capture_delay_ = rclcpp::Duration(dur, 0.0);
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_->get_parameter("intrinsic_file", url))
  {
    url = "file://" + url;
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Invalid camera info URL %s", node_->get_name(), url.c_str());
    }
  }

  info_ = info_manager_.getCameraInfo();

  // If zero distortion, just pass the message along
  bool zero_distortion = true;

  for (size_t i = 0; i < info_.d.size(); ++i)
  {
      if (info_.d[i] != 0.0)
      {
          zero_distortion = false;
          break;
      }
  }

  // This will be true if D is empty/zero sized
  if (zero_distortion)
  {
      RCLCPP_ERROR(node_->get_logger(), "[%s] No distortion coefficients found, rectification cannot be done", node_->get_name());
      return;
  }

  cv::Mat K = cv::Mat(3, 3, CV_64F, info_.k.data());
  cv::Mat R = cv::Mat(3, 3, CV_64F, info_.r.data());
  cv::Mat P = cv::Mat(3, 4, CV_64F, info_.p.data());

  // select depending on distortion model
  if (info_.distortion_model == "plumb_bob")
  {
      cv::Mat D = cv::Mat(1, 5, CV_64F, info_.d.data());
      cv::initUndistortRectifyMap(K, D, R, P, cv::Size(info_.width, info_.height), CV_16SC2, map1_, map2_);
  }
  else if (info_.distortion_model == "equidistant")
  {
      cv::Mat D = cv::Mat(1, 4, CV_64F, info_.d.data());
      cv::fisheye::initUndistortRectifyMap(K, D, R, P, cv::Size(info_.width, info_.height), CV_16SC2, map1_, map2_);
  }
  else
  {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Unsupported distortion model: %s", node_->get_name(), info_.distortion_model.c_str());
      return;
  }

  rescale_camera_info_ = false;
  node_->get_parameter_or("rescale_camera_info", rescale_camera_info_, rescale_camera_info_);


  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_->get_parameter(param_for_code, code) || !node_->get_parameter(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Setting with code %d and value %f failed", node_->get_name(),code, value);
    }
  }
}

void Capture::rescaleCameraInfo(uint width, uint height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.k[0] *= width_coeff;
  info_.k[2] *= width_coeff;
  info_.k[4] *= height_coeff;
  info_.k[5] *= height_coeff;

  info_.p[0] *= width_coeff;
  info_.p[2] *= width_coeff;
  info_.p[5] *= height_coeff;
  info_.p[6] *= height_coeff;
}

bool Capture::open(int32_t device_id)
{
  cap_.open(device_id, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    return false;
  }
  
  loadCameraInfo();
  return true;
}

bool Capture::open(const std::string &port)
{
  std::string device;
  
  if (det_device_path(port.c_str()) != "-1")
  {
    device = "/dev/video" + det_device_path(port.c_str());
  }
  else
  {
    RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Unable to determine device for port %s.", node_->get_name(), port.c_str());
    return false;
  }
  
  cap_.open(device, cv::CAP_V4L2);
  
  if (!cap_.isOpened())
  {
    return false;
  }
  
  loadCameraInfo();
  return true;
}

void Capture::open()
{
  open(0);
}

bool Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to open file %s.", file_path.c_str());
    return false;
  }
  
  video_path_ = file_path;
  loadCameraInfo();
  return true;
}

bool Capture::grab()
{
  // Restart the video when it ends when video playback mode
  if (video_path_ != "" && cap_.get(cv::CAP_PROP_POS_FRAMES) >= cap_.get(cv::CAP_PROP_FRAME_COUNT))
  {
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
  }

  return cap_.grab();
}

bool Capture::capture()
{
  if (!cap_.retrieve(bridge_.image)) return false;
  if (flip_) cv::flip(bridge_.image, bridge_.image, -1);

  sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());

  // Pack the OpenCV image into the ROS image.
  timestamp_ = node_->now();
  msg->header.stamp = timestamp_;
  msg->header.frame_id = frame_id_;
  msg->height = bridge_.image.rows;
  msg->width = bridge_.image.cols;
  msg->encoding = mat_type2encoding(bridge_.image.type());
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(bridge_.image.step);
  msg->data.assign(bridge_.image.datastart, bridge_.image.dataend);

  // Dont publish image if empty
  if (bridge_.image.empty())
  {
    RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Frame is empty.", node_->get_name());
    return false;
  }

  m_pub_image_ptr->publish(std::move(msg));

  // Fill the cam info message.
  info_.header.stamp = timestamp_;
  info_.header.frame_id = frame_id_;

  m_pub_camera_info_ptr->publish(info_);

  if (rectify_) rectify();

  return true;
}


void Capture::rectify()
{
    // Dont publish image if empty
  if (bridge_.image.empty())
  {
    RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Frame is empty.", node_->get_name());
    return;
  }

  cv::Mat rect_image = bridge_.image;

  // return if map empty
  if (map1_.empty() || map2_.empty())
  {
      RCLCPP_WARN(node_->get_logger(), "[%s] Map1 or Map2 is empty", node_->get_name());
      return;
  }
  cv::remap(rect_image, rect_image, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

  // Update message
  sensor_msgs::msg::Image::UniquePtr msg_image(new sensor_msgs::msg::Image());
  msg_image->header.stamp = timestamp_;
  msg_image->header.frame_id = frame_id_;
  msg_image->height = rect_image.rows;
  msg_image->width = rect_image.cols;
  msg_image->encoding = mat_type2encoding(rect_image.type());
  msg_image->is_bigendian = false;
  msg_image->step = static_cast<sensor_msgs::msg::Image::_step_type>(rect_image.step);
  msg_image->data.assign(rect_image.datastart, rect_image.dataend);

  // Publish rectified image
  m_pub_rect_image_ptr->publish(std::move(msg_image));
}

void Capture::close()
{
  cap_.release();
}

bool Capture::is_opened()
{
  return cap_.isOpened();
}


bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_->get_parameter(param_name, value))
    {
      if (!cap_.set(property_id, value) && value != getProperty(property_id))
      {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Setting with code %d and value %f failed", node_->get_name(), property_id, value);
        return false;
      }
    }
  }
  return true;
}

// Get VideoCapture properties
double Capture::getProperty(int property_id)
{
  if (cap_.isOpened())
  {
    return cap_.get(property_id);
  }
  return 0.0;
}

std::string Capture::execute_command(const char* command)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
  {
    result += buffer.data();
  }
  return result;
}

std::string Capture::det_device_path(const char* port)
{
  std::string video_device = "-1";
  // TODO: instead of reading the output from shell, iter the directory
  std::string video_devices = execute_command("ls /dev/video*");
  std::string delimiter = "\n";
  
  size_t pos = 0;
  std::string pre_token;
  std::string token;
  std::string output_command;
  std::vector<int> devices;
  
  while ((pos = video_devices.find(delimiter)) != std::string::npos)
  {
    // get /dev/videoX substring
    pre_token = video_devices.substr(0, pos);
    // get number of the cam device
    token = pre_token.substr(10, 2);
    devices.push_back(std::stoi(token));

    video_devices.erase(0, pos + delimiter.length());
  }
  
  // Sort the vector to get devices in order
  std::sort(devices.begin(), devices.end());
  
  // Iter the devices to identify which port correspond to which videoX
  for (const auto& cam : devices)
  {
    output_command = "udevadm info --query=path --name=/dev/video" + std::to_string(cam);
    std::string camera_device_info = execute_command(output_command.c_str());
    if (camera_device_info.find(port) != std::string::npos)
    {
        video_device = std::to_string(cam);
        return video_device;
    }
  }
  
  return video_device;
}

std::string Capture::mat_type2encoding(int mat_type)
  {
      switch (mat_type)
      {
          case CV_8UC1:
              return "mono8";
          case CV_8UC3:
              return "bgr8";
          case CV_16SC1:
              return "mono16";
          case CV_8UC4:
              return "rgba8";
          default:
              throw std::runtime_error("Unsupported encoding type");
      }
}

void Capture::set_now(builtin_interfaces::msg::Time& time)
{
    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (now <= std::chrono::nanoseconds(0))
    {
        time.sec = time.nanosec = 0;
    }
    else
    {
        time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
        time.nanosec = now.count() % 1000000000;
    }
}
}  // namespace cv_camera
