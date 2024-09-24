// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"
#include <string>

namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{

Driver::Driver(const rclcpp::NodeOptions& options) : Node("cv_camera", options)
{
  auto ptr = std::shared_ptr<Driver>(this, [](Driver*) {});
  this->parameters_setup();
  this->setup();
}

void Driver::parameters_setup()
{
  name_ = this->get_fully_qualified_name();

  // OpenCV Parameters
  this->declare_parameter("fourcc", rclcpp::PARAMETER_STRING_ARRAY);
  this->declare_parameter("cv_cap_prop_fourcc", 0.0);
  this->get_parameter("fourcc", fourcc_);
  // Decode fourcc as CV2 only accepts double values for its parameters
  this->set_parameter(rclcpp::Parameter("cv_cap_prop_fourcc", (double)cv::VideoWriter::fourcc(
                      *fourcc_[0].c_str(), *fourcc_[1].c_str(), *fourcc_[2].c_str(), *fourcc_[3].c_str())));

  // Environment Variables | ROS Parameters
  param_manager_ = NodeParamManager(this);
  param_manager_.addParameter<std::string>(port_, "port", "");
  param_manager_.addParameter(device_id_, "device_id", -1);
  param_manager_.addParameter(publish_rate_, "publish_rate", 15.0f);
  param_manager_.addParameter(read_rate_, "read_rate", 15.0f);
  param_manager_.addParameter(cam_info_period_, "cam_info_period", 5);
  param_manager_.addParameter(flip_, "flip", false);
  param_manager_.addParameter(roi_exposure_, "roi_exposure", false);
  param_manager_.addParameter(rectify_, "rectify", false);
  param_manager_.addParameter(always_rectify_, "always_rectify", false);
  param_manager_.addParameter(always_publish_, "always_publish", false);
  param_manager_.addParameter<std::string>(intrinsic_file_, "intrinsic_file", "");
  param_manager_.addParameter<std::string>(video_path_, "video_path", "");
  param_manager_.addParameter<std::string>(frame_id_, "frame_id", "camera_id");
  param_manager_.addParameter(video_stream_recovery_time_, "video_stream_recovery_time", 2);
  param_manager_.addParameter(video_stream_recovery_tries_, "video_stream_recovery_tries", 10);

  // Video capture parameters
  param_manager_.addParameter(width_, "width", 640);
  param_manager_.addParameter(height_, "height", 360);
  this->declare_parameter("cv_cap_prop_frame_width", 640.0);
  this->declare_parameter("cv_cap_prop_frame_height", 360.0);
  this->set_parameter(rclcpp::Parameter("cv_cap_prop_frame_width", (double)width_));
  this->set_parameter(rclcpp::Parameter("cv_cap_prop_frame_height", (double)height_));
  param_manager_.addParameter(cv_cap_prop_brightness_, "cv_cap_prop_brightness", 0.0f);
  param_manager_.addParameter(cv_cap_prop_contrast_, "cv_cap_prop_contrast", 32.0f);
  param_manager_.addParameter(cv_cap_prop_saturation_, "cv_cap_prop_saturation", 56.0f);
  param_manager_.addParameter(cv_cap_prop_hue_, "cv_cap_prop_hue", 0.0f);
  param_manager_.addParameter(cv_cap_prop_gain_, "cv_cap_prop_gain", 0.0f);
  param_manager_.addParameter(cv_cap_prop_exposure_, "cv_cap_prop_exposure", 156.0f);
  param_manager_.addParameter(cv_cap_prop_auto_exposure_, "cv_cap_prop_auto_exposure", 3.0f);

  // Subscribers
  undistort_req_sub_ = 
    this->create_subscription<std_msgs::msg::Bool>("/video_mapping/un_distort", 1, 
                    [&](const std_msgs::msg::Bool::SharedPtr msg) -> void { undistort_img_req_bool_ = msg->data; });

  // Publishers
  // In intra process communication, we cant use transient local,
  // so we disable the intra process for this publisher
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  pub_cam_status_ = this->create_publisher<std_msgs::msg::UInt8>("/video_mapping" + name_ + "/status", rclcpp::QoS(1).keep_all().transient_local().reliable(), options);
  pub_cam_diagnostic_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

  // Services
  restart_srv_ = this->create_service<std_srvs::srv::Trigger>(
    name_ + "/restart", std::bind(&Driver::RestartNodeCb, this, _1, _2, _3));
  pause_img_srv_ = this->create_service<std_srvs::srv::SetBool>(
    name_ + "/pause_img_pub", std::bind(&Driver::PauseImageCb, this, _1, _2, _3));
  release_cam_srv_ = this->create_service<std_srvs::srv::SetBool>(
    name_ + "/release", std::bind(&Driver::ReleaseCamCb, this, _1, _2, _3));

  params_callback_handle_ =
    this->add_on_set_parameters_callback(std::bind(&Driver::parameters_cb, this, _1));
}

bool Driver::setup()
{

  camera_.reset(new Capture(shared_from_this(),
                            "/video_mapping" + name_ + "/image_raw",
                            "/video_mapping" + name_ + "/camera_info",
                            "/video_mapping" + name_ + "/image_rect",
                            frame_id_,
                            roi_exposure_,
                            PUBLISHER_BUFFER_SIZE));

  if (video_path_ != "")
  {
    camera_->openFile(video_path_);
  }
  else if (device_id_ >= 0)
  {
    if (!camera_->open(device_id_))
    {
      RCLCPP_WARN(get_logger(), "[%s] Couldn't open camera by device_id [%d]", name_.c_str(), device_id_);
      return false;
    }
  }
  else if (port_ != "")
  {
    if (!camera_->open(port_))
    {
      RCLCPP_WARN(get_logger(), "[%s] Couldn't open camera by port [%s]", name_.c_str(), port_.c_str());
      return false;
    }
  }

  camera_->setPropertyFromParam(cv::CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
  camera_->setPropertyFromParam(cv::CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
  camera_->setPropertyFromParam(cv::CAP_PROP_FPS, "cv_cap_prop_fps");
  camera_->setPropertyFromParam(cv::CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
  camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
  camera_->setPropertyFromParam(cv::CAP_PROP_FORMAT, "cv_cap_prop_format");
  camera_->setPropertyFromParam(cv::CAP_PROP_MODE, "cv_cap_prop_mode");
  camera_->setPropertyFromParam(cv::CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
  camera_->setPropertyFromParam(cv::CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
  camera_->setPropertyFromParam(cv::CAP_PROP_SATURATION, "cv_cap_prop_saturation");
  camera_->setPropertyFromParam(cv::CAP_PROP_HUE, "cv_cap_prop_hue");
  camera_->setPropertyFromParam(cv::CAP_PROP_GAIN, "cv_cap_prop_gain");
  camera_->setPropertyFromParam(cv::CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
  camera_->setPropertyFromParam(cv::CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");
  camera_->setPropertyFromParam(cv::CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
  camera_->setPropertyFromParam(cv::CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif  // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif  // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
    camera_->setPropertyFromParam(cv::CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif  // CV_CAP_PROP_BUFFERSIZE

  // Timers
  read_tmr_ =
    this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / read_rate_)), std::bind(&Driver::read, this));
  publish_tmr_ =
    this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / publish_rate_)), std::bind(&Driver::proceed, this));

  cam_status_ = std::make_shared<std_msgs::msg::UInt8>();
  cam_status_->data = ONLINE;
  pub_cam_status_->publish(*cam_status_);
  publish_diagnostic(ONLINE);

  // set error image
  std::stringstream error_msg;
  auto upper_label = str_toupper(name_.c_str());
  error_msg << upper_label << " CONNECTING...";
  camera_->set_error_image(error_msg.str());

  // Log camera starting configuration
  RCLCPP_INFO(get_logger(), "(GOT VIDEO) %s: DEVICE: %d - SIZE: %dX%d - RATE: %d/%d - PROP_MODE: %f - EXPOSURE: %d",
              name_.c_str(), device_id_, int(camera_->getProperty(cv::CAP_PROP_FRAME_WIDTH)),
              int(camera_->getProperty(cv::CAP_PROP_FRAME_HEIGHT)), int(read_rate_), int(camera_->getProperty(cv::CAP_PROP_FPS)),
              float(camera_->getProperty(cv::CAP_PROP_FOURCC)), int(camera_->getProperty(cv::CAP_PROP_AUTO_EXPOSURE)));
  return true;
}

void Driver::read()
{
  if (!camera_->is_opened()) return;

  if (!camera_->grab())
  {
    camera_->close();
  }
}

void Driver::proceed()
{
  if (video_path_ != "") camera_->capture(flip_);

  else if (!camera_->is_opened())
  {
    read_tmr_->cancel();

    // set error image
    std::stringstream error_msg;
    auto upper_label = str_toupper(name_.c_str());
    error_msg << upper_label << " " << status_map_[DISCONNECTED];
    camera_->set_error_image(error_msg.str());

    cam_status_->data = DISCONNECTED;
    pub_cam_status_->publish(*cam_status_);
    publish_diagnostic(DISCONNECTED);

    while (reconnection_attempts_ < video_stream_recovery_tries_)
    {
      RCLCPP_WARN(get_logger(), "[%s] Reconnecting... attempt %d/%d", name_.c_str(), reconnection_attempts_ + 1,
                  video_stream_recovery_tries_);
      if (camera_->open(port_))
      {
        if (camera_->grab() && camera_->capture(flip_))
        {
          read_tmr_->reset();
          RCLCPP_WARN(get_logger(), "[%s] Reconnected", name_.c_str());
          reconnection_attempts_ = 0;
          cam_status_->data = ONLINE;
          pub_cam_status_->publish(*cam_status_);
          publish_diagnostic(ONLINE);
          break;
        }
        else
        {
          // set error image
          std::stringstream error_msg;
          auto upper_label = str_toupper(name_.c_str());
          error_msg << upper_label << " " << status_map_[READING_ERROR];
          camera_->set_error_image(error_msg.str());

          cam_status_->data = READING_ERROR;
          pub_cam_status_->publish(*cam_status_);
          publish_diagnostic(READING_ERROR);
        }
      }
      std::stringstream error_msg;
      auto upper_label = str_toupper(name_.c_str());
      error_msg << upper_label << " " << status_map_[DISCONNECTED];
      camera_->set_error_image(error_msg.str());
      reconnection_attempts_++;
      std::this_thread::sleep_for(std::chrono::seconds(video_stream_recovery_time_));
    }
    if (reconnection_attempts_ >= video_stream_recovery_tries_)
    {
      RCLCPP_ERROR(get_logger(), "[%s] Camera lost", name_.c_str());
      camera_->close();
      cam_status_->data = LOST;
      pub_cam_status_->publish(*cam_status_);
      publish_diagnostic(LOST);

      // set error image
      std::stringstream error_msg;
      auto upper_label = str_toupper(name_.c_str());
      error_msg << upper_label << " CAMERA " << status_map_[LOST];
      camera_->set_error_image(error_msg.str());

      read_tmr_->cancel();
      publish_tmr_->cancel();
    }
  }
  else
  {
    if (!camera_->capture(flip_))
    {
      RCLCPP_WARN(get_logger(), "[%s] Couldn't capture frame", name_.c_str());
    }
    else
    {
      if (always_rectify_ || (rectify_ && undistort_img_req_bool_))
        camera_->rectify();
    }
  }
}

rcl_interfaces::msg::SetParametersResult Driver::parameters_cb(const std::vector<rclcpp::Parameter>& parameters)
{
    auto result = param_manager_.parametersCb(parameters);
    /* Some extra logic after catch the new values if you need it */
    // reset the timer if publishing rate changed
    for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) 
    {
      if (name == "read_rate") 
      {
        read_rate_ = parameter.as_double();
        // Create timer with new read rate
        RCLCPP_WARN(get_logger(), "Setting new read rate to %f", read_rate_);
        read_tmr_->cancel();
        read_tmr_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / read_rate_)), 
                                  std::bind(&Driver::read, this));
      } 
      else if (name  == "publish_rate") 
      {
        publish_rate_ = parameter.as_double();
        // Create timer with new publish rate
        RCLCPP_WARN(get_logger(), "Setting new publish rate to %f", publish_rate_);
        publish_tmr_->cancel();
        publish_tmr_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / publish_rate_)),
                                               std::bind(&Driver::proceed, this));
      }
      else if (name == "cv_cap_prop_frame_width" || name == "cv_cap_prop_frame_height")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
        camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
      }
      else if (name == "cv_cap_prop_brightness")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
      }
      else if (name == "cv_cap_prop_contrast")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
      }
      else if (name == "cv_cap_prop_saturation")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_SATURATION, "cv_cap_prop_saturation");
      }
      else if (name == "cv_cap_prop_hue")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_HUE, "cv_cap_prop_hue");
      }
      else if (name == "cv_cap_prop_gain")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_GAIN, "cv_cap_prop_gain");
      }
      else if (name == "cv_cap_prop_exposure")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
      }
      else if (name == "cv_cap_prop_exposure")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
      }
      else if (name == "cv_cap_prop_auto_exposure")
      {
        camera_->setPropertyFromParam(cv::CAP_PROP_AUTO_EXPOSURE, "cv_cap_prop_auto_exposure");
      }
    }
    else if (type == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      if (name == "width")
      {
        width_ = parameter.as_int();
        this->set_parameter(rclcpp::Parameter("cv_cap_prop_frame_width", (double)width_));
        camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
      }
      else if (name == "height")
      {
        height_ = parameter.as_int();
        this->set_parameter(rclcpp::Parameter("cv_cap_prop_frame_height", (double)height_));
        camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
      }
    }
    else if (type == rclcpp::ParameterType::PARAMETER_STRING)
    {
      if (name == "intrinsic_file")
      {
        camera_->loadCameraInfo();
      }
    }
    else if (type == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      if (name == "roi_exposure")
      {
        roi_exposure_ = parameter.as_bool();
        setup();
      }
      else if (name == "always_publish")
      {
        always_publish_ = parameter.as_bool();
        if (cam_status_->data == PAUSED && always_publish_)
        {
          read_tmr_->reset();
          publish_tmr_->reset();
          cam_status_->data = ONLINE;
          pub_cam_status_->publish(*cam_status_);
          publish_diagnostic(ONLINE);
        }
      }
    }
  }
    return result;
}

void Driver::RestartNodeCb(shared_ptr_request_id const, shared_ptr_trigger_request const,
                          shared_ptr_trigger_response response)
{
  RCLCPP_WARN(get_logger(), "[%s] Restarting camera setup...", name_.c_str());
  reconnection_attempts_ = 0;
  setup();
  response->success = true;
  response->message = "Camera setup will be restarted";
}

void Driver::PauseImageCb(shared_ptr_request_id const, shared_ptr_bool_request const request,
                            shared_ptr_bool_response response)
{
  if (request->data && !always_publish_)
  {
    read_tmr_->cancel();
    publish_tmr_->cancel();
    cam_status_->data = PAUSED;
    pub_cam_status_->publish(*cam_status_);
    publish_diagnostic(PAUSED);
    response->success = true;
    response->message = "Camera read and pub paused";
    return;
  }
  else if (cam_status_->data == PAUSED)
  {
    read_tmr_->reset();
    publish_tmr_->reset();
    cam_status_->data = ONLINE;
    pub_cam_status_->publish(*cam_status_);
    publish_diagnostic(ONLINE);
    response->success = true;
    response->message = "Camera read and pub resumed";
    return;
  }
}


void Driver::ReleaseCamCb(shared_ptr_request_id const, shared_ptr_bool_request const request,
                            shared_ptr_bool_response response)
{
  if (request->data)
  {
    read_tmr_->cancel();
    publish_tmr_->cancel();
    camera_->close();
    cam_status_->data = TURNED_OFF;
    pub_cam_status_->publish(*cam_status_);
    publish_diagnostic(TURNED_OFF);
    response->success = true;
    response->message = "Camera device released";
    return;
  }
  else
  {
    setup();
    read_tmr_->reset();
    publish_tmr_->reset();
    cam_status_->data = ONLINE;
    pub_cam_status_->publish(*cam_status_);
    publish_diagnostic(ONLINE);
    response->success = true;
    response->message = "Camera read and pub resumed";
    return;
  }
}
void Driver::publish_diagnostic(Status status)
{
  auto message = diagnostic_msgs::msg::DiagnosticArray();
  message.header.stamp = this->get_clock()->now();
  auto status_msg = diagnostic_msgs::msg::DiagnosticStatus();
  status_msg.name = "USB Camera Status";
  status_msg.hardware_id = "/USB" + name_;

  switch (status) {
    case ONLINE:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status_msg.message = "Camera is online";
        break;
    case DISCONNECTED:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Camera is disconnected";
        break;
    case LOST:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status_msg.message = "Camera lost";
        break;
    case READING_ERROR:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Reading error from camera";
        break;
    case PAUSED:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Camera is paused";
        break;
    case TURNED_OFF:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Camera is turned off";
        break;
    default:
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status_msg.message = "Unrecognized camera status";
  }

  message.status.push_back(status_msg);
  pub_cam_diagnostic_->publish(message);
}

Driver::~Driver()
{
}

}  // namespace cv_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(cv_camera::Driver)
