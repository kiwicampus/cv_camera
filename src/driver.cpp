// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.hpp"
#include <string>

namespace {
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}  // namespace

namespace cv_camera {

Driver::Driver(const rclcpp::NodeOptions& options) : Node("cv_camera", options)
{
    auto ptr = std::shared_ptr<Driver>(this, [](Driver*) {});
    this->setup();
}

bool Driver::setup()
{
    double hz_pub(DEFAULT_RATE), hz_read(DEFAULT_RATE);
    std::string frame_id("camera");
    std::string file_path("");

    // Declare Custom Parameters
    this->declare_parameter("port", "");
    this->declare_parameter("name", "cam_name");
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("read_rate", 30.0);
    this->declare_parameter("flip", 0);
    this->declare_parameter("topic_name", "/video_mapping/test");
    this->declare_parameter("cam_info_topic", "None");
    this->declare_parameter("topic_sim", "/gazebo/streaming_cam_test/image_raw");
    this->declare_parameter("cam_info_period", 5);
    this->declare_parameter("intrinsic", false);
    this->declare_parameter("video_path", "None");
    this->declare_parameter("object_detection", false);
    this->declare_parameter("semantic_segmentation", false);
    this->declare_parameter("qr_scan", false);
    this->declare_parameter("data_capture_csv", false);
    this->declare_parameter("data_capture_video", false);
    this->declare_parameter("device_id", -1);
    // this->declare_parameter("file_path", "");
    // this->declare_parameter("frame_id", "camera");

    // Declare CV2 parameters
    this->declare_parameter("width", 640.0);
    this->declare_parameter("height", 360.0);
    this->declare_parameter("fourcc", (double)cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Get Custom Parameters
    this->get_parameter("publish_rate", hz_pub);
    this->get_parameter("read_rate", hz_read);
    this->get_parameter("device_id", device_id);
    // this->get_parameter("frame_id", frame_id);
    this->get_parameter("topic_name", topic_name);
    this->get_parameter("name", name);

    // Timers
    m_proceed_tmr =
        this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / hz_read)), std::bind(&Driver::proceed, this));

    camera_.reset(new Capture(shared_from_this(), topic_name, PUBLISHER_BUFFER_SIZE, frame_id));

    if (this->get_parameter("file", file_path) && file_path != "")
    {
        camera_->openFile(file_path);
    }
    else if (this->get_parameter("port", port) && port != "")
    {
        if (!camera_->open(port))
        {
            RCLCPP_WARN(get_logger(), "Couldnt open camera by port");
            // return false;
        }
    }
    else
    {
        if (!camera_->open(device_id))
        {
            RCLCPP_WARN(get_logger(), "Couldnt open camera by device id");
            // return false;
        }
    }

    camera_->setPropertyFromParam(cv::CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
    camera_->setPropertyFromParam(cv::CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
    camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_WIDTH, "width");
    camera_->setPropertyFromParam(cv::CAP_PROP_FRAME_HEIGHT, "height");
    camera_->setPropertyFromParam(cv::CAP_PROP_FPS, "cv_cap_prop_fps");
    camera_->setPropertyFromParam(cv::CAP_PROP_FOURCC, "fourcc");
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

    m_pub_cam_status = this->create_publisher<std_msgs::msg::UInt8>("/video_mapping/" + name + "/status", 1);

    cam_status = std::make_shared<std_msgs::msg::UInt8>();
    cam_status->data = 1;
    m_pub_cam_status->publish(*cam_status);

    publish_tmr_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / hz_pub)), [&]() {
        if (camera_->capture())
        {
        }
    });
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif  // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
    camera_->setPropertyFromParam(cv::CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif  // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
    camera_->setPropertyFromParam(cv::CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif  // CV_CAP_PROP_BUFFERSIZE

    rate_.reset(new rclcpp::Rate(hz_read));
    return true;
}

void Driver::proceed()
{
    if (!camera_->grab())
    {
        std::chrono::milliseconds video_recovery_time(VIDEO_STREAM_CAM_RECOVERY_TIME * 1000);

        while ((!camera_->open(port)) && m_reconnection_attempts < VIDEO_STREAM_CAM_RECOVERY_TRIES)
        {
            m_reconnection_attempts += 1;
            RCLCPP_ERROR(get_logger(), "not possible to open %s. (device %s), retrying... %d/%d ", name.c_str(),
                         port.c_str(), m_reconnection_attempts, VIDEO_STREAM_CAM_RECOVERY_TRIES);
            camera_->open(port);
            std::this_thread::sleep_for(video_recovery_time);
            cam_status->data = 2;
            m_pub_cam_status->publish(*cam_status);
            if (camera_->grab())
            {
                m_reconnection_attempts = 0;
                cam_status->data = 1;
                m_pub_cam_status->publish(*cam_status);
                break;
            }
        }
        if (m_reconnection_attempts >= VIDEO_STREAM_CAM_RECOVERY_TRIES)
        {
            RCLCPP_ERROR(get_logger(), "%s camera Lost", name.c_str());
            camera_->close();
            cam_status->data = 3;
            m_pub_cam_status->publish(*cam_status);
            m_proceed_tmr->cancel();
            return;
        }
    };
    rate_->sleep();
}

Driver::~Driver() {}

}  // namespace cv_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(cv_camera::Driver)