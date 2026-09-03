// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <algorithm>
#include <sstream>
#include <string>

namespace cv_camera {

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(rclcpp::Node::SharedPtr node, const std::string& img_topic_name,
                 const std::string& cam_info_topic_name, const std::string& rect_img_topic_name,
                 const std::string& frame_id, const bool roi_exposure, double focus_threshold,
                 bool check_focus_in_img_center, int stale_pixel_intensity_threshold,
                 double stale_min_changed_pixels_pct, int stale_window_size, uint32_t buffer_size)
    : node_(node)
    , it_(node_)
    , img_topic_name_(img_topic_name)
    , cam_info_topic_name_(cam_info_topic_name)
    , rect_img_topic_name_(rect_img_topic_name)
    , frame_id_(frame_id)
    , roi_exposure_(roi_exposure)
    , focus_threshold_(focus_threshold)
    , check_focus_in_img_center_(check_focus_in_img_center)
    , stale_pixel_intensity_threshold_(stale_pixel_intensity_threshold)
    , stale_min_changed_pixels_pct_(stale_min_changed_pixels_pct)
    , stale_window_size_(static_cast<size_t>(stale_window_size))
    , buffer_size_(buffer_size)
    , info_manager_(node_.get(), frame_id)
    , capture_delay_(rclcpp::Duration(0, 0.0))
{
    int dur = 0;
    m_pub_image_ptr =
        node->create_publisher<sensor_msgs::msg::Image>(img_topic_name_, rclcpp::QoS(rclcpp::SensorDataQoS()));
    m_pub_rect_image_ptr =
        node->create_publisher<sensor_msgs::msg::Image>(rect_img_topic_name_, rclcpp::QoS(rclcpp::SensorDataQoS()));
    m_pub_camera_info_ptr = node->create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic_name_,
                                                                                 rclcpp::QoS(rclcpp::SensorDataQoS()));
    node_->get_parameter_or("capture_delay", dur, dur);
    this->capture_delay_ = rclcpp::Duration(dur, 0.0);
}

void Capture::loadCameraInfo()
{
    std::string url;
    if (node_->get_parameter("intrinsic_file", url))
    {
        if (url == "") return;
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
        RCLCPP_ERROR(node_->get_logger(), "[%s] No distortion coefficients found, rectification cannot be done",
                     node_->get_name());
        return;
    }

    initUndistortRectifyMap();

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
            RCLCPP_ERROR(node_->get_logger(), "[%s] Setting with code %d and value %f failed", node_->get_name(), code,
                         value);
        }
    }
}

void Capture::initUndistortRectifyMap()
{
    cv::Mat K = cv::Mat(3, 3, CV_64F, info_.k.data());
    cv::Mat R = cv::Mat(3, 3, CV_64F, info_.r.data());
    cv::Mat P = cv::Mat(3, 4, CV_64F, info_.p.data());

    // select depending on distortion model
    if (info_.distortion_model == "plumb_bob")
    {
        cv::Mat D = cv::Mat(1, 5, CV_64F, info_.d.data());
        cv::initUndistortRectifyMap(K, D, R, P, cv::Size(info_.width, info_.height), CV_16SC2, map1_, map2_);
    }
    else if (info_.distortion_model == "equidistant" || info_.distortion_model == "fisheye")
    {
        cv::Mat D = cv::Mat(1, 4, CV_64F, info_.d.data());
        cv::fisheye::initUndistortRectifyMap(K, D, R, P, cv::Size(info_.width, info_.height), CV_16SC2, map1_, map2_);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Unsupported distortion model: %s", node_->get_name(),
                     info_.distortion_model.c_str());
        return;
    }
}

void Capture::rescaleCameraInfo(uint width, uint height)
{
    if (info_.width == width && info_.height == height)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] No rescaling needed", node_->get_name());
        return;
    }

    if (rescaleFromFile(width, height)) return;

    RCLCPP_WARN(node_->get_logger(), "[%s] Rescaling camera parameters from %dx%d to %dx%d", node_->get_name(),
                info_.width, info_.height, width, height);

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

    initUndistortRectifyMap();
}

bool Capture::rescaleFromFile(uint width, uint height)
{
    std::string hd_url;
    if (!node_->get_parameter("hd_intrinsic_file", hd_url) && hd_url == "") return false;

    hd_url = "file://" + hd_url;

    if (!info_manager_.validateURL(hd_url)) return false;

    info_manager_.loadCameraInfo(hd_url);
    sensor_msgs::msg::CameraInfo hd_info = info_manager_.getCameraInfo();

    // If width and height are the same, we can use the hd_info instead of rescaling
    if (hd_info.width != width || hd_info.height != height) return false;

    RCLCPP_INFO(node_->get_logger(), "[%s] Changing camera parameters from %dx%d to %dx%d as HD parameters were found",
                node_->get_name(), hd_info.width, hd_info.height, width, height);

    info_ = hd_info;

    initUndistortRectifyMap();

    return true;
}

bool Capture::open(int32_t device_id)
{
    cap_.open(device_id, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
        return false;
    }

    configureRawDecode();
    loadCameraInfo();
    return true;
}

bool Capture::open(const std::string& port)
{
    std::string device;

    if (det_device_path(port.c_str()) != "-1")
    {
        device = "/dev/video" + det_device_path(port.c_str());
    }
    else
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Unable to determine device for port %s.", node_->get_name(),
                         port.c_str());
        return false;
    }

    cap_.open(device, cv::CAP_V4L2);

    if (!cap_.isOpened())
    {
        return false;
    }

    configureRawDecode();
    loadCameraInfo();
    return true;
}

void Capture::open() { open(0); }

bool Capture::openFile(const std::string& file_path)
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

    // OpenCV 4.11 (JP5) throws cv::Exception from the V4L2 MJPG path when the camera returns an empty
    // frame (imdecode assertion); uncaught it aborts the whole component container (2026-09-03).
    try
    {
        return cap_.grab();
    } catch (const cv::Exception& e)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] grab threw: %s", node_->get_name(),
                             e.what());
        return false;
    }
}

bool Capture::capture(bool flip_vertical, bool flip_horizontal)
{
    try
    {
        if (raw_mjpg_)
        {
            // Raw compressed buffer: Mat(1, bytesused, CV_8U) wrapping the V4L2 mmap area.
            cv::Mat raw;
            if (!cap_.retrieve(raw) || raw.empty()) return false;
            const size_t n = raw.total() * raw.elemSize();
            const size_t max_n = static_cast<size_t>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)) *
                                     static_cast<size_t>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)) * 2 +
                                 4096;
            if (n < 4 || n > max_n || raw.data[0] != 0xFF || raw.data[1] != 0xD8)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "[%s] dropping invalid MJPG frame (%zu bytes)", node_->get_name(), n);
                return false;
            }
            cv::imdecode(raw, cv::IMREAD_COLOR, &bridge_.image);
            if (bridge_.image.empty()) return false;
        }
        else if (!cap_.retrieve(bridge_.image))
        {
            return false;
        }
    } catch (const cv::Exception& e)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] retrieve threw: %s",
                             node_->get_name(), e.what());
        return false;
    }
    if (flip_vertical) cv::flip(bridge_.image, bridge_.image, 0);
    if (flip_horizontal) cv::flip(bridge_.image, bridge_.image, 1);

    // Our custom made exposure set depending on ROI
    if (roi_exposure_) custom_roi_exposure(bridge_.image);

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

    // Publish the image.
    publish(std::move(msg));

    // Fill the cam info message.
    info_.header.stamp = timestamp_;
    info_.header.frame_id = frame_id_;

    m_pub_camera_info_ptr->publish(info_);

    return true;
}

void Capture::custom_roi_exposure(cv::Mat& frame)
{
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // disable auto exposure
    double exposure = cap_.get(cv::CAP_PROP_EXPOSURE);

    // define ROI for the center of the image
    cv::Rect roi(frame.cols / 4, frame.rows / 4, frame.cols / 2, frame.rows / 2);
    cv::Mat roiFrame = frame(roi);

    // convert the ROI to grayscale
    cv::Mat grayRoi;
    cv::cvtColor(roiFrame, grayRoi, cv::COLOR_BGR2GRAY);

    // calculate the histogram
    int histSize = 256;
    float range[] = {0, 256};  // the upper boundary is exclusive
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&grayRoi, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // analyze the histogram to decide on exposure adjustment
    double underExposedThreshold = 0.1 * grayRoi.total();
    double overExposedThreshold = 0.1 * grayRoi.total();
    // Compute overexposed and underexposed pixels
    double underExposedPixels = 0;
    double overExposedPixels = 0;
    for (int i = 0; i < histSize; i++)
    {
        if (i < 50)
        {
            underExposedPixels += hist.at<float>(i);
        }
        else if (i > 200)
        {
            overExposedPixels += hist.at<float>(i);
        }
    }

    if (underExposedPixels > underExposedThreshold)
    {  // underexposed
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Underexposed: %f vs cap exposure: %f", node_->get_name(),
                     underExposedPixels, cap_.get(cv::CAP_PROP_EXPOSURE));
        exposure += 1;
    }
    else if (overExposedPixels > overExposedThreshold)
    {  // overexposed
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Overexposed: %f vs cap exposure: %f", node_->get_name(),
                     overExposedPixels, cap_.get(cv::CAP_PROP_EXPOSURE));
        exposure -= 1;
    }

    cap_.set(cv::CAP_PROP_EXPOSURE, exposure);  // set new exposure
}

void Capture::rectify()
{
    // Dont publish image if empty
    if (bridge_.image.empty())
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Frame is empty.", node_->get_name());
        return;
    }

    rect_image_ = bridge_.image;

    // return if map empty
    if (map1_.empty() || map2_.empty())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "[%s] Map1 or Map2 empty. Wont rectify the image. Check that calibration params are available",
            node_->get_name());
        return;
    }
    cv::remap(rect_image_, rect_image_, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    // Update message
    sensor_msgs::msg::Image::UniquePtr msg_image(new sensor_msgs::msg::Image());
    msg_image->header.stamp = timestamp_;
    msg_image->header.frame_id = frame_id_;
    msg_image->height = rect_image_.rows;
    msg_image->width = rect_image_.cols;
    msg_image->encoding = mat_type2encoding(rect_image_.type());
    msg_image->is_bigendian = false;
    msg_image->step = static_cast<sensor_msgs::msg::Image::_step_type>(rect_image_.step);
    msg_image->data.assign(rect_image_.datastart, rect_image_.dataend);

    // Publish rectified image
    m_pub_rect_image_ptr->publish(std::move(msg_image));
}

void Capture::publish(sensor_msgs::msg::Image::UniquePtr msg) { m_pub_image_ptr->publish(std::move(msg)); }

void Capture::close() { cap_.release(); }

bool Capture::is_opened() { return cap_.isOpened(); }

sensor_msgs::msg::Image::SharedPtr Capture::getImageMsgPtr()
{
    sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    msg->header.stamp = timestamp_;
    msg->header.frame_id = frame_id_;
    msg->height = bridge_.image.rows;
    msg->width = bridge_.image.cols;
    msg->encoding = mat_type2encoding(bridge_.image.type());
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(bridge_.image.step);
    msg->data.assign(bridge_.image.datastart, bridge_.image.dataend);

    return msg;
}

sensor_msgs::msg::Image::SharedPtr Capture::getUndistortedImageMsgPtr()
{
    rectify();

    sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    msg->header.stamp = timestamp_;
    msg->header.frame_id = frame_id_;
    msg->height = rect_image_.rows;
    msg->width = rect_image_.cols;
    msg->encoding = mat_type2encoding(rect_image_.type());
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(rect_image_.step);
    msg->data.assign(rect_image_.datastart, rect_image_.dataend);

    return msg;
}

bool Capture::setPropertyFromParam(int property_id, const std::string& param_name)
{
    if (cap_.isOpened())
    {
        double value = 0.0;
        if (node_->get_parameter(param_name, value))
        {
            if (!cap_.set(property_id, value) && value != getProperty(property_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Setting with code %d and value %f failed", node_->get_name(),
                             property_id, value);
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

void Capture::setFocusThreshold(double focus_threshold) { focus_threshold_ = focus_threshold; }

void Capture::setCheckFocusInImgCenter(bool check_focus_in_img_center)
{
    check_focus_in_img_center_ = check_focus_in_img_center;
}

void Capture::setStalePixelIntensityThreshold(int stale_pixel_intensity_threshold)
{
    stale_pixel_intensity_threshold_ = stale_pixel_intensity_threshold;
}

void Capture::setStaleMinChangedPixelsPct(double stale_min_changed_pixels_pct)
{
    stale_min_changed_pixels_pct_ = stale_min_changed_pixels_pct;
}

void Capture::setStaleWindowSize(int stale_window_size)
{
    stale_window_size_ = static_cast<size_t>(stale_window_size);
    stale_sample_window_.clear();
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
void Capture::set_error_image(const std::string& error_msg, int width, int height)
{
    // Create frame with black image
    cv::Mat frame_aux = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);

    // Set font parameters
    int font = cv::FONT_HERSHEY_SIMPLEX;
    // empirical formula so that: when w=640 -> font=1.5 and w=1920 -> font=3.5
    double fontScale = 0.0015625 * width + 0.3;
    int thickness = 4;

    // Get text size & calculate text position
    cv::putText(frame_aux, error_msg, cv::Point(int(width * 0.15), int(height * 0.5)), font, fontScale,
                cv::Scalar(255, 255, 255), thickness);

    // Update message
    sensor_msgs::msg::Image::UniquePtr msg_image(new sensor_msgs::msg::Image());
    msg_image->header.stamp = timestamp_;
    msg_image->header.frame_id = frame_id_;
    msg_image->height = frame_aux.rows;
    msg_image->width = frame_aux.cols;
    msg_image->encoding = mat_type2encoding(frame_aux.type());
    msg_image->is_bigendian = false;
    msg_image->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_aux.step);
    msg_image->data.assign(frame_aux.datastart, frame_aux.dataend);

    // publish error image
    publish(std::move(msg_image));
}

bool Capture::is_empty()
{
    if (bridge_.image.empty())
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] Frame is empty.", node_->get_name());
        return true;
    }
    return false;
}

bool Capture::isFocused()
{
    // Calculate image focus using Brenner score method
    // Higher score indicates more edges/details are in focus
    double brennerScore = getBrennerScore();
    RCLCPP_INFO(node_->get_logger(), "[%s] Brenner score: %f", node_->get_name(), brennerScore);
    RCLCPP_INFO(node_->get_logger(), "[%s] focus_threshold_: %f", node_->get_name(), focus_threshold_);

    // Return true if score is above threshold (image is focused)
    return brennerScore >= focus_threshold_;
}

double Capture::getLaplacianVariance()
{
    cv::Mat frame = bridge_.image;
    if (check_focus_in_img_center_)
    {
        // Get the center of the image
        frame = frame(cv::Rect(frame.cols / 4, frame.rows / 4, frame.cols / 2, frame.rows / 2));
    }
    cv::Mat gray, laplacian;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);  // Convert to grayscale
    cv::Laplacian(gray, laplacian, CV_64F);         // Apply Laplacian filter

    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);

    double variance = stddev[0] * stddev[0];
    return variance;
}

double Capture::getBrennerScore()
{
    cv::Mat frame = bridge_.image;
    cv::Mat gray, gray_int16, focus_measure;

    if (check_focus_in_img_center_)
    {
        // Get the center of the image
        frame = frame(cv::Rect(frame.cols / 4, frame.rows / 4, frame.cols / 2, frame.rows / 2));
    }

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray_int16, CV_16S);

    int height = gray_int16.rows;
    int width = gray_int16.cols;

    cv::Mat horizontal_diff = cv::Mat::zeros(height, width, CV_16S);
    cv::Mat vertical_diff = cv::Mat::zeros(height, width, CV_16S);

    // Compute horizontal differences each 2 pixels
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width - 2; j++)
        {
            int diff = gray_int16.at<short>(i, j + 2) - gray_int16.at<short>(i, j);
            horizontal_diff.at<short>(i, j) = std::max(0, diff);
        }
    }

    // Compute vertical differences each 2 pixels
    for (int i = 0; i < height - 2; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int diff = gray_int16.at<short>(i + 2, j) - gray_int16.at<short>(i, j);
            vertical_diff.at<short>(i, j) = std::max(0, diff);
        }
    }

    // Compute focus measure: max(horizontal_diff, vertical_diff)^2
    focus_measure = cv::Mat::zeros(height, width, CV_32F);
    cv::max(horizontal_diff, vertical_diff, focus_measure);
    cv::Scalar mean_focus = cv::mean(focus_measure);

    double mean_focus_value = mean_focus[0];

    return mean_focus_value;
}

double Capture::computeChangedPixelsPct(const cv::Mat& current, const cv::Mat& previous, int intensity_threshold)
{
    cv::Mat current_small, previous_small, current_gray, previous_gray, diff, thresholded;

    cv::resize(current, current_small, cv::Size(64, 64), 0, 0, cv::INTER_AREA);
    cv::resize(previous, previous_small, cv::Size(64, 64), 0, 0, cv::INTER_AREA);

    cv::cvtColor(current_small, current_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(previous_small, previous_gray, cv::COLOR_BGR2GRAY);

    cv::absdiff(current_gray, previous_gray, diff);
    cv::threshold(diff, thresholded, intensity_threshold, 255, cv::THRESH_BINARY);

    int changed_pixels = cv::countNonZero(thresholded);
    int total_pixels = thresholded.rows * thresholded.cols;

    return (static_cast<double>(changed_pixels) / total_pixels) * 100.0;
}

void Capture::updateStaleFrameEvidence()
{
    // First sample ever (or right after a reset)
    if (!has_prior_sample_)
    {
        if (bridge_.image.empty()) return;
        previous_bridge_.image = bridge_.image.clone();
        has_prior_sample_ = true;
        return;
    }

    // true  = this sample looked "frozen" (too little change) -> a vote TOWARD stale
    // false = this sample showed real change                  -> a vote AGAINST stale
    bool sample_looked_frozen;

    if (bridge_.image.empty() || previous_bridge_.image.empty() ||
        bridge_.image.size() != previous_bridge_.image.size() || bridge_.image.type() != previous_bridge_.image.type())
    {
        // Can't tell maybe some (resolution/format change, reconnection, etc.)
        sample_looked_frozen = false;
    }
    else
    {
        double changed_pct =
            computeChangedPixelsPct(bridge_.image, previous_bridge_.image, stale_pixel_intensity_threshold_);
        sample_looked_frozen = changed_pct < stale_min_changed_pixels_pct_;
        RCLCPP_DEBUG(node_->get_logger(), "[%s] stale-check sample: changed_pct=%.3f%% (min=%.3f%%) -> %s",
                     frame_id_.c_str(), changed_pct, stale_min_changed_pixels_pct_,
                     sample_looked_frozen ? "frozen" : "changed");
    }

    stale_sample_window_.push_back(sample_looked_frozen);
    while (stale_sample_window_.size() > stale_window_size_)
    {
        stale_sample_window_.pop_front();
    }

    previous_bridge_.image = bridge_.image.clone();
}

void Capture::resetStaleEvidence()
{
    stale_sample_window_.clear();
    has_prior_sample_ = false;
}

bool Capture::isFrameStale()
{
    // Stale only once the window is full AND every single sample looked frozen,
    // one sample with real change (false) anywhere in the window clears it.
    if (stale_sample_window_.size() < stale_window_size_) return false;
    return std::all_of(stale_sample_window_.begin(), stale_sample_window_.end(),
                       [](bool looked_frozen) { return looked_frozen; });
}

}  // namespace cv_camera

void Capture::configureRawDecode()
{
    const int fcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    const std::string fourcc{static_cast<char>(fcc & 0xFF), static_cast<char>((fcc >> 8) & 0xFF),
                             static_cast<char>((fcc >> 16) & 0xFF), static_cast<char>((fcc >> 24) & 0xFF)};
    raw_mjpg_ = (fourcc == "MJPG") && cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);
    RCLCPP_INFO(node_->get_logger(), "[%s] fourcc %s, raw MJPG decode %s", node_->get_name(), fourcc.c_str(),
                raw_mjpg_ ? "enabled" : "disabled");
}
