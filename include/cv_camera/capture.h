// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include <deque>
#include <string>
#include "cv_camera/exception.h"

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "cv_camera/srv/grab_frame.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "utils/console.hpp"
#include "utils/parameters.hpp"
#include "utils/string_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/**
 * @brief namespace of this package
 */
namespace cv_camera {

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
     * @param rect_image_topic_name name of topic to publish rectified image (this may be image_rect).
     * @param frame_id frame_id of publishing messages.
     * @param roi_exposure enable/disable ROI exposure control.
     * @param focus_threshold focus threshold.
     * @param check_focus_in_img_center check focus in image center.
     * @param stale_pixel_intensity_threshold gray-level change (0-255) for a pixel to count as "changed" when checking
     * for staleness.
     * @param stale_min_changed_pixels_pct minimum percentage of changed pixels for a sample to count as "real change"
     * rather than stale.
     * @param stale_window_size number of consecutive stale-check samples required, all unchanged, to report the camera
     * as stale.
     * @param buffer_size size of publisher buffer.
     */
    Capture(rclcpp::Node::SharedPtr node, const std::string& img_topic_name, const std::string& cam_info_topic_name,
            const std::string& rect_image_topic_name, const std::string& frame_id, const bool roi_exposure,
            double focus_threshold, bool check_focus_in_img_center, int stale_pixel_intensity_threshold,
            double stale_min_changed_pixels_pct, int stale_window_size, uint32_t buffer_size);

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
    bool open(const std::string& port);

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
    bool openFile(const std::string& file_path);

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
     * @param flip_vertical flip the image around vertical axis if true
     * @param flip_horizontal flip the image around horizontal axis if true
     * @return true if success to capture, false if not captured.
     */
    bool capture(bool flip_vertical, bool flip_horizontal);

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
    inline const sensor_msgs::msg::CameraInfo& getInfo() const { return info_; }

    /**
     * @brief accessor of cv::Mat
     *
     * you have to call capture() before call this.
     *
     * @return captured cv::Mat
     */
    inline const cv::Mat& getCvImage() const { return bridge_.image; }

    /**
     * @brief accessor of ROS Image message.
     *
     * you have to call capture() before call this.
     *
     * @return message pointer.
     */
    sensor_msgs::msg::Image::SharedPtr getImageMsgPtr();

    /**
     * @brief accessor of ROS Image message.
     *
     * you have to call capture() before call this.
     *
     * @return message pointer.
     */
    sensor_msgs::msg::Image::SharedPtr getUndistortedImageMsgPtr();

    /**
     * @brief try capture image width
     * @return true if success
     */
    inline bool setWidth(int32_t width) { return cap_.set(cv::CAP_PROP_FRAME_WIDTH, width); }

    /**
     * @brief try capture image height
     * @return true if success
     */
    inline bool setHeight(int32_t height) { return cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height); }

    /**
     * @brief set CV_PROP_*
     * @return true if success
     */
    bool setPropertyFromParam(int property_id, const std::string& param_name);

    /**
     * @brief get CV_PROP_*
     * @return value of property
     */
    double getProperty(int property_id);

    /**
     * @brief Set focus threshold
     * @param focus_threshold focus threshold
     */
    void setFocusThreshold(double focus_threshold);

    /**
     * @brief Set check focus in image center
     * @param check_focus_in_img_center check focus in image center
     */
    void setCheckFocusInImgCenter(bool check_focus_in_img_center);

    /**
     * @brief Set the pixel intensity threshold used to decide if a single pixel changed.
     */
    void setStalePixelIntensityThreshold(int stale_pixel_intensity_threshold);

    /**
     * @brief Set the minimum percentage of changed pixels for a sample to count as an actual change.
     */
    void setStaleMinChangedPixelsPct(double stale_min_changed_pixels_pct);

    /**
     * @brief Set the number of consecutive unchanged samples required to report the camera as stale.
     *        Resets any in-progress sample window, since a mid-window size change can't be evaluated consistently.
     */
    void setStaleWindowSize(int stale_window_size);

    /**
     * @brief Set black error image in case of error.
     */
    void set_error_image(const std::string& error_msg, int width = 640, int height = 360);

    /**
     * @brief Check if the current frame is stale (sustained lack of change)
     * @return true if the last stale_window_size_ samples all showed no real change
     */
    bool isFrameStale();

    /**
     * @brief Take one stale-detection sample, diff the current frame against the frame
     *        from the previous call
     */
    void updateStaleFrameEvidence();

    /**
     * @brief Discard all stale-frame evidence: clears the sample window and the reference
     *        frame. Call whenever sampling stops (pause, device release), so evidence
     *        gathered before the stop can't be reported later as if it were current.
     */
    void resetStaleEvidence();

    /**
     * @brief Compute the percentage of pixels that changed by more than intensity_threshold
     *        between two frames, after downsampling both to a small grayscale image.
     * @param current current frame
     * @param previous previous frame (must be the same size/type as current)
     * @param intensity_threshold per-pixel gray-level change (0-255) to count as "changed"
     * @return percentage (0-100) of pixels that changed
     */
    static double computeChangedPixelsPct(const cv::Mat& current, const cv::Mat& previous, int intensity_threshold);

    /**
     * @brief rescale camera calibration to another resolution
     */
    void rescaleCameraInfo(uint width, uint height);

    /**
     * @brief rescale camera calibration to another resolution
     * @param width new width
     * @param height new height
     * @return true if success using a hd_intrinsic_file instead of rescaling
     */
    bool rescaleFromFile(uint width, uint height);

    /**
     * @brief initialize undistort rectify map
     */
    void initUndistortRectifyMap();
    /**
     * @brief Check if the image is empty
     * @return True if image is empty, false otherwise
     */
    bool is_empty();
    /**
     * @brief Check if the image is focused
     * @return True if image is focused, false otherwise
     */
    bool isFocused();

   private:
    /**
     * @brief Get Laplacian variance of last captured image
     * @return Laplacian variance value
     */
    double getLaplacianVariance();

    /**
     * @brief Get Brenner score of last captured image
     * @return Brenner score value
     */
    double getBrennerScore();

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
     * @brief Focus threshold for focus detection
     */
    double focus_threshold_;
    /**
     * @brief Check focus in image center
     */
    bool check_focus_in_img_center_;
    /**
     * @brief Gray-level change (0-255) for a pixel to count as "changed" in the stale check
     */
    int stale_pixel_intensity_threshold_;
    /**
     * @brief Minimum percentage of changed pixels for a sample to count as "real change"
     */
    double stale_min_changed_pixels_pct_;
    /**
     * @brief Number of consecutive unchanged samples required to report the camera as stale
     */
    size_t stale_window_size_;
    /**
     * @brief Sliding window of the most recent stale-check samples (true = unchanged)
     */
    std::deque<bool> stale_sample_window_;
    /**
     * @brief Whether updateStaleFrameEvidence() has captured a reference frame yet
     */
    bool has_prior_sample_ = false;
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
     * @brief JP5: decode MJPG frames ourselves (CAP_PROP_CONVERT_RGB=0) with size/marker validation.
     * OpenCV 4.11's V4L2 backend segfaults in imdecode when uvcvideo (5.10) reports a bogus bytesused.
     */
    bool raw_mjpg_ = false;
    void configureRawDecode();

    /**
     * @brief this stores last captured image.
     */
    cv_bridge::CvImage bridge_;

    /**
     * @brief this stores previous captured image for stale frame detection.
     */
    cv_bridge::CvImage previous_bridge_;

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

}  // namespace cv_camera

#endif  // CV_CAMERA_CAPTURE_H