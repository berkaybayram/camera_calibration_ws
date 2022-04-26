#include "camera_calibration/CameraCalibrationNode.hpp"

#include <string>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace CameraCalibration {

CameraCalibrationNode::CameraCalibrationNode(
    const rclcpp::NodeOptions &node_options)
    : Node("CameraCalibrationNode", node_options),
      name_(rclcpp::Node::get_name()), // Get node name
      calibrated_image_publisher_(
          this->create_publisher<sensor_msgs::msg::Image>("/calibrated_img",
                                                          10)) // Init publisher
{
  // Declare parameters in the class.
  this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>());
  this->declare_parameter<std::vector<double>>("distortion", std::vector<double>());

  // Get from parameters from param server and set in class parameters.
  this->get_parameter("camera_matrix", camera_matrix_param_);
  this->get_parameter("distortion", distortion_param_);

  // Init parameters as CV Matrix.
  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_param_.data());
  distortion_ = cv::Mat_<double>(distortion_param_);

  // Bind callback function with lambda.
  auto callback = [this](sensor_msgs::msg::Image::SharedPtr msg) {
    this->topic_callback(msg);
  };

  // Init subscriber.
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, callback);
}

void CameraCalibrationNode::topic_callback(
    sensor_msgs::msg::Image::SharedPtr msg) const {

  // Convert ROS image message to OpenCV image.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "e");
    return;
  }

  // Correct the input image.
  std::shared_ptr<cv::Mat> img_p(new cv::Mat());
  cv::undistort(cv_ptr->image, *img_p, camera_matrix_, distortion_);

  // Convert OpenCV image to ROS message.
  sensor_msgs::msg::Image::SharedPtr msg_to_pub =
      cv_bridge::CvImage(msg->header, "bgr8", *img_p).toImageMsg();

  // Publish ros image message.
  calibrated_image_publisher_->publish(*msg_to_pub);
}

} // namespace CameraCalibration

RCLCPP_COMPONENTS_REGISTER_NODE(CameraCalibration::CameraCalibrationNode)