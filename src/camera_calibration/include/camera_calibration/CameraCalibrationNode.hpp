//
// Created by berkay on 25.04.2022.
//

#ifndef BUILD_CAMERACALIBRATIONNODE_HPP
#define BUILD_CAMERACALIBRATIONNODE_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/mat.hpp>


namespace CameraCalibration {

class CameraCalibrationNode : public rclcpp::Node {
public:
  CameraCalibrationNode(const rclcpp::NodeOptions &node_options);

private:
  // Node
  std::string name_;

  // Parameters
  std::vector<double> camera_matrix_param_;
  std::vector<double> distortion_param_;

  cv::Mat_<double> camera_matrix_;
  cv::Mat1d distortion_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr calibrated_image_publisher_;

  // Callbacks
  void topic_callback(sensor_msgs::msg::Image::SharedPtr msg) const;

};

} // namespace CameraCalibration

#endif // BUILD_CAMERACALIBRATIONNODE_HPP
