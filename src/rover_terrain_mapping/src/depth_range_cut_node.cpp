// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// depth_range_cut node: /camera/depth_points in, range-cut and voxel-downsampled cloud out
// (default /terrain/depth_points). Runs on the Orin on the rover and stands in for it in sim.

#include <memory>
#include <string>

#include "nanopcl/bridge/ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_terrain_mapping/depth_range_cut.hpp"

namespace rover_terrain_mapping
{

class DepthRangeCutNode : public rclcpp::Node
{
public:
  explicit DepthRangeCutNode(const rclcpp::NodeOptions & o)
  : rclcpp::Node("depth_range_cut", o)
  {
    p_.min_range = static_cast<float>(declare_parameter<double>("min_range", p_.min_range));
    p_.max_range = static_cast<float>(declare_parameter<double>("max_range", p_.max_range));
    p_.voxel_size = static_cast<float>(declare_parameter<double>("voxel_size", p_.voxel_size));
    const auto in = declare_parameter<std::string>("input_topic", "/camera/depth_points");
    const auto out = declare_parameter<std::string>("output_topic", "/terrain/depth_points");
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        pub_->publish(nanopcl::to(rangeCut(nanopcl::from(*m), p_)));
      });
    RCLCPP_INFO(
      get_logger(), "depth_range_cut: %s -> %s, range %.2f to %.2f m, voxel %.3f m",
      in.c_str(), out.c_str(), p_.min_range, p_.max_range, p_.voxel_size);
  }

private:
  RangeCutParams p_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace rover_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_terrain_mapping::DepthRangeCutNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
