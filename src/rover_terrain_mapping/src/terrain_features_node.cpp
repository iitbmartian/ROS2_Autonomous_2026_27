// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// terrain_features_node: feature layers of the SPEC.md map contract from an elevation map.
//
//   in   /terrain/local           elevation, variance, observations
//   out  /terrain/local_features  the input layers plus slope, slope_std, roughness, step,
//                                 normal_x, normal_y, normal_z, curvature, negative_obstacle
//
// Window sizes come from rover_physical.yaml: the plane fit spans wheelbase / 2 (chassis scale),
// the step window a wheel radius (wheel scale). See features.hpp for the definitions.

#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rover_terrain_mapping/features.hpp"
#include "rover_traversability/grid_map_msg.hpp"
#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/pose_evaluation.hpp"
#include "rover_traversability/core/yaw_evaluation.hpp"
#include "rover_traversability/layer_names.hpp"

namespace rover_terrain_mapping
{

namespace L = rover_traversability::layers;
using rover_traversability::geometryOf;
using rover_traversability::GridGeometry;
using rover_traversability::readLayer;
using rover_traversability::writeLayer;

class TerrainFeaturesNode : public rclcpp::Node
{
public:
  explicit TerrainFeaturesNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("terrain_features", options)
  {
    const auto in = declare_parameter<std::string>("input_topic", "/terrain/local");
    const auto out = declare_parameter<std::string>("output_topic", "/terrain/local_features");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar3d_link");
    const auto phys_path = declare_parameter<std::string>(
      "physical_params_file", rover_traversability::core::defaultPhysicalParamsPath());
    fp_.min_fill = declare_parameter<double>("min_fill", fp_.min_fill);
    fp_.spread_fraction = declare_parameter<double>("spread_fraction", fp_.spread_fraction);
    fp_.threads = static_cast<int>(declare_parameter<int>("feature_threads", fp_.threads));
    np_.max_range = declare_parameter<double>("negative.max_range", np_.max_range);
    np_.min_gap = declare_parameter<double>("negative.min_gap", np_.min_gap);
    np_.row_spacing = declare_parameter<double>("negative.row_spacing", np_.row_spacing);
    np_.gap_factor = declare_parameter<double>("negative.gap_factor", np_.gap_factor);
    np_.min_beam_angle = declare_parameter<double>("negative.min_beam_angle", np_.min_beam_angle);
    np_.min_range = declare_parameter<double>("negative.min_range", np_.min_range);
    np_.azimuth_bins = static_cast<int>(
      declare_parameter<int>("negative.azimuth_bins", np_.azimuth_bins));

    yaw_.yaw_bins = static_cast<int>(declare_parameter<int>("yaw_bins", yaw_.yaw_bins));
    yaw_.threads = static_cast<int>(declare_parameter<int>("threads", yaw_.threads));
    eval_.risk_kappa = declare_parameter<double>("risk_kappa", eval_.risk_kappa);
    evaluate_yaws_ = declare_parameter<bool>("evaluate_yaws", true);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    fill_footprint_ = declare_parameter<bool>("fill_footprint", true);
    // Same padding as the mapping node's self filter, which empties exactly this box.
    footprint_padding_ = declare_parameter<double>("footprint_padding", 0.10);
    const auto phys = rover_traversability::core::loadPhysicalParams(phys_path);
    phys_ = phys;
    const auto warn = rover_traversability::core::placeholderWarning(phys);
    if (!warn.empty()) {
      RCLCPP_WARN(get_logger(), "%s", warn.c_str());
    }
    fp_.plane_radius = phys.wheelbase / 2.0;
    wheel_radius_ = phys.wheel_radius;
    fp_.step_radius = phys.wheel_radius;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<grid_map_msgs::msg::GridMap>(out, 1);
    sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      in, 1, [this](grid_map_msgs::msg::GridMap::ConstSharedPtr m) {onMap(*m);});
    RCLCPP_INFO(
      get_logger(), "terrain_features: %s -> %s, plane radius %.3f m, step radius %.3f m, "
      "negative obstacles from '%s'", in.c_str(), out.c_str(), fp_.plane_radius,
      fp_.step_radius, sensor_frame_.c_str());
  }

private:
  void onMap(const grid_map_msgs::msg::GridMap & in)
  {
    Eigen::MatrixXf z, var;
    if (!readLayer(in, L::kElevation, z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input map has no elevation layer");
      return;
    }
    if (!readLayer(in, L::kVariance, var)) {
      var = Eigen::MatrixXf::Constant(z.rows(), z.cols(), 1e-2f);
    }
    const auto t0 = std::chrono::steady_clock::now();
    const GridGeometry g = geometryOf(in);
    // The published elevation and variance stay as observed. Only features see the fill.
    const Eigen::MatrixXf z_obs = z, var_obs = var;
    if (fill_footprint_) {fillUnderRover(in, g, z, var);}
    fp_.resolution = g.resolution;
    np_.resolution = g.resolution;
    // On a coarse grid (the global map) a wheel-radius window would hold a single cell and step
    // would never have two values to compare.
    fp_.step_radius = std::max(wheel_radius_, 1.5 * g.resolution);
    const FeatureLayers f = computeFeatures(z, var, fp_);

    Eigen::MatrixXf neg = Eigen::MatrixXf::Constant(
      z.rows(), z.cols(), std::numeric_limits<float>::quiet_NaN());
    try {
      const auto tf = tf_buffer_->lookupTransform(
        in.header.frame_id, sensor_frame_, in.header.stamp, tf2::durationFromSec(0.1));
      const auto & p = tf.transform.translation;
      RCLCPP_INFO_ONCE(
        get_logger(), "sensor at (%.2f, %.2f, %.2f) in %s, cell (%.1f, %.1f) of %dx%d, "
        "map centre (%.2f, %.2f)", p.x, p.y, p.z, in.header.frame_id.c_str(), g.row(p.x),
        g.col(p.y), g.rows, g.cols, g.center_x, g.center_y);
      neg = negativeObstacles(z, g.row(p.x), g.col(p.y), p.z, np_);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no sensor pose, negative obstacles skipped: %s",
        e.what());
    }

    grid_map_msgs::msg::GridMap out;
    out.header = in.header;      // same stamp: consumers judge data age by it
    out.info = in.info;
    out.outer_start_index = 0;
    out.inner_start_index = 0;
    writeLayer(out, L::kElevation, z_obs);
    writeLayer(out, L::kVariance, var_obs);
    Eigen::MatrixXf obs;
    if (readLayer(in, L::kObservations, obs)) {writeLayer(out, L::kObservations, obs);}
    writeLayer(out, L::kSlope, f.slope);
    writeLayer(out, "slope_std", f.slope_std);
    writeLayer(out, L::kRoughness, f.roughness);
    writeLayer(out, L::kStep, f.step);
    writeLayer(out, L::kNormalX, f.normal_x);
    writeLayer(out, L::kNormalY, f.normal_y);
    writeLayer(out, L::kNormalZ, f.normal_z);
    writeLayer(out, L::kCurvature, f.curvature);
    writeLayer(out, L::kNegativeObstacle, neg);

    double yaw_ms = 0.0;
    if (evaluate_yaws_) {
      const auto ty = std::chrono::steady_clock::now();
      rover_traversability::core::TerrainGrid tg;
      tg.resolution = g.resolution;
      tg.center_x = g.center_x;
      tg.center_y = g.center_y;
      tg.elevation = z;
      tg.variance = var;
      tg.step = f.step;
      tg.negative_obstacle = neg;
      // Roughness under the support polygon, precomputed once per map (rough_radius 0 below).
      tg.roughness = rover_traversability::core::discMax(
        f.roughness, std::hypot(phys_.wheelbase / 2.0, phys_.track / 2.0) / g.resolution);
      auto e = eval_;
      e.rough_radius = 0.0;
      const auto y = rover_traversability::core::evaluateYaws(tg, phys_, e, yaw_);
      writeLayer(out, L::kSafeYawMask, y.safe_yaw_mask);
      writeLayer(out, L::kBestYaw, y.best_yaw);
      writeLayer(out, L::kBestUtilisation, y.best_utilisation);
      writeLayer(out, "limiting_check", y.limiting_check);
      yaw_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - ty).count();
    }
    out.basic_layers = {L::kElevation};
    pub_->publish(out);

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(
      get_logger(), "features for %dx%d cells in %.1f ms (heading evaluation %.1f ms)", g.rows,
      g.cols, ms, yaw_ms);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 30000, "features %.1f ms per map, heading evaluation %.1f ms",
      ms, yaw_ms);
  }

  void fillUnderRover(
    const grid_map_msgs::msg::GridMap & in, const GridGeometry & g, Eigen::MatrixXf & z,
    Eigen::MatrixXf & var)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        in.header.frame_id, base_frame_, in.header.stamp, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no rover pose, footprint not filled: %s", e.what());
      return;
    }
    const auto & q = tf.transform.rotation;
    const Eigen::Matrix3d R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    rover_terrain_mapping::FootprintPlane fp;
    fp.x = tf.transform.translation.x;
    fp.y = tf.transform.translation.y;
    fp.z = tf.transform.translation.z;
    fp.yaw = std::atan2(R(1, 0), R(0, 0));
    fp.nx = R(0, 2);
    fp.ny = R(1, 2);
    fp.nz = R(2, 2);
    fp.half_length = phys_.footprint_length / 2.0 + footprint_padding_;
    fp.half_width = phys_.footprint_width / 2.0 + footprint_padding_;
    std::vector<double> row_x(g.rows), col_y(g.cols);
    for (int i = 0; i < g.rows; ++i) {row_x[i] = g.x(i);}
    for (int j = 0; j < g.cols; ++j) {col_y[j] = g.y(j);}
    // 1 cm, the Kalman floor of observed cells: the wheels pin the plane about as well.
    rover_terrain_mapping::fillFootprint(z, var, row_x, col_y, fp, 1e-4f);
  }

  std::string base_frame_;
  bool fill_footprint_{true};
  double footprint_padding_{0.10};
  FeatureParams fp_;
  rover_traversability::core::PhysicalParams phys_;
  rover_traversability::core::EvalParams eval_;
  rover_traversability::core::YawParams yaw_;
  bool evaluate_yaws_{true};
  double wheel_radius_{0.15};
  NegativeObstacleParams np_;
  std::string sensor_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
};

}  // namespace rover_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<rover_terrain_mapping::TerrainFeaturesNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
