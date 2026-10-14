// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// terrain_mapping_node: fuses point clouds into a robot-centric 2.5D elevation map with FastDEM
// and publishes it in the SPEC.md layer contract.
//
//   /terrain/local           elevation, variance, observations   (grid_map_msgs/GridMap, odom)
//   /terrain/local_features  elevation, slope (rad), step, roughness, curvature, normal_*
//
// Phase 0 scope: lidar input, local map. The feature map is FastDEM's own feature extraction,
// a temporary bridge that terrain_features_node replaces in Phase 2. Depth input, per-sensor
// noise beyond FastDEM's model, gravity alignment, the self filter and the global map are Phase 2.

#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "fastdem/config/fastdem.hpp"
#include "fastdem/config/postprocess.hpp"
#include "fastdem/fastdem.hpp"
#include "fastdem/postprocess/feature_extraction.hpp"
#include "fastdem/postprocess/inpainting.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nanogrid/bridge/ros2.hpp"
#include "nanopcl/bridge/ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_terrain_mapping/contract_layers.hpp"
#include "rover_terrain_mapping/tf_provider.hpp"

namespace rover_terrain_mapping
{

class TerrainMappingNode : public rclcpp::Node
{
public:
  explicit TerrainMappingNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("terrain_mapping", options)
  {
    const auto config_file = declare_parameter<std::string>("fastdem_config", "");
    const auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics", std::vector<std::string>{"/lidar/points"});
    const auto base_frame = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto map_frame = declare_parameter<std::string>("map_frame", "odom");
    const auto map_length = declare_parameter<double>("map_length", 10.0);
    const auto resolution = declare_parameter<double>("resolution", 0.05);
    const auto publish_rate = declare_parameter<double>("publish_rate", 5.0);
    const auto tf_timeout = declare_parameter<double>("tf_timeout", 0.1);
    publish_features_ = declare_parameter<bool>("publish_features", true);

    if (config_file.empty()) {
      throw std::runtime_error("terrain_mapping: parameter fastdem_config is required");
    }
    // One file holds FastDEM's pipeline and post-processing sections, in FastDEM's own format.
    cfg_ = fastdem::loadConfig(config_file);
    post_ = fastdem::config::loadPostProcess(config_file);
    if (cfg_.mapping.mode != fastdem::MappingMode::LOCAL) {
      throw std::runtime_error("terrain_mapping: Phase 0 supports mapping.mode 'local' only");
    }

    map_.setGeometry(map_length, map_length, resolution);
    map_.setFrameId(map_frame);
    mapper_ = std::make_unique<fastdem::FastDEM>(map_, cfg_);
    tf_ = std::make_shared<TfProvider>(*this, base_frame, map_frame, tf_timeout);
    mapper_->setTransformProvider(tf_);

    pub_elevation_ = create_publisher<grid_map_msgs::msg::GridMap>("/terrain/local", 1);
    pub_features_ = create_publisher<grid_map_msgs::msg::GridMap>("/terrain/local_features", 1);

    // Scans and publishing in separate groups, so feature extraction never delays integration.
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = scan_group_;
    for (const auto & topic : input_topics) {
      subs_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onScan(*msg);},
          sub_opts));
    }
    // Node clock, so the rate follows /clock in simulation.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate),
      [this]() {publish();}, publish_group_);

    RCLCPP_INFO(
      get_logger(),
      "terrain_mapping: %.1f x %.1f m at %.2f m in '%s', base '%s', %zu input(s), "
      "features %s (analysis radius %.2f m, inpainting %s)",
      map_length, map_length, resolution, map_frame.c_str(), base_frame.c_str(),
      input_topics.size(), publish_features_ ? "on" : "off",
      post_.feature_extraction.analysis_radius, post_.inpainting.enabled ? "on" : "off");
    RCLCPP_WARN(
      get_logger(),
      "terrain_mapping: /terrain/local_features carries FastDEM's features, a Phase 0 bridge. "
      "Their definitions differ from SPEC.md, see PROGRESS.md.");
  }

private:
  void onScan(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto cloud = std::make_shared<fastdem::PointCloud>(nanopcl::from(msg));
    std::unique_lock lock(map_mutex_);
    if (mapper_->integrate(cloud)) {
      // FastDEM leaves the map stamp at 0. The newest integrated scan's stamp lets consumers
      // judge how old the terrain data is.
      map_.setTimestamp(cloud->timestamp());
      ++integrated_;
    }
  }

  void publish()
  {
    fastdem::ElevationMap snap;
    {
      std::shared_lock lock(map_mutex_);
      if (integrated_ == 0 || map_.isEmpty()) {
        return;
      }
      snap = map_.snapshot(
        {fastdem::layer::elevation, fastdem::layer::variance, fastdem::layer::n_points});
    }

    pub_elevation_->publish(
      nanogrid::ros2::toMsg(toElevationContract(snap), elevationContractLayers()));

    if (!publish_features_) {
      return;
    }
    const auto & pp = post_;
    if (pp.inpainting.enabled) {
      fastdem::applyInpainting(
        snap, pp.inpainting.max_iterations, pp.inpainting.min_valid_neighbors, true);
    }
    fastdem::applyFeatureExtraction(
      snap, pp.feature_extraction.analysis_radius, pp.feature_extraction.min_valid_neighbors,
      pp.feature_extraction.step_lower_percentile, pp.feature_extraction.step_upper_percentile);
    pub_features_->publish(
      nanogrid::ros2::toMsg(toFeatureContract(snap), featureContractLayers()));
  }

  fastdem::Config cfg_;
  fastdem::config::PostProcess post_;
  bool publish_features_{true};

  fastdem::ElevationMap map_;
  std::unique_ptr<fastdem::FastDEM> mapper_;
  std::shared_ptr<TfProvider> tf_;
  std::shared_mutex map_mutex_;
  uint64_t integrated_{0};

  rclcpp::CallbackGroup::SharedPtr scan_group_, publish_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_elevation_, pub_features_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rover_terrain_mapping::TerrainMappingNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
