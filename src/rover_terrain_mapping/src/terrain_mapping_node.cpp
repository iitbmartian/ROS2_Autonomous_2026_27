// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// terrain_mapping_node: fuses point clouds into 2.5D elevation maps with FastDEM and publishes
// them in the SPEC.md layer contract.
//
//   /terrain/local    elevation, variance, observations, last_seen   (odom, robot-centric, rolling)
//   /terrain/global   the same layers, coarse, fixed in the map frame (optional)
//
// Every input cloud has its own sensor model and range limits (lidar: near-constant range
// noise, RGB-D: noise growing with range squared), set on the mapper before each scan.
// A body box from rover_physical.yaml removes the rover's own points. Roll and pitch come from
// TF or from the IMU (gravity_source), position and yaw always from TF.
//
// With publish_features the node also publishes FastDEM's own features on
// /terrain/local_features, the Phase 0 bridge. terrain_features_node replaces it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fastdem/config/fastdem.hpp"
#include "fastdem/config/postprocess.hpp"
#include "fastdem/fastdem.hpp"
#include "fastdem/postprocess/feature_extraction.hpp"
#include "fastdem/postprocess/inpainting.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nanogrid/bridge/ros2.hpp"
#include "nanopcl/bridge/ros2.hpp"
#include "nanopcl/core/transform.hpp"
#include "nanopcl/filters/crop.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_terrain_mapping/contract_layers.hpp"
#include "rover_terrain_mapping/tf_provider.hpp"
#include "rover_traversability/core/physical_params.hpp"

namespace rover_terrain_mapping
{

namespace
{

fastdem::SensorType sensorType(const std::string & s)
{
  if (s == "lidar") {return fastdem::SensorType::LiDAR;}
  if (s == "rgbd") {return fastdem::SensorType::RGBD;}
  if (s == "constant") {return fastdem::SensorType::Constant;}
  throw std::runtime_error("terrain_mapping: unknown sensor_model '" + s + "'");
}

}  // namespace

class TerrainMappingNode : public rclcpp::Node
{
  struct Input
  {
    std::string name, topic, model;
    fastdem::SensorType type;
    double range_min, range_max;
    uint64_t scans{0};
  };

  // One FastDEM map with its mapper and TF provider: the local map, or the global one.
  struct Mapper
  {
    fastdem::ElevationMap map;
    std::unique_ptr<fastdem::FastDEM> fastdem;
    std::shared_ptr<TfProvider> tf;
    std::shared_mutex mutex;
    uint64_t integrated{0};
    // Since the last status line: scans integrated, scans whose pose lookup failed, scans
    // FastDEM rejected with a pose available (empty after filtering, no extrinsic).
    uint64_t ok{0}, no_pose{0}, rejected{0};
  };

public:
  explicit TerrainMappingNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("terrain_mapping", options), start_(now())
  {
    const auto config_file = declare_parameter<std::string>("fastdem_config", "");
    const auto inputs = declare_parameter<std::vector<std::string>>(
      "inputs", std::vector<std::string>{"lidar"});
    const auto base_frame = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto map_frame = declare_parameter<std::string>("map_frame", "odom");
    const auto map_length = declare_parameter<double>("map_length", 10.0);
    const auto resolution = declare_parameter<double>("resolution", 0.05);
    const auto publish_rate = declare_parameter<double>("publish_rate", 5.0);
    const auto tf_timeout = declare_parameter<double>("tf_timeout", 0.1);
    publish_features_ = declare_parameter<bool>("publish_features", false);
    const auto gravity = declare_parameter<std::string>("gravity_source", "tf");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    self_filter_ = declare_parameter<bool>("self_filter.enabled", true);
    const auto pad = declare_parameter<double>("self_filter.padding", 0.10);
    const auto body_h = declare_parameter<double>("self_filter.height", 1.0);
    const auto global_on = declare_parameter<bool>("global.enabled", false);
    const auto global_frame = declare_parameter<std::string>("global.frame", "map");
    const auto global_len = declare_parameter<double>("global.length", 100.0);
    const auto global_res = declare_parameter<double>("global.resolution", 0.2);
    const auto global_rate = declare_parameter<double>("global.publish_rate", 1.0);
    const auto phys_path = declare_parameter<std::string>(
      "physical_params_file", rover_traversability::core::defaultPhysicalParamsPath());

    if (config_file.empty()) {
      throw std::runtime_error("terrain_mapping: parameter fastdem_config is required");
    }
    // One file holds FastDEM's pipeline and post-processing sections, in FastDEM's own format.
    cfg_ = fastdem::loadConfig(config_file);
    post_ = fastdem::config::loadPostProcess(config_file);

    const auto phys = rover_traversability::core::loadPhysicalParams(phys_path);
    const auto warn = rover_traversability::core::placeholderWarning(phys);
    if (!warn.empty()) {RCLCPP_WARN(get_logger(), "%s", warn.c_str());}
    // Body box in base_footprint: the footprint plus padding, from the ground to body height.
    body_min_ = fastdem::Point(-phys.footprint_length / 2 - pad, -phys.footprint_width / 2 - pad,
        -0.05);
    body_max_ = fastdem::Point(phys.footprint_length / 2 + pad, phys.footprint_width / 2 + pad,
        body_h);

    auto local_cfg = cfg_;
    local_cfg.mapping.mode = fastdem::MappingMode::LOCAL;
    local_.map.setGeometry(map_length, map_length, resolution);
    local_.map.setFrameId(map_frame);
    local_.map.add(kLastSeenInternal, NAN);
    local_.fastdem = std::make_unique<fastdem::FastDEM>(local_.map, local_cfg);
    local_.tf = std::make_shared<TfProvider>(*this, base_frame, map_frame, tf_timeout);
    local_.fastdem->setTransformProvider(local_.tf);

    if (global_on) {
      auto global_cfg = cfg_;
      global_cfg.mapping.mode = fastdem::MappingMode::GLOBAL;
      global_ = std::make_unique<Mapper>();
      global_->map.setGeometry(global_len, global_len, global_res);
      global_->map.setFrameId(global_frame);
      global_->map.add(kLastSeenInternal, NAN);
      global_->fastdem = std::make_unique<fastdem::FastDEM>(global_->map, global_cfg);
      global_->tf = std::make_shared<TfProvider>(*this, base_frame, global_frame, tf_timeout);
      global_->tf->setOdomFrame(map_frame);
      global_->fastdem->setTransformProvider(global_->tf);
    }

    for (const auto & name : inputs) {
      Input in;
      in.name = name;
      in.topic = declare_parameter<std::string>(name + ".topic", "/" + name + "/points");
      in.model = declare_parameter<std::string>(name + ".sensor_model", "lidar");
      in.type = sensorType(in.model);
      in.range_min = declare_parameter<double>(name + ".range_min", cfg_.point_filter.range_min);
      in.range_max = declare_parameter<double>(name + ".range_max", cfg_.point_filter.range_max);
      inputs_.push_back(in);
    }

    if (gravity == "imu") {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});
    } else if (gravity != "tf") {
      throw std::runtime_error("terrain_mapping: gravity_source must be 'tf' or 'imu'");
    }

    pub_local_ = create_publisher<grid_map_msgs::msg::GridMap>("/terrain/local", 1);
    pub_features_ = create_publisher<grid_map_msgs::msg::GridMap>("/terrain/local_features", 1);
    pub_global_ = create_publisher<grid_map_msgs::msg::GridMap>("/terrain/global", 1);

    // Scans and publishing in separate groups, so post-processing never delays integration.
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = scan_group_;
    for (size_t k = 0; k < inputs_.size(); ++k) {
      subs_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          inputs_[k].topic, rclcpp::SensorDataQoS(),
          [this, k](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onScan(k, *msg);},
          sub_opts));
    }
    // Node clock, so rates follow /clock in simulation.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate),
      [this]() {publishLocal();}, publish_group_);
    status_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {logStatus();});
    if (global_) {
      global_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0 / global_rate),
        [this]() {publishGlobal();}, publish_group_);
    }

    for (const auto & in : inputs_) {
      RCLCPP_INFO(
        get_logger(), "terrain_mapping: input '%s' on %s, %s model, range %.2f to %.2f m",
        in.name.c_str(), in.topic.c_str(), in.model.c_str(), in.range_min, in.range_max);
    }
    RCLCPP_INFO(
      get_logger(), "terrain_mapping: local %.1f m at %.2f m in '%s'%s, gravity from %s, "
      "self filter %s", map_length, resolution, map_frame.c_str(),
      global_ ? (", global " + std::to_string(global_len) + " m in '" + global_frame + "'").c_str() :
      "", gravity.c_str(), self_filter_ ? "on" : "off");
    if (publish_features_) {
      RCLCPP_WARN(
        get_logger(), "terrain_mapping: /terrain/local_features carries FastDEM's features, "
        "the Phase 0 bridge. Their definitions differ from SPEC.md, see PROGRESS.md.");
    }
  }

private:
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const auto & o = m.orientation;
    const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    const uint64_t t = rclcpp::Time(m.header.stamp).nanoseconds();
    local_.tf->setImuAttitude(q, m.header.frame_id, t);
    if (global_) {global_->tf->setImuAttitude(q, m.header.frame_id, t);}
  }

  // Drops points inside the rover's body box: into base_footprint, crop, back to the sensor
  // frame, since FastDEM's sensor models need sensor-frame ranges.
  fastdem::PointCloud selfFilter(fastdem::PointCloud cloud, const Eigen::Isometry3d & T_base_sensor)
  {
    const std::string frame = cloud.frameId();
    const uint64_t stamp = cloud.timestamp();
    auto in_base = nanopcl::transformCloud(std::move(cloud), T_base_sensor);
    auto kept = nanopcl::filters::cropBox(
      std::move(in_base), body_min_, body_max_, nanopcl::filters::FilterMode::OUTSIDE);
    auto out = nanopcl::transformCloud(std::move(kept), T_base_sensor.inverse());
    out.setFrameId(frame);
    out.setTimestamp(stamp);
    return out;
  }

  bool integrate(Mapper & m, const Input & in, const std::shared_ptr<fastdem::PointCloud> & cloud)
  {
    std::unique_lock lock(m.mutex);
    // FastDEM reports its own failures through spdlog, which buffers when not on a terminal, so
    // the pose is checked here to tell a missing pose from a FastDEM rejection.
    if (!m.tf->getPoseAt(cloud->timestamp())) {
      ++m.no_pose;
      return false;
    }
    m.fastdem->setSensorModel(in.type);
    m.fastdem->setRangeFilter(static_cast<float>(in.range_min), static_cast<float>(in.range_max));
    // _last_seen is updated where the point count grew. The previous count is kept as a map
    // layer so it moves with the map when the local map recentres during integrate.
    auto & n = m.map.get(fastdem::layer::n_points);
    if (!m.map.exists("_prev_n_points")) {m.map.add("_prev_n_points", n);}
    m.map.get("_prev_n_points") = n;
    if (!m.fastdem->integrate(cloud)) {
      ++m.rejected;
      return false;
    }
    ++m.ok;
    const float t = static_cast<float>((rclcpp::Time(cloud->timestamp(), get_clock()->get_clock_type()) -
      start_).seconds());
    auto & seen = m.map.get(kLastSeenInternal);
    const auto & now_n = m.map.get(fastdem::layer::n_points);
    const auto & prev = m.map.get("_prev_n_points");
    // Cells new since the move hold NaN in _prev_n_points, so (now > prev) is false there and a
    // separate check catches cells that gained their first points.
    seen = (now_n.array() > prev.array() ||
      (prev.array().isNaN() && now_n.array() > 0)).select(t, seen);
    // FastDEM leaves the map stamp at 0. The newest integrated scan's stamp lets consumers judge
    // how old the terrain data is. Never backwards: depth clouds arrive late through the depth
    // pipeline, and taking their stamp made fresh maps look seconds old.
    m.map.setTimestamp(std::max(m.map.getTimestamp(), cloud->timestamp()));
    ++m.integrated;
    return true;
  }

  void onScan(size_t k, const sensor_msgs::msg::PointCloud2 & msg)
  {
    const Input & in = inputs_[k];
    auto raw = nanopcl::from(msg);
    if (self_filter_) {
      const auto T = local_.tf->getExtrinsic(raw.frameId());
      if (!T) {return;}
      raw = selfFilter(std::move(raw), *T);
    }
    auto cloud = std::make_shared<fastdem::PointCloud>(std::move(raw));
    if (integrate(local_, in, cloud)) {++inputs_[k].scans;}
    if (global_) {integrate(*global_, in, cloud);}
  }

  void logStatus()
  {
    auto line = [](const char * name, Mapper & m) {
        std::unique_lock lock(m.mutex);
        std::string s = std::string(name) + " " + std::to_string(m.ok) + " ok, " +
          std::to_string(m.no_pose) + " no pose, " + std::to_string(m.rejected) + " rejected";
        m.ok = m.no_pose = m.rejected = 0;
        return s;
      };
    std::string s = line("local", local_);
    if (global_) {s += " | " + line("global", *global_);}
    for (const auto & in : inputs_) {s += " | " + in.name + " " + std::to_string(in.scans);}
    RCLCPP_INFO(get_logger(), "terrain_mapping last 10 s (wall): %s", s.c_str());
  }

  double secondsSinceStart(uint64_t stamp_ns) const
  {
    return (rclcpp::Time(stamp_ns, get_clock()->get_clock_type()) - start_).seconds();
  }

  void publishLocal()
  {
    fastdem::ElevationMap snap;
    {
      std::shared_lock lock(local_.mutex);
      if (local_.integrated == 0 || local_.map.isEmpty()) {return;}
      snap = local_.map.snapshot(
        {fastdem::layer::elevation, fastdem::layer::variance, fastdem::layer::n_points,
          kLastSeenInternal});
    }
    const double now_s = secondsSinceStart(snap.getTimestamp());
    pub_local_->publish(
      nanogrid::ros2::toMsg(toElevationContract(snap, now_s), elevationContractLayers()));

    if (!publish_features_) {return;}
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

  void publishGlobal()
  {
    fastdem::ElevationMap snap;
    {
      std::shared_lock lock(global_->mutex);
      if (global_->integrated == 0) {return;}
      snap = global_->map.snapshot(
        {fastdem::layer::elevation, fastdem::layer::variance, fastdem::layer::n_points,
          kLastSeenInternal});
    }
    const double now_s = secondsSinceStart(snap.getTimestamp());
    pub_global_->publish(
      nanogrid::ros2::toMsg(toElevationContract(snap, now_s), elevationContractLayers()));
  }

  rclcpp::Time start_;
  fastdem::Config cfg_;
  fastdem::config::PostProcess post_;
  bool publish_features_{false};
  bool self_filter_{true};
  fastdem::Point body_min_, body_max_;

  std::vector<Input> inputs_;
  Mapper local_;
  std::unique_ptr<Mapper> global_;

  rclcpp::CallbackGroup::SharedPtr scan_group_, publish_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_local_, pub_features_, pub_global_;
  rclcpp::TimerBase::SharedPtr timer_, global_timer_, status_timer_;
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
