// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// terrain_safety_monitor (SPEC.md, speed and safety):
//  - Lethal terrain cells inside the braking zone go out as a point cloud on
//    /terrain/hazard_points, a source for Nav2's collision monitor. A cell is lethal when no
//    heading is safe there, its best utilisation exceeds 1, or it is a negative obstacle. Points
//    sit 0.5 m above the rover's base, inside the collision monitor's height band.
//  - Measured roll or pitch (IMU, rotated into the base frame) within tip_margin of its limit
//    cancels every NavigateToPose goal, once per episode.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/speed_safety.hpp"
#include "terrain_map_input.hpp"

namespace rover_traversability
{

class SafetyMonitor : public rclcpp::Node
{
public:
  explicit SafetyMonitor(const rclcpp::NodeOptions & o)
  : rclcpp::Node("terrain_safety_monitor", o)
  {
    tip_margin_ = declare_parameter<double>("tip_margin", 0.05);
    brake_decel_ = declare_parameter<double>("braking_decel", 0.5);
    brake_margin_ = declare_parameter<double>("braking_margin", 0.5);
    map_timeout_ = declare_parameter<double>("map_timeout", 1.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    abort_enabled_ = declare_parameter<bool>("abort_on_tip_risk", true);
    const auto rate = declare_parameter<double>("rate", 10.0);
    const auto phys_path = declare_parameter<std::string>(
      "physical_params_file", core::defaultPhysicalParamsPath());
    p_ = core::loadPhysicalParams(phys_path);
    const auto w = core::placeholderWarning(p_);
    if (!w.empty()) {RCLCPP_WARN(get_logger(), "%s", w.c_str());}
    core::tipLimits(p_, roll_limit_, pitch_limit_);

    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("hazard_topic", "/terrain/hazard_points"),
      rclcpp::SensorDataQoS());
    nav_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, declare_parameter<std::string>("navigate_action", "navigate_to_pose"));
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      declare_parameter<std::string>("map_topic", "/terrain/local_features"), 1,
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr m) {
        auto s = snapshotFrom(*m, p_, core::EvalParams(), get_clock()->get_clock_type());
        std::lock_guard<std::mutex> lock(mutex_);
        if (s) {terrain_ = s;}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_ = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {publishHazards();});
    RCLCPP_INFO(
      get_logger(), "safety monitor: roll limit %.1f deg, pitch limit %.1f deg, abort within "
      "%.1f deg", roll_limit_ * 180 / M_PI, pitch_limit_ * 180 / M_PI, tip_margin_ * 180 / M_PI);
  }

private:
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    // Base attitude: R_world_base = R_world_imu * R_imu_base, from the static mount TF.
    geometry_msgs::msg::TransformStamped mount;
    try {
      mount = tf_->lookupTransform(base_frame_, m.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const auto & q = m.orientation;
    const auto & r = mount.transform.rotation;
    const Eigen::Matrix3d R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix() *
      Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix().transpose();
    const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    const double roll = std::atan2(R(2, 1), R(2, 2));
    const bool risk = core::tipRisk(roll, pitch, p_, tip_margin_);
    if (risk && !in_episode_) {
      RCLCPP_ERROR(
        get_logger(), "tip risk: roll %.1f deg, pitch %.1f deg within %.1f deg of the limits",
        roll * 180 / M_PI, pitch * 180 / M_PI, tip_margin_ * 180 / M_PI);
      if (abort_enabled_ && nav_->action_server_is_ready()) {
        nav_->async_cancel_all_goals();
        RCLCPP_ERROR(get_logger(), "cancelled all navigation goals");
      }
    }
    in_episode_ = risk;
  }

  void publishHazards()
  {
    std::shared_ptr<const TerrainSnapshot> t;
    double v;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      t = terrain_;
      v = speed_;
    }
    if (!t || (now() - t->stamp).seconds() > map_timeout_) {return;}
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(t->frame, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const double rx = tf.transform.translation.x, ry = tf.transform.translation.y;
    const double rz = tf.transform.translation.z;
    // Braking zone: the stopping distance at the current speed plus a margin, beyond the
    // footprint's circumscribed radius.
    const double circ = std::hypot(p_.footprint_length / 2, p_.footprint_width / 2);
    const double reach = circ + v * v / (2.0 * brake_decel_) + brake_margin_;
    const auto & g = t->grid;
    std::vector<std::array<float, 3>> pts;
    for (int i = 0; i < g.rows(); ++i) {
      for (int j = 0; j < g.cols(); ++j) {
        const double x = g.x(i), y = g.y(j);
        if (std::hypot(x - rx, y - ry) > reach) {continue;}
        const float u = t->best_utilisation.size() ? t->best_utilisation(i, j) : NAN;
        const float mask = t->safe_yaw_mask.size() ? t->safe_yaw_mask(i, j) : NAN;
        const float neg = g.negative_obstacle.size() ? g.negative_obstacle(i, j) : 0.0f;
        const bool lethal = neg == 1.0f || (std::isfinite(mask) && mask == 0.0f) ||
          (std::isfinite(u) && u > 1.0f);
        if (lethal) {pts.push_back({static_cast<float>(x), static_cast<float>(y),
              static_cast<float>(rz + 0.5)});}
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = t->frame;
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    for (const auto & p : pts) {*ix = p[0]; *iy = p[1]; *iz = p[2]; ++ix; ++iy; ++iz;}
    cloud_pub_->publish(cloud);
  }

  core::PhysicalParams p_;
  double roll_limit_{0}, pitch_limit_{0}, tip_margin_{0.05};
  double brake_decel_{0.5}, brake_margin_{0.5}, map_timeout_{1.0};
  bool abort_enabled_{true}, in_episode_{false};
  std::string base_frame_;
  std::mutex mutex_;
  std::shared_ptr<const TerrainSnapshot> terrain_;
  double speed_{0.0};
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_traversability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_traversability::SafetyMonitor>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
