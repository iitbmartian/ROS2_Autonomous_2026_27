// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// terrain_speed_governor: publishes /speed_limit from the terrain ahead (SPEC.md, speed and
// safety). The rover's current twist is projected over lookahead_time, every pose on that path
// is evaluated on /terrain/local_features, and the worst utilisation u_ahead sets
//   v_max = v_nominal * (1 - alpha * u_ahead), floored at v_min.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/speed_safety.hpp"
#include "terrain_map_input.hpp"

namespace rover_traversability
{

class SpeedGovernor : public rclcpp::Node
{
public:
  explicit SpeedGovernor(const rclcpp::NodeOptions & o)
  : rclcpp::Node("terrain_speed_governor", o)
  {
    gp_.v_nominal = declare_parameter<double>("v_nominal", gp_.v_nominal);
    gp_.v_min = declare_parameter<double>("v_min", gp_.v_min);
    gp_.alpha = declare_parameter<double>("alpha", gp_.alpha);
    lookahead_ = declare_parameter<double>("lookahead_time", 2.0);
    dt_ = declare_parameter<double>("prediction_dt", 0.2);
    unknown_u_ = declare_parameter<double>("unknown_utilisation", 0.5);
    map_timeout_ = declare_parameter<double>("map_timeout", 2.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    e_.risk_kappa = declare_parameter<double>("risk_kappa", 1.0);
    const auto rate = declare_parameter<double>("rate", 5.0);
    const auto phys_path = declare_parameter<std::string>(
      "physical_params_file", core::defaultPhysicalParamsPath());
    p_ = core::loadPhysicalParams(phys_path);
    const auto w = core::placeholderWarning(p_);
    if (!w.empty()) {RCLCPP_WARN(get_logger(), "%s", w.c_str());}

    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
      declare_parameter<std::string>("speed_limit_topic", "/speed_limit"), 1);
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      declare_parameter<std::string>("map_topic", "/terrain/local_features"), 1,
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr m) {
        auto s = snapshotFrom(*m, p_, e_, get_clock()->get_clock_type());
        std::lock_guard<std::mutex> lock(mutex_);
        if (s) {terrain_ = s;}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_ = m->twist.twist;
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "speed governor: %.2f m/s nominal, floor %.2f, alpha %.2f, lookahead %.1f s",
      gp_.v_nominal, gp_.v_min, gp_.alpha, lookahead_);
  }

private:
  void publish(double v)
  {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = now();
    msg.percentage = false;
    msg.speed_limit = v;
    pub_->publish(msg);
  }

  void tick()
  {
    std::shared_ptr<const TerrainSnapshot> t;
    geometry_msgs::msg::Twist tw;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      t = terrain_;
      tw = twist_;
    }
    // No fresh terrain: hold the floor speed rather than trust old ground.
    if (!t || (now() - t->stamp).seconds() > map_timeout_) {
      publish(gp_.v_min);
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(t->frame, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      publish(gp_.v_min);
      return;
    }
    const core::Pose2 start{tf.transform.translation.x, tf.transform.translation.y,
      tf2::getYaw(tf.transform.rotation)};
    // At a standstill project the nominal speed straight ahead, so the limit is ready before
    // the rover moves.
    double vx = tw.linear.x, vy = tw.linear.y;
    if (std::hypot(vx, vy) < 0.02) {vx = gp_.v_nominal; vy = 0.0;}
    const auto poses = core::predictPoses(start, vx, vy, tw.angular.z, lookahead_, dt_);
    const double u = core::worstUtilisation(t->grid, p_, e_, poses, unknown_u_, &t->cache);
    publish(core::speedLimit(u, gp_));
  }

  core::PhysicalParams p_;
  core::EvalParams e_;
  core::GovernorParams gp_;
  double lookahead_{2.0}, dt_{0.2}, unknown_u_{0.5}, map_timeout_{1.0};
  std::string base_frame_;
  std::mutex mutex_;
  std::shared_ptr<const TerrainSnapshot> terrain_;
  geometry_msgs::msg::Twist twist_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_traversability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_traversability::SpeedGovernor>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
