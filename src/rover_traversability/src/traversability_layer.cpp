// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rover_traversability/layer_names.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace rover_traversability
{

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: failed to lock the parent node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue(std::string("/terrain/local_features")));
  declareParameter("use_precomputed_traversability", rclcpp::ParameterValue(false));
  declareParameter("traversability_layer", rclcpp::ParameterValue(std::string("traversability")));
  declareParameter("slope_layer", rclcpp::ParameterValue(std::string(layers::kSlope)));
  declareParameter("roughness_layer", rclcpp::ParameterValue(std::string(layers::kRoughness)));
  declareParameter("step_layer", rclcpp::ParameterValue(std::string(layers::kStep)));
  declareParameter("max_slope", rclcpp::ParameterValue(0.45));
  declareParameter("max_roughness", rclcpp::ParameterValue(0.05));
  declareParameter("max_step", rclcpp::ParameterValue(0.15));
  declareParameter("slope_weight", rclcpp::ParameterValue(0.5));
  declareParameter("roughness_weight", rclcpp::ParameterValue(0.2));
  declareParameter("step_weight", rclcpp::ParameterValue(0.3));
  declareParameter("unknown_cost", rclcpp::ParameterValue(200));
  declareParameter("max_soft_cost", rclcpp::ParameterValue(252));
  declareParameter("lethal_dilation_radius", rclcpp::ParameterValue(0.0));
  declareParameter("map_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("cost_model", rclcpp::ParameterValue(std::string("legacy")));
  declareParameter("utilisation_layer", rclcpp::ParameterValue(std::string(layers::kBestUtilisation)));
  declareParameter("mask_layer", rclcpp::ParameterValue(std::string(layers::kSafeYawMask)));
  declareParameter("odom_topic", rclcpp::ParameterValue(std::string("/odom")));
  declareParameter("braking_decel", rclcpp::ParameterValue(0.5));
  declareParameter("braking_margin", rclcpp::ParameterValue(0.25));
  declareParameter("unknown_lethal_in_braking", rclcpp::ParameterValue(true));

  int unknown_cost_i = 200, max_soft_cost_i = 252;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".map_topic", map_topic_);
  node->get_parameter(name_ + ".use_precomputed_traversability", use_precomputed_traversability_);
  node->get_parameter(name_ + ".traversability_layer", traversability_layer_);
  node->get_parameter(name_ + ".slope_layer", slope_layer_);
  node->get_parameter(name_ + ".roughness_layer", roughness_layer_);
  node->get_parameter(name_ + ".step_layer", step_layer_);
  node->get_parameter(name_ + ".max_slope", max_slope_);
  node->get_parameter(name_ + ".max_roughness", max_roughness_);
  node->get_parameter(name_ + ".max_step", max_step_);
  node->get_parameter(name_ + ".slope_weight", slope_weight_);
  node->get_parameter(name_ + ".roughness_weight", roughness_weight_);
  node->get_parameter(name_ + ".step_weight", step_weight_);
  node->get_parameter(name_ + ".unknown_cost", unknown_cost_i);
  node->get_parameter(name_ + ".max_soft_cost", max_soft_cost_i);
  node->get_parameter(name_ + ".lethal_dilation_radius", lethal_dilation_radius_);
  node->get_parameter(name_ + ".map_timeout", map_timeout_);
  node->get_parameter(name_ + ".combination_method", combination_method_);
  node->get_parameter(name_ + ".cost_model", cost_model_);
  node->get_parameter(name_ + ".utilisation_layer", utilisation_layer_);
  node->get_parameter(name_ + ".mask_layer", mask_layer_);
  node->get_parameter(name_ + ".odom_topic", odom_topic_);
  node->get_parameter(name_ + ".braking_decel", braking_decel_);
  node->get_parameter(name_ + ".braking_margin", braking_margin_);
  node->get_parameter(name_ + ".unknown_lethal_in_braking", unknown_lethal_in_braking_);
  if (cost_model_ != "legacy" && cost_model_ != "utilisation") {
    throw std::runtime_error("TraversabilityLayer: cost_model must be legacy or utilisation");
  }
  unknown_cost_ = static_cast<unsigned char>(std::clamp(unknown_cost_i, 0, 252));
  max_soft_cost_ = static_cast<unsigned char>(std::clamp(max_soft_cost_i, 1, 252));

  use_explicit_dilation_radius_ = (lethal_dilation_radius_ > 0.0);
  if (!use_explicit_dilation_radius_) {
    lethal_dilation_radius_ = layered_costmap_->getInscribedRadius();
  }

  // Unobserved terrain is the default, not free space.
  default_value_ = NO_INFORMATION;
  matchSize();

  // A GridMap message is large and arrives at map rate. Give it a dedicated
  // callback group and executor thread so it never contends with the costmap
  // update loop, which runs on the parent node's own executor.
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // Reliable, newest map only. A feature map is about 2 MB: it does not fit Fast DDS's shared
  // memory segment, travels as UDP fragments, and with best effort one lost fragment drops the
  // whole map (in sim the layer saw maps up to 8 s old while the node published every 0.4 s).
  map_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    map_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&TraversabilityLayer::gridMapCallback, this, std::placeholders::_1),
    sub_options);

  if (cost_model_ == "utilisation") {
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TraversabilityLayer::odomCallback, this, std::placeholders::_1), sub_options);
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node->get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);

  latest_map_stamp_ = node->now();
  current_ = false;

  RCLCPP_INFO(
    logger_, "TraversabilityLayer '%s' (%s) subscribed to %s, inscribed dilation %.2f m",
    name_.c_str(), cost_model_.c_str(), map_topic_.c_str(), lethal_dilation_radius_);
}

void TraversabilityLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(), master->getOriginX(), master->getOriginY());

  // resolution_ is only valid after resizeMap, and the kernel is expressed in
  // cells, so it has to be rebuilt here rather than in onInitialize.
  dilation_.setRadius(lethal_dilation_radius_, resolution_);
}

void TraversabilityLayer::onFootprintChanged()
{
  if (!use_explicit_dilation_radius_) {
    lethal_dilation_radius_ = layered_costmap_->getInscribedRadius();
    dilation_.setRadius(lethal_dilation_radius_, resolution_);
    RCLCPP_DEBUG(
      logger_, "TraversabilityLayer: footprint changed, dilation radius now %.3f m (%d cells)",
      lethal_dilation_radius_, dilation_.radiusCells());
  }
}

void TraversabilityLayer::activate() {}
void TraversabilityLayer::deactivate() {}

void TraversabilityLayer::reset()
{
  resetMaps();
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_.reset();
  current_ = false;
}

void TraversabilityLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  auto map = std::make_shared<grid_map::GridMap>();
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, *map)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "TraversabilityLayer: could not convert incoming GridMap");
    return;
  }

  const std::vector<std::string> required =
    cost_model_ == "utilisation" ? std::vector<std::string>{utilisation_layer_, mask_layer_} :
    use_precomputed_traversability_ ? std::vector<std::string>{traversability_layer_} :
    std::vector<std::string>{slope_layer_, roughness_layer_, step_layer_};

  for (const auto & layer : required) {
    if (!map->exists(layer)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "TraversabilityLayer: incoming GridMap has no layer '%s'", layer.c_str());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_ = map;                 // cheap pointer swap, no deep copy under lock
  latest_map_stamp_ = msg->header.stamp;
}

unsigned char TraversabilityLayer::computeCost(
  float slope_rad, float roughness_m, float step_m) const
{
  if (!std::isfinite(slope_rad) || !std::isfinite(roughness_m) || !std::isfinite(step_m)) {
    return unknown_cost_;
  }

  // Hard gate first. Any single channel over its physical limit is untraversable,
  // no averaging allowed. A 30 cm step is lethal even on perfectly flat ground.
  if (slope_rad > max_slope_ || roughness_m > max_roughness_ || step_m > max_step_) {
    return LETHAL_OBSTACLE;
  }

  const double s = std::clamp(slope_rad / max_slope_, 0.0, 1.0);
  const double r = std::clamp(roughness_m / max_roughness_, 0.0, 1.0);
  const double h = std::clamp(step_m / max_step_, 0.0, 1.0);

  // Quadratic blending keeps mild terrain nearly free while sharpening the
  // gradient near the limits, which is what the planner needs to discriminate.
  const double blended =
    slope_weight_ * s * s + roughness_weight_ * r * r + step_weight_ * h * h;
  const double norm = slope_weight_ + roughness_weight_ + step_weight_;
  const double cost = std::clamp(blended / norm, 0.0, 1.0);

  return static_cast<unsigned char>(cost * static_cast<double>(max_soft_cost_));
}

void TraversabilityLayer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & v = msg->twist.twist.linear;
  speed_.store(std::hypot(v.x, v.y));
}

unsigned char TraversabilityLayer::utilisationCost(
  float u, float mask, double dist, bool & braking) const
{
  braking = false;
  if (!std::isfinite(u) || !std::isfinite(mask)) {
    if (!unknown_lethal_in_braking_) {return unknown_cost_;}
    // Stopping distance at the current speed plus a margin, measured from the footprint's
    // circumscribed circle. Cells inside the footprint are exempt: every sensor has a blind
    // zone under the rover, and marking it lethal would put the rover inside its own obstacle.
    const double v = speed_.load();
    const double brake = v * v / (2.0 * braking_decel_) + braking_margin_;
    const double outside = dist - layered_costmap_->getCircumscribedRadius();
    braking = outside > 0.0 && outside <= brake;
    return braking ? LETHAL_OBSTACLE : unknown_cost_;
  }
  if (mask == 0.0f || u > 1.0f) {
    return LETHAL_OBSTACLE;
  }
  const double c = static_cast<double>(max_soft_cost_) * static_cast<double>(u) * u;
  return static_cast<unsigned char>(std::clamp(c, 0.0, static_cast<double>(max_soft_cost_)));
}

void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }

  std::shared_ptr<grid_map::GridMap> map;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
    stamp = latest_map_stamp_;
  }

  if (!map) {
    current_ = false;
    return;
  }

  const double age = (clock_->now() - stamp).seconds();
  if (age > map_timeout_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "TraversabilityLayer: elevation map is %.2f s old", age);
    current_ = false;
    return;
  }

  // The elevation map is republished in full every cycle and is robot centric,
  // so accumulating across cycles would only preserve stale terrain. Wipe and
  // rewrite. Everything not covered stays NO_INFORMATION and is skipped by
  // updateWithMax, which leaves other layers in control of those cells.
  resetMaps();
  braking_mask_.assign(static_cast<size_t>(size_x_) * size_y_, 0);

  // The GridMap frame and the costmap global frame should both be odom. If they
  // differ, look up the transform once and apply a planar rigid transform per
  // cell rather than calling tf per cell.
  double cos_t = 1.0, sin_t = 0.0, tx = 0.0, ty = 0.0;
  const std::string & global_frame = layered_costmap_->getGlobalFrameID();
  if (map->getFrameId() != global_frame) {
    try {
      auto tf = tf_->lookupTransform(
        global_frame, map->getFrameId(), tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      const double yaw = tf2::getYaw(tf.transform.rotation);
      cos_t = std::cos(yaw);
      sin_t = std::sin(yaw);
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "TraversabilityLayer: %s", ex.what());
      current_ = false;
      return;
    }
  }

  const bool util = cost_model_ == "utilisation";
  const grid_map::Matrix * slope = nullptr;
  const grid_map::Matrix * rough = nullptr;
  const grid_map::Matrix * step = nullptr;
  const grid_map::Matrix * trav = nullptr;
  const grid_map::Matrix * u_layer = nullptr;
  const grid_map::Matrix * mask_layer = nullptr;
  if (util) {
    u_layer = &(*map)[utilisation_layer_];
    mask_layer = &(*map)[mask_layer_];
  } else if (use_precomputed_traversability_) {
    trav = &(*map)[traversability_layer_];
  } else {
    slope = &(*map)[slope_layer_];
    rough = &(*map)[roughness_layer_];
    step = &(*map)[step_layer_];
  }

  double touched_min_x = std::numeric_limits<double>::max();
  double touched_min_y = std::numeric_limits<double>::max();
  double touched_max_x = std::numeric_limits<double>::lowest();
  double touched_max_y = std::numeric_limits<double>::lowest();

  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx = *it;

    grid_map::Position p;
    map->getPosition(idx, p);
    const double wx = cos_t * p.x() - sin_t * p.y() + tx;
    const double wy = sin_t * p.x() + cos_t * p.y() + ty;

    unsigned char cost;
    bool braking = false;
    if (util) {
      cost = utilisationCost(
        (*u_layer)(idx(0), idx(1)), (*mask_layer)(idx(0), idx(1)),
        std::hypot(wx - robot_x, wy - robot_y), braking);
    } else if (use_precomputed_traversability_) {
      const float t = (*trav)(idx(0), idx(1));
      if (!std::isfinite(t)) {
        cost = unknown_cost_;
      } else if (t >= 1.0f) {
        cost = LETHAL_OBSTACLE;
      } else {
        cost = static_cast<unsigned char>(
          std::clamp(t, 0.0f, 1.0f) * static_cast<float>(max_soft_cost_));
      }
    } else {
      cost = computeCost(
        (*slope)(idx(0), idx(1)), (*rough)(idx(0), idx(1)), (*step)(idx(0), idx(1)));
    }

    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) {
      continue;
    }

    // Several grid map cells can fall into one costmap cell when the costmap is
    // coarser. Keep the worst case, never the average, or a single lethal rock
    // gets diluted away by its neighbours.
    const unsigned int index = getIndex(mx, my);
    if (costmap_[index] == NO_INFORMATION || cost > costmap_[index]) {
      costmap_[index] = cost;
    }
    if (braking) {braking_mask_[index] = 1;}

    touched_min_x = std::min(touched_min_x, wx);
    touched_min_y = std::min(touched_min_y, wy);
    touched_max_x = std::max(touched_max_x, wx);
    touched_max_y = std::max(touched_max_y, wy);
  }

  if (touched_min_x > touched_max_x) {
    current_ = false;
    return;
  }

  // Expand the bounds to also cover the previous footprint of the map, so that
  // cells the rover has driven past get rewritten instead of keeping old costs.
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, std::min(touched_min_x, last_min_x_));
    *min_y = std::min(*min_y, std::min(touched_min_y, last_min_y_));
    *max_x = std::max(*max_x, std::max(touched_max_x, last_max_x_));
    *max_y = std::max(*max_y, std::max(touched_max_y, last_max_y_));
  } else {
    *min_x = std::min(*min_x, touched_min_x);
    *min_y = std::min(*min_y, touched_min_y);
    *max_x = std::max(*max_x, touched_max_x);
    *max_y = std::max(*max_y, touched_max_y);
  }
  last_min_x_ = touched_min_x;
  last_min_y_ = touched_min_y;
  last_max_x_ = touched_max_x;
  last_max_y_ = touched_max_y;
  has_last_bounds_ = true;

  current_ = true;
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !current_) {
    return;
  }

  const int sx = static_cast<int>(size_x_), sy = static_cast<int>(size_y_);
  if (cost_model_ == "utilisation") {
    // Only lethal cells grow by the inscribed radius. Graded costs already describe the whole
    // rover at that cell (pose evaluation), and dilating the unknown cost would flood every
    // known cell next to an unobserved one.
    lethal_buf_.assign(costmap_, costmap_ + static_cast<size_t>(sx) * sy);
    for (size_t k = 0; k < lethal_buf_.size(); ++k) {
      const bool terrain_lethal = lethal_buf_[k] == LETHAL_OBSTACLE &&
        !(k < braking_mask_.size() && braking_mask_[k]);
      lethal_buf_[k] = terrain_lethal ? LETHAL_OBSTACLE : 0;
    }
    dilation_.apply(lethal_buf_.data(), sx, sy, min_i, min_j, max_i, max_j);
    for (int j = std::max(0, min_j); j < std::min(sy, max_j); ++j) {
      for (int i = std::max(0, min_i); i < std::min(sx, max_i); ++i) {
        const unsigned int k = getIndex(i, j);
        if (lethal_buf_[k] == LETHAL_OBSTACLE) {costmap_[k] = LETHAL_OBSTACLE;}
      }
    }
  } else {
    dilation_.apply(costmap_, sx, sy, min_i, min_j, max_i, max_j);
  }

  if (combination_method_ == 0) {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } else {
    // updateWithMax skips cells equal to NO_INFORMATION, which is why unknown
    // terrain is written as a finite penalty rather than 255.
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
}

}  // namespace rover_traversability

PLUGINLIB_EXPORT_CLASS(rover_traversability::TraversabilityLayer, nav2_costmap_2d::Layer)
