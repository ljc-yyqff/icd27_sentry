// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "icd2027sentry_nav2_plugins/layers/intensity_voxel_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define VOXEL_BITS 16

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;

namespace icd2027sentry_nav2_costmap_2d
{

void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(name_ + ".combination_method").as_int();

  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);

  const auto tunnel_config_file =
    node->declare_parameter<std::string>(name_ + ".tunnel_config_file", "");
  tunnel_mode_ = node->declare_parameter<std::string>(name_ + ".tunnel_mode", "local");
  if (!tunnel_config_file.empty()) {
    try {
      if (tunnel_mode_ != "global" && tunnel_mode_ != "local") {
        throw std::invalid_argument("tunnel_mode must be global or local");
      }
      tunnel_config_ = sentry_tunnel::Config::load(tunnel_config_file);
      tunnel_filter_enabled_ = !tunnel_config_.tunnels.empty();
      if (tunnel_filter_enabled_ && tunnel_mode_ == "local") {
        const auto joint_states_topic = node->declare_parameter<std::string>(
          name_ + ".tunnel_joint_states_topic", "joint_states");
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = callback_group_;
        joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
          joint_states_topic, rclcpp::SensorDataQoS(),
          std::bind(&IntensityVoxelLayer::jointStateCallback, this, std::placeholders::_1),
          subscription_options);
        const auto ready_topic = node->declare_parameter<std::string>(
          name_ + ".tunnel_ready_topic", "tunnel_local_clearance_ready");
        tunnel_ready_pub_ = node->create_publisher<std_msgs::msg::Bool>(ready_topic, 1);
      }
    } catch (const std::exception & error) {
      // An invalid tunnel configuration must preserve the original obstacle marking.
      tunnel_filter_enabled_ = false;
      RCLCPP_ERROR(node->get_logger(), "Tunnel roof filtering disabled: %s", error.what());
    }
  }

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() {}

void IntensityVoxelLayer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto received_ns = clock_->now().nanoseconds();
  const auto stamp_ns = rclcpp::Time(msg->header.stamp, clock_->get_clock_type()).nanoseconds();
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    sentry_tunnel::JointSample * feedback = nullptr;
    if (msg->name[i] == "chassis_lift_joint") {
      feedback = &lift_feedback_;
    } else if (msg->name[i] == "gimbal_pitch_joint") {
      feedback = &pitch_feedback_;
    }
    if (feedback == nullptr) {
      continue;
    }
    feedback->valid = i < msg->position.size() && std::isfinite(msg->position[i]);
    if (feedback->valid) {
      feedback->position = msg->position[i];
      feedback->received_ns = received_ns;
      feedback->stamp_ns = stamp_ns;
    }
  }
}

double IntensityVoxelLayer::tunnelRobotHeight()
{
  if (tunnel_mode_ == "global") {
    // The global plan expresses capability. The local layer and BT enforce readiness.
    return tunnel_config_.lowered_height;
  }

  const auto now_ns = clock_->now().nanoseconds();
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (sentry_tunnel::loweredReady(tunnel_config_, lift_feedback_, pitch_feedback_, now_ns)) {
    return tunnel_config_.lowered_height;
  }
  return tunnel_config_.normal_height;
}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  // update origin information for rolling costmap publication
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // reset maps each iteration
  resetMaps();

  // This layer rebuilds its entire map, so the master must also revisit the old
  // obstacle cells. Otherwise a roof that was marked before lowering can persist
  // indefinitely even though it no longer appears in the current observations.
  touch(origin_x_, origin_y_, min_x, min_y, max_x, max_y);
  touch(origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY(), min_x, min_y, max_x, max_y);

  // if not enabled, stop here
  if (!enabled_) {
    if (tunnel_ready_pub_) {
      tunnel_ready_pub_->publish(std_msgs::msg::Bool());
    }
    return;
  }

  // get the maximum sized window required to operate
  useExtraBounds(min_x, min_y, max_x, max_y);

  // get the marking observations
  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;

  // update the global current status
  current_ = current;

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  const double tunnel_robot_height = tunnel_filter_enabled_ ? tunnelRobotHeight() : 0.0;
  bool local_ready = tunnel_filter_enabled_ && tunnel_mode_ == "local" &&
                     tunnel_robot_height == tunnel_config_.lowered_height && current &&
                     !observations.empty();
  for (const auto & obs : observations) {
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    tf2::Transform map_from_global;
    map_from_global.setIdentity();
    bool can_filter_roof = tunnel_filter_enabled_;
    if (local_ready) {
      const auto cloud_stamp = rclcpp::Time(obs.cloud_->header.stamp, clock_->get_clock_type());
      const auto cloud_age = (clock_->now() - cloud_stamp).seconds();
      local_ready = !obs.cloud_->data.empty() && cloud_stamp.nanoseconds() > 0 &&
                    cloud_age >= -0.05 && cloud_age <= tunnel_config_.joint_timeout;
    }
    if (can_filter_roof && tunnel_config_.frame != global_frame_) {
      try {
        if (obs.cloud_->header.stamp.sec == 0 && obs.cloud_->header.stamp.nanosec == 0) {
          can_filter_roof = false;
        } else {
          const auto transform = tf_->lookupTransform(
            tunnel_config_.frame, global_frame_,
            rclcpp::Time(obs.cloud_->header.stamp, clock_->get_clock_type()),
            rclcpp::Duration::from_seconds(0.0));
          tf2::fromMsg(transform.transform, map_from_global);
        }
      } catch (const tf2::TransformException & error) {
        can_filter_roof = false;
        if (auto node = node_.lock()) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *clock_, 5000,
            "Keeping tunnel roof obstacles: cloud-time transform unavailable: %s", error.what());
        }
      }
    }
    local_ready = local_ready && can_filter_roof;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      double px = *it_x, py = *it_y, pz = *it_z;

      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) || !std::isfinite(*it_i)) {
        continue;
      }

      // if the obstacle is too low/high, we won't add it
      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
        continue;
      }

      // if the intensity is not in the range we want, we won't add it
      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far/close enough away... we won't consider it
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) {
        continue;
      }

      if (can_filter_roof) {
        const auto map_point = map_from_global * tf2::Vector3(px, py, pz);
        if (sentry_tunnel::knownRoofPoint(
              tunnel_config_, map_point.x(), map_point.y(), map_point.z(), tunnel_robot_height)) {
          continue;
        }
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch(static_cast<double>(px), static_cast<double>(py), min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  if (tunnel_ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = local_ready && tunnelRobotHeight() == tunnel_config_.lowered_height;
    tunnel_ready_pub_->publish(ready);
  }
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // namespace icd2027sentry_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(icd2027sentry_nav2_costmap_2d::IntensityVoxelLayer, nav2_costmap_2d::Layer)
