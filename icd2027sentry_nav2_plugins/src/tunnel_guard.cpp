// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "icd2027sentry_nav2_plugins/tunnel_clearance.hpp"
#include "icd2027sentry_nav2_plugins/tunnel_motion_collision.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sentry_tunnel
{
class TunnelGuard : public rclcpp::Node
{
public:
  TunnelGuard() : Node("tunnel_guard")
  {
    config_ = Config::load(declare_parameter<std::string>("tunnel_config_file", ""));
    chassis_frame_ = declare_parameter<std::string>("chassis_frame", "chassis");
    command_frame_ = declare_parameter<std::string>("command_frame", "gimbal_yaw");
    policy_timeout_ = declare_parameter("policy_timeout", 0.5);
    command_timeout_ = declare_parameter("command_timeout", 0.3);
    pose_timeout_ = declare_parameter("pose_timeout", 0.5);
    normal_speed_limit_ = declare_parameter("normal_speed_limit", 1.8);
    if (!std::isfinite(normal_speed_limit_) || normal_speed_limit_ <= 0.0) {
      throw std::invalid_argument("normal_speed_limit must be positive");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    output_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    state_ = create_publisher<std_msgs::msg::Float64MultiArray>("tunnel_guard_state", 10);
    input_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_unguarded", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // STOP and ALIGN cannot retain commands that could be replayed on release.
        if (policy_ == STOP || policy_ == ALIGN) {
          return;
        }
        if (!finiteTwist(*msg)) {
          command_ns_ = 0;
          return;
        }
        command_ = *msg;
        command_ns_ = now().nanoseconds();
      });
    policy_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "tunnel_policy", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        policy_ns_ = now().nanoseconds();
        policy_seen_ = true;
        if (
          msg->data.size() < 3 || !std::isfinite(msg->data[0]) || !std::isfinite(msg->data[1]) ||
          !std::isfinite(msg->data[2]) || msg->data[0] != std::round(msg->data[0]) ||
          msg->data[0] < NORMAL || msg->data[0] > TRAVERSE || msg->data[2] < 0.0) {
          policy_ = STOP;
          command_ns_ = 0;
          return;
        }
        policy_ = static_cast<Policy>(static_cast<int>(msg->data[0]));
        target_yaw_ = msg->data[1];
        speed_limit_ = msg->data[2];
        if (policy_ == STOP || policy_ == ALIGN) {
          command_ns_ = 0;
        }
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        const auto received_ns = now().nanoseconds();
        const auto stamp_ns =
          rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).nanoseconds();
        for (size_t i = 0; i < msg->name.size(); ++i) {
          JointSample * sample = nullptr;
          if (msg->name[i] == "chassis_lift_joint") {
            sample = &lift_;
          } else if (msg->name[i] == "gimbal_pitch_joint") {
            sample = &pitch_;
          }
          if (sample == nullptr) {
            continue;
          }
          sample->valid = i < msg->position.size() && std::isfinite(msg->position[i]);
          if (sample->valid) {
            sample->position = msg->position[i];
            sample->received_ns = received_ns;
            sample->stamp_ns = stamp_ns;
          }
        }
      });
    clearance_sub_ = create_subscription<std_msgs::msg::Bool>(
      "tunnel_local_clearance_ready", 10, [this](const std_msgs::msg::Bool & msg) {
        clearance_ready_ = msg.data;
        clearance_ns_ = now().nanoseconds();
      });
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      "local_costmap/costmap_raw", rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav2_msgs::msg::Costmap & message) {
        const auto & meta = message.metadata;
        const auto & origin = meta.origin;
        const size_t cells = static_cast<size_t>(meta.size_x) * meta.size_y;
        // Costmap2D is axis-aligned; never silently discard a rotated origin.
        if (
          message.header.frame_id.empty() || meta.size_x == 0 || meta.size_y == 0 ||
          cells != message.data.size() || !std::isfinite(meta.resolution) ||
          meta.resolution <= 0.0 || !std::isfinite(origin.position.x) ||
          !std::isfinite(origin.position.y) || !std::isfinite(origin.orientation.x) ||
          !std::isfinite(origin.orientation.y) || !std::isfinite(origin.orientation.z) ||
          !std::isfinite(origin.orientation.w) || std::abs(origin.orientation.x) > 1e-6 ||
          std::abs(origin.orientation.y) > 1e-6 || std::abs(origin.orientation.z) > 1e-6 ||
          std::abs(std::abs(origin.orientation.w) - 1.0) > 1e-6) {
          costmap_valid_ = false;
          return;
        }
        costmap_.resizeMap(
          meta.size_x, meta.size_y, meta.resolution, origin.position.x, origin.position.y);
        std::copy(message.data.begin(), message.data.end(), costmap_.getCharMap());
        costmap_frame_ = message.header.frame_id;
        costmap_stamp_ns_ =
          rclcpp::Time(message.header.stamp, get_clock()->get_clock_type()).nanoseconds();
        costmap_received_ns_ = now().nanoseconds();
        costmap_valid_ = true;
      });
    const auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const auto stamp_ns =
        rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).nanoseconds();
      const auto & twist = msg->twist.twist;
      if (finiteTwist(twist)) {
        odom_speed_ = std::hypot(twist.linear.x, twist.linear.y);
        odom_yaw_rate_ = twist.angular.z;
        odom_ns_ = stamp_ns;
      }
    };
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(), odom_callback);
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "chassis_odometry_gt", rclcpp::SensorDataQoS(), odom_callback);
    timer_ =
      create_wall_timer(std::chrono::milliseconds(20), std::bind(&TunnelGuard::update, this));
  }

private:
  struct Pose
  {
    int64_t stamp_ns;
    double x, y, yaw;
  };

  static bool finiteTwist(const geometry_msgs::msg::Twist & twist)
  {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }

  bool fresh(int64_t stamp_ns, int64_t now_ns, double timeout) const
  {
    const double age = (now_ns - stamp_ns) * 1e-9;
    return stamp_ns > 0 && age >= -0.05 && age <= timeout;
  }

  bool readPose(int64_t now_ns, tf2::Transform & map_from_command, double & roll, double & pitch)
  {
    try {
      const auto chassis =
        tf_buffer_->lookupTransform(config_.frame, chassis_frame_, tf2::TimePointZero);
      const auto command =
        tf_buffer_->lookupTransform(config_.frame, command_frame_, tf2::TimePointZero);
      const auto chassis_stamp =
        rclcpp::Time(chassis.header.stamp, get_clock()->get_clock_type()).nanoseconds();
      const auto command_stamp =
        rclcpp::Time(command.header.stamp, get_clock()->get_clock_type()).nanoseconds();
      if (
        !fresh(chassis_stamp, now_ns, pose_timeout_) ||
        !fresh(command_stamp, now_ns, pose_timeout_)) {
        return false;
      }
      tf2::Transform map_from_chassis;
      tf2::fromMsg(chassis.transform, map_from_chassis);
      tf2::fromMsg(command.transform, map_from_command);
      const auto & position = map_from_chassis.getOrigin();
      tf2::Matrix3x3(map_from_chassis.getRotation()).getRPY(roll, pitch, pose_yaw_);
      pose_x_ = position.x();
      pose_y_ = position.y();
      if (
        !std::isfinite(pose_x_) || !std::isfinite(pose_y_) || !std::isfinite(pose_yaw_) ||
        !std::isfinite(roll) || !std::isfinite(pitch)) {
        return false;
      }

      if (!poses_.empty() && chassis_stamp < poses_.back().stamp_ns) {
        poses_.clear();
      }
      if (poses_.empty() || chassis_stamp > poses_.back().stamp_ns) {
        poses_.push_back({chassis_stamp, pose_x_, pose_y_, pose_yaw_});
      }
      while (poses_.size() > 2 && chassis_stamp - poses_.front().stamp_ns > 150000000) {
        poses_.pop_front();
      }
      const double span = poses_.empty() ? 0.0 : (chassis_stamp - poses_.front().stamp_ns) * 1e-9;
      if (span >= 0.02) {
        linear_speed_ = std::hypot(pose_x_ - poses_.front().x, pose_y_ - poses_.front().y) / span;
        yaw_rate_ = angleDifference(pose_yaw_, poses_.front().yaw) / span;
      } else if (fresh(odom_ns_, now_ns, pose_timeout_)) {
        linear_speed_ = odom_speed_;
        yaw_rate_ = odom_yaw_rate_;
      } else {
        linear_speed_ = std::numeric_limits<double>::quiet_NaN();
        yaw_rate_ = std::numeric_limits<double>::quiet_NaN();
      }
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Tunnel guard waiting for fresh chassis and command transforms: %s", error.what());
      return false;
    }
  }

  bool wouldOverlap(
    const geometry_msgs::msg::Twist & output, const tf2::Transform & map_from_command) const
  {
    // Keep the outgoing linear coordinates unchanged. Rotate a copy solely to
    // predict world motion; the existing base interface performs the actual conversion.
    const auto map_velocity =
      map_from_command.getBasis() * tf2::Vector3(output.linear.x, output.linear.y, 0.0);
    for (const auto & tunnel : config_.tunnels) {
      for (int i = 0; i <= 10; ++i) {
        const double t = i * 0.1;
        if (config_.overlaps(
              tunnel, pose_x_ + map_velocity.x() * t, pose_y_ + map_velocity.y() * t,
              pose_yaw_ + output.angular.z * t)) {
          return true;
        }
      }
    }
    return false;
  }

  bool safeFinalMotion(const PlanarVelocity & output, int64_t now_ns)
  {
    if (!costmap_valid_ || !freshMotionCostmap(costmap_stamp_ns_, costmap_received_ns_, now_ns)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Tunnel guard stopping: no fresh full local costmap");
      return false;
    }
    try {
      // Both paths include a dynamic chassis transform. A static map->odom
      // alone cannot certify that the physical robot pose is still current.
      const auto chassis =
        tf_buffer_->lookupTransform(costmap_frame_, chassis_frame_, tf2::TimePointZero);
      const auto command =
        tf_buffer_->lookupTransform(costmap_frame_, command_frame_, tf2::TimePointZero);
      const auto chassis_stamp =
        rclcpp::Time(chassis.header.stamp, get_clock()->get_clock_type()).nanoseconds();
      const auto command_stamp =
        rclcpp::Time(command.header.stamp, get_clock()->get_clock_type()).nanoseconds();
      if (
        !fresh(chassis_stamp, now_ns, pose_timeout_) ||
        !fresh(command_stamp, now_ns, pose_timeout_)) {
        return false;
      }
      tf2::Transform grid_from_chassis, grid_from_command;
      tf2::fromMsg(chassis.transform, grid_from_chassis);
      tf2::fromMsg(command.transform, grid_from_command);
      const auto velocity = grid_from_command.getBasis() * tf2::Vector3(output.x, output.y, 0.0);
      const auto & position = grid_from_chassis.getOrigin();
      const double yaw = tf2::getYaw(grid_from_chassis.getRotation());
      MotionCollision detail;
      const bool collision = sweptMotionCollision(
        costmap_, config_.footprint, position.x(), position.y(), yaw,
        PlanarVelocity{velocity.x(), velocity.y(), output.yaw_rate}, 1.0, &detail);
      if (collision && rejected_motion_.reason == MotionCollision::Reason::NONE) {
        // Keep the FIRST rejected proposal, not the binary search's later
        // probes. A successful adaptive slowdown is not a blocked navigator.
        rejected_motion_ = detail;
      }
      return !collision;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Tunnel guard stopping: local costmap TF missing: %s",
        error.what());
      return false;
    }
  }

  void update()
  {
    const auto now_ns = now().nanoseconds();
    geometry_msgs::msg::Twist output;
    tf2::Transform map_from_command;
    double roll = 0.0, pitch = 0.0;
    const bool pose_valid = readPose(now_ns, map_from_command, roll, pitch);
    const bool ready = pose_valid && loweredReady(config_, lift_, pitch_, now_ns) &&
                       std::abs(roll) <= config_.pitch_tolerance &&
                       std::abs(pitch) <= config_.pitch_tolerance;

    std_msgs::msg::Float64MultiArray state;
    const auto invalid = std::numeric_limits<double>::quiet_NaN();
    state.data = {pose_valid ? pose_x_ : invalid,   pose_valid ? pose_y_ : invalid,
                  pose_valid ? pose_yaw_ : invalid, pose_valid ? linear_speed_ : invalid,
                  pose_valid ? yaw_rate_ : invalid, ready ? 1.0 : 0.0};
    state_->publish(state);

    bool inside_roof = false;
    if (pose_valid) {
      for (const auto & tunnel : config_.tunnels) {
        inside_roof = inside_roof || config_.overlaps(tunnel, pose_x_, pose_y_, pose_yaw_);
      }
    }
    GuardConditions conditions;
    conditions.configured = !config_.tunnels.empty();
    conditions.pose_valid = pose_valid;
    conditions.policy_live = fresh(policy_ns_, now_ns, policy_timeout_);
    conditions.command_live = fresh(command_ns_, now_ns, command_timeout_);
    conditions.lowered_ready = ready;
    conditions.clearance_ready =
      clearance_ready_ && fresh(clearance_ns_, now_ns, config_.joint_timeout);
    conditions.policy_seen = policy_seen_;
    conditions.inside_roof = inside_roof;
    conditions.policy = policy_;
    conditions.yaw = pose_yaw_;
    conditions.target_yaw = target_yaw_;
    conditions.speed_limit = speed_limit_;
    conditions.tunnel_speed = config_.speed;
    auto guarded = guardedVelocity(
      conditions, PlanarVelocity{command_.linear.x, command_.linear.y, command_.angular.z},
      [this, &map_from_command](const PlanarVelocity & proposed) {
        geometry_msgs::msg::Twist prediction;
        prediction.linear.x = proposed.x;
        prediction.linear.y = proposed.y;
        prediction.angular.z = proposed.yaw_rate;
        return wouldOverlap(prediction, map_from_command);
      });
    if (conditions.configured && policy_ != STOP) {
      if (policy_ == NORMAL && speed_limit_ == 0.0) {
        // Certify one second inside the 5 m rolling map, including the
        // square's .51 m circumscribed radius while spinning.
        const double speed = std::hypot(guarded.x, guarded.y);
        if (speed > normal_speed_limit_) {
          guarded.x *= normal_speed_limit_ / speed;
          guarded.y *= normal_speed_limit_ / speed;
        }
      }
      const auto proposed = guarded;
      rejected_motion_ = MotionCollision();
      guarded = collisionCheckedVelocity(
        policy_, speed_limit_, guarded,
        [this, now_ns](const PlanarVelocity & proposed) {
          return safeFinalMotion(proposed, now_ns);
        },
        yaw_rate_);
      if (rejected_motion_.reason != MotionCollision::Reason::NONE) {
        const double proposed_speed = std::hypot(proposed.x, proposed.y);
        const double selected_speed = std::hypot(guarded.x, guarded.y);
        if (
          selected_speed == 0.0 && guarded.yaw_rate == 0.0 &&
          (proposed_speed > 0.0 || proposed.yaw_rate != 0.0)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Tunnel guard stopped policy=%d: %s at %s (%.3f, %.3f), cell=(%u,%u), "
            "predicted t=%.3f s; command=(%.3f, %.3f, %.3f)",
            static_cast<int>(policy_), collisionReasonName(rejected_motion_.reason),
            costmap_frame_.c_str(), rejected_motion_.x, rejected_motion_.y, rejected_motion_.cell_x,
            rejected_motion_.cell_y, rejected_motion_.time, proposed.x, proposed.y,
            proposed.yaw_rate);
        } else if (proposed_speed > 0.0 && selected_speed < proposed_speed) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Tunnel guard reduced speed %.3f -> %.3f m/s; full 1 s footprint "
            "sweep remains clear (original rejection: %s at %.3f, %.3f)",
            proposed_speed, selected_speed, collisionReasonName(rejected_motion_.reason),
            rejected_motion_.x, rejected_motion_.y);
        }
      }
    }
    output.linear.x = guarded.x;
    output.linear.y = guarded.y;
    output.angular.z = guarded.yaw_rate;
    output_->publish(output);
  }

  Config config_;
  std::string chassis_frame_, command_frame_;
  double policy_timeout_, command_timeout_, pose_timeout_, normal_speed_limit_;
  Policy policy_{NORMAL};
  int64_t policy_ns_{0}, command_ns_{0}, odom_ns_{0}, clearance_ns_{0};
  bool clearance_ready_{false}, policy_seen_{false};
  bool costmap_valid_{false};
  int64_t costmap_stamp_ns_{0}, costmap_received_ns_{0};
  std::string costmap_frame_;
  nav2_costmap_2d::Costmap2D costmap_;
  MotionCollision rejected_motion_;
  double target_yaw_{0.0}, speed_limit_{0.0};
  double pose_x_{0.0}, pose_y_{0.0}, pose_yaw_{0.0};
  double linear_speed_{0.0}, yaw_rate_{0.0}, odom_speed_{0.0}, odom_yaw_rate_{0.0};
  JointSample lift_, pitch_;
  geometry_msgs::msg::Twist command_;
  std::deque<Pose> poses_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr policy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clearance_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, ground_truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace sentry_tunnel

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<sentry_tunnel::TunnelGuard>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("tunnel_guard"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
