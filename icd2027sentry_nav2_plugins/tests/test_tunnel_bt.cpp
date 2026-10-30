// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using namespace std::chrono_literals;

geometry_msgs::msg::PoseStamped goal(double x, double y)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}

struct NavigationStub
{
  double x{-4.0}, y{3.475}, yaw{0.0}, lift{0.0};
  bool complete{false}, fail_follower{false}, fail_planner{false};
  std::vector<geometry_msgs::msg::PoseStamped> planner_goals;
  std::vector<nav_msgs::msg::Path> follower_paths;
  std::vector<std::string> controllers;
  nav_msgs::msg::Path current_path;
};

// Stateful actions have the same reset contract as Nav2's asynchronous action
// nodes: a completed planner is not a new request until its status is reset.
class PlannerStub : public BT::StatefulActionNode
{
public:
  PlannerStub(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<NavigationStub> state)
  : BT::StatefulActionNode(name, config), state_(std::move(state))
  {
  }
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("planner_id"), BT::OutputPort<nav_msgs::msg::Path>("path")};
  }
  BT::NodeStatus onStart() override
  {
    geometry_msgs::msg::PoseStamped target;
    if (!getInput("goal", target)) {
      return BT::NodeStatus::FAILURE;
    }
    state_->planner_goals.push_back(target);
    if (state_->fail_planner) {
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.poses = {goal(state_->x, state_->y), target};
    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void onHalted() override {}

private:
  std::shared_ptr<NavigationStub> state_;
};

class FollowerStub : public BT::StatefulActionNode
{
public:
  FollowerStub(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<NavigationStub> state)
  : BT::StatefulActionNode(name, config), state_(std::move(state))
  {
  }
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("controller_id"),
      BT::InputPort<std::string>("goal_checker_id")};
  }
  BT::NodeStatus onStart() override
  {
    std::string controller;
    if (
      !getInput("path", state_->current_path) || !getInput("controller_id", controller) ||
      state_->current_path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    state_->follower_paths.push_back(state_->current_path);
    state_->controllers.push_back(controller);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    getInput("path", state_->current_path);
    if (state_->fail_follower) {
      return BT::NodeStatus::FAILURE;
    }
    if (state_->complete) {
      state_->complete = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  std::shared_ptr<NavigationStub> state_;
};

class TunnelBt : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    static unsigned int count = 0;
    const auto ns = "/tunnel_bt_test_" + std::to_string(++count);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    parent_ = std::make_shared<rclcpp::Node>("parent", ns, options);
    parent_->declare_parameter("tunnel_config_file", std::string(TUNNEL_BT_TEST_CONFIG));
    driver_ = std::make_shared<rclcpp::Node>("driver", ns, options);
    state_pub_ =
      driver_->create_publisher<std_msgs::msg::Float64MultiArray>("tunnel_guard_state", 10);
    joints_pub_ = driver_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    ready_pub_ = driver_->create_publisher<std_msgs::msg::Bool>("tunnel_local_clearance_ready", 10);
    phase_sub_ = driver_->create_subscription<std_msgs::msg::String>(
      "tunnel_navigation_state", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String & msg) {
        phases_.push_back(msg.data.substr(0, msg.data.find(':')));
      });
    lift_sub_ = driver_->create_subscription<std_msgs::msg::Float64>(
      "chassis_lift_cmd", 10, [this](const std_msgs::msg::Float64 & msg) {
        lift_commands_.push_back(msg.data);
        if (
          msg.data == 0.0 && std::abs(stub_->y - 3.46) < 0.76 && stub_->x + 0.37 >= -2.675 &&
          stub_->x - 0.37 <= -2.325) {
          raised_under_roof_ = true;
        }
        if (actuate_lift_) {
          stub_->lift = msg.data;
        }
      });
    policy_sub_ = driver_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "tunnel_policy", 10, [this](const std_msgs::msg::Float64MultiArray & msg) {
        if (msg.data.size() >= 3) {
          last_policy_ = static_cast<int>(msg.data[0]);
          last_heading_ = msg.data[1];
          last_speed_limit_ = msg.data[2];
        }
      });
    factory_.registerFromPlugin(TUNNEL_BT_PLUGIN);
    stub_ = std::make_shared<NavigationStub>();
    factory_.registerBuilder<PlannerStub>(
      "ComputePathToPose", [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<PlannerStub>(name, config, stub_);
      });
    factory_.registerBuilder<FollowerStub>(
      "FollowPath", [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<FollowerStub>(name, config, stub_);
      });
    board_ = BT::Blackboard::create();
    board_->set("node", parent_);
  }
  void TearDown() override
  {
    if (tree_.rootNode()) {
      tree_.haltTree();
    }
    rclcpp::spin_some(driver_);
  }
  void create(const geometry_msgs::msg::PoseStamped & target)
  {
    board_->set("goal", target);
    tree_ = factory_.createTreeFromFile(
      std::string(TUNNEL_BT_TREE_DIR) + "/navigate_to_pose_with_tunnels.xml", board_);
    discover();
  }
  void create(const std::vector<geometry_msgs::msg::PoseStamped> & targets)
  {
    board_->set("goals", targets);
    tree_ = factory_.createTreeFromFile(
      std::string(TUNNEL_BT_TREE_DIR) + "/navigate_through_poses_with_tunnels.xml", board_);
    discover();
  }
  void discover()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (state_pub_->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(driver_);
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(state_pub_->get_subscription_count(), 0u);
  }
  void publish()
  {
    if (publish_state_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {stub_->x, stub_->y, stub_->yaw, 0.0, 0.0, stub_->lift > 0.052 ? 1.0 : 0.0};
      state_pub_->publish(msg);
    }
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = driver_->now();
    joints.name = {"chassis_lift_joint", "gimbal_pitch_joint"};
    joints.position = {stub_->lift, 0.0};
    joints_pub_->publish(joints);
    if (publish_ready_) {
      std_msgs::msg::Bool ready;
      ready.data = ready_;
      ready_pub_->publish(ready);
    }
  }
  BT::NodeStatus tick()
  {
    publish();
    std::this_thread::sleep_for(3ms);
    rclcpp::spin_some(driver_);
    status_ = tree_.tickRoot();
    std::this_thread::sleep_for(3ms);
    rclcpp::spin_some(driver_);
    return status_;
  }
  bool until(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition() && std::chrono::steady_clock::now() < deadline) {
      tick();
      if (status_ == BT::NodeStatus::FAILURE) {
        return false;
      }
    }
    return condition();
  }
  bool phaseIs(const std::string & phase) const
  {
    return !phases_.empty() && phases_.back() == phase;
  }
  void expectStoppedPolicy()
  {
    // Publishing STOP is asynchronous DDS delivery. Do not tick a failed tree
    // again (that starts a new navigation); drain the output subscription until
    // the bounded stop message arrives instead of assuming a 3-ms round trip.
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (last_policy_ != 1 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(driver_);
      std::this_thread::sleep_for(2ms);
    }
    EXPECT_EQ(last_policy_, 1);
  }
  void completeFollower()
  {
    ASSERT_FALSE(stub_->current_path.poses.empty());
    const auto & target = stub_->current_path.poses.back().pose.position;
    stub_->x = target.x;
    stub_->y = target.y;
    stub_->complete = true;
  }
  bool reachCrossing()
  {
    if (!until([this] { return phaseIs("approach_wait_point"); }) || !until([this] {
          return !stub_->follower_paths.empty();
        })) {
      return false;
    }
    completeFollower();
    return until([this] { return phaseIs("crossing"); }) &&
           until([this] { return stub_->controllers.back() == "TunnelFollowPath"; });
  }
  rclcpp::Node::SharedPtr parent_, driver_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr policy_sub_;
  std::shared_ptr<NavigationStub> stub_;
  std::vector<std::string> phases_;
  std::vector<double> lift_commands_;
  bool publish_state_{true}, actuate_lift_{true}, raised_under_roof_{false};
  bool publish_ready_{true}, ready_{true};
  int last_policy_{-1};
  double last_heading_{0.0}, last_speed_limit_{0.0};
  BT::NodeStatus status_{BT::NodeStatus::IDLE};
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr board_;
  BT::Tree tree_;
};

TEST_F(TunnelBt, NormalFollowingIssuesANewPlannerRequestOnFirstReplan)
{
  stub_->y = 0.0;
  create(goal(0.0, 0.0));
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
  ASSERT_EQ(stub_->planner_goals.size(), 1u);
  std::this_thread::sleep_for(350ms);
  tick();
  EXPECT_GE(stub_->planner_goals.size(), 2u);
}

TEST_F(TunnelBt, OpenStagingApproachKeepsNormalMotionAndReplanningUntilStationaryAlignment)
{
  stub_->yaw = 0.6;
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return last_policy_ == 0 && last_speed_limit_ == 0.0; }));
  const auto previous = stub_->planner_goals.size();
  std::this_thread::sleep_for(350ms);
  tick();
  EXPECT_GT(stub_->planner_goals.size(), previous);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("align_chassis_and_gimbal"); }));
  ASSERT_TRUE(until([this] { return last_policy_ == 2; }));
  EXPECT_NEAR(last_heading_, 0.0, 1e-6);
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
}

TEST_F(TunnelBt, BlueTunnelUsesTheSameLowerCrossRaiseSequenceAndCruiseLimit)
{
  stub_->x = 0.425;
  stub_->y = -3.475;
  create(goal(4.0, -3.475));
  ASSERT_TRUE(reachCrossing());
  ASSERT_TRUE(until([this] { return last_policy_ == 3; }));
  EXPECT_NEAR(last_speed_limit_, 1.2, 1e-6);
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_GT(stub_->current_path.poses.back().pose.position.x, 3.0);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
}

TEST_F(TunnelBt, OrdinaryGoalFromInsideTheExitGuideFirstReachesItsOpenEndWithoutLowering)
{
  stub_->x = -1.6;
  create(goal(0.0, 0.0));
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelApproach");
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.425, 1e-6);
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, 3.475, 1e-6);
  ASSERT_TRUE(until([this] { return last_policy_ == 0 && last_speed_limit_ > 0.0; }));
  EXPECT_NEAR(last_heading_, 0.0, 1e-6);
  EXPECT_TRUE(lift_commands_.empty());
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
  ASSERT_TRUE(until([this] { return stub_->controllers.back() == "FollowPath"; }));
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
}

TEST_F(TunnelBt, OrdinaryEntryToAHighGuideGoalAlignsAtItsOpenStagingPoint)
{
  stub_->x = 0.0;
  stub_->y = 0.0;
  create(goal(-1.6, 3.54));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.425, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 2u; }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelApproach");
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.6, 1e-6);
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, 3.475, 1e-6);
  EXPECT_TRUE(lift_commands_.empty());
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_EQ(std::count(stub_->controllers.begin(), stub_->controllers.end(), "FollowPath"), 0);
}

TEST_F(TunnelBt, FailedGlobalPlanInsideTheGuideGetsOnlyOneSameSideEscapeAttempt)
{
  stub_->x = -1.6;
  stub_->fail_planner = true;
  create(goal(0.0, 0.0));
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.425, 1e-6);
  EXPECT_TRUE(lift_commands_.empty());
  completeFollower();
  until([] { return false; });
  EXPECT_EQ(status_, BT::NodeStatus::FAILURE);
  EXPECT_EQ(stub_->planner_goals.size(), 2u);
  EXPECT_EQ(stub_->follower_paths.size(), 1u);
  expectStoppedPolicy();
}

TEST_F(TunnelBt, MisalignedHighPoseInsideTheGuideDoesNotTryABlindTurn)
{
  stub_->x = -1.6;
  stub_->yaw = 0.3;
  create(goal(0.0, 0.0));
  until([] { return false; });
  ASSERT_EQ(status_, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(stub_->follower_paths.empty());
  EXPECT_TRUE(lift_commands_.empty());
  EXPECT_EQ(std::count(phases_.begin(), phases_.end(), "align_chassis_and_gimbal"), 0);
  expectStoppedPolicy();
}

TEST_F(TunnelBt, AGoalUpdateWhileLeavingAHighGuideKeepsItsAxisControllerUntilOpenSpace)
{
  stub_->x = 0.0;
  stub_->y = 0.0;
  create(goal(-1.6, 3.54));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 2u; }));
  board_->set("goal", goal(0.0, 0.0));
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  completeFollower();
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 3u; }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelApproach");
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.425, 1e-6);
  EXPECT_TRUE(lift_commands_.empty());
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
}

TEST_F(TunnelBt, LoweredPoseLeavingTheGuideUsesVerifiedLowTraversalBeforeRestoring)
{
  stub_->x = -1.6;
  stub_->lift = 0.055;
  create(goal(0.0, 0.0));
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelFollowPath");
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.425, 1e-6);
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
}

TEST_F(TunnelBt, ExecutesLowerCrossRaiseAndContinuesToTheFinalGoal)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  ASSERT_EQ(stub_->controllers.back(), "TunnelFollowPath");
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("normal_follow"); }));
  ASSERT_TRUE(until([this] { return stub_->controllers.back() == "FollowPath"; }));
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, GoalInsideTheTunnelFinishesWithTheChassisLowered)
{
  create(goal(-2.5, 3.475));
  ASSERT_TRUE(reachCrossing());
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, ASecondGoalCanStartFromTheAlreadyCenteredApproach)
{
  stub_->x = -1.5;
  create(goal(-4.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return !stub_->controllers.empty(); }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelFollowPath");
  EXPECT_EQ(stub_->planner_goals.size(), 1u);
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, ClearRoofGoalOnTheExitCenterlineRestoresWithoutResumingSpin)
{
  create(goal(-1.6, 3.475));
  ASSERT_TRUE(reachCrossing());
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.6, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  EXPECT_EQ(std::count(stub_->controllers.begin(), stub_->controllers.end(), "FollowPath"), 0);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, SlightlyOffCenterGuideGoalFinishesAtSafeCenterlineWithoutResumingSpin)
{
  create(goal(-1.52, 3.54));
  ASSERT_TRUE(reachCrossing());
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, -1.52, 1e-6);
  for (const auto & point : stub_->current_path.poses) {
    EXPECT_NEAR(point.pose.position.y, 3.475, 1e-6);
  }
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_NEAR(stub_->y, 3.475, 1e-6);
  EXPECT_LT(std::hypot(stub_->x + 1.52, stub_->y - 3.54), 0.15);
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  EXPECT_EQ(std::count(stub_->controllers.begin(), stub_->controllers.end(), "FollowPath"), 0);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, OffCenterGuideGoalIsProjectedBeforePlanningButRetainsTheUserRequest)
{
  create(goal(-1.52, 3.54));
  ASSERT_TRUE(reachCrossing());
  ASSERT_FALSE(stub_->planner_goals.empty());
  EXPECT_NEAR(stub_->planner_goals.front().pose.position.x, -1.52, 1e-6);
  EXPECT_NEAR(stub_->planner_goals.front().pose.position.y, 3.475, 1e-6);
  const auto requested = board_->get<geometry_msgs::msg::PoseStamped>("goal");
  EXPECT_NEAR(requested.pose.position.x, -1.52, 1e-6);
  EXPECT_NEAR(requested.pose.position.y, 3.54, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_LT(
    std::hypot(stub_->x - requested.pose.position.x, stub_->y - requested.pose.position.y), 0.15);
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, SlightlyOffCenterRoofGoalFinishesLowOnTheSafeCenterline)
{
  create(goal(-2.5, 3.54));
  ASSERT_TRUE(reachCrossing());
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, 3.475, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_LT(std::hypot(stub_->x + 2.5, stub_->y - 3.54), 0.15);
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_EQ(std::count(stub_->controllers.begin(), stub_->controllers.end(), "FollowPath"), 0);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, UnsafeOffCenterRoofGoalFailsLowRatherThanCuttingTheSideWall)
{
  stub_->x = -2.5;
  stub_->lift = 0.055;
  create(goal(-2.4, 3.70));
  until([] { return false; });
  ASSERT_EQ(status_, BT::NodeStatus::FAILURE);
  expectStoppedPolicy();
  EXPECT_TRUE(stub_->follower_paths.empty());
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, ExactReverseExitGoalToleratesFloatingPointArithmetic)
{
  stub_->x = -1.5;
  create(goal(-3.575, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return !stub_->controllers.empty(); }));
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_EQ(std::count(stub_->controllers.begin(), stub_->controllers.end(), "FollowPath"), 0);
  EXPECT_NEAR(stub_->lift, 0.0, 1e-6);
}

TEST_F(TunnelBt, BlueGuideAlignmentUsesOpenStagingThenTwoAxisLegs)
{
  parent_->set_parameter(
    rclcpp::Parameter("tunnel_config_file", std::string(TUNNEL_BT_STAGE_TEST_CONFIG)));
  stub_->x = 0.0;
  stub_->y = 0.0;
  stub_->yaw = 0.4;
  create(goal(1.8, -3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, 1.425, 1e-6);
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, -2.675, 1e-6);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("align_chassis_and_gimbal"); }));
  stub_->yaw = 0.0;
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 2u; }));
  for (const auto & point : stub_->current_path.poses) {
    if (std::abs(point.pose.position.y + 3.475) > 1e-6) {
      EXPECT_NEAR(point.pose.position.x, 1.425, 1e-6);
    }
  }
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.x, 1.8, 1e-6);
  EXPECT_TRUE(lift_commands_.empty());
}

TEST_F(TunnelBt, ExactGuideEndpointGoalStillOwnsAxisAlignment)
{
  parent_->set_parameter(
    rclcpp::Parameter("tunnel_config_file", std::string(TUNNEL_BT_STAGE_TEST_CONFIG)));
  stub_->x = 0.0;
  stub_->y = 0.0;
  stub_->yaw = 0.4;
  create(goal(1.425, -3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, -2.675, 1e-6);
  ASSERT_TRUE(until([this] { return last_policy_ == 0 && last_speed_limit_ == 0.0; }));
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("align_chassis_and_gimbal"); }));
  stub_->yaw = 0.0;
  ASSERT_TRUE(until([this] { return phaseIs("following_axis_aligned_guide"); }));
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 2u; }));
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  EXPECT_EQ(stub_->follower_paths.size(), 2u);
}

TEST_F(TunnelBt, RecentersOutsideBeforeStartingThePassage)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  completeFollower();
  // Model a small displacement while the chassis stops and aligns. Correct
  // it outside instead of interpolating a diagonal through the narrow roof.
  stub_->y += 0.03;
  ASSERT_TRUE(until([this] { return phaseIs("centering_before_entry"); }));
  ASSERT_TRUE(until([this] { return stub_->controllers.back() == "TunnelApproach"; }));
  ASSERT_TRUE(until([this] { return last_policy_ == 3; }));
  EXPECT_NEAR(last_speed_limit_, 0.15, 1e-6);
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, 3.475, 1e-6);
  EXPECT_LT(stub_->current_path.poses.back().pose.position.x, -3.0);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, AGoalUpdateWhileLoweringDoesNotReleaseOrdinarySpinInTheApproach)
{
  actuate_lift_ = false;
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("lowering"); }));
  const auto planner_count = stub_->planner_goals.size();
  board_->set("goal", goal(0.0, 0.0));
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(phaseIs("lowering"));
  EXPECT_EQ(stub_->planner_goals.size(), planner_count);
  expectStoppedPolicy();
  actuate_lift_ = true;
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return stub_->controllers.back() == "TunnelFollowPath"; }));
  EXPECT_NEAR(stub_->current_path.poses.back().pose.position.y, 3.475, 1e-6);
}

TEST_F(TunnelBt, AGoalUpdateWhileCenteringFinishesTheCommittedManeuverFirst)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("approach_wait_point"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  completeFollower();
  stub_->y += 0.03;
  ASSERT_TRUE(until([this] { return phaseIs("centering_before_entry"); }));
  ASSERT_TRUE(until([this] { return stub_->controllers.back() == "TunnelApproach"; }));
  const auto planner_count = stub_->planner_goals.size();
  board_->set("goal", goal(0.0, 0.0));
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(phaseIs("centering_before_entry"));
  EXPECT_EQ(stub_->planner_goals.size(), planner_count);
  completeFollower();
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
}

TEST_F(TunnelBt, NewGoalDuringAnInsideTargetDoesNotSucceedAtTheOldGoal)
{
  create(goal(-2.5, 3.475));
  ASSERT_TRUE(reachCrossing());
  board_->set("goal", goal(-1.0, 3.475));
  tick();
  completeFollower();
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(raised_under_roof_);
  ASSERT_TRUE(until([this] { return stub_->current_path.poses.back().pose.position.x > -1.5; }));
}

TEST_F(TunnelBt, CancellingInsideKeepsItLowAndNewGoalResumesTowardTheOtherExit)
{
  stub_->x = -2.5;
  stub_->lift = 0.055;
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  tree_.haltTree();
  board_->set("goal", goal(-4.0, 3.475));
  phases_.clear();
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return !stub_->current_path.poses.empty(); }));
  tick();
  EXPECT_LT(stub_->current_path.poses.back().pose.position.x, -3.0);
  EXPECT_TRUE(stub_->planner_goals.empty());
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, WaitsForInitialPoseBeforeDecidingWhetherItIsInside)
{
  stub_->x = -2.5;
  stub_->lift = 0.055;
  publish_state_ = false;
  create(goal(-1.0, 3.475));
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  publish_state_ = true;
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  EXPECT_TRUE(stub_->planner_goals.empty());
}

TEST_F(TunnelBt, RepeatedIdenticalWaypointMissionStartsAgainAtTheFirstWaypoint)
{
  stub_->x = -1.0;
  stub_->y = 0.0;
  create(std::vector<geometry_msgs::msg::PoseStamped>{goal(0.0, 0.0), goal(1.0, 0.0)});
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 1; }));
  completeFollower();
  ASSERT_TRUE(until([this] { return stub_->follower_paths.size() == 2; }));
  completeFollower();
  ASSERT_TRUE(until([this] { return status_ == BT::NodeStatus::SUCCESS; }));
  tree_.haltTree();
  stub_->x = -1.0;
  tick();
  ASSERT_FALSE(stub_->planner_goals.empty());
  EXPECT_DOUBLE_EQ(stub_->planner_goals.back().pose.position.x, 0.0);
}

TEST_F(TunnelBt, BlockedCrossingFailsAndHoldsTheLoweredTarget)
{
  stub_->x = -2.5;
  stub_->lift = 0.055;
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(until([this] { return phaseIs("crossing"); }));
  ASSERT_TRUE(until([this] { return !stub_->follower_paths.empty(); }));
  stub_->fail_follower = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  expectStoppedPolicy();
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, ClearanceBecomingFalseDuringCrossingStopsAndKeepsItLow)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  ready_ = false;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  expectStoppedPolicy();
  until([] { return false; });  // until() deliberately returns false on failure.
  ASSERT_EQ(status_, BT::NodeStatus::FAILURE);
  expectStoppedPolicy();
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, ClearanceFeedbackLossDuringCrossingStopsAndKeepsItLow)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  publish_ready_ = false;
  tick();  // Drain the last queued update before timing out its heartbeat.
  std::this_thread::sleep_for(550ms);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  expectStoppedPolicy();
  until([] { return false; });
  ASSERT_EQ(status_, BT::NodeStatus::FAILURE);
  expectStoppedPolicy();
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, TransientLiftDeviationStopsThenResumesOnlyAfterValidFeedback)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  stub_->x = -1.8;
  const auto previous = stub_->follower_paths.size();
  actuate_lift_ = false;
  stub_->lift = 0.050;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  expectStoppedPolicy();
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
    expectStoppedPolicy();
    EXPECT_EQ(stub_->follower_paths.size(), previous);
  }
  stub_->lift = 0.055;
  ASSERT_TRUE(until([this, previous] { return stub_->follower_paths.size() > previous; }));
  EXPECT_NEAR(stub_->current_path.poses.front().pose.position.x, -1.8, 1e-6);
  EXPECT_FALSE(raised_under_roof_);
}

TEST_F(TunnelBt, PermanentLiftDeviationFailsHeldLowWithoutResuming)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  actuate_lift_ = false;
  stub_->lift = 0.050;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  expectStoppedPolicy();
  until([] { return false; });
  EXPECT_EQ(status_, BT::NodeStatus::FAILURE);
  expectStoppedPolicy();
  EXPECT_FALSE(raised_under_roof_);
  ASSERT_FALSE(lift_commands_.empty());
  EXPECT_DOUBLE_EQ(lift_commands_.back(), 0.055);
}

TEST_F(TunnelBt, TransientClearanceLossStopsThenResumesOnlyWithFreshData)
{
  create(goal(-1.0, 3.475));
  ASSERT_TRUE(reachCrossing());
  stub_->x = -1.8;  // Recovery must not restart from the old waiting point.
  const size_t paths_before_pause = stub_->follower_paths.size();
  ready_ = false;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  expectStoppedPolicy();
  EXPECT_NEAR(stub_->lift, 0.055, 1e-6);
  ready_ = true;
  ASSERT_TRUE(until(
    [this, paths_before_pause] { return stub_->follower_paths.size() > paths_before_pause; }));
  EXPECT_EQ(stub_->controllers.back(), "TunnelFollowPath");
  EXPECT_NEAR(stub_->current_path.poses.front().pose.position.x, -1.8, 1e-6);
  for (const auto & point : stub_->current_path.poses) {
    EXPECT_GE(point.pose.position.x, -1.8 - 1e-6);
  }
  EXPECT_FALSE(raised_under_roof_);
}
}  // namespace
