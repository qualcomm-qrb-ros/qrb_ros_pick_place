// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pick_place/pick_place_node.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using moveit::core::MoveItErrorCode;

namespace qrb_ros_pick_place
{

PickPlaceNode::PickPlaceNode(const rclcpp::NodeOptions & options) : Node("pick_place_node", options)
{
  arm_group_name_ = declare_parameter<std::string>("arm_group_name", "arm");
  gripper_action_name_ = declare_parameter<std::string>(
      "gripper_action_name", "/gripper_action_controller/gripper_cmd");
  planning_time_ = declare_parameter<double>("planning_time", 5.0);
  max_velocity_scaling_ = declare_parameter<double>("max_velocity_scaling", 0.5);
  max_acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling", 0.5);
  gripper_max_effort_ = declare_parameter<double>("gripper_max_effort", 50.0);

  RCLCPP_INFO(get_logger(), "arm_group_name       : %s", arm_group_name_.c_str());
  RCLCPP_INFO(get_logger(), "gripper_action_name  : %s", gripper_action_name_.c_str());
  RCLCPP_INFO(get_logger(), "planning_time        : %.1f s", planning_time_);
  RCLCPP_INFO(get_logger(), "velocity_scaling     : %.2f", max_velocity_scaling_);
  RCLCPP_INFO(get_logger(), "acceleration_scaling : %.2f", max_acceleration_scaling_);

  // MoveGroupInterface is initialised lazily on the first goal (see execute())
  // so this node can start before move_group is ready.
  planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);

  action_server_ = rclcpp_action::create_server<PickPlace>(this, "pick_place",
      std::bind(&PickPlaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "PickPlaceNode ready - action server: /pick_place");
}

PickPlaceNode::~PickPlaceNode() = default;

rclcpp_action::GoalResponse PickPlaceNode::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const PickPlace::Goal> goal)
{
  RCLCPP_INFO(get_logger(),
      "Received pick-and-place goal: pick frame='%s', place frame='%s', object_id='%s'",
      goal->pick_pose.header.frame_id.c_str(), goal->place_pose.header.frame_id.c_str(),
      goal->object_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlaceNode::handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received cancel request - stopping arm motion");
  if (move_group_) {
    move_group_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlaceNode::handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  // Spin execution in a separate thread so the action server stays responsive
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void PickPlaceNode::execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PickPlace::Result>();

  // Resolve defaults for optional goal fields
  const double approach_dist = (goal->approach_distance > 0.0) ? goal->approach_distance : 0.10;
  const double retreat_dist = (goal->retreat_distance > 0.0) ? goal->retreat_distance : 0.10;
  const double open_width = (goal->gripper_open_width > 0.0) ? goal->gripper_open_width : 0.08;
  const double close_width = goal->gripper_close_width;  // 0.0 is valid

  if (!move_group_) {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), arm_group_name_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialise MoveGroupInterface: %s", e.what());
      result->success = false;
      result->message = std::string("MoveGroupInterface init failed: ") + e.what();
      result->error_code = -1;
      goal_handle->abort(result);
      return;
    }
  }

  move_group_->setPlanningTime(planning_time_);
  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  auto check_cancel = [&]() -> bool { return goal_handle->is_canceling(); };

  auto abort_with = [&](const std::string & msg, int code) {
    RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
    result->success = false;
    result->message = msg;
    result->error_code = code;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  };

  const bool manage_object = !goal->object_id.empty();
  if (manage_object) {
    publish_feedback(goal_handle, "ADD_COLLISION_OBJECT", 0.05f);
    add_collision_object(goal->object_id, goal->pick_pose);
    rclcpp::sleep_for(200ms);  // let the scene update propagate
  }

  publish_feedback(goal_handle, "OPEN_GRIPPER", 0.10f);
  if (!control_gripper(open_width)) {
    abort_with("Failed to open gripper", -2);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled after open gripper", -99);
    return;
  }

  publish_feedback(goal_handle, "MOVE_TO_PRE_PICK", 0.20f);
  auto pre_pick = compute_approach_pose(goal->pick_pose, approach_dist);
  auto ec = move_to_pose(pre_pick);
  if (ec != MoveItErrorCode::SUCCESS) {
    abort_with("Failed to reach pre-pick pose (code=" + std::to_string(ec.val) + ")", ec.val);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled after pre-pick", -99);
    return;
  }

  publish_feedback(goal_handle, "MOVE_TO_PICK", 0.30f);
  ec = move_to_pose(goal->pick_pose);
  if (ec != MoveItErrorCode::SUCCESS) {
    abort_with("Failed to reach pick pose (code=" + std::to_string(ec.val) + ")", ec.val);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled at pick pose", -99);
    return;
  }

  publish_feedback(goal_handle, "CLOSE_GRIPPER", 0.40f);
  if (!control_gripper(close_width)) {
    abort_with("Failed to close gripper", -3);
    return;
  }

  if (manage_object) {
    publish_feedback(goal_handle, "ATTACH_OBJECT", 0.45f);
    move_group_->attachObject(goal->object_id);
  }

  publish_feedback(goal_handle, "RETREAT_FROM_PICK", 0.50f);
  ec = move_to_pose(pre_pick);
  if (ec != MoveItErrorCode::SUCCESS) {
    abort_with("Failed to retreat from pick (code=" + std::to_string(ec.val) + ")", ec.val);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled after pick retreat", -99);
    return;
  }

  publish_feedback(goal_handle, "MOVE_TO_PRE_PLACE", 0.60f);
  auto pre_place = compute_approach_pose(goal->place_pose, retreat_dist);
  ec = move_to_pose(pre_place);
  if (ec != MoveItErrorCode::SUCCESS) {
    abort_with("Failed to reach pre-place pose (code=" + std::to_string(ec.val) + ")", ec.val);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled after pre-place", -99);
    return;
  }

  publish_feedback(goal_handle, "MOVE_TO_PLACE", 0.70f);
  ec = move_to_pose(goal->place_pose);
  if (ec != MoveItErrorCode::SUCCESS) {
    abort_with("Failed to reach place pose (code=" + std::to_string(ec.val) + ")", ec.val);
    return;
  }
  if (check_cancel()) {
    abort_with("Cancelled at place pose", -99);
    return;
  }

  publish_feedback(goal_handle, "OPEN_GRIPPER", 0.80f);
  if (!control_gripper(open_width)) {
    abort_with("Failed to open gripper at place", -4);
    return;
  }

  if (manage_object) {
    publish_feedback(goal_handle, "DETACH_OBJECT", 0.85f);
    move_group_->detachObject(goal->object_id);
    rclcpp::sleep_for(100ms);
    remove_collision_object(goal->object_id);
  }

  publish_feedback(goal_handle, "RETREAT_FROM_PLACE", 0.90f);
  ec = move_to_pose(pre_place);
  if (ec != MoveItErrorCode::SUCCESS) {
    // Non-fatal: object is already placed
    RCLCPP_WARN(
        get_logger(), "Retreat from place failed (code=%d) - object placed successfully", ec.val);
  }

  publish_feedback(goal_handle, "DONE", 1.0f);
  result->success = true;
  result->message = "Pick and place completed successfully";
  result->error_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Pick-and-place succeeded");
}

moveit::core::MoveItErrorCode PickPlaceNode::move_to_pose(
    const geometry_msgs::msg::PoseStamped & target_pose)
{
  move_group_->setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ec = move_group_->plan(plan);
  if (ec != MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "Planning failed (code=%d)", ec.val);
    return ec;
  }
  ec = move_group_->execute(plan);
  if (ec != MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "Execution failed (code=%d)", ec.val);
  }
  move_group_->clearPoseTargets();
  return ec;
}

geometry_msgs::msg::PoseStamped PickPlaceNode::compute_approach_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    double distance)
{
  // Step back along the end-effector Z-axis by distance
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  tf2::Matrix3x3 rot(q);
  tf2::Vector3 z_axis = rot.getColumn(2);

  auto approach = pose;
  approach.pose.position.x -= z_axis.x() * distance;
  approach.pose.position.y -= z_axis.y() * distance;
  approach.pose.position.z -= z_axis.z() * distance;
  return approach;
}

bool PickPlaceNode::control_gripper(double width)
{
  if (!gripper_client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(
        get_logger(), "Gripper action server not available: %s", gripper_action_name_.c_str());
    return false;
  }

  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = width;
  goal_msg.command.max_effort = gripper_max_effort_;

  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

  std::promise<bool> promise;
  auto future = promise.get_future();

  send_goal_options.result_callback =
      [&promise](const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult & result) {
        promise.set_value(result.code == rclcpp_action::ResultCode::SUCCEEDED);
      };

  gripper_client_->async_send_goal(goal_msg, send_goal_options);

  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Gripper command timed out");
    return false;
  }
  return future.get();
}

void PickPlaceNode::add_collision_object(const std::string & id,
    const geometry_msgs::msg::PoseStamped & pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header = pose.header;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Represent the object as a small box (5 cm cube)
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose.pose);

  planning_scene_->applyCollisionObject(obj);
  RCLCPP_INFO(get_logger(), "Added collision object '%s' to planning scene", id.c_str());
}

void PickPlaceNode::remove_collision_object(const std::string & id)
{
  planning_scene_->removeCollisionObjects({id});
  RCLCPP_INFO(get_logger(), "Removed collision object '%s' from planning scene", id.c_str());
}

void PickPlaceNode::publish_feedback(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const std::string & state,
    float progress)
{
  auto feedback = std::make_shared<PickPlace::Feedback>();
  feedback->current_state = state;
  feedback->progress = progress;
  goal_handle->publish_feedback(feedback);
  RCLCPP_INFO(get_logger(), "[%.0f%%] %s", progress * 100.0f, state.c_str());
}

}  // namespace qrb_ros_pick_place
