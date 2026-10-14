// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#pragma once

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "qrb_ros_pick_place/action/pick_place.hpp"

namespace qrb_ros_pick_place
{

/**
 * @brief Pick-and-place control node.
 *
 * Exposes a single ROS 2 action server `/pick_place` that drives a robot arm
 * through a complete grasp-and-release cycle using MoveIt 2:
 *
 *   Open gripper -> Pre-pick -> Pick -> Close gripper -> Retreat
 *   -> Pre-place -> Place -> Open gripper -> Retreat -> Done
 *
 * Parameters
 * ----------
 * arm_group_name          (string,  default "arm")
 * gripper_action_name     (string,  default "/gripper_action_controller/gripper_cmd")
 * planning_time           (double,  default 5.0)   seconds
 * max_velocity_scaling    (double,  default 0.5)
 * max_acceleration_scaling(double,  default 0.5)
 * gripper_max_effort      (double,  default 50.0)  Newtons
 */
class PickPlaceNode : public rclcpp::Node
{
public:
  using PickPlace = qrb_ros_pick_place::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;
  using GripperCommand = control_msgs::action::GripperCommand;

  explicit PickPlaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PickPlaceNode() override;

private:
  // ROS interfaces
  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  // MoveIt 2
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  // Parameters
  std::string arm_group_name_;
  std::string gripper_action_name_;
  double planning_time_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
  double gripper_max_effort_;

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PickPlace::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  // Execution (runs in a dedicated thread)
  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  /**
   * @brief Move the arm end-effector to @p target_pose.
   * @return MoveIt error code (SUCCESS == 1).
   */
  moveit::core::MoveItErrorCode move_to_pose(const geometry_msgs::msg::PoseStamped & target_pose);

  /**
   * @brief Compute a standoff pose offset along the end-effector Z-axis.
   *
   * A positive @p distance moves the pose *away* from the approach direction
   * (i.e. the pre-pick / pre-place waypoint).
   */
  geometry_msgs::msg::PoseStamped compute_approach_pose(
      const geometry_msgs::msg::PoseStamped & pose,
      double distance);

  /**
   * @brief Send a gripper command and wait for completion.
   * @param width  Target gap in metres.
   * @return true on success.
   */
  bool control_gripper(double width);

  /**
   * @brief Add a small box collision object at @p pose with the given @p id.
   */
  void add_collision_object(const std::string & id, const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Remove a collision object by @p id.
   */
  void remove_collision_object(const std::string & id);

  void publish_feedback(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
      const std::string & state,
      float progress);
};

}  // namespace qrb_ros_pick_place
