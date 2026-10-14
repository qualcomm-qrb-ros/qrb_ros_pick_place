// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#pragma once

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vector>

#include "qrb_ros_pick_place/action/pick_place.hpp"

namespace qrb_ros_pick_place
{

/**
 * @brief Pick-and-place action server built on MoveIt Task Constructor.
 *
 * For every goal the node adds the object to the planning scene, builds an MTC
 * task (open hand, pick, place, optional return home), plans the whole task and
 * only then executes the best solution.
 *
 * The node must be started with the MoveIt configuration of the robot
 * (robot_description, robot_description_semantic, kinematics and planning
 * pipeline parameters), and move_group must load
 * move_group/ExecuteTaskSolutionCapability.
 */
class PickPlaceNode : public rclcpp::Node
{
public:
  using PickPlace = qrb_ros_pick_place::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

  explicit PickPlaceNode(const rclcpp::NodeOptions & options);
  ~PickPlaceNode() override;

private:
  enum class Phase
  {
    IDLE,
    PLANNING,
    EXECUTING
  };

  struct Config
  {
    std::string arm_group;
    std::string hand_group;
    std::string hand_frame;
    std::string hand_open_pose;
    std::string hand_close_pose;
    std::string home_pose;
    std::string planning_pipeline;
    std::string planner_id;
    std::vector<double> grasp_frame_xyz;
    std::vector<double> grasp_frame_rpy;
    double grasp_angle_delta;
    int grasp_max_ik_solutions;
    double approach_min_distance;
    double approach_max_distance;
    double lift_min_distance;
    double lift_max_distance;
    double retreat_min_distance;
    double retreat_max_distance;
    double planning_timeout;
    int max_solutions;
    double max_velocity_scaling;
    double max_acceleration_scaling;
  };

  template <typename T>
  T get_or_declare(const std::string & name, const T & default_value);

  void load_parameters();

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PickPlace::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  bool add_object_to_scene(const PickPlace::Goal & goal);

  std::unique_ptr<moveit::task_constructor::Task> create_task(const PickPlace::Goal & goal);

  void finish(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
      bool success,
      int32_t error_code,
      const std::string & message);

  void publish_state(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
      const std::string & state);

  Config config_;

  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
  moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner_;

  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<Phase> phase_{Phase::IDLE};
  std::atomic<moveit::task_constructor::Task *> active_task_{nullptr};
};

}  // namespace qrb_ros_pick_place
