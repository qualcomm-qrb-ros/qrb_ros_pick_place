// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pick_place/pick_place_node.hpp"

#include <moveit/task_constructor/stages.h>

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sstream>
#include <utility>

namespace mtc = moveit::task_constructor;
using moveit_msgs::msg::MoveItErrorCodes;

namespace qrb_ros_pick_place
{

PickPlaceNode::PickPlaceNode(const rclcpp::NodeOptions & options) : Node("pick_place_node", options)
{
  load_parameters();

  action_server_ = rclcpp_action::create_server<PickPlace>(this, "pick_place",
      std::bind(&PickPlaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "PickPlaceNode ready, action server: pick_place");
}

PickPlaceNode::~PickPlaceNode()
{
  if (auto * task = active_task_.load()) {
    task->preempt();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

template <typename T>
T PickPlaceNode::get_or_declare(const std::string & name, const T & default_value)
{
  // The node may be started with automatically_declare_parameters_from_overrides
  if (!has_parameter(name)) {
    declare_parameter<T>(name, default_value);
  }
  return get_parameter(name).get_value<T>();
}

void PickPlaceNode::load_parameters()
{
  config_.arm_group = get_or_declare<std::string>("arm_group", "panda_arm");
  config_.hand_group = get_or_declare<std::string>("hand_group", "hand");
  config_.hand_frame = get_or_declare<std::string>("hand_frame", "panda_hand");
  config_.hand_open_pose = get_or_declare<std::string>("hand_open_pose", "open");
  config_.hand_close_pose = get_or_declare<std::string>("hand_close_pose", "close");
  config_.home_pose = get_or_declare<std::string>("home_pose", "");
  config_.planning_pipeline = get_or_declare<std::string>("planning_pipeline", "ompl");
  config_.planner_id = get_or_declare<std::string>("planner_id", "");

  // Grasp frame in the hand frame, so that the object frame is aligned with
  // this frame when the hand is at the grasp pose. Defaults are the values of
  // the MTC pick-and-place tutorial for the Panda hand.
  config_.grasp_frame_xyz =
      get_or_declare<std::vector<double>>("grasp_frame_xyz", std::vector<double>{0.0, 0.0, 0.1});
  config_.grasp_frame_rpy = get_or_declare<std::vector<double>>(
      "grasp_frame_rpy", std::vector<double>{M_PI, -M_PI / 2.0, 0.0});
  config_.grasp_angle_delta = get_or_declare<double>("grasp_angle_delta", M_PI / 12.0);
  config_.grasp_max_ik_solutions = get_or_declare<int>("grasp_max_ik_solutions", 8);

  config_.approach_min_distance = get_or_declare<double>("approach_min_distance", 0.1);
  config_.approach_max_distance = get_or_declare<double>("approach_max_distance", 0.15);
  config_.lift_min_distance = get_or_declare<double>("lift_min_distance", 0.1);
  config_.lift_max_distance = get_or_declare<double>("lift_max_distance", 0.3);
  config_.retreat_min_distance = get_or_declare<double>("retreat_min_distance", 0.1);
  config_.retreat_max_distance = get_or_declare<double>("retreat_max_distance", 0.3);

  config_.planning_timeout = get_or_declare<double>("planning_timeout", 5.0);
  config_.max_solutions = get_or_declare<int>("max_solutions", 5);
  config_.max_velocity_scaling = get_or_declare<double>("max_velocity_scaling", 0.5);
  config_.max_acceleration_scaling = get_or_declare<double>("max_acceleration_scaling", 0.5);

  if (config_.grasp_frame_xyz.size() != 3 || config_.grasp_frame_rpy.size() != 3) {
    throw std::invalid_argument("grasp_frame_xyz and grasp_frame_rpy must have 3 elements");
  }
}

rclcpp_action::GoalResponse PickPlaceNode::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const PickPlace::Goal> goal)
{
  if (goal->object_id.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: object_id is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->object_size.x <= 0.0 || goal->object_size.y <= 0.0 || goal->object_size.z <= 0.0) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: object_size must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->object_pose.header.frame_id.empty() || goal->target_pose.header.frame_id.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: pose frame_id is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: another pick-and-place is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlaceNode::handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace> /*goal_handle*/)
{
  // Cancellation is honoured while planning. Once execution has started the
  // trajectory runs to the end, because MTC has no execution preempt.
  if (phase_.load() == Phase::EXECUTING) {
    RCLCPP_WARN(get_logger(), "Cancel rejected: the task is already executing");
    return rclcpp_action::CancelResponse::REJECT;
  }
  if (auto * task = active_task_.load()) {
    task->preempt();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlaceNode::handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  if (worker_.joinable()) {
    worker_.join();
  }
  worker_ = std::thread([this, goal_handle]() { execute(goal_handle); });
}

bool PickPlaceNode::add_object_to_scene(const PickPlace::Goal & goal)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = goal.object_id;
  object.header = goal.object_pose.header;
  object.pose = goal.object_pose.pose;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {goal.object_size.x, goal.object_size.y, goal.object_size.z};
  object.primitives.push_back(box);

  geometry_msgs::msg::Pose origin;
  origin.orientation.w = 1.0;
  object.primitive_poses.push_back(origin);

  moveit::planning_interface::PlanningSceneInterface psi;
  return psi.applyCollisionObject(object);
}

std::unique_ptr<mtc::Task> PickPlaceNode::create_task(const PickPlace::Goal & goal)
{
  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("pick and place");
  task->loadRobotModel(shared_from_this());

  const auto robot_model = task->getRobotModel();
  if (!robot_model->hasJointModelGroup(config_.arm_group) ||
      !robot_model->hasJointModelGroup(config_.hand_group)) {
    throw std::runtime_error("arm_group or hand_group not found in the robot model");
  }

  const std::string & arm = config_.arm_group;
  const std::string & hand = config_.hand_group;
  const std::string & hand_frame = config_.hand_frame;
  const std::string & object = goal.object_id;

  task->setProperty("group", arm);
  task->setProperty("eef", hand);
  task->setProperty("ik_frame", hand_frame);

  // PipelinePlanner needs shared_from_this(), so it is created on first use
  if (!sampling_planner_) {
    sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(
        shared_from_this(), config_.planning_pipeline, config_.planner_id);
  }
  sampling_planner_->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  sampling_planner_->setMaxAccelerationScalingFactor(config_.max_acceleration_scaling);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  interpolation_planner->setMaxAccelerationScalingFactor(config_.max_acceleration_scaling);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(config_.max_acceleration_scaling);
  cartesian_planner->setStepSize(0.01);

  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::Stage * current_state_ptr = current_state.get();
  task->add(std::move(current_state));

  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  open_hand->setGroup(hand);
  open_hand->setGoal(config_.hand_open_pose);
  task->add(std::move(open_hand));

  auto move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner_}});
  move_to_pick->setTimeout(config_.planning_timeout);
  move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task->add(std::move(move_to_pick));

  mtc::Stage * attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task->properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(config_.approach_min_distance, config_.approach_max_distance);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(config_.hand_open_pose);
      stage->setObject(object);
      stage->setAngleDelta(config_.grasp_angle_delta);
      stage->setMonitoredStage(current_state_ptr);

      const auto & rpy = config_.grasp_frame_rpy;
      const Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.linear() = q.toRotationMatrix();
      grasp_frame_transform.translation() = Eigen::Vector3d(
          config_.grasp_frame_xyz[0], config_.grasp_frame_xyz[1], config_.grasp_frame_xyz[2]);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(config_.grasp_max_ik_solutions);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object,
          robot_model->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(), true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand);
      stage->setGoal(config_.hand_close_pose);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(config_.lift_min_distance, config_.lift_max_distance);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = robot_model->getModelFrame();
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      grasp->insert(std::move(stage));
    }

    task->add(std::move(grasp));
  }

  {
    auto move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner_}});
    move_to_place->setTimeout(config_.planning_timeout);
    move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task->add(std::move(move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object);
      stage->setPose(goal.target_pose);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand);
      stage->setGoal(config_.hand_open_pose);
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object,
          robot_model->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(), false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object, hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(config_.retreat_min_distance, config_.retreat_max_distance);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Retreat backwards along the approach axis of the hand
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = -1.0;
      stage->setDirection(direction);
      place->insert(std::move(stage));
    }

    task->add(std::move(place));
  }

  if (!config_.home_pose.empty()) {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(config_.home_pose);
    task->add(std::move(stage));
  }

  return task;
}

void PickPlaceNode::finish(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    bool success,
    int32_t error_code,
    const std::string & message)
{
  auto result = std::make_shared<PickPlace::Result>();
  result->success = success;
  result->error_code = error_code;
  result->message = message;

  if (success) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    goal_handle->succeed(result);
  } else {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  }

  phase_ = Phase::IDLE;
  busy_ = false;
}

void PickPlaceNode::publish_state(const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const std::string & state)
{
  auto feedback = std::make_shared<PickPlace::Feedback>();
  feedback->state = state;
  goal_handle->publish_feedback(feedback);
}

void PickPlaceNode::execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  phase_ = Phase::PLANNING;
  publish_state(goal_handle, "PLANNING");

  if (!add_object_to_scene(*goal)) {
    finish(goal_handle, false, MoveItErrorCodes::FAILURE,
        "Failed to add object '" + goal->object_id + "' to the planning scene");
    return;
  }

  std::unique_ptr<mtc::Task> task;
  try {
    task = create_task(*goal);
    task->init();
  } catch (const mtc::InitStageException & e) {
    std::ostringstream os;
    os << "Task initialisation failed: " << e;
    finish(goal_handle, false, MoveItErrorCodes::FAILURE, os.str());
    return;
  } catch (const std::exception & e) {
    finish(goal_handle, false, MoveItErrorCodes::FAILURE,
        std::string("Failed to build the task: ") + e.what());
    return;
  }

  active_task_ = task.get();
  const auto plan_result = task->plan(static_cast<size_t>(config_.max_solutions));
  active_task_ = nullptr;

  if (goal_handle->is_canceling()) {
    finish(goal_handle, false, MoveItErrorCodes::PREEMPTED, "Cancelled while planning");
    return;
  }
  if (!plan_result || task->solutions().empty()) {
    std::ostringstream os;
    task->explainFailure(os);
    finish(
        goal_handle, false, MoveItErrorCodes::PLANNING_FAILED, "Task planning failed: " + os.str());
    return;
  }

  task->introspection().publishSolution(*task->solutions().front());

  phase_ = Phase::EXECUTING;
  publish_state(goal_handle, "EXECUTING");

  const auto exec_result = task->execute(*task->solutions().front());
  if (exec_result.val != MoveItErrorCodes::SUCCESS) {
    finish(goal_handle, false, exec_result.val,
        "Task execution failed, error code " + std::to_string(exec_result.val));
    return;
  }

  finish(goal_handle, true, MoveItErrorCodes::SUCCESS, "Pick and place completed successfully");
}

}  // namespace qrb_ros_pick_place
