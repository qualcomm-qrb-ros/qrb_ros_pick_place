// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "qrb_ros_pick_place/pick_place_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveIt parameters (robot_description, kinematics, planning pipelines) are
  // passed in as overrides, so they must be declared automatically.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<qrb_ros_pick_place::PickPlaceNode>(options);

  // Planning and execution block inside the worker thread and need the
  // executor to keep serving their action and service clients.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
