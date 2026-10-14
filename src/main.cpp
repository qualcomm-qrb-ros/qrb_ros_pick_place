// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <rclcpp/rclcpp.hpp>

#include "qrb_ros_pick_place/pick_place_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Use a MultiThreadedExecutor so the action server callbacks and the
  // execution thread can run concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<qrb_ros_pick_place::PickPlaceNode>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
