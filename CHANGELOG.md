# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-16

### Added
- Initial implementation of the pick-and-place control node (`pick_place_node`).
- Custom ROS 2 action interface `qrb_ros_pick_place/action/PickPlace` with goal
  fields for pick pose, place pose, object ID, approach/retreat distances, and
  gripper widths; feedback for current state and progress; result for success and
  error code.
- MoveIt 2 integration via `MoveGroupInterface` for arm trajectory planning and
  execution (lazy-initialised on first goal).
- Gripper control via `control_msgs/action/GripperCommand` action client.
- Optional planning-scene collision object management: add, attach, detach, and
  remove a box-shaped object keyed by `object_id`.
- 11-step execution state machine: open gripper, pre-pick, pick, close gripper,
  attach, retreat, pre-place, place, open gripper, detach, retreat.
- Cancellation support at every motion step.
- ROS 2 parameters: `arm_group_name`, `gripper_action_name`, `planning_time`,
  `max_velocity_scaling`, `max_acceleration_scaling`, `gripper_max_effort`.
- README with architecture overview, API tables, usage examples, and build
  instructions.

### Changed
- Copyright header updated to
  `Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.`
  with SPDX identifier `BSD-3-Clause-Clear`.
- Maintainer set to Peng Wang `<penwang@qti.qualcomm.com>`.

### Fixed
- Removed decorative separator comments and non-ASCII characters from all source
  files to comply with project code-style rules.
