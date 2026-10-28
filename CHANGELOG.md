# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-16

### Added
- Pick-and-place action server `pick_place_node`, built on MoveIt Task
  Constructor (MTC). The whole task is planned first and executed only if
  planning succeeds.
- Action interface `qrb_ros_pick_place/action/PickPlace`. The goal holds the
  object id, size, current pose and target pose; the feedback reports PLANNING
  or EXECUTING; the result holds success, message and a MoveIt error code.
- MTC task following the MoveIt pick-and-place tutorial: open hand, approach,
  sampled grasp pose with IK, close hand, attach, lift, sampled place pose with
  IK, open hand, detach, retreat, optional return home.
- The object is added to the planning scene as a box for every goal.
- Cancellation while planning through `Task::preempt()`.
- ROS 2 parameters for the group, frame and named pose names of the robot, the
  grasp frame, the motion distances, the planning limits and the scaling
  factors. Defaults are in `config/pick_place.yaml`.
- Launch file `pick_place.launch.py`, which loads the MoveIt configuration of
  the robot with `MoveItConfigsBuilder`.
- README with the architecture, the API, the setup of `move_group` and a usage
  example.
