<div align="center">
  <h1>qrb_ros_pick_place</h1>
  <p align="center">
    <!-- Add images or videos to showcase your project demo, use case, or logo -->
  </p>
  <p>A ROS 2 action server that performs pick-and-place with MoveIt Task Constructor, given the current pose and the target pose of an object.</p>

  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>

</div>

---

## 👋 Overview

`qrb_ros_pick_place` provides a ROS 2 action server, `pick_place`. A client sends one goal with the object (id, size, current pose) and the pose the object should have after placing. The node then:

1. Adds the object to the MoveIt planning scene as a box.
2. Builds a [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) (MTC) task for the whole pick-and-place sequence.
3. Plans the complete task first. If planning fails, the robot does not move.
4. Executes the best solution.

The stage structure follows the MTC pick-and-place tutorial:

```
current state -> open hand -> move to pick
  -> pick object:  approach -> generate grasp pose + IK -> allow collision
                   -> close hand -> attach object -> lift
  -> move to place
  -> place object: generate place pose + IK -> open hand -> forbid collision
                   -> detach object -> retreat
  -> return home (optional)
```

The grasp pose is sampled around the object by MTC (`GenerateGraspPose` and `ComputeIK`), so the client does not give a grasp pose.

**Architecture:**

```
 client --PickPlace action--> PickPlaceNode
                                 |  builds mtc::Task per goal
                                 v
                        MoveIt Task Constructor
                          |                 |
                plan (PipelinePlanner,   execute (ExecuteTaskSolution
                 CartesianPath, ...)      action of move_group)
                          \                 /
                           v               v
                              move_group
```

## 🔎 Table of Contents

  * [APIs](#-apis)
  * [Supported Targets](#-supported-targets)
  * [Installation](#-installation)
  * [Usage](#-usage)
  * [Build from Source](#-build-from-source)
  * [Contributing](#-contributing)
  * [FAQs](#-faqs)
  * [License](#-license)

## ⚓ APIs

### 🔹 qrb_ros_pick_place APIs

#### ROS Interfaces

| Kind | Name | Type | Description |
|------|------|------|-------------|
| Action Server | `pick_place` | `qrb_ros_pick_place/action/PickPlace` | Pick-and-place of one object |

The node also talks to `move_group` through the interfaces MTC uses (planning, planning scene, and the `ExecuteTaskSolution` action).

##### Action: `qrb_ros_pick_place/action/PickPlace`

**Goal**

| Field | Type | Description |
|-------|------|-------------|
| `object_id` | `string` | Name of the object in the planning scene. Must not be empty |
| `object_pose` | `geometry_msgs/PoseStamped` | Current pose of the object frame |
| `target_pose` | `geometry_msgs/PoseStamped` | Pose of the object frame after placing |
| `object_size` | `geometry_msgs/Vector3` | Box size in metres. All values must be positive |

**Feedback**

| Field | Type | Description |
|-------|------|-------------|
| `state` | `string` | `PLANNING` or `EXECUTING` |

**Result**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if the sequence was executed |
| `message` | `string` | Outcome, or the MTC failure explanation |
| `error_code` | `int32` | `moveit_msgs/MoveItErrorCodes` value |

A goal is rejected if a field is invalid or if another goal is running. A cancel request is accepted while planning and rejected once execution has started, because MTC cannot interrupt an execution.

#### ROS Parameters

Defaults are in [config/pick_place.yaml](config/pick_place.yaml). The defaults for the robot names are those of the Panda in the MTC tutorial and must be changed for other robots.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `arm_group` | `string` | `panda_arm` | Planning group of the arm |
| `hand_group` | `string` | `hand` | Planning group of the gripper |
| `hand_frame` | `string` | `panda_hand` | IK frame of the hand |
| `hand_open_pose` | `string` | `open` | Named pose of `hand_group` for open |
| `hand_close_pose` | `string` | `close` | Named pose of `hand_group` for closed |
| `home_pose` | `string` | `""` | Named pose of `arm_group` to return to. Empty to skip |
| `planning_pipeline` | `string` | `ompl` | Pipeline used for free-space motion |
| `planner_id` | `string` | `""` | Planner of the pipeline. Empty for the default |
| `grasp_frame_xyz` | `double[3]` | `[0, 0, 0.1]` | Grasp frame position in the hand frame |
| `grasp_frame_rpy` | `double[3]` | `[pi, -pi/2, 0]` | Grasp frame orientation in the hand frame |
| `grasp_angle_delta` | `double` | `pi/12` | Angle step of the sampled grasps (rad) |
| `grasp_max_ik_solutions` | `int` | `8` | Maximum IK solutions for the grasp pose |
| `approach_min_distance`, `approach_max_distance` | `double` | `0.1`, `0.15` | Approach distance range (m) |
| `lift_min_distance`, `lift_max_distance` | `double` | `0.1`, `0.3` | Lift distance range (m) |
| `retreat_min_distance`, `retreat_max_distance` | `double` | `0.1`, `0.3` | Retreat distance range (m) |
| `planning_timeout` | `double` | `5.0` | Timeout of each `Connect` stage (s) |
| `max_solutions` | `int` | `5` | Number of solutions planned before one is picked |
| `max_velocity_scaling` | `double` | `0.5` | Velocity scaling factor |
| `max_acceleration_scaling` | `double` | `0.5` | Acceleration scaling factor |

The node must also receive the MoveIt parameters of the robot (`robot_description`, `robot_description_semantic`, kinematics and planning pipeline parameters). The launch file does this.

## 🎯 Supported Targets

Any ROS 2 Jazzy robot that has a MoveIt 2 configuration with:

- an arm group and a gripper group, and the gripper joints are controlled through a MoveIt controller;
- named poses for the open and closed gripper in the SRDF;
- an end effector defined for the arm group in the SRDF.

---

## ✨ Installation

```bash
sudo apt update
sudo apt install \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-capabilities
```

The optional package `ros-jazzy-moveit-task-constructor-visualization` adds the MTC panel in RViz, which shows the result of every stage.

## 🚀 Usage

### 1. Load the MTC execution capability in move_group

MTC executes through an action server that is a `move_group` capability. Add it to the `capabilities` parameter of `move_group`:

```yaml
capabilities: move_group/ExecuteTaskSolutionCapability
```

Without it, planning works but execution fails.

### 2. Start move_group and the controllers of your robot

This package does not start them.

### 3. Start the node

```bash
ros2 launch qrb_ros_pick_place pick_place.launch.py \
  robot_name:=<robot_name> \
  moveit_config_package:=<moveit_config_package> \
  params_file:=<path to a yaml with your group and frame names>
```

`moveit_config_package` defaults to `<robot_name>_moveit_config`. Copy [config/pick_place.yaml](config/pick_place.yaml) and change the group, frame and named pose parameters to match your robot.

### 4. Send a goal

```bash
ros2 action send_goal /pick_place qrb_ros_pick_place/action/PickPlace "{
  object_id: 'object',
  object_size: {x: 0.02, y: 0.02, z: 0.1},
  object_pose: {
    header: {frame_id: 'world'},
    pose: {position: {x: 0.5, y: -0.25, z: 0.05}, orientation: {w: 1.0}}
  },
  target_pose: {
    header: {frame_id: 'world'},
    pose: {position: {x: 0.5, y: 0.25, z: 0.05}, orientation: {w: 1.0}}
  }
}" --feedback
```

`target_pose` is the pose of the object, not of the hand. The frame of the box is its centre, so `z` is half the height when the object stands on the ground plane.

---

## 👨‍💻 Build from Source

```bash
# Clone into the workspace
cd ~/ros2_ws/src
git clone https://github.com/qualcomm-linux/qrb_ros_pick_place.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build from the workspace root
colcon build --packages-select qrb_ros_pick_place
source install/setup.bash
```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).  
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## ❔ FAQs

<details>
<summary>Planning succeeds but execution fails</summary><br>
Check that <code>move_group</code> loads <code>move_group/ExecuteTaskSolutionCapability</code> and that the controllers for both the arm and the gripper are configured in MoveIt.
</details>

<details>
<summary>Planning fails</summary><br>
The failure explanation of MTC is returned in the result message. Typical causes are an unreachable object pose or target pose, a wrong <code>grasp_frame_xyz</code> and <code>grasp_frame_rpy</code> for your gripper, or approach and lift distances that collide with the scene. Increase <code>max_solutions</code> or <code>planning_timeout</code> to give the planner more room.
</details>

<details>
<summary>Can I set the gripper force?</summary><br>
No. The gripper is moved to the named poses through MoveIt, which only controls the joint positions.
</details>

## 📜 License

This project is licensed under the [BSD 3-Clause Clear License](LICENSE).
