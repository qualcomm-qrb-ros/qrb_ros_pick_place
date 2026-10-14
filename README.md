<div align="center">
  <h1>qrb_ros_pick_place</h1>
  <p align="center">
    <!-- Add images or videos to showcase your project demo, use case, or logo -->
  </p>
  <p>A ROS 2 control node that performs pick-and-place operations via MoveIt 2, given a pick pose and a place pose.</p>

  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>

</div>

---

## 👋 Overview

`qrb_ros_pick_place` provides a single ROS 2 action server node that drives a robot arm through a complete pick-and-place cycle using MoveIt 2. A client sends one action goal containing:

- **`pick_pose`** — the end-effector pose at which the gripper should grasp the object.
- **`place_pose`** — the end-effector pose at which the gripper should release the object.

The node then autonomously executes the following state machine:

```
Open Gripper → Move to Pre-Pick → Move to Pick → Close Gripper
  → Attach Object → Retreat → Move to Pre-Place → Move to Place
  → Open Gripper → Detach Object → Retreat → Done
```

**Key features:**

- ROS 2 Action interface with per-stage feedback and cancellation support.
- Automatic pre-pick / pre-place waypoint generation (approach along the end-effector Z-axis).
- Optional MoveIt 2 planning-scene collision object management (attach / detach).
- Gripper control via the standard `control_msgs/action/GripperCommand` action.
- Fully parameterised: arm group name, gripper action name, velocity/acceleration scaling, approach distances, and gripper widths are all ROS 2 parameters.

**Architecture:**

```
┌─────────────────────────────────────────────────────────┐
│                   qrb_ros_pick_place                    │
│                                                         │
│  PickPlaceNode (rclcpp::Node)                           │
│  ┌─────────────────────────────────────────────────┐   │
│  │  Action Server: /pick_place                     │   │
│  │  (qrb_ros_pick_place/action/PickPlace)          │   │
│  └──────────────────┬──────────────────────────────┘   │
│                     │                                   │
│  ┌──────────────────▼──────────────────────────────┐   │
│  │  State Machine                                  │   │
│  │  OPEN_GRIPPER → PRE_PICK → PICK →               │   │
│  │  CLOSE_GRIPPER → RETREAT → PRE_PLACE →          │   │
│  │  PLACE → OPEN_GRIPPER → RETREAT → DONE          │   │
│  └──────┬──────────────────────────┬───────────────┘   │
│         │                          │                   │
│  ┌──────▼──────┐          ┌────────▼────────┐          │
│  │ MoveIt 2    │          │ Gripper Action  │          │
│  │ MoveGroup   │          │ Client          │          │
│  │ Interface   │          │ (GripperCommand)│          │
│  └─────────────┘          └─────────────────┘          │
└─────────────────────────────────────────────────────────┘
```

## 🔎 Table of Contents

  * [APIs](#-apis)
  * [Supported Targets](#-supported-targets)
  * [Installation](#-installation)
  * [Usage](#-usage)
  * [Build from Source](#-build-from-source)
  * [Contributing](#-contributing)
  * [License](#-license)

## ⚓ APIs

### 🔹 qrb_ros_pick_place ROS APIs

#### ROS Interfaces

| Kind | Name | Type | Description |
|------|------|------|-------------|
| Action Server | `/pick_place` | `qrb_ros_pick_place/action/PickPlace` | Main pick-and-place action |
| Action Client | `<gripper_action_name>` | `control_msgs/action/GripperCommand` | Gripper open/close control |

##### Action: `qrb_ros_pick_place/action/PickPlace`

**Goal**

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `pick_pose` | `geometry_msgs/PoseStamped` | — | End-effector pose for grasping |
| `place_pose` | `geometry_msgs/PoseStamped` | — | End-effector pose for releasing |
| `object_id` | `string` | `""` | Collision object ID in the planning scene (optional) |
| `approach_distance` | `float64` | `0.10` | Pre-pick / pre-place standoff distance along EEF Z-axis (m) |
| `retreat_distance` | `float64` | `0.10` | Post-pick / post-place retreat distance along EEF Z-axis (m) |
| `gripper_open_width` | `float64` | `0.08` | Gripper gap when open (m) |
| `gripper_close_width` | `float64` | `0.00` | Gripper gap when closed (m) |

**Feedback**

| Field | Type | Description |
|-------|------|-------------|
| `current_state` | `string` | Human-readable name of the current execution stage |
| `progress` | `float32` | Overall progress in [0.0, 1.0] |

**Result**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if the full cycle completed successfully |
| `message` | `string` | Human-readable outcome or error description |
| `error_code` | `int32` | MoveIt error code (0 = SUCCESS) |

#### ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `arm_group_name` | `string` | `"arm"` | MoveIt 2 planning group name for the arm |
| `gripper_action_name` | `string` | `"/gripper_action_controller/gripper_cmd"` | Action server name for gripper control |
| `planning_time` | `double` | `5.0` | Maximum planning time per motion segment (s) |
| `max_velocity_scaling` | `double` | `0.5` | Joint velocity scaling factor (0, 1] |
| `max_acceleration_scaling` | `double` | `0.5` | Joint acceleration scaling factor (0, 1] |
| `gripper_max_effort` | `double` | `50.0` | Maximum gripper effort (N) |

## 🎯 Supported Targets

| Target | Notes |
|--------|-------|
| Qualcomm RB3 Gen 2 | Primary target platform |
| Any ROS 2 Jazzy robot with MoveIt 2 | Requires a valid MoveIt 2 configuration and a `control_msgs/GripperCommand` action server |

---

## ✨ Installation

```bash
sudo apt update
sudo apt install ros-jazzy-qrb-ros-pick-place
```

## 🚀 Usage

### 1. Launch the node

```bash
ros2 run qrb_ros_pick_place pick_place_node \
  --ros-args \
  -p arm_group_name:=arm \
  -p gripper_action_name:=/gripper_action_controller/gripper_cmd \
  -p planning_time:=5.0 \
  -p max_velocity_scaling:=0.5
```

### 2. Send a pick-and-place goal

```bash
ros2 action send_goal /pick_place qrb_ros_pick_place/action/PickPlace "{
  pick_pose: {
    header: {frame_id: 'world'},
    pose: {
      position: {x: 0.4, y: 0.0, z: 0.2},
      orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
    }
  },
  place_pose: {
    header: {frame_id: 'world'},
    pose: {
      position: {x: 0.4, y: 0.3, z: 0.2},
      orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
    }
  },
  approach_distance: 0.1,
  retreat_distance: 0.1,
  gripper_open_width: 0.08,
  gripper_close_width: 0.0
}" --feedback
```

### 3. Expected output

The action server publishes feedback at each stage:

```
Feedback: current_state='OPEN_GRIPPER'   progress=0.10
Feedback: current_state='MOVE_TO_PRE_PICK' progress=0.20
Feedback: current_state='MOVE_TO_PICK'   progress=0.30
Feedback: current_state='CLOSE_GRIPPER'  progress=0.40
Feedback: current_state='RETREAT'        progress=0.50
Feedback: current_state='MOVE_TO_PRE_PLACE' progress=0.60
Feedback: current_state='MOVE_TO_PLACE'  progress=0.70
Feedback: current_state='OPEN_GRIPPER'   progress=0.80
Feedback: current_state='RETREAT'        progress=0.90
Result:   success=True  message='Pick and place completed successfully'
```

### 4. Cancel a goal in progress

```bash
ros2 action cancel /pick_place <goal_id>
```

---

## 👨‍💻 Build from Source

```bash
# 1. Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/qualcomm-linux/qrb_ros_pick_place.git

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --packages-select qrb_ros_pick_place

# 4. Source and verify
source install/setup.bash
ros2 pkg list | grep qrb_ros_pick_place
```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).  
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## ❔ FAQs

<details>
<summary>The action goal is rejected immediately — what should I check?</summary><br>
Ensure that the MoveIt 2 <code>move_group</code> node is running and that the planning group name matches the <code>arm_group_name</code> parameter. Also verify that the gripper action server is available with <code>ros2 action list</code>.
</details>

<details>
<summary>Planning fails with "No motion plan found"</summary><br>
Try increasing <code>planning_time</code>, reducing <code>max_velocity_scaling</code>, or checking that the target poses are reachable and collision-free in the planning scene.
</details>

<details>
<summary>How do I use collision object management?</summary><br>
Set a non-empty <code>object_id</code> in the goal. The node will add a box-shaped collision object at the pick pose before planning, attach it to the end-effector after grasping, and detach/remove it after placing.
</details>

## 📜 License

This project is licensed under the [BSD 3-Clause License](LICENSE).
