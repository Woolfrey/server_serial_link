<a id="top"></a>
# Serial Link Action Server

This ROS2 package provides a set of action servers for controlling serial link robot arms.

> [!NOTE]
> This package is still under development. More features will be added.

## Table of Contents

- [Overview](#overview)
  - [Components](#components)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Nodes](#launching-the-nodes)
  - [Testing the Action Server](#testing-the-action-server)
- [Contributing](#contributing)
- [License](#license)

## Overview

### Components

1. **ModelUpdater**
   - **File:** `ModelUpdater.h`
   - **Description:** A ROS2 node that subscribes to a joint state topic and updates the robot's kinematics and dynamics as it receives messages.

2. **TrackJointTrajectory**
   - **File:** `TrackJointTrajectory.h`
   - **Description:** A ROS2 action server that handles joint trajectory tracking for a serial link robot arm. It processes requests, tracks joint trajectories, and provides feedback on position and velocity errors.

3. **Velocity Control Server**
   - **File:** `velocity_control_server.cpp`
   - **Description:** The main executable for launching the control server, which includes initializing nodes and handling robot control and trajectory tracking.
  
4. **Torque Control Server**
   - **File:** `torque_control_server.cpp`
   - **Description:** Launches a control server, but computes the inverse dynamics necessary to execute control actions, such as joint trajectory tracking, and Cartesian control.

> [!WARNING]
> The torque control actions are still not fully complete.

## Prerequisites

- ROS2 (e.g., Foxy, Galactic)
- `RobotLibrary` (see [RobotLibrary GitHub](https://github.com/Woolfrey/software_robot_library))
- `mujoco_interface` (see [mujoco_interface GitHub](https://github.com/Woolfrey/interface_mujoco_ros2))

## Installation

1. **Clone the repository:**

   ```
   git clone https://github.com/Woolfrey/server_serial_link.git
   ```

2. **Install dependencies:**

Ensure you have ROS2 installed and sourced. Also, install RobotLibrary and mujoco_interface as needed.

3. **Build the package:**

Navigate to your ROS2 workspace and build the package:

```
cd your-ros2-workspace
colcon build
```

4: **Source the workspace:**
```
source install/setup.bash
```

### Usage
#### Launching the Nodes

There is a launch file designed to work with the [mujoco_interface](https://github.com/Woolfrey/interface_mujoco_ros2) package.

You may modify/write your own to work with any robot.

#### Testing the Action Server

1. Launch the MuJoCo simulation. There are two options, `velocity_mode.py`, or `torque_mode.py`:
```
ros2 launch mujoco_interface velocity_mode.py
```
2. Launch the action server. It must match the control mode:
```
ros2 launch serial_link_action_server mujoco_velocity_control.py
```
or `mujoco_torque_control.py` if running torque mode.

3. Run the client:

```
ros2 run serial_link_action_server demo_client <numberOfJoints>
```
Replace <numberOfJoints> with the number of joints of the robot.


> [!TIP]
> The client is independent of the control mode! :+1:

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.
