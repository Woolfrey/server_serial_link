<a id="top"></a>
# Serial Link Action Server

This ROS2 package provides a set of action servers for controlling serial link robot arms. It builds upon [RobotLibrary](https://github.com/Woolfrey/software_robot_library) C++ library which contains the fundamental algorithms for kinematics, dynamics, and control.

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

These are the current basic components that show how to coordinate the different modules to enable real-time control of a robot arm. The design is modular, and you can create your own action servers & action clients as needed. More actions & features will be added in future.

- `include/ModelUpdater.h`: This uses a pointer to a `RobotLibrary::KinematicTree` object which defines a robot model. It subscribes to a `sensor_msgs/msg/JointState.msg` topic and uses it to compute the forward kinematics and inverse dynamics based on the current state. The subscription topic name can be set as a constructor argument.
- `src/velocity_control_server.cpp`: This creates the `RobotLibrary::KinematicTree` model, a `RobotLibrary::SerialKinematicControl` controller, and links together the `ModelUpdater` node, as well as creating & advertising the `TrackJointTrajectory` and `TrackCartesianTrajectory` action servers. It publishes joint commands using a custom `JointCommand.msg` message. You can write your own ROS2 subscriber to subscribe and send these to your robot.
- `src/demo_client.cpp`: This communicates with the `velocity_control_server` to demonstrate how to perform joint & Cartesian control actions.
- `src/mujoco_relay.cpp`: This node works in conjunction with this [MuJoCo interface](https://github.com/Woolfrey/interface_mujoco_ros2) 3D simulation. It subscribes to the `JointCommand.msg` topic from the action servers, and passes on a `std_msgs::msg::Float64MultiArray.msg` which is then used to control the simulated robot.
- `launch/mujoco_velocity_mode.py`: This contains various launch parameters, such as the URDF file for the robot model, control gains in the `/config` directory, subscription & publisher names, and launches the `mujoco_relay` node in conjunction with the MuJoCo simulation.
  
## Prerequisites

- [ROS2](https://docs.ros.org/en/foxy/index.html), obviousyl!
- [RobotLibrary](https://github.com/Woolfrey/software_robot_library): This contains the fundamental algorithms for forward kinematics, inverse dynamics, and feedback control.
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): This is actually utilized by RobotLibrary, so if that is working then this is satisfied.
- [MuJoCo Interface](https://github.com/Woolfrey/interface_mujoco_ros2): An optional package. It is used to demonstrate this package with a 3D simulation.

## Installation

1. **Clone the repository:**

    ```
    git clone https://github.com/Woolfrey/server_serial_link.git
    ```

2. **Build the package:**

    Navigate to your ROS2 workspace and build the package:

    ```
    cd your-ros2-workspace
    colcon build
    ```

3: **Source the workspace:**

    ```
    source install/setup.bash
    ```

### Usage
#### Launching the Nodes

There is a launch file designed to work with the [mujoco_interface](https://github.com/Woolfrey/interface_mujoco_ros2) package.

You may modify/write your own to work with any robot.

#### Testing the Action Server

1. Launch the MuJoCo simulation. There are two options, `velocity_mode.py`, or `torque_mode.py`, but currently the action server is only working in velocity mode:
```
ros2 launch mujoco_interface velocity_mode.py
```
2. Launch the action server. It must match the control mode:
```
ros2 launch serial_link_action_server mujoco_velocity_mode.py
```

3. Run the client:

```
ros2 run serial_link_action_server demo_client <numberOfJoints>
```
Replace <numberOfJoints> with the number of joints of the robot. Type `options` to see a list of commands.


> [!TIP]
> The client is independent of the control mode! :+1:

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.
