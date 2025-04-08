# :cartwheeling: Serial Link Action Server

This package contains [ROS2 action servers](https://design.ros2.org/articles/actions.html) defined in the [serial link interface package](https://github.com/Woolfrey/interface_serial_link), using [Robot Library](https://github.com/Woolfrey/software_robot_library) for the underlying control algorithms.

#### :sparkles: Features:
- Real-time control of serial link robot arms,
- Joint space & Cartesian control,
- Seemless integration with the [serial link action client](https://github.com/Woolfrey/client_serial_link) package, and
- Classes for implementing your own custom control algorithms.

#### :compass: Navigation
- [Requirements](#clipboard-requirements)
- [Installation](#floppy_disk-installation)
- [Usage](#wrench-usage)
- [Classes](#toolbox-classes)
- [Release Notes](#package-release-notes---v100-april-2025)
- [Contributing](#handshake-contributing)
- [License](#scroll-license)

## :clipboard: Requirements

- [Ubuntu 22.04](https://ubuntu.com/blog/tag/22-04-lts), or later,
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html), or later,
- [RobotLibrary](https://github.com/Woolfrey/software_robot_library), and
- The [serial link interfaces](https://github.com/Woolfrey/interface_serial_link) package.

> [!NOTE]
> This package was built and tested using Ubuntu 22.04, and ROS2 Humble.

## :floppy_disk: Installation

Your directory structure should end up looking something like this:
```
ros2_workspace/
├── build/
├── install/
├── log/
└── src/
    ├── interface_serial_link/
    └── server_serial_link/
        ├── doc/
        ├── include/
        ├── src/
        ├── CMakeLists.txt
        ├── LICENSE
        ├── package.xml
        └── README.md
```

1. In the `src/` directory of your ROS2 workspace, clone the interfaces repository:

```
git clone https://github.com/Woolfrey/interface_serial_link.git
```

2. Clone the action server repository:

```
git clone http://github.com/Woolfrey/server_serial_link.git
```

3. Navigate back to the root of your ROS2 workspace and build:

```
colcon build
```

4. Source the local directory (if you haven't yet altered your .bashrc file):

```
source ./install/setup.bash
```

5. Check successful installation:

```
ros2 pkg list
```
   
If you scroll down the list, you should see both `serial_link_action_server`, and `serial_link_interfaces`.

[:top: Back to Top.](#cartwheeling-serial-link-action-server)

## :wrench: Usage

> [!TIP]
> Check out my [Kuka iiwa14 velocity control package](https://github.com/Woolfrey/control_kuka_velocity) on how to get the action server(s) up and running.

> [!NOTE]
> The actions have been fully implemented, and all you have to do is run / launch the action server nodes provided in this package. But you can also inherit the `ActionServerBase` class to implement your own methods.

1. Define the `handle_goal`, `handle_accepted`, `handle_cancel`, and `execute` methods:

(TO DO).

2. Create an executable, attach a node, and spin:

(TO DO).

## :toolbox: Classes

:construction: Under construction.

### SerialLinkBase

### FollowTransform

### FollowTwist

### ModelUpdater

### TrackCartesianTrajectory

### TrackJointTrajectory

[:top: Back to Top.](#cartwheeling-serial-link-action-server)

## :package: Release Notes - v1.0.0 (April 2025)

To do.
 
[:top: Back to Top.](#cartwheeling-serial-link-action-server)

## :handshake: Contributing

Contributions are always welcome. Feel free to fork the repository, make changes, and issue a pull request.

You can also raise an issue asking for new features.

[:top: Back to Top.](#cartwheeling-serial-link-action-server)

## :scroll: License

This software package is licensed under the [GNU General Public License v3.0 (GPL-3.0)](https://choosealicense.com/licenses/gpl-3.0/). You are free to use, modify, and distribute this package, provided that any modified versions also comply with the GPL-3.0 license. All modified versions must make the source code available and be licensed under GPL-3.0. The license also ensures that the software remains free and prohibits the use of proprietary restrictions such as Digital Rights Management (DRM) and patent claims. For more details, please refer to the [full license text](LICENSE).

[:top: Back to Top.](#cartwheeling-serial-link-action-server)
