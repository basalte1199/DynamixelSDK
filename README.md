[![kinetic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/kinetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/kinetic-devel)
[![melodic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/melodic-devel)
[![noetic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/noetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/noetic-devel)
[![dashing-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/dashing-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/dashing-devel)
[![foxy-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/foxy-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel)
[![galactic-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/galactic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/galactic-devel)
[![humble-devel Status](https://github.com/ROBOTIS-GIT/DynamixelSDK/workflows/humble-devel/badge.svg)](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel)

<img src="http://emanual.robotis.com/assets/images/sw/sdk/dynamixel_sdk/overview/dynamixel_sdk_concept_logo.jpg">

## Dynamixel SDK
The ROBOTIS Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The API is designed for Dynamixel actuators and Dynamixel-based platforms. For more information on Dynamixel SDK, please refer to the e-manual below.
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

## Supported Programming Languages
DynamixelSDK supports various programming languages.
- **C**: *Dynamic library and source code of this library and examples
- **C#** / **Java** / **MATLAB** / **LabVIEW**: Support based on dynamic library using C language
- **C++**: *Dynamic library and source code of this library and examples
- **Python**: Python module and examples
(* Dynamic library (*.dll, *.so, and *.dylib files) / .dll: dynamic-link library on Windows / .so: shared object on Linux / .dylib: dynamic library on MacOS)

For more information on ROS Packages for Dynamixel SDK, please refer to the ROS wiki pages below.
- http://wiki.ros.org/dynamixel_sdk
- http://wiki.ros.org/dynamixel_workbench
- http://wiki.ros.org/dynamixel_workbench_msgs

---

## Additional Notes for `dynamixel_sdk_examples` (PyLoT)

This section documents how to use the following example nodes in this repository:

- `read_write_position_node`
- `read_write_position_arm_node`

### 1. Installation

Build the required packages from your ROS 2 workspace root:

```bash
colcon build --packages-select dynamixel_sdk_custom_interfaces dynamixel_sdk_examples
source install/setup.bash
```

If your environment is not sourced yet, source your ROS 2 distro first (example):

```bash
source /opt/ros/humble/setup.bash
```

### 2. Usage

#### 2.1 `read_write_position_node`

Start node:

```bash
ros2 run dynamixel_sdk_examples read_write_position_node
```

Send target position (example):

```bash
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 1, position: 1000}"
```

Read current position (service):

```bash
ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 1}"
```

#### 2.2 `read_write_position_arm_node`

Start node:

```bash
ros2 run dynamixel_sdk_examples read_write_position_arm_node
```

Single motor command (example):

```bash
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 11, position: 1500}"
```

Four-motor command (example):

```bash
ros2 topic pub -1 /set_position_four_motor dynamixel_sdk_custom_interfaces/msg/SetPositionFourMotor "{id_1: 11, position_1: 1500, id_2: 12, position_2: 1500, id_3: 13, position_3: 1500, id_4: 14, position_4: 1500}"
```

### 3. Original custom topics used by these nodes

#### `read_write_position_node`

- Subscribe: `/set_position` (`dynamixel_sdk_custom_interfaces/msg/SetPosition`)

#### `read_write_position_arm_node`

- Subscribe: `/set_position` (`dynamixel_sdk_custom_interfaces/msg/SetPosition`)
- Subscribe: `/set_position_four_motor` (`dynamixel_sdk_custom_interfaces/msg/SetPositionFourMotor`)
- Publish: `/get_present_position_five_motor` (`dynamixel_sdk_custom_interfaces/msg/SetPositionFiveMotor`)
- Publish: `/get_present_current_five_motor` (`dynamixel_sdk_custom_interfaces/msg/SetPositionFiveMotor`)
