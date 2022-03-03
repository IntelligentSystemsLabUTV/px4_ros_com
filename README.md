# IntelligentSystemsLabUTV/px4_ros_com

**This is a fork of PX4/px4_ros_com, for internal use only.**

**The main difference from the official version is that this is now a full ROS 2 application, completely integrated in the ROS 2 modules ecosystem.**

This package materializes the ROS 2 side of the PX4-FastDDS bridge, establishing a bridge between the PX4 autopilot stack and ROS2. It has a straight dependency on the [`IntelligentSystemsLabUTV/px4_msgs`](https://github.com/IntelligentSystemsLabUTV/px4_msgs) package, as it depends on its IDL files to generate the microRTPS Agent, and on the ROS 2 interfaces and typesupport to build and run the example nodes. It also depends on the [`IntelligentSystemsLabUTV/ros2_signal_handler`](https://github.com/IntelligentSystemsLabUTV/ros2_signal_handler) package to handle application shutdown.

## Install, build and usage

Check the [RTPS/ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Development Guide for details on how to install the required dependencies, build the package (composed by the two branches) and use it. Once the dependencies listed there are met, one just needs to place the three packages in a ROS 2 workspace and build them with `colcon`.
