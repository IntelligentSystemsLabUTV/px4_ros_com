# IntelligentSystemsLabUTV/px4_ros_com

**This is a fork of PX4/px4_ros_com, for internal use only.**

This package materializes the ROS2 side of PX4-FastRTPS/DDS bridge, establishing a bridge between the PX4 autopilot stack through a micro-RTPS bridge, Fast-RTPS(DDS) and ROS2. It has a straight dependency on the [`px4_msgs`](https://github.com/IntelligentSystemsLabUTV/px4_msgs) package, as it depends on the IDL files, to generate the micro-RTPS bridge agent, and on the ROS interfaces and typesupport, to allow building and running the example nodes. It also depends on the `IntelligentSystemsLabUTV/ros2_signal_handler` package to handle application shutdown.

**The main difference from the official version is that this is now a full ROS 2 application, completely integrated in the ROS 2 modules ecosystem.**

## Install, build and usage

Check the [RTPS/ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Devguide for details on how to install the required dependencies, build the package (composed by the two branches) and use it.
