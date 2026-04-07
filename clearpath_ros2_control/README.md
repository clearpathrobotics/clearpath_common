# clearpath_ros2_control

Generic ROS 2 hardware components and controllers.

## clearpath_bms_broadcaster

ros2_control broadcaster for publishing `sensor_msgs/msg/BatteryState`.

This implementation is based on the upstream PR found [here](https://github.com/ros-controls/ros2_controllers/pull/1888).

It adds the following:

* cell_voltage state interfaces
* cell_temperature state interfaces

Intent is to stage changes here and contribute upstream via our fork [here](https://github.com/clearpathrobotics/ros2_controllers).
