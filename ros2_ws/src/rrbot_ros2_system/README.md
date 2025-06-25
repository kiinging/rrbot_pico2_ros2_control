i git clone from https://github.com/grzesiek2201/servobot_pico_driver

# rrbot_ros2_system 

## Description
Servobot is a 3dof robot based on hobby servos. This repo serves as a Hardware Interface driver for the robot, which in turn can be used to interact with the robot through other packages, such as moveit2.

The driver uses a Serial connection (e.g. USB) to talk to Raspberry Pi Pico (e.g. using this freeRTOS [example](https://github.com/grzesiek2201/pico-rtos-servo-control)), or any other microcontroller serving as an interface between the robot and the ROS2.

Built and tested on ROS2 Humble.

This package is based on the official RRBot example from the [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) repo.
