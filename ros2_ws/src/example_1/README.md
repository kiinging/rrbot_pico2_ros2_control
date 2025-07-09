# example_1

   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html).



# serial


# Hardware interpace
example_1/hardware/rrbot.cpp -simulation
example_1/hardware/rrbot_serial.cpp - real hardware


# useful commands
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1.5
- 0.9
- -1.5"

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 1.0]"


ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.0
- 1.0"