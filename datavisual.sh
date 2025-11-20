#!/bin/bash

# Launch Gazebo
gnome-terminal --tab -- bash -c "source install/setup.bash;
                                ros2 run data_visual data_visual_node ;
                                 echo Press any key to close;
                                 read -n 1"

# Launch Navigation
# gnome-terminal --tab -- bash -c "source install/setup.bash;
#                                  ros2 launch robot_properties nav2_def.launch.py;
#                                  echo Press any key to close;
#                                  read -n 1"

# gnome-terminal --tab -- bash -c "source install/setup.bash;
#                                  ros2 launch robot_properties localization_launch.py;
#                                  echo Press any key to close;
#                                  read -n 1"

gnome-terminal --tab -- bash -c "source install/setup.bash;
                                ros2 run gamepad_pkg gamepad_testing;
                                echo Press any key to close;
                                read -n 1"

gnome-terminal --tab -- bash -c "source install/setup.bash;
                                ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_18025870-if00;
                                echo Press any key to close;
                                read -n 1"
# Wait a bit to make sure everything loads
sleep 3

# Launch RViz with custom config

# gnome-terminal --tab -- bash -c "source install/setup.bash ; ros2 launch robot_properties localization_launch.py; echo Press any key to close; read -n 1"
