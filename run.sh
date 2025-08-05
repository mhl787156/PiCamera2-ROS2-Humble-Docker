#!/bin/bash

source /opt/ros/humble/setup.bash
# source /camera_ws/install/setup.bash

ros2 run camera_ros camera_node # -–ros-args -p camera:=0 -p role:=viewfinder
