#!/bin/bash

#Source up the ros distro and the camera node
source /opt/ros/noetic/setup.bash
source /opt/camera_ws/devel/setup.bash

cd ./catkin_ws
catkin_make