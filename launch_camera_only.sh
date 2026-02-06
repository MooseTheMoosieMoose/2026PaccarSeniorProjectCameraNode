#!/bin/bash

#Source up the ros distro and the camera node
source /opt/ros/noetic/setup.bash
source /opt/camera_ws/devel/setup.bash

#Launch the camera node only
roslaunch realsense2_camera rs_camera.launch \
    enable_depth:=true \
    enable_color:=true \