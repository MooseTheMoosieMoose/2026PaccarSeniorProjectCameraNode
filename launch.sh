#!/bin/bash

#Check that the node is built
bash ./build.sh

#Source ROS itself
source /opt/ros/noetic/setup.bash

#Source the camera node
source /opt/camera_ws/devel/setup.bash

#Source up our node
source catkin_ws/devel/setup.bash

#Launch it (auto starts the camera node as well)
roslaunch camera_inference_pkg start_node.launch
