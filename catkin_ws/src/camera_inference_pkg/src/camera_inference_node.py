#!/usr/bin/env python3

import rospy
from std_msgs.msg import CameraInfo

CAMERA_INFO_TOPIC = "/camera/color/camera_info"

def fetch_camera_info() -> CameraInfo:
    """
    Blocking wait to receive camera intrinsic info
    
    :return: a CameraInfo message with the cameras intrinsics
    :rtype: CameraInfo
    """
    try:
        cam_info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=None)
        return cam_info
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Shutdown occured before camera node could fetch camera metadata!")


def main():
    #Declare our node
    rospy.init_node("camera_inference_node", anonymous=False)

    #Wait on the camera node to be up and running, fetch its camera data
    cam_meta = fetch_camera_info()
    focal_x = cam_meta.P[0]
    focal_y = cam_meta.P[5]
    principle_x = cam_meta.P[2]
    principle_y = cam_meta.P[6]
    loginfo = f"Camera has focal lengths: {focal_x}, {focal_y}, with principle points: {principle_x}, {principle_y}"
    rospy.loginfo(loginfo)

    rospy.spin()

if __name__ == "__main__":
    main()