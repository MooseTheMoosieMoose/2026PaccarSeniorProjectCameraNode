#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import CameraInfo, Image

import numpy as np
import cv2
from ultralytics import YOLO

CAMERA_INFO_TOPIC = "/camera/color/camera_info"
CAMERA_IMAGE_TOPIC = "/camera/camera/color/image_raw"

last_image = None

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

def camera_image_callback(data: Image) -> None:
    global last_image
    last_image = data

def main():
    #Ref our last image
    global last_image

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

    #Create a subscriber to bind to the camera topic
    rospy.Subscriber(CAMERA_IMAGE_TOPIC, Image, camera_image_callback)

    #Load up YOLO
    model = YOLO("/workspace/yolov8s.pt")
    rospy.loginfo("YOLO Model loaded and ready!")

    #Loop and process
    while not rospy.is_shutdown():

        if (last_image != None):
            rospy.loginfo("Performing inference...")

            #Swap to a form that YOLO likes
            numpy_image = np.frombuffer(last_image.data, dtype=np.uint8).reshape(
                last_image.height, last_image.width - 1
            )

            #Swap to RGB from camera's BGR
            rgb_image = cv2.cvtColor(numpy_image, cv2.COLOR_BGR2RGB)

            results = model(rgb_image)
            last_image = None

            for result in results:
                class_names = result.names
                for box in result.boxes:
                    class_id_tensor = box.cls
                    class_id = int(class_id_tensor.item())
                    class_name = class_names[class_id]
                    rospy.loginfo(f"Detected: {class_name}, Confidence: {box.conf.item():.2f}")


    rospy.spin()

if __name__ == "__main__":
    main()