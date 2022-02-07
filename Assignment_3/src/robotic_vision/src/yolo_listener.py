#!/usr/bin/python3

import os
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Roslib to have relative path
import roslib

PKG = 'robotic_vision'
roslib.load_manifest(PKG)

# Questo script deve prendere un'immagine dal topic /camera_low/color/image_raw e salvarla
# in una cartella condivisa con gli eseguibili di c++, che dovranno darla in pasto a python
RV_PATH = roslib.packages.get_pkg_dir(PKG)
MAIN_PATH = RV_PATH+'/src'

bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        #cv2.imwrite(MAIN_PATH+'detect_this.jpeg', cv2_img)
        filename = os.path.join(MAIN_PATH, "detect_this.jpeg")
        cv2.imwrite(filename, cv2_img)

rospy.init_node('yolo_listener')
image_topic = "/camera_low/color/image_raw"
#image_topic = "/camera/color/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)
rospy.spin()