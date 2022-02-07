#!/usr/bin/python3

import rospy, roslib, os

PKG = 'robotic_vision'
roslib.load_manifest(PKG)
RV_PATH = roslib.packages.get_pkg_dir(PKG)
SRC_PATH = RV_PATH+'/src/'

rospy.init_node('yolo_starter',log_level=rospy.INFO)

os.system("python3 " + SRC_PATH + "yolov5/my_detect.py --weights " + SRC_PATH + "yolov5/best.pt --img 1024 --conf 0.1 --source " + SRC_PATH + "detect_this.jpeg --save-txt --save-conf")
