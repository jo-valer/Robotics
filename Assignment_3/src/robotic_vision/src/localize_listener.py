#!/usr/bin/python3

import os, math
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# custom ROS message
from robotic_vision.msg import Localize
# OpenCV2 for saving an image
import cv2
# Roslib to have relative path
import roslib

# Coefficients to map image coordinates to gazebo coordinates
STRETCH = 0.0008837891
X_SHIFT = -0.4525
# Y_SHIFT depends on where we put the upper camera (+0.9 in respect of robot centre, +0.9-0.3 in respect of camera)
Y_SHIFT = 0.95-0.3-0.4525

PKG = 'mir_controller'
roslib.load_manifest(PKG)

# Questo script deve prendere un'immagine dal topic /camera_up/color/image_raw e salvarla
# in localization.jpeg, affinché localize.py riconosca la posizione degli oggetti presenti
MIR_PATH = roslib.packages.get_pkg_dir(PKG)
MAIN_PATH = MIR_PATH+'/src/'

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
        #cv2.imwrite(MAIN_PATH+'localization.jpeg', cv2_img)
        # converting image into grayscale image
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

        # setting threshold of gray image
        ret,threshold = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        # using a findContours() function
        contours,ret = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # localize publisher
        pub = rospy.Publisher('localize', Localize, queue_size=10)
        msg = Localize()

        #Initialization
        msg.lego1_imgx = 0
        msg.lego1_imgy = 0
        msg.lego1_x = 0.0
        msg.lego1_y = 0.0
        msg.lego1_q = 0.0
        msg.lego1_w = 0.0
        msg.lego1_h = 0.0
        msg.lego1_p = 0
        msg.lego2_imgx = 0
        msg.lego2_imgy = 0
        msg.lego2_x = 0.0
        msg.lego2_y = 0.0
        msg.lego2_q = 0.0
        msg.lego2_w = 0.0
        msg.lego2_h = 0.0
        msg.lego2_p = 0
        msg.lego3_imgx = 0
        msg.lego3_imgy = 0
        msg.lego3_x = 0.0
        msg.lego3_y = 0.0
        msg.lego3_q = 0.0
        msg.lego3_w = 0.0
        msg.lego3_h = 0.0
        msg.lego3_p = 0

        #Initialization of counter for lego
        numLego=0

        # list for storing names of shapes
        for contour in contours:
            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(contour, 7, True)
            p = -1 #the pose specifies how the lego is (0=standing, 1=lying on its side)

            #LEGO STANDING
            if len(approx)==4:
                numLego=numLego+1
                p=0
                (xu, yu), (w, h), ang = cv2.minAreaRect(contour)
                
                x1 = approx[0][0][0]
                y1 = approx[0][0][1]
                x2 = approx[1][0][0]
                y2 = approx[1][0][1]
                x3 = approx[2][0][0]
                y3 = approx[2][0][1]

                # finding center point of shape
                M = cv2.moments(approx)
                if M['m00'] != 0.0:
                    x = M['m10']/M['m00']
                    y = M['m01']/M['m00']

                # finding angular orientation
                q = math.atan2(y1-y2,x1-x2)
                if math.sqrt((x1-x2)**2 + (y1-y2)**2) < math.sqrt((x2-x3)**2 + (y2-y3)**2):
                    q = math.atan2(y2-y3,x2-x3)
                q = -(q + math.pi/2)

                print("x: " + str((x*STRETCH) + X_SHIFT ) + ", y: " + str( ((1024-y)*STRETCH) + Y_SHIFT ) + ", q: " + str(q) + " (ang: " + str(ang) + "), w:" + str(w) + ", h: " + str(h) + "\n")

                # We will instantiate the topic publisher to send this data to mir_controller
                if numLego==1: #this is the first brick we have localized
                    msg.lego1_imgx = int(x)
                    msg.lego1_imgy = int(y)
                    msg.lego1_x = (x*STRETCH) + X_SHIFT
                    msg.lego1_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego1_q = q
                    msg.lego1_w = w
                    msg.lego1_h = h
                    msg.lego1_p = p
                elif numLego==2: #this is the second brick we have localized
                    msg.lego2_imgx = int(x)
                    msg.lego2_imgy = int(y)
                    msg.lego2_x = (x*STRETCH) + X_SHIFT
                    msg.lego2_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego2_q = q
                    msg.lego2_w = w
                    msg.lego2_h = h
                    msg.lego2_p = p
                elif numLego==3: #this is the third brick we have localized
                    msg.lego3_imgx = int(x)
                    msg.lego3_imgy = int(y)
                    msg.lego3_x = (x*STRETCH) + X_SHIFT
                    msg.lego3_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego3_q = q
                    msg.lego3_w = w
                    msg.lego3_h = h
                    msg.lego3_p = p

                #with open(MAIN_PATH + 'localization.txt', 'w') as ff:
                    # Notice that y has to be inverted (i.e. 1024-y) because the y-axis in gazebo is in opposite direction if compared to the image y-axis
                    #ff.write(str(int(x)) + " " + str(int(y)) + " " + str( (x*STRETCH) + X_SHIFT ) + " " + str( ((1024-y)*STRETCH) + Y_SHIFT ) + " " + str(q) + " " + str(w) + " " + str(h) + "\n")
                    #ff.close()

            #LEGO LYING
            if len(approx)>4:
                numLego=numLego+1
                p=1
                (xu, yu), (w, h), ang = cv2.minAreaRect(contour)
                
                x1 = approx[0][0][0]
                y1 = approx[0][0][1]
                x2 = approx[1][0][0]
                y2 = approx[1][0][1]
                x3 = approx[2][0][0]
                y3 = approx[2][0][1]

                # finding center point of shape
                M = cv2.moments(approx)
                if M['m00'] != 0.0:
                    x = M['m10']/M['m00']
                    y = M['m01']/M['m00']

                # finding angular orientation
                i = 1 #counter
                #these first lines before the WHILE are needed to compute the values for the segment connecting first and last vertices
                x0 = approx[0][0][0]
                y0 = approx[0][0][1]
                x1 = approx[len(approx)-1][0][0]
                y1 = approx[len(approx)-1][0][1]
                max_dist = math.sqrt((x0-x1)**2 + (y0-y1)**2) #maximum distance between two consecutive vertices in contour (approx)
                q = math.atan2(y0-y1, x0-x1) #angular orientation
                while i<len(approx):
                    x0 = approx[i-1][0][0]
                    y0 = approx[i-1][0][1]
                    x1 = approx[i][0][0]
                    y1 = approx[i][0][1]
                    act_dist = math.sqrt((x0-x1)**2 + (y0-y1)**2) #distance between the actual vertex and the previous one
                    if(act_dist > max_dist):
                        max_dist = act_dist
                        q = math.atan2(y0-y1, x0-x1)
                    i=i+1
                
                q = math.fmod(-(q + math.pi/2), math.pi)

                print("x: " + str((x*STRETCH) + X_SHIFT ) + ", y: " + str( ((1024-y)*STRETCH) + Y_SHIFT ) + ", q: " + str(q) + " (ang: " + str(ang) + "), w:" + str(w) + ", h: " + str(h) + "\n")
                
                # We will instantiate the topic publisher to send this data to mir_controller
                if numLego==1: #this is the first brick we have localized
                    msg.lego1_imgx = int(x)
                    msg.lego1_imgy = int(y)
                    msg.lego1_x = (x*STRETCH) + X_SHIFT
                    msg.lego1_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego1_q = q
                    msg.lego1_w = w
                    msg.lego1_h = h
                    msg.lego1_p = p
                elif numLego==2: #this is the second brick we have localized
                    msg.lego2_imgx = int(x)
                    msg.lego2_imgy = int(y)
                    msg.lego2_x = (x*STRETCH) + X_SHIFT
                    msg.lego2_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego2_q = q
                    msg.lego2_w = w
                    msg.lego2_h = h
                    msg.lego2_p = p
                elif numLego==3: #this is the third brick we have localized
                    msg.lego3_imgx = int(x)
                    msg.lego3_imgy = int(y)
                    msg.lego3_x = (x*STRETCH) + X_SHIFT
                    msg.lego3_y = ((1024-y)*STRETCH) + Y_SHIFT
                    msg.lego3_q = q
                    msg.lego3_w = w
                    msg.lego3_h = h
                    msg.lego3_p = p


        #Set number of lego bricks localized
        msg.numLego = numLego
        if not rospy.is_shutdown(): # Cause we will re-execute all this code thanks to the node in the startcomplete.launch file there is no meaning to do a while loop to publish the message on the topic
            rospy.loginfo(msg)
            pub.publish(msg)

rospy.init_node('localize_listener')
image_topic = "/camera_up/color/image_raw"
#image_topic = "/camera/color/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)
rospy.spin()