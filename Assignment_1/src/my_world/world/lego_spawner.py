#!/usr/bin/python3

import rospy, tf, random, math, roslib, os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point

PKG = 'my_world'
roslib.load_manifest(PKG)
MW_PATH = roslib.packages.get_pkg_dir(PKG)
WORLD_PATH = MW_PATH+'/world'

NUM = 4

rospy.init_node('insert_object',log_level=rospy.INFO)

i = -3
j = 0

output = os.path.join(WORLD_PATH, "dynamic_links.txt")
with open(output, 'w') as ff:
    ff.write(str(NUM) + '\n')
    while(i<NUM):
        brick = random.randint(0,10)
        f = open(WORLD_PATH + '/models/' + str(brick) + '/model.sdf', 'r')
        sdff = f.read()
        lego_pose = Pose()
        area_x = i
        area_y = 2.9
        rho = random.uniform(0, 0.1)
        theta = random.uniform(-math.pi, math.pi)
        lego_pose.position.x = area_x + rho*math.cos(theta)
        lego_pose.position.y = area_y + rho*math.sin(theta)
        lego_pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,random.uniform(-math.pi, math.pi))
        lego_pose.orientation.x = quaternion[0]
        lego_pose.orientation.y = quaternion[1]
        lego_pose.orientation.z = quaternion[2]
        lego_pose.orientation.w = quaternion[3]
        #lego_name = "brick" + str(brick) + "_" + str(j)
        lego_name = "brick" + str(j)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(lego_name, sdff, "", lego_pose, "world")

        # Stampo su file: NOME DEL BRICK IN GAZEBO, X, Y, CLASSE (separati da spazio)
        ff.write(str(lego_name) + ' ' + str(lego_pose.position.x) + ' ' + str(lego_pose.position.y) + ' ' + str(brick) + '\n')

        i = i+2
        j = j+1

    ff.close()

