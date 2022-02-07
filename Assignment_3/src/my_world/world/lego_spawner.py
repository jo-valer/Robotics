#!/usr/bin/python3

import rospy, tf, random, math, roslib, os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point

PKG = 'my_world'
roslib.load_manifest(PKG)
MW_PATH = roslib.packages.get_pkg_dir(PKG)
WORLD_PATH = MW_PATH+'/world'

#Number of target area in the map
NUM_TARGET_AREA=4

#Max number of lego in a target area
MAX_LEGO=3

#Min number of lego in a target area
MIN_LEGO=1

#Define target area coordinates
area1_x=2.0
area1_y=2.0

area2_x=3.0
area2_y=4.0

area3_x=0.0
area3_y=-2.0

area4_x=-2.0
area4_y=1.0

area_x = [area1_x, area2_x, area3_x, area4_x] 
area_y = [area1_y, area2_y, area3_y, area4_y] 
#------------------------------

#Support variables
lego_counter=0

rospy.init_node('insert_object',log_level=rospy.INFO)

print("Spawner node start")

output = os.path.join(WORLD_PATH, "dynamic_links.txt")
with open(output, 'w') as outputFile:

    for area in range(NUM_TARGET_AREA):

        #Decide how many lego to spawn in the area
        #min: MIN_LEGO ; max: MAX_LEGO
        #num_lego = 1
        num_lego = random.randint(MIN_LEGO, MAX_LEGO)
        
        #Write how many lego are there in the area
        outputFile.write(str(num_lego)+'\n')

        #Set up spawn configuration according to the number of lego to spawn
        if num_lego == 1:

            #Decide a random brick
            brick = random.randint(0,10)

            #Read the model from the corresponding file
            modelFile = open(WORLD_PATH + '/models/' + str(brick) + '/model.sdf', 'r')
            legoModel = modelFile.read()

            #===DECIDE POSITION OF THE LEGO===
            lego_pose = Pose()
            
            #Decide coordinate (x,y)
            rho = random.uniform(0, 0.1)
            theta = random.uniform(-math.pi, math.pi)
            lego_pose.position.x = area_x[area] + rho*math.cos(theta)
            lego_pose.position.y = area_y[area] + rho*math.sin(theta)
            lego_pose.position.z = 0.2
            
            #Decide rotation of the lego
            roll = 0
            #Randomly choose natural rotation or not
            brick_rotation = random.randint(0,1)
            
            if brick == 0:
                brick_rotation=0
                
            if brick == 1:
                brick_rotation=0
            
            if brick == 7:
                brick_rotation=0

            if brick_rotation==0:
                pitch=0
            else:
                pitch=math.pi/2.0

            yaw = random.uniform(-math.pi, math.pi)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            lego_pose.orientation.x = quaternion[0]
            lego_pose.orientation.y = quaternion[1]
            lego_pose.orientation.z = quaternion[2]
            lego_pose.orientation.w = quaternion[3]
            #=============================================

            #Set lego name
            lego_name = "brick" + str(lego_counter)

            #Spawn the lego
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_prox(lego_name, legoModel, "", lego_pose, "world")

            # Print to file: lego name in Gazebo, lego_x, lego_y, lego class
            outputFile.write(str(lego_name) + ' ' + str(lego_pose.position.x) + ' ' + str(lego_pose.position.y) + ' ' + str(brick) + '\n')

            lego_counter=lego_counter+1

        elif num_lego == 2:
            
            for l in range(num_lego):
                #Decide random brick
                brick = random.randint(0,10)

                #Read the model from the corresponding file
                modelFile = open(WORLD_PATH + '/models/' + str(brick) + '/model.sdf', 'r')
                legoModel = modelFile.read()

                #===DECIDE POSITION OF THE LEGO===
                lego_pose = Pose()
                
                #Decide coordinate (x,y)
                if l==0:
                    lego_pose.position.x = area_x[area] + 0.12
                else:
                    lego_pose.position.x = area_x[area] - 0.12

                lego_pose.position.y = area_y[area]
                lego_pose.position.z = 0.2
                
                #Decide rotation of the lego
                roll = 0

                #Randomly choose natural rotation or not
                brick_rotation = random.randint(0,1)
                
                if brick == 0:
                    brick_rotation=0
                
                if brick == 1:
                    brick_rotation=0
            
                if brick == 7:
                    brick_rotation=0

                if brick_rotation==0:
                    pitch=0
                else:
                    pitch=math.pi/2.0
                

                yaw = random.uniform(-math.pi, math.pi)
                quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
                lego_pose.orientation.x = quaternion[0]
                lego_pose.orientation.y = quaternion[1]
                lego_pose.orientation.z = quaternion[2]
                lego_pose.orientation.w = quaternion[3]
                #=============================================

                #Set lego name
                lego_name = "brick" + str(lego_counter)

                #Spawn the lego
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(lego_name, legoModel, "", lego_pose, "world")

                # Print to file: lego name in Gazebo, lego_x, lego_y, lego class
                outputFile.write(str(lego_name) + ' ' + str(lego_pose.position.x) + ' ' + str(lego_pose.position.y) + ' ' + str(brick) + '\n')

                lego_counter=lego_counter+1

        elif num_lego == 3:
            for l in range(num_lego):
                #Decide random brick
                brick = random.randint(0,10)

                #Read the model from the corresponding file
                modelFile = open(WORLD_PATH + '/models/' + str(brick) + '/model.sdf', 'r')
                legoModel = modelFile.read()

                #===DECIDE POSITION OF THE LEGO===
                lego_pose = Pose()
                
                #Decide coordinate (x,y)
                if l==0:
                    lego_pose.position.x = area_x[area] - 0.054
                    lego_pose.position.y = area_y[area] - 0.114
                elif l==1:
                    lego_pose.position.x = area_x[area] - 0.084
                    lego_pose.position.y = area_y[area] + 0.086
                else:
                    lego_pose.position.x = area_x[area] + 0.118
                    lego_pose.position.y = area_y[area] + 0.038

                lego_pose.position.z = 0.2
                
                #Decide rotation of the lego
                roll = 0

                #Randomly choose natural rotation or not
                brick_rotation = random.randint(0,1)

                if brick == 0:
                    brick_rotation=0
                
                if brick == 1:
                    brick_rotation=0
            
                if brick == 7:
                    brick_rotation=0

                if brick_rotation==0:
                    pitch=0
                else:
                    pitch=math.pi/2.0

                yaw = random.uniform(-math.pi, math.pi)
                quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
                lego_pose.orientation.x = quaternion[0]
                lego_pose.orientation.y = quaternion[1]
                lego_pose.orientation.z = quaternion[2]
                lego_pose.orientation.w = quaternion[3]
                #=============================================

                #Set lego name
                lego_name = "brick" + str(lego_counter)

                #Spawn the lego
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(lego_name, legoModel, "", lego_pose, "world")

                # Print to file: lego name in Gazebo, lego_x, lego_y, lego class
                outputFile.write(str(lego_name) + ' ' + str(lego_pose.position.x) + ' ' + str(lego_pose.position.y) + ' ' + str(brick) + '\n')

                lego_counter=lego_counter+1
    outputFile.close()
    print("Spawner end")

