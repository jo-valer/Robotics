/**
 * ASSIGNMENT 3
 * @author Pietro Fronza, Giovanni Valer, Stefano Genetti
 */

//===C++ LIBRARIES===
#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>
#include <cmath>
//===================

//===EIGEN LIBRARY===
#include <eigen3/Eigen/Dense>
//===================

//===KINEMATIC LIBRARY===
#include <kinematic_library/Position.h>
#include <kinematic_library/RPYRotation.h>
#include <kinematic_library/EndEffector.h>
#include <kinematic_library/Ur5JointConfiguration.h>
#include <kinematic_library/ur5Kinematic.h>
#include <kinematic_library/MyUrd5.h>
#include <kinematic_library/Callback_jointconfig.h>
//=======================

//===ROS LIBRARY===
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include "control_msgs/JointControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"
//=================

//===WORLD COMMON LIBRARY===
#include "my_world/World.h"
#include "my_world/common.h"
#include "my_world/Lego.h"
#include "my_world/Coordinate.h"
#include "my_world/TargetArea.h"
//==========================

//===MOBILE CONTROL LIBRARY===
#include "mobile_control_library/MobileRobot.h"
#include "mobile_control_library/mir_control_lib.h"
#include "mobile_control_library/Callback_odometry.h"
//============================

//===DINAMIC LINK LIBRARY===
#include "gazebo_ros_link_attacher/gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/dynamic_link.h"
//==========================

//===ROBOTIC VISION LIBRARY===
#include "robotic_vision/vision_library.h"
#include "robotic_vision/Callback_detect.h"
#include "robotic_vision/Callback_localization.h"
#include "robotic_vision/Callback_pointcloud.h"
//============================

using namespace Eigen;
using namespace std;

const int loop_rate_val = 100;

ofstream logFile(LOG_FILE);    //log file
ofstream outFile(OUTPUT_FILE); //output file

int main(int argc, char **argv){

  //===ROS PARAMETER INITIALIZATION===
	ros::init(argc, argv, "mir_control");
  std::string urdf_path = ros::package::getPath("ur5-joint-position-control");
	if(urdf_path.empty()) {
		ROS_ERROR("ur5-joint-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
	ros::init(argc, argv, "tcp_control");

	ros::NodeHandle n;

  ros::Rate loop_rate(loop_rate_val);
  //==================================

  //Callback for PointCloud data
  Callback_pointcloud pointCloud_data(n);
  //...

  //Create callback to recieve actual position from odometry sensors
  Callback_odometry pose2d(n);
  pose2d.enable();
  //...

	//Create publishers to send position commands to all joints
	ros::Publisher mirPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  //...

  //Create subscribers for all joint states
	Callback_jointconfig joint_data(n);

	//Define publishers' advertise, to send position commands to all joints
  ros::Publisher joint_com_pub[6];
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);
  //...

  //Create callback to get localization data
  Callback_localization localization_data(n);

  //Create callback to get yolo data
  Callback_detect detection_data(n);

  //Define gripper publisher's advertise
  ros::Publisher gripperTopic = n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1000);
  //...

  //Define client for dinamic link services
  ros::ServiceClient client_attach = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  ros::ServiceClient client_detach = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
  ros::ServiceClient client_delete = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  //...

  cout<<"Assignment 3"<<endl;

  //Make sure we have received proper joint angles already
  joint_data.enable();
  for(int i=0; i<5; i++) {
      ros::spinOnce();
      loop_rate.sleep();
  }

  //Initialize world map
  World world;

  //Debug: print the map
  //cout<<"DEPUG[print_map()]"<<endl;
  //world.print_map();
  //...
  
  //Initialize legos vector
  vector<Lego> legos;
  init_dynamic_links(legos);

  //Setup robotic arm before starting
  EndEffector initialPose;  //end effector configuration
  if(ros::ok()){
    //Initial pose
    ur5Direct(joint_data.jnt_pos_start, initialPose);

    sleep(2);

    //Raise the arm straight
    point2point_motion_plan(initialPose, -0.1, 0.1, 0.9, 0.0, 0.0, 0.0, 5, loop_rate, joint_com_pub); sleep(3);
    joint_data.disable();
    ros::spinOnce();
    loop_rate.sleep();
  }
  //...

  sleep(2);

  //===WORKFLOW VARIABLES===

  //---Support variables---
  int minDistance;
  int tmp_nextArea;
  double distance_tgp1;
  double distance_tgp2;
  int futureAreaId;
  Coordinate futureAreaCenter;
  Coordinate mirCoordinates;
  //-----------------------


  bool goOn=true;       //set to false if some error occurs
  int currentArea=0;    //target area's id where the mir is
  Lego currentLego;     //which kind of lego is in the gripper
  bool has_brick=false; //TRUE iff we have a lego in the gripper

  //---DIJKSTRA DATA STRUCTURES---
  vector<int> distance(world.N, INT32_MAX);  //per Dijkstra inizializzo a infinito - nota: infinito = 60000
  vector<int> parent(world.N, 0);            //array dei padri per stampare percorso minimo
  vector<int> path;                          //sequence of areas, represents the path
  //------------------------------

  //---TRAVELLING---
  double actual_x;              //position x read from /base_pose_ground_truth topic
  double actual_y;              //position y read from /base_pose_ground_truth topic
  double actual_th;             //position th read from /base_pose_ground_truth topic

  double desired_actual_x=0.0;  //theoretical position x (where we think the robot is)
  double desired_actual_y=0.0;  //theoretical position y (where we think the robot is)
  double desired_actual_th=0.0; //theoretical position th (where we think the robot is)

  int nextArea;                 //area number where the robot has to go
  double xf;                    //target x position of the mir
  double yf;                    //target y position of the mir
  double thf;                   //target th position of the mir
  //----------------

  //---OBJECT CLASSIFICATION AND LOCALIZATION---
  vector<Lego> lego_output(MAX_AREA_LEGO);  //the vector is populated with the legos which are found in the current target area
  Lego lego;                                //support variable of the object classification and localization process
  //Lego class
  int decided_class=11;
  std::string classe_lego_str;
  //Lego coordinates with respect to gazebo frame
  double lego_x;
  double lego_y;
  //Localization variables from upper camera
  int pixel_x;
  int pixel_y;
  double center_x;
  double center_y;
  double orientation;
  double width;
  double height;
  int pose;
  int actual_y_dim;
  int actual_x_dim;
  int lego_dim_x;
  int lego_surface;

  //Dynamic link support variables
  int dyn_cls;             //class written in dynamic_links.txt, by default is 11=UNKNOWN
  string gaz_name;         //brick's name in gazebo
  double epsi;             //maximum error (hope so) in localization
  
  //Point cloud support variables
  PointCloud rotated_pointcloud;
  vector<vector<double>> matrixPointcloud;
  matrixPointcloud.resize(POINT_CLOUD_HEIGHT);
  for(int iii=0; iii<POINT_CLOUD_HEIGHT; iii++){
      matrixPointcloud[iii].resize(POINT_CLOUD_WIDTH);
  }
  double etha;
  int lego_height;
  int lego_caps;
  //--------------------------------------------

  //========================

  while(ros::ok() && goOn && (!world.complete() || has_brick==true) ){    //Go on until all the area have been explored and completed (note: at the end all the area has been explored and completed but we have still a lego in the gripper)
    //===COMPUTE MIN DISTANCE TREE WITH DIJKSTRA, CURRENT_AREA IS THE ROOT OF THE TREE===
    //Dijkstra
    for(int d=0; d<distance.size(); d++){
      distance[d] = INT32_MAX;
    }
    dijkstra(world.graph, currentArea, distance, parent);
    //---
    //===end min distance tree computation===============================================

    //===COMPUTE NEXT DESTINATION===========
    if(has_brick){  //IF WE HAVE A LEGO

      //Next destination is the basket for our lego's class
      
      //Assumption:
      // basket for class 0 is in mapArea[CARD_TARGET+1]
      // basket for class 1 is in mapArea[CARD_TARGET+2]
      // ...
      // basket for class NUM_CLASSES is in mapArea[CARD_TARGET+CARD_BASKET]
      nextArea=currentLego.getClasse()+CARD_TARGET+1;

    }else{          //IF WE DO NOT HAVE A LEGO
      
      //FIND NEAREST TARGET AREA (do not consider baskets)
      tmp_nextArea=-1;
      minDistance=INT32_MAX;
      for(int d=1; d<=CARD_TARGET; d++){
        if(distance[d]<minDistance && (world.targetArea[d].getNumLego()==-1 || world.targetArea[d].getNumLego()>0) ){ //consider only the target areas with at least one lego or with an unknown number of legos
          minDistance=distance[d];
          tmp_nextArea=d;
        }
      }

      if(tmp_nextArea==-1){
        cout<<"Error: there is no more lego in the map"<<endl;
        goOn=false;
      }

      nextArea=tmp_nextArea;

    }
    //===end next destination computation===

    //===GO TO NEXT DESTINATION===
    //Read position of the robot from the topic (two decimals for x and y)
    //actual_x=round(pose2d.x*100)/100.0;
    //actual_y=round(pose2d.y*100)/100.0;
    actual_x=pose2d.x;
    actual_y=pose2d.y;
    actual_th=pose2d.theta;

    if(goOn){ //go on iff there have been no errors previously

      //Initialize the path to the destination
      path.clear();
      printPath(currentArea, nextArea, parent, path);

      //Follow the path
      for(int i=1; i<path.size(); i++){
        
        //Next area to travel to
        nextArea=path[i];

        //Get the two tangent points
        pair<Coordinate, Coordinate> tg_points = tangent_point(world.mapArea[nextArea].getX(), world.mapArea[nextArea].getY(), desired_actual_x, desired_actual_y);

        //IFF WE ARE NOT IN THE PENULTIMATE AREA - Choose the tangent point which is nearer to the next area than the other
        if(i<path.size()-1){
          //Get next area
          futureAreaId = path[i+1];
          futureAreaCenter.setX(world.mapArea[futureAreaId].getX());
          futureAreaCenter.setY(world.mapArea[futureAreaId].getY());

          //Compute distance
          distance_tgp1 = two_points_distance(futureAreaCenter, tg_points.first);
          distance_tgp2 = two_points_distance(futureAreaCenter, tg_points.second);

          //Choose the shortest
          if(distance_tgp1<distance_tgp2){
            xf = tg_points.first.getX();
            yf = tg_points.first.getY();
          }else{
            xf = tg_points.second.getX();
            yf = tg_points.second.getY();
          }
        }else{  //If we are in the penultimate area, choose the tangent point which is nearer to the current position of the mir robot
          
          //Get current coordinates of the mir
          mirCoordinates.setX(pose2d.x);
          mirCoordinates.setY(pose2d.y);

          //Compute distance
          distance_tgp1 = two_points_distance(mirCoordinates, tg_points.first);
          distance_tgp2 = two_points_distance(mirCoordinates, tg_points.second);

          //Choose the shortest
          if(distance_tgp1<distance_tgp2){
            xf = tg_points.first.getX();
            yf = tg_points.first.getY();
          }else{
            xf = tg_points.second.getX();
            yf = tg_points.second.getY();
          }

        }
        //..........................................................................

        //IFF
        //WE ARE IN THE FIRST AREA OF THE PATH
        //CURRENT AREA != 0
        //THE NUMBER OF LEGO IN THE CURRENT AREA IS UNKNOW OF GREATER THAN 0
        // ==> CHECK IF THE TRAJECTORY IS DANGEROUS, ie if we follow the straight trajectory we could bounce into the lego of the current target area

        if( i==1 &&
            currentArea!=0 &&
            (currentArea>CARD_TARGET || world.targetArea[currentArea].getNumLego()==-1 || world.targetArea[currentArea].getNumLego()>0) &&
            dangerous_trajectory(desired_actual_x, desired_actual_y, world.mapArea[currentArea].getX(), world.mapArea[currentArea].getY(), radius_targetArea, xf, yf)
          )
        {
          //Choose a safe intermediate point
          Coordinate safePoint = nearest_safe_point(desired_actual_x, desired_actual_y, world.mapArea[currentArea].getX(), world.mapArea[currentArea].getY(), radius_targetArea);
          
          //Go to the safe intermediate point
          travel(desired_actual_x, desired_actual_y, desired_actual_th, safePoint.getX(), safePoint.getY(), loop_rate, pose2d, mirPublisher);

          //Update current mir configuration
          desired_actual_th=pose2d.theta;
          desired_actual_x=safePoint.getX();
          desired_actual_y=safePoint.getY();
          ros::spinOnce();
          loop_rate.sleep();
        }
        //......................................................................

        //TRAVEL TO DESTINATION
        travel(desired_actual_x, desired_actual_y, desired_actual_th, xf, yf, loop_rate, pose2d, mirPublisher);
        
        //Update theoretical position (generic coordinates) of the mir robot
        desired_actual_th=pose2d.theta;
        desired_actual_x=xf;
        desired_actual_y=yf;
        ros::spinOnce();
        loop_rate.sleep();

        currentArea=nextArea;

        //If we have reach an unexplored target area or the final destination we rotate to the center
        if(i==path.size()-1 || (currentArea!=0 && currentArea<=CARD_TARGET && world.visited[currentArea]==false)){ //WE ARE IN AN UNEXPLORED TARGET AREA OR WE HAVE REACHED THE DESTINATION

          //Rotate to point to the center of the area
          rotateToAreaCenter(desired_actual_x, desired_actual_y, world.mapArea[currentArea].getX(), world.mapArea[currentArea].getY(), loop_rate, pose2d, mirPublisher);

          //Update theoretical position (generic coordinates) of the mir robot
          desired_actual_th=pose2d.theta;
          desired_actual_x=xf;
          desired_actual_y=yf;
          ros::spinOnce();
          loop_rate.sleep();

          if(currentArea<=CARD_TARGET && world.visited[currentArea]==false){
            sleep(2);
            //===OBJECT CLASSIFICATION AND LOCALIZATION===
            localization_data.enable();
            ros::spinOnce();
            loop_rate.sleep();

            //cout<<"OBJECT CLASSIFICATION AND LOCALIZATION"<<endl;
            int num_lego_localized = localization_data.numLegoLocalized;

            for(int l=0; l<num_lego_localized; l++){
              decided_class=11;
              ros::spinOnce();
              loop_rate.sleep();

              //Read lego x,y,z dimensions from upper camera data
              //tie(pixel_x, pixel_y, center_x, center_y, orientation, width, height) = get_localize_data();
              pixel_x = localization_data.localized[l].img_x;
              pixel_y = localization_data.localized[l].img_y;
              center_x = localization_data.localized[l].x;
              center_y = localization_data.localized[l].y;
              orientation = localization_data.localized[l].q; 
              width = localization_data.localized[l].w; 
              height = localization_data.localized[l].h;
              pose = localization_data.localized[l].p;

              lego_x = pose2d.x + 0.3*cos(pose2d.theta) + sqrt(pow(center_x, 2)+pow(center_y, 2)) * cos(pose2d.theta - M_PI_2 + calculate_trajectory(0.0,0.0,center_x,center_y));
              lego_y = pose2d.y + 0.3*sin(pose2d.theta) + sqrt(pow(center_x, 2)+pow(center_y, 2)) * sin(pose2d.theta - M_PI_2 + calculate_trajectory(0.0,0.0,center_x,center_y));
      
              //===POINT CLOUD HANDLING===
              pointCloud_data.enable();
              ros::spinOnce();
              loop_rate.sleep();

              //Wait until point cloud data is ready
              while(!pointCloud_data.message_ready()){
                ros::spinOnce();
                loop_rate.sleep();
              }
              pointCloud_data.disable();
              pointCloud_data.reset_new_message();

              //Rotate the current point cloud frame and use it to understand Z property of the lego
              //Init rotation matrix
              etha = -(1.570796) - (M_PI_2); // etha is the angle (along x-axis) needed to rotate the camera frame
              // WARNING: etha has to be that number because in the .urdf there's that one, which is NOT exactly M_PI_2
              Matrix4d RotMatrix;
              RotMatrix << 1.0, 0.0,        0.0,        0.0,
                          0.0, cos(etha),  -sin(etha), 0.0,
                          0.0, sin(etha),  cos(etha),  2.99,
                          0.0,             0.0, 0.0,   1.0;

              pcl::transformPointCloud(pointCloud_data.myPointCloud, rotated_pointcloud, RotMatrix);
              matrix_pointcloud(matrixPointcloud, rotated_pointcloud);

              //LEGO LYING
              if(pose==1){
                //Get comparable lego properties [X, surface] computed by the localization process and pointcloud
                tie(lego_dim_x, lego_surface) = get_lego_property4lying_x_and_surface(pixel_x, pixel_y, matrixPointcloud);

                //===OBJECT CLASSIFICATION===
                decided_class=compute_class4lying(lego_dim_x, lego_surface);               
              }

              //LEGO STANDING
              if(pose==0){
                //Get comparable lego properties [X dim, Y dim] computed by the localization process
                actual_y_dim = get_lego_property_y_localized(width, height);
                actual_x_dim = get_lego_property_x_localized(width, height);

                //Get lego height and number of caps looking at point cloud data
                tie(lego_height, lego_caps) = get_lego_property_z_and_caps(pixel_x, pixel_y, matrixPointcloud);
                
                //Get the lego's number of ramps (it has a concrete meaning just for bricks 3 and 4)
                int lego_ramps = get_lego_property_ramp(pixel_x, pixel_y, matrixPointcloud);
                
                //===OBJECT CLASSIFICATION===
                decided_class=compute_class4standing(actual_x_dim, actual_y_dim, lego_height, lego_caps, lego_ramps);
                if(decided_class==11){
                  //YOLO is needed
                  //decided_class = yolo_scan(loop_rate, pose2d, localization_data, detection_data, mirPublisher);
                  cout<<"Yolo_scan"<<endl;
                }
              }

              //DECISIONS ARE SAVED IN THE LEGO ITSELF
              lego.setX(lego_x);
              lego.setY(lego_y);
              lego.setClasse(decided_class);
              lego.setName(get_lego_string(decided_class));
              //lego.setOrientation(orientation);
              lego.setOrientation(orientation+pose2d.theta);
              lego.setPose(pose);
              
              world.targetArea[currentArea].addLego(lego);

              //Write output
              cout<<"AREA: "<<currentArea<<" - LEGO class: "<<lego.getClasse()<<", name: "<<lego.getName()<<", x: "<<lego.getX()<<", y: "<<lego.getY()<<endl;
              outFile<<"AREA: "<<currentArea<<" - LEGO class: "<<lego.getClasse()<<", name: "<<lego.getName()<<", x: "<<lego.getX()<<", y: "<<lego.getY()<<endl;
            }
            
            world.visited[currentArea]=true;  
            localization_data.disable();
            //====================================================================
          }

        }

        //Update theoretical position (generic coordinates) of the mir robot
        desired_actual_th=pose2d.theta;
        desired_actual_x=xf;
        desired_actual_y=yf;

      }
      //===end of travelling=========================
      joint_data.enable();

      if(currentArea<=CARD_TARGET){ //WE ARE IN A TARGET AREA

        if(world.targetArea[currentArea].getNumLego()>0){ //check the presence of legos in the current area
          
          //Choose which lego to pick up
          currentLego=world.targetArea[currentArea].getNextLego();

          dyn_cls=11;          //class written in dynamic_links.txt, by default is 11=UNKNOWN
          epsi = 0.075;        //maximum error (hope so) in localization

          for(Lego gaz_lego : legos){
            if(sqrt( pow(abs(gaz_lego.getX() - currentLego.getX()),2) +  pow(abs(gaz_lego.getY() - currentLego.getY()),2)) < epsi){
              dyn_cls = gaz_lego.getClasse();
              gaz_name = gaz_lego.getName();
              break;
            }
          }
          //---

          if(dyn_cls==11){  //localization error

            cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"LOCALIZATION ERROR"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

          }else if(currentLego.getClasse()==11){  //YOLO error

            cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"YOLO ERROR: NO OUTPUT"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

          }else if(currentLego.getClasse() != dyn_cls){ //dynamic linking error

            cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"ME DESPLAS, EN ZAPÀ N LEGO PAR N'AUTER"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

          }else{  //No error in object classification and localization

            //Pick up the lego
            pick_brick(initialPose, gaz_name, currentLego, pose2d, loop_rate, joint_com_pub, joint_data, gripperTopic, client_attach);          

          }

          //Remove the lego from the area
          world.targetArea[currentArea].removeLego();

          //Keep note that we have a lego in the gripper now
          has_brick=true;  

          //If there are no more legos in the area, keep track that an area has been completed
          if(world.targetArea[currentArea].isEmpty()){
            world.areaCompleted();
          }

        }else{
          cout<<"There is no lego in this area"<<endl;
        }


      }else{  //WE ARE IN A BASKET AREA
        //Put the lego into the basket
        throw_brick(initialPose, gaz_name, loop_rate, joint_com_pub, joint_data, gripperTopic, client_detach);
        dynamic_delete_model(gaz_name, client_delete);

        //Keep note that we have not a lego in the gripper now
        has_brick=false;

      }
      joint_data.disable();
    }//...if(goOn)
  }

  return 0;
}