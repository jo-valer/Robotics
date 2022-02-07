/**
 * ASSIGNMENT 2
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
#include "my_world/common.h"
#include "my_world/Lego.h"
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
#include "robotic_vision/Callback_pointcloud.h"
#include "robotic_vision/Callback_detect.h"
#include "robotic_vision/Callback_localization.h"
//============================

using namespace Eigen;
using namespace std;

const int loop_rate_val = 100;

ofstream logFile(LOG_FILE);    //log file
ofstream outFile(OUTPUT_FILE); //output file

//===WORLD INITIALIZATION FUNCTIONS===

/**
 * @brief 
 * Initialize gloabal vector targetPoints
 */
void init_targetPoints(vector<MobileRobot>& targetPoints);

/**
 * @brief 
 * Initialize data structure which stores the location of each basket
 */
void init_basket_coordinates();
MobileRobot baskets[NUM_CLASSES]; //baskets[i] corresponds to the mobile robot generic coordinates of the basket for lego of class i

//===================================

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

	//Create publishers to send position commands to the MIR
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

  //Define gripper publisher's advertise
  ros::Publisher gripperTopic = n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1000);
  //...

  //Define client for dinamic link services
  ros::ServiceClient client_attach = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  ros::ServiceClient client_detach = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
  ros::ServiceClient client_delete = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  //...

  //Create callback to get localization data
  Callback_localization localization_data(n);

  //Create callback to get yolo data
  Callback_detect detection_data(n);

  cout<<"Assignment 2"<<endl;

  //Make sure we have received proper joint angles already
  joint_data.enable();
  for(int i=0; i<5; i++) {
      ros::spinOnce();
      loop_rate.sleep();
  }

  //Initialize target locations
  vector<MobileRobot> targetPoints; //vector of target poses for the mobile robot initialized in init_targetPoints()
  init_targetPoints(targetPoints);
  //...

  //Initialize basket coordinates
  init_basket_coordinates();
  //...

  //Initialize legos vector
  vector<Lego> legos; //vector of lego dynamic links, for dynamic_link_attach()
  init_dynamic_links(legos);
  //...

  EndEffector initialPose;      //end effector configuration

  //---TRAVELLING---
  double actual_x;              //position x read from /base_pose_ground_truth topic
  double actual_y;              //position y read from /base_pose_ground_truth topic
  double actual_th;             //position th read from /base_pose_ground_truth topic

  double desired_actual_x=0.0;  //theoretical position x (where we think the robot is)
  double desired_actual_y=0.0;  //theoretical position y (where we think the robot is)
  double desired_actual_th=0.0; //theoretical position th (where we think the robot is)

  double xf;                    //target x position of the mir
  double yf;                    //target y position of the mir
  double thf;                   //target th position of the mir
  //----------------

  if(ros::ok()){

    sleep(2); //Wait two seconds before starting the execution

    for(MobileRobot target_pos : targetPoints){  //reach all the target area in sequence

      //Update end effector configuration before moving the robotic arm
      ros::spinOnce();
      loop_rate.sleep();
      ur5Direct(joint_data.jnt_pos_start, initialPose);

      //Raise robotic arm in a straight configuration
      point2point_motion_plan(initialPose, -0.1, 0.1, 0.9, 0.0, 0.0, 0.0, 5, loop_rate, joint_com_pub); sleep(3);
      joint_data.disable();
      ros::spinOnce();
      loop_rate.sleep();

      //Read position of the robot from the topic (two decimals for x and y)
      //actual_x=round(pose2d.x*100)/100.0;
      //actual_y=round(pose2d.y*100)/100.0;
      actual_x=pose2d.x;
      actual_y=pose2d.y;
      actual_th=pose2d.theta;

      //Read final configuration (generic coordinates)
      xf=target_pos.getX();
      yf=target_pos.getY();
      thf=target_pos.getTh();

      //TRAVEL TO DESTINATION
      travel(desired_actual_x, desired_actual_y, desired_actual_th, xf, yf, thf, loop_rate, pose2d, mirPublisher);
      
      //Update theoretical position (generic coordinates) of the mir robot
      desired_actual_th=thf;
      desired_actual_x=xf;
      desired_actual_y=yf;

      pose2d.disable();
      localization_data.enable();
      ros::spinOnce();
      loop_rate.sleep();

      //per essere sicuri che localize.py parta su un'immagine nuova
      sleep(2);

      //===OBJECT CLASSIFICATION AND LOCALIZATION===

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
      int actual_y_dim;
      int actual_x_dim;
      
      //Point cloud support variables
      PointCloud rotated_pointcloud;
      double etha;
      int lego_height;
      int lego_caps;

      //===OBJECT LOCALIZATION===
      
      //Read lego x,y,z dimensions from upper camera data
      //tie(pixel_x, pixel_y, center_x, center_y, orientation, width, height) = get_localize_data();
      pixel_x = localization_data.localized[0].img_x;
      pixel_y = localization_data.localized[0].img_y;
      center_x = localization_data.localized[0].x;
      center_y = localization_data.localized[0].y;
      orientation = localization_data.localized[0].q; 
      width = localization_data.localized[0].w; 
      height = localization_data.localized[0].h;
      //...

      //Compute lego's coordinates in gazebo
      //double lego_x = pose2d.x + cos(pose2d.theta)*(localize_data[i][0] + 0.3);
      //double lego_y = pose2d.y + sin(pose2d.theta)*(localize_data[i][1] + 0.3);
      lego_x = pose2d.x + center_x;
      lego_y = pose2d.y + center_y + 0.3; //+0.3 because the robotic arm is not mounted in the middle of the mir

      //Get a comparable lego properties [X dim, Y dim] computed by the localization process
      actual_y_dim = get_lego_property_y_localized(width, height);
      actual_x_dim = get_lego_property_x_localized(width, height);
      
      //===POINT CLOUD HANDLING===
      pointCloud_data.enable();
      ros::spinOnce();
      loop_rate.sleep();

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

      //Get lego height and number of caps looking at point cloud data
      tie(lego_height, lego_caps) = get_lego_property_z_and_caps(pixel_x, pixel_y, rotated_pointcloud);
      
      pointCloud_data.disable();
      localization_data.disable();
      sleep(2);

      //===OBJECT CLASSIFICATION===

      //Having localization data in some cases YOLO computation is not needed
      //Class 0: X1_Y1_Z2
      if(actual_y_dim==1)                         {decided_class=0;}
      //Class 9: X2_Y2_Z2
      else if(actual_x_dim==2 && lego_caps>2)     {decided_class=9;}
      //Class 10: X2_Y2_Z2_FILLET
      else if(actual_x_dim==2)                    {decided_class=10;}

      //Class 1: X1_Y2_Z1
      else if(actual_y_dim==2 && lego_height==1)  {decided_class=1;}
      //Class 2: X1_Y2_Z2
      else if(actual_y_dim==2 && lego_caps>1)     {decided_class=2;}
      //Class 3 - 4: X1_Y2_Z2_CHAMFER - X1_Y2_Z2_TWINFILLET
      else if(actual_y_dim==2){
        int lego_ramps = get_lego_property_ramp(pixel_x, pixel_y, rotated_pointcloud);
        if(lego_ramps==1)                         {decided_class=3;}
        else                                      {decided_class=4;}
      }

      //Class 5: X1_Y3_Z2
      else if(actual_y_dim==3 && lego_caps>1)     {decided_class=5;}
      //Class 6: X1_Y3_Z2_FILLET
      else if(actual_y_dim==3)                    {decided_class=6;}
      //Class 7: X1_Y4_Z1
      else if(actual_y_dim==4 && lego_height==1)  {decided_class=7;}
      //Class 8: X1_Y4_Z2
      else if(actual_y_dim==4)                    {decided_class=8;}

      else{  //YOLO is needed
        detection_data.enable();
        ros::spinOnce();
        sleep(15);
        decided_class = yolo_scan(loop_rate, pose2d, localization_data, detection_data, mirPublisher);
        detection_data.disable();
      }

      classe_lego_str = get_lego_string(decided_class);

      // Write final output
      outFile<<"LEGO class: "<<decided_class<<", name: "<<classe_lego_str<<", x:"<<lego_x<<", y:"<<lego_y<<endl;
      cout<<"LEGO class: "<<decided_class<<", name: "<<classe_lego_str<<", x:"<<lego_x<<", y:"<<lego_y<<endl;

      pose2d.enable();
      joint_data.enable();
      //====================================================================
      ros::spinOnce();
      loop_rate.sleep();
      //=========================== ASSIGNMENT 2 ===========================
      double grip = 0.1;      //how much the gripper is closed
      double altitude = -0.1; //at wich z the lego has to be aproached
      int dyn_cls=11;         //class written in dynamic_links.txt, by default is 11=UNKNOWN
      string gaz_name;        //brick's name in gazebo
      double epsi = 0.01;     //maximum error (hope so) in localization

      //Get data for dynamic linking
      for (Lego lego : legos){
        if( (lego.getX() > lego_x) && (lego.getX()-lego_x < epsi) || (lego.getX() < lego_x) && (lego_x-lego.getX() < epsi) ){
          if( (lego.getY() > lego_y) && (lego.getY()-lego_y < epsi) || (lego.getY() < lego_y) && (lego_y-lego.getY() < epsi) ){
            dyn_cls = lego.getClasse();
            gaz_name = lego.getName();
            grip = lego.getGrip();
            altitude = lego.getZapproach();
            break;
          }
        }
      }
      //---

      if(dyn_cls==11){  //localization error

        cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"LOCALIZATION ERROR"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

      }else if(decided_class==11){  //YOLO error
      
        cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"YOLO ERROR: NO OUTPUT"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

      }else if(decided_class != dyn_cls){ //dynamic linking error

        //rot_brick(initialPose, gaz_name, localization_output_center_x, localization_output_center_y, localization_output_orientation, grip);
        cout<<endl<<endl<<endl<<"-----!!!-----"<<endl<<"ME DESPLAS, EN ZAPÀ N LEGO PAR N'AUTER"<<endl<<"-----!!!-----"<<endl<<endl<<endl<<endl;

      }else{  //No error in object classification and localization
        
        //Pick up the lego with the gripper
        pick_brick(initialPose, gaz_name, center_x, center_y, orientation, grip, altitude, loop_rate, joint_com_pub, joint_data, gripperTopic, client_attach);

        //Depending on the lego class, go to the proper basket
        travel(desired_actual_x, desired_actual_y, desired_actual_th, baskets[decided_class].getX(), baskets[decided_class].getY(), baskets[decided_class].getTh(), loop_rate, pose2d, mirPublisher);

        sleep(0.5);

        //Drop the lego in the basket
        throw_brick(initialPose, gaz_name, loop_rate, joint_com_pub, joint_data, gripperTopic, client_detach);
        dynamic_delete_model(gaz_name, client_delete);

        //Update mir position
        desired_actual_th=baskets[decided_class].getTh();
        desired_actual_x=baskets[decided_class].getX();
        desired_actual_y=baskets[decided_class].getY();
      }
      //=================================================================================

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}

//===WORLD INITIALIZATION FUNCTIONS===
void init_targetPoints(vector<MobileRobot>& targetPoints){
  MobileRobot target(0.0, 2.0, 1.57);
  for(int k=-3; k<=3; k=k+2){
    target.setX(k/1.0);
    targetPoints.push_back(target);
  }
}

void init_basket_coordinates(){
  int i=0;
  for(int b=-5; b<=5; b++){
    baskets[i].setX(b/1.0);
    baskets[i].setY(-1.0);
    baskets[i].setTh(-1.57);
    i++;
  }
}
//============================