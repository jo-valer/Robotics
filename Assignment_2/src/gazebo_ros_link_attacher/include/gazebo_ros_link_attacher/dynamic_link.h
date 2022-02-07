#ifndef DYNAMIC_LINK_H
#define DYNAMIC_LINK_H
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include "my_world/common.h"
#include "my_world/Lego.h"
#include "gazebo_ros_link_attacher/gazebo_ros_link_attacher.h"
#include "gazebo_msgs/DeleteModel.h"

//===KINEMATIC LIBRARY===
#include <kinematic_library/EndEffector.h>
#include <kinematic_library/Ur5JointConfiguration.h>
#include <kinematic_library/ur5Kinematic.h>
#include <kinematic_library/Callback_jointconfig.h>
//=======================

using namespace std;

/**
 * @brief 
 * Init vector legos
 */
void init_dynamic_links(vector<Lego>& legos);

/**
 * @brief 
 * Attach/detach the lego with a dynamic link
 * @param lego_dynamic_link lego to attach/detach
 */
void dynamic_link_attach(string lego_dynamic_link, ros::ServiceClient& client_attach);
void dynamic_link_detach(string lego_dynamic_link, ros::ServiceClient& client_detach);

/**
 * @brief 
 * Delete the lego
 * @param lego_dynamic_link lego to delete
 */
void dynamic_delete_model(string lego_dynamic_link, ros::ServiceClient& client_delete);

/**
 * @brief 
 * Function to pick up a brick with the robotic arm
 * 
 * @param initialPose  end effector configuration
 * @param lego_dynamic_link dynamic link services
 * 
 * @param lego_x_rel lego x position
 * @param lego_y_rel lego y position
 * 
 * @param lego_q_rel required rotation of the gripper
 * @param grip required aperture of the gripper
 * 
 * @param altitude gripper desired height
 * 
 * @param loop_rate ros::Rate
 * 
 * @param joint_com_pub publisher for the robotic arm
 * @param joint_data joint configuration
 * 
 * @param gripperTopic gripper publisher
 * 
 * @param client_attach dynamic linking client
 */
void pick_brick(EndEffector &initialPose, string lego_dynamic_link, double lego_x_rel, double lego_y_rel, double lego_q_rel, double grip, double altitude, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_attach);

/**
 * @brief 
 * Function to relese a brick with the robotic arm into a basket
 * 
 * @param initialPose  end effector configuration
 * @param lego_dynamic_link dynamic link services
 *  
 * @param loop_rate ros::Rate
 *  
 * @param joint_com_pub publisher for the robotic arm 
 * @param joint_data joint configuration
 *  
 * @param gripperTopic gripper publisher
 * 
 * @param client_detach dynamic linking client 
 */
void throw_brick(EndEffector &initialPose, string lego_dynamic_link, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_detach);

#endif
