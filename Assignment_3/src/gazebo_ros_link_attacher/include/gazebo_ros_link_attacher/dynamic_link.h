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
#include "mobile_control_library/Callback_odometry.h"
#include "mobile_control_library/mir_control_lib.h"

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

void pick_brick(EndEffector &initialPose, string lego_dynamic_link, Lego lego, Callback_odometry& pose2d, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_attach);

void throw_brick(EndEffector &initialPose, string lego_dynamic_link, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_detach);

#endif
