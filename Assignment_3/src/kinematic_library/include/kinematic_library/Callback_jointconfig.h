#ifndef CALLBACK_JOINTCONFIG_H
#define CALLBACK_JOINTCONFIG_H

#include "ros/ros.h"
#include <ros/package.h>

//===EIGEN LIBRARY===
#include <eigen3/Eigen/Dense>
//===================

#include "control_msgs/JointControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"

#include <kinematic_library/ur5Kinematic.h>

class Callback_jointconfig{
    private:
        ros::NodeHandle n;
        ros::Subscriber shoulder_pan_joint_sub;
        ros::Subscriber shoulder_lift_joint_sub;
        ros::Subscriber elbow_joint_sub;
        ros::Subscriber wrist_1_joint_sub;
        ros::Subscriber wrist_2_joint_sub;
        ros::Subscriber wrist_3_joint_sub;
        bool enabled;

    public:
        Ur5JointConfiguration jnt_pos_start;

        Callback_jointconfig(ros::NodeHandle& _nh);
        void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);
        void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);
        void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);
        void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);
        void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);
        void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg);

        void disable();
        void enable();
        bool is_enable();
};
#endif
