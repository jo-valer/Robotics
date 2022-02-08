#ifndef UR5KINEMATIC_H
#define UR5KINEMATIC_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "EndEffector.h"
#include "Ur5JointConfiguration.h"
#include "kinematic_library/MyUrd5.h"
#include "control_msgs/JointControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "std_msgs/Float64.h"

#define JOINTS 6

/**
 * @brief
 * Constructing the rotation matrix given the eular angles
 * The rotation matrix R is constructed as follows:
 *      ct = [cz cy cx] and st = [sy sy sx]
        R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
               cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
               -sy            cy*sx             cy*cx]
        = Rz(tz) * Ry(ty) * Rx(tx)
 *  
 * Where:
 * -ct = cos(eul)
 * -st = sin(eul);
 */
void eul2rotm(double eul[3], Matrix3d& rotMatrix);

/**
 * @brief 
 * Converting a rotation matrix to Euler angles
 * The solution is not unique in most cases.
 * The output of the following code should exactly
 * match the output of MATLAB’s rotm2euler
 */
void rotm2eul(double eul[3], Matrix3d& rotMatrix);

/**
 * @brief 
 * Direct Kinematics of the UR5
 * @param jointConfiguration six joint angles
 * @param endEffector the procedure computes pose and orientation of the endEffector
 */
void ur5Direct(Ur5JointConfiguration &jointConfiguration, EndEffector &endEffector);

/**
 * @brief 
 * Inverse Kinematics of the UR5
 * There are multiple solutions for each angle. We choose the
 * first of these solutions.
 * @param jointConfiguration six joint angles
 * @param endEffector the procedure computes pose and orientation of the endEffector
 */
void ur5Inverse(Ur5JointConfiguration &jointConfiguration, EndEffector &endEffector);

/**
 * @brief
 * Function to control the gripper
 * @param grip how much close the gripper
 */
void gripper_controller(double grip, ros::Publisher& gripperTopic);

/**
 * @brief 
 * Move the robotic arm's end effector to the pose specified by the input parameters
 */
void point2point_motion_plan(EndEffector &initialPose, double x, double y, double z, double input_roll, double input_pitch, double input_yaw, double t_max, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6]);

#endif
