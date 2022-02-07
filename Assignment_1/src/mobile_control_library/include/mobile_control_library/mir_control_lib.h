#ifndef MIR_CONTROL_LIB_H
#define MIR_CONTROL_LIB_H

#include <iostream>
#include <ros/ros.h>
#include "mobile_control_library/Callback_odometry.h"

#define VEL_LIN 0.5     //constant linear velocity
#define VEL_ANG 0.3     //constant angular velocity

//===TRAJECTORY CONTROL===

/**Initial pose values of the vehicle*/
double x0a = 0.0;
double y0a = 0.0;
double th0a = -0.1;
/**---*/

/**Initial desired position of the vehicle*/
double x0d = 2.0;
double y0d = 2.0;
double th0d = 0.5;
/**---*/

/**Controller gains*/
double kp = 10.0;
double kth = 10.0;

/*The type of container used to hold the state vector, used in solving differential equations with odeint*/
typedef std::vector<double> state_type;

/**
 * @brief 
 * Desired position
 */
state_type posD(double t);

/**
 * @brief 
 * Auxiliary function 
 */
double sinc(double t);

/**
 * @brief 
 * LYAPUNOV BASED CONTROL
 * Control algorithm to make sure that the robot follows
 * the ideal point 
 */
std::tuple<double, double> lineControl(const state_type &x, const state_type &xd, double vd, double omegad);


//========================

//===TRAJECTORY PLANNING===

/**
 * @brief 
 * Return the rotation angle to reach point [to_x, to_y] from [from_x, from_y]
 */
double calculate_trajectory(double from_x, double from_y, double from_th, double to_x, double to_y);


//========================

//===TRAVELLING===
/**
 * @brief 
 * Move the mir from [from_x, from_y, from_th] to [to_x, to_y, to_th]
*/
void travel(double from_x, double from_y, double from_th, double to_x, double to_y, double to_th, ros::Rate& loop_rate, Callback_odometry& pose2d, ros::Publisher& mirPublisher);
//================

#endif
