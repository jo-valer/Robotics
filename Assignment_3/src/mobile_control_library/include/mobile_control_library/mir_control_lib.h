#ifndef MIR_CONTROL_LIB_H
#define MIR_CONTROL_LIB_H

#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <boost/numeric/odeint.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>

#include "my_world/common.h"
#include "my_world/Coordinate.h" 

#include "mobile_control_library/Callback_odometry.h"

#define VEL_LIN 0.5     //constant linear velocity
#define VEL_ANG 0.3     //constant angular velocity

//===TRAJECTORY CONTROL===

/**Initial pose values of the vehicle*/
double x0a;
double y0a;
double th0a;
/**---*/

/**Initial desired position of the vehicle*/
double x0d;
double y0d;
double th0d;
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
double calculate_trajectory(double from_x, double from_y, double to_x, double to_y);


//========================

//===SAFE TRAJECTORY CHECKING===

/**
 * @brief 
 * When we live a target area we have to check if our trajectory is dangerous.
 * 
 * Dangerous meaning: in some cases if we follow a stright trajectory to the destination we bounce into the legos of the current area.
 * We have to detect this cases in order to avoid lego collisions.
 * 
 * Assumption: this function is called only at the beginning of the travel to the destination, we want to check
 * intersections between the trajectory and the target area where the mir is.
 * 
 * @param mir_x current coordinate x of the mir 
 * @param mir_y current coordinate y of the mir
 * @param center_x coordinate x of the center of the target area where the mir is
 * @param center_y coordinate y of the center of the target area where the mir is
 * @param radius radius of the target area where the mir is
 * @param destination_x coordinate x of the destination point
 * @param destination_y coordinate y of the destination point
 * @return true the trajectory is dangerous --> avoid to follow the straight trajectory
 * @return false the trajectory is safe --> follow the straight trajectory
 */
bool dangerous_trajectory(double mir_x, double mir_y, double center_x, double center_y, double radius, double destination_x, double destination_y);

/**
 * @brief 
 * When the straight trajectory is dangerous because we might bounce into some legos, we have to find safe intermediate points.
 * This function returns the nearest safe point.
 * @param mir_x current coordinate x of the mir
 * @param mir_y current coordinate y of the mir
 * @param center_x coordinate x of the center of the target area where the mir is
 * @param center_y coordinate y of the center of the target area where the mir is
 * @param radius radius of the target area where the mir is
 * @return Coordinate (x,y) coordinates of the nearest point
 */
Coordinate nearest_safe_point(double mir_x, double mir_y, double center_x, double center_y, double radius);

//==============================

//===TRAVELING===
/**
 * @brief 
 * Move the mir from [from_x, from_y, from_th] to [to_x, to_y, to_th]
*/
void travel(double from_x, double from_y, double from_th, double to_x, double to_y, ros::Rate& loop_rate, Callback_odometry& pose2d, ros::Publisher& mirPublisher);

/**
 * @brief 
 * Rotate to point to the center of the area where the mir is
 */
void rotateToAreaCenter(double mir_x, double mir_y, double area_x, double area_y, ros::Rate& loop_rate, Callback_odometry& pose2d, ros::Publisher& mirPublisher);
//===============
#endif
