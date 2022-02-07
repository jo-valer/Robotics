#ifndef CALLBACK_ODOMETRY_H
#define CALLBACK_ODOMETRY_H

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

class Callback_odometry{
    private:
        ros::NodeHandle n;
        ros::Subscriber subscriber;
        bool enabled;

    public:
        //actual position (x, y, th) of the robot
        double x;
        double y;
        double theta;

        Callback_odometry(ros::NodeHandle& _nh);
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void disable();
        void enable();
        bool is_enable();
};

#endif
