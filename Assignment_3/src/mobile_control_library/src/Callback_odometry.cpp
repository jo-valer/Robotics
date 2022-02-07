#include "mobile_control_library/Callback_odometry.h"

using namespace std;

Callback_odometry::Callback_odometry(ros::NodeHandle& _nh){
    n = _nh;
    subscriber = n.subscribe("/base_pose_ground_truth", 1, &Callback_odometry::odometryCallback, this);
    enabled = false;
}

void Callback_odometry::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  if(enabled){
    //Get actual position
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;
    
    //Get orientation in quaternion form
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    //Convert quaternion in RPY angles
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //Complete info about actual pose
    this->theta = yaw;
  }
}

void Callback_odometry::disable(){
    this->enabled = false;
}

void Callback_odometry::enable(){
    this->enabled = true;
}

bool Callback_odometry::is_enable(){
    return enabled;
}