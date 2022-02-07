#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "control_msgs/JointControllerState.h"

using namespace Eigen;
using namespace std;

const int loop_rate_val = 100;

int main(int argc, char **argv){

  //Ros parameter initialization
  std::string urdf_path = ros::package::getPath("ur5-joint-position-control");
	if(urdf_path.empty()) {
		ROS_ERROR("ur5-joint-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
	ros::init(argc, argv, "tcp_control");

	ros::NodeHandle n;

	ros::Rate loop_rate(loop_rate_val);

  ros::Publisher gripperTopic; 
  gripperTopic=n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1000);
  //...
  cout<<"Ciao Vittoria"<<endl;

  const float t_step = 1/((float)loop_rate_val);
	int count = 0;

  while (ros::ok()) {
    std_msgs::Float64 position;
    control_msgs::GripperCommandActionGoal gripperValue;
    float userInput;

    cout<<"Insert value: ";
    cin>>userInput;
    cout<<endl;
    gripperValue.goal.command.position=userInput;
    //gripperValue.goal.command.position=0.5;
    gripperTopic.publish(gripperValue);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}