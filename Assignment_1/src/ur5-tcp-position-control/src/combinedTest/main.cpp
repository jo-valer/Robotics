#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <kinematic_library/Position.h>
#include <kinematic_library/RPYRotation.h>
#include <kinematic_library/EndEffector.h>
#include <kinematic_library/Ur5JointConfiguration.h>
#include <kinematic_library/ur5Kinematic.h>
#include <kinematic_library/MyUrd5.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"

//TODO: Togliere kdl
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

using namespace Eigen;
using namespace std;

//Global publishers needed to enable point2point_motion_plan() and gripper_controller() to publish
ros::Publisher joint_com_pub[6];
ros::Publisher gripperTopic;

const int Joints = 6;
KDL::JntArray jnt_pos_start_vecchio(Joints);
Ur5JointConfiguration jnt_pos_start;

void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(0) = ctr_msg->process_value;
	jnt_pos_start.setTh1(ctr_msg->process_value);
}

void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(1) = ctr_msg->process_value;
  jnt_pos_start.setTh2(ctr_msg->process_value);
}

void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(2) = ctr_msg->process_value;
  jnt_pos_start.setTh3(ctr_msg->process_value);
}

void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(3) = ctr_msg->process_value;
  jnt_pos_start.setTh4(ctr_msg->process_value);
}

void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(4) = ctr_msg->process_value;
  jnt_pos_start.setTh5(ctr_msg->process_value);
}

void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  jnt_pos_start_vecchio(5) = ctr_msg->process_value;
  jnt_pos_start.setTh6(ctr_msg->process_value);
}

void point2point_motion_plan(EndEffector &initialPose, double x, double y, double z, double input_roll, double input_pitch, double input_yaw, double t_max){
  x = -1*x;
  //y = (-1*y)-0.0823;
  y = -1*y;
  z = z;

  cout<<"Modified: "<<"x: "<<x<<"; y="<<y<<"; z="<<z<<endl;

  //Final pose
  EndEffector finalPose;
  finalPose.setPostion(x, y, z);
  finalPose.setOrientation(input_roll, input_pitch, input_yaw);
  //finalPose.setOrientation(0.0, M_PI_2, M_PI);
  Matrix3d fiRotMatrix;
  double fiEul[3]={finalPose.getOrientation().getR(), finalPose.getOrientation().getP(), finalPose.getOrientation().getY()};
  eul2rotm(fiEul, fiRotMatrix);
  finalPose.setRotationMatrix(fiRotMatrix);

  //Time
  double minT=0;
  double maxT=t_max;
  float deltaT = 0.01;

  //Compute initial and final join configurations with inverse kinematic
  Ur5JointConfiguration initialJointConfig;
  Ur5JointConfiguration finalJointConfig;
  ur5Inverse(initialJointConfig, initialPose);
  ur5Inverse(finalJointConfig, finalPose);

  //q(t) = a3t^3 + a2t^2 + a1t + a0
    
  //Compute coefficients a0 a1 a2 a3
  Matrix<double,6,4>A;  //Matrix of coefficients

  //Support variables
  Matrix4d M;
  Matrix<double,4,1>b;
  Matrix<double,4,1>a;
  double qEs[6]={ initialJointConfig.getTh1(),
                  initialJointConfig.getTh2(),
                  initialJointConfig.getTh3(),
                  initialJointConfig.getTh4(),
                  initialJointConfig.getTh5(),
                  initialJointConfig.getTh6()
                };
  double qEf[6]={ finalJointConfig.getTh1(),
                  finalJointConfig.getTh2(),
                  finalJointConfig.getTh3(),
                  finalJointConfig.getTh4(),
                  finalJointConfig.getTh5(),
                  finalJointConfig.getTh6()
                };

  M<<   1.0, minT, pow(minT, 2), pow(minT, 3),
        0.0, 1.0,  2.0*minT,     3.0*pow(minT, 2),
        1.0, maxT, pow(maxT, 2), pow(maxT, 3),
        0.0, 1.0,  2.0*maxT,     3.0*pow(maxT, 2);
    
  for(int i=0; i<6; i++){
    b(0,0)=qEs[i];
    b(1,0)=0.0;
    b(2,0)=qEf[i];
    b(3,0)=0.0;
    a=M.inverse()*b;

    //A = [A; a']
    for(int j=0; j<4; j++){
      A(i,j)=a(j,0);
    }
  }

  //Compute the sequence of positions (time dependent) to reach the final configuration
  Ur5JointConfiguration tmpJointConfiguration;
  EndEffector tmpEndEffectorPose;
  for(float t=minT; t<maxT; t=t+deltaT){
    //Compute the expression q(t) = a3t^3 + a2t^2 + a1t + a0
    //for each angle.
    double qt;
    
    //the configuration of the robotic arm at time t
    //Ur5JointConfiguration tmpJointConfiguration;
    double tmpJointConfigutationAngles[6];

    for(int j=0; j<6; j++){
      qt=A(j,0)+A(j,1)*t+A(j,2)*pow(t,2)+A(j,3)*pow(t,3);
      tmpJointConfigutationAngles[j]=qt;
    }
    tmpJointConfiguration.setAngles(tmpJointConfigutationAngles);

    std_msgs::Float64 position[6];
		//Compute next position step for all joints
		for(int i=0; i<Joints; i++) {
			position[i].data = tmpJointConfigutationAngles[i];
			joint_com_pub[i].publish(position[i]);
		}

    //Using direct kinematic compute the end effector position and orientation (RPY)
    //EndEffector tmpEndEffectorPose;
    ur5Direct(tmpJointConfiguration, tmpEndEffectorPose);
    cout<<"POINT:"<<endl;
    cout<<"Position: "<<endl;
    cout<<" x: "<<tmpEndEffectorPose.getPosition().getX()<<endl;
    cout<<" y: "<<tmpEndEffectorPose.getPosition().getY()<<endl;
    cout<<" z: "<<tmpEndEffectorPose.getPosition().getZ()<<endl;
    cout<<" R: "<<tmpEndEffectorPose.getOrientation().getR()<<endl;
    cout<<" P: "<<tmpEndEffectorPose.getOrientation().getP()<<endl;
    cout<<" Y: "<<tmpEndEffectorPose.getOrientation().getY()<<endl;
    cout<<"---"<<endl;
   }

  //Wait for the robotic arm to adjust itself
  sleep(3);

  //Get new initial pose from the last joint configuration computed
  ur5Direct(tmpJointConfiguration, initialPose);
}  

void gripper_controller(double grip){
  std_msgs::Float64 position;
  control_msgs::GripperCommandActionGoal gripperValue;
  gripperValue.goal.command.position=grip;
  gripperTopic.publish(gripperValue);
}
 
const int loop_rate_val = 100;
ofstream logFile("log.txt");  //log file

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

  //Create subscribers for all joint states
	ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
	ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
	ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
	ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
	ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
	ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);

	//Define publishers' advertise, to send position commands to all joints
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);
  //...

  //Define gripper publisher's advertise
  gripperTopic=n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1000);
  //...

  cout<<"Ciao Vittoria"<<endl;

  /**KDL OPERATIONS --> TODO remove*/
  //Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
	if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
		ROS_ERROR("Failed to construct kdl tree");
   		return false;
	}

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

  //Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);
  /**---*/

  //Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}

  const float t_step = 1/((float)loop_rate_val);
	int count = 0;

  EndEffector initialPose; 

  //FIRST TIME initialPose is computed reading the joint configuration from the subscribed topic
  if(ros::ok()){
    //Initial pose
    ROS_INFO("Posizione iniziale secondo mio eseguibile:");
    ur5Direct(jnt_pos_start, initialPose);

    cout<<"jnt_pos_start[1]="<<jnt_pos_start.getTh1();
    cout<<"jnt_pos_start[2]="<<jnt_pos_start.getTh2();
    cout<<"jnt_pos_start[3]="<<jnt_pos_start.getTh3();
    cout<<"jnt_pos_start[4]="<<jnt_pos_start.getTh4();
    cout<<"jnt_pos_start[5]="<<jnt_pos_start.getTh5();
    cout<<"jnt_pos_start[6]="<<jnt_pos_start.getTh6()<<endl;

    //initialPose.setPostion(-1*initialPose.getPosition().getX(), -1*(initialPose.getPosition().getY()+0.0823), initialPose.getPosition().getZ());
    ROS_INFO("Position: %lf %lf %lf", initialPose.getPosition().getX(), initialPose.getPosition().getY(), initialPose.getPosition().getZ());		
		ROS_INFO("Orientation: %lf %lf %lf", initialPose.getOrientation().getR(), initialPose.getOrientation().getP(), initialPose.getOrientation().getY());
    cout<<"Rotation: "<<initialPose.getRotationMatrix()<<endl;
    ROS_INFO("---");

		//Compute current tcp position
    ROS_INFO("Posizione iniziale secondo eseguibile vecchio:");
		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start_vecchio, tcp_pos_start);

		ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));
    double kdlEularAngles[3];
    Matrix3d kdlRotMatrix;
    kdlRotMatrix<<tcp_pos_start.M(0,0), tcp_pos_start.M(0,1), tcp_pos_start.M(0,2),
                  tcp_pos_start.M(1,0), tcp_pos_start.M(1,1), tcp_pos_start.M(1,2),
                  tcp_pos_start.M(2,0), tcp_pos_start.M(2,1), tcp_pos_start.M(2,2);
    rotm2eul(kdlEularAngles, kdlRotMatrix);
    ROS_INFO("Orientation: %lf %lf %lf", kdlEularAngles[0], kdlEularAngles[1], kdlEularAngles[2]);
    ROS_INFO("RotMatrix: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(0,1), tcp_pos_start.M(0,2));
    ROS_INFO("RotMatrix: %f %f %f", tcp_pos_start.M(1,0), tcp_pos_start.M(1,1), tcp_pos_start.M(1,2));
    ROS_INFO("RotMatrix: %f %f %f", tcp_pos_start.M(2,0), tcp_pos_start.M(2,1), tcp_pos_start.M(2,2));		
    ROS_INFO("---");
  }


  while (ros::ok()) {
    //Get user input
		double x,y,z,t_max,grip;
    double input_roll, input_pitch, input_yaw;
		std::cout << "x:";
		std::cin >> x;
		std::cout << "y:";
		std::cin >> y;
		std::cout << "z:";
		std::cin >> z;
    //std::cout << "roll:";
    //std::cin >> input_roll;
    //std::cout << "pitch:";
    //std::cin >> input_pitch;
    //std::cout << "yaw:";
    //std::cin >> input_yaw;
		std::cout << "Time:";
		std::cin >> t_max;
    std::cout << "Gripper:";
    std::cin >> grip;
    cout<<"Inserted: "<<"x: "<<x<<"; y="<<y<<"; z="<<z<<"; grip="<<grip<<endl;

    point2point_motion_plan(initialPose, x, y, z+0.2, 0.0, 0.0, M_PI, t_max);
    point2point_motion_plan(initialPose, x, y, z, 0.0, 0.0, M_PI, t_max);
    gripper_controller(grip);

    ros::spinOnce();
	  loop_rate.sleep();
	}

  return 0;
}