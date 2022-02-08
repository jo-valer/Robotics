#include "kinematic_library/Callback_jointconfig.h"

using namespace std;

Callback_jointconfig::Callback_jointconfig(ros::NodeHandle& _nh){
    n = _nh;
    
    shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, &Callback_jointconfig::get_shoulder_pan_joint_position, this);
	shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, &Callback_jointconfig::get_shoulder_lift_joint_position, this);
	elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, &Callback_jointconfig::get_elbow_joint_position, this);
	wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, &Callback_jointconfig::get_wrist_1_joint_position, this);
	wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, &Callback_jointconfig::get_wrist_2_joint_position, this);
	wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, &Callback_jointconfig::get_wrist_3_joint_position, this);

    enabled = false;
}

void Callback_jointconfig::get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
    if(enabled){
	    jnt_pos_start.setTh1(ctr_msg->process_value);
    }
}

void Callback_jointconfig::get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  if(enabled){
    jnt_pos_start.setTh2(ctr_msg->process_value);
  }
}

void Callback_jointconfig::get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  if(enabled){
    jnt_pos_start.setTh3(ctr_msg->process_value);
  }
}

void Callback_jointconfig::get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  if(enabled){
    jnt_pos_start.setTh4(ctr_msg->process_value);
  }
}

void Callback_jointconfig::get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  if(enabled){
    jnt_pos_start.setTh5(ctr_msg->process_value);
  }
}

void Callback_jointconfig::get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
  if(enabled){    
    jnt_pos_start.setTh6(ctr_msg->process_value);
  }
}

void Callback_jointconfig::disable(){
    this->enabled = false;
}

void Callback_jointconfig::enable(){
    this->enabled = true;
}

bool Callback_jointconfig::is_enable(){
    return enabled;
}