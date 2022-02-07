#include "gazebo_ros_link_attacher/dynamic_link.h"

void init_dynamic_links(vector<Lego>& legos){
  
  //Read from file
  ifstream myfile (DYNAMIC_LINK_PATH);

  //Support variables
  int num_rows;
  std::string gazebo_class;
  double pos_x;
  double pos_y;
  int classe;

  Lego input_lego;  
  if (myfile.is_open()){
    myfile>>num_rows;
    for(int i=0; i<num_rows; i++){
      myfile>>gazebo_class>>pos_x>>pos_y>>classe;
      input_lego.setName(gazebo_class);
      input_lego.setX(pos_x);
      input_lego.setY(pos_y);
      input_lego.setClasse(classe);
      legos.push_back(input_lego);
    }
    myfile.close();
  }else{
    cout << "Unable to open dynamic_links.txt"<<endl;
  }
}

//===DYNAMIC LINKING===
void dynamic_link_attach(string lego_dynamic_link, ros::ServiceClient& client_attach){
  gazebo_ros_link_attacher::Attach srv_attach;
  srv_attach.request.model_name_1 = "mir";
  srv_attach.request.link_name_1 = "wrist_3_link";
  srv_attach.request.model_name_2 = lego_dynamic_link;
  srv_attach.request.link_name_2 = "link";
  
  client_attach.call(srv_attach);
}

void dynamic_link_detach(string lego_dynamic_link, ros::ServiceClient& client_detach){
  gazebo_ros_link_attacher::Attach srv_detach;
  srv_detach.request.model_name_1 = "mir";
  srv_detach.request.link_name_1 = "wrist_3_link";
  srv_detach.request.model_name_2 = lego_dynamic_link;
  srv_detach.request.link_name_2 = "link";

  client_detach.call(srv_detach);
}

void dynamic_delete_model(string lego_dynamic_link, ros::ServiceClient& client_delete){
  gazebo_msgs::DeleteModel srv_delete;
  srv_delete.request.model_name = lego_dynamic_link;
  client_delete.call(srv_delete);
}
//=====================

void pick_brick(EndEffector &initialPose, string lego_dynamic_link, double lego_x_rel, double lego_y_rel, double lego_q_rel, double grip, double altitude, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_attach){
  double gripper_x;
  double gripper_y;
  double gripper_z;
  double gripper_roll;
  double gripper_pitch;
  double gripper_yaw;
  double kinematic_time;

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Move robotic arm over the lego, and wait for the robotic arm to adjust itself
  gripper_x = lego_x_rel;
  gripper_y = lego_y_rel;
  gripper_z = altitude+0.12;
  gripper_roll = lego_q_rel;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 10;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Lower the gripper 
  gripper_x = lego_x_rel;
  gripper_y = lego_y_rel;
  gripper_z = altitude+0.01;
  gripper_roll = lego_q_rel;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 5;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...
  
  //Open the gripper accordingly to the lego class to pick up
  gripper_controller(grip, gripperTopic); sleep(3);
  //...

  //Dynamic attach
  dynamic_link_attach(lego_dynamic_link, client_attach);
  //...

  /*
  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(jnt_pos_start, initialPose);

  //Raise the robotic arm
  gripper_x = lego_x_rel;
  gripper_y = lego_y_rel;
  gripper_z = 0.0;
  gripper_roll = lego_q_rel;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 5;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...
  */

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Raise the robotic arm in straight configuration to avoid problems during the motion of the mir
  gripper_x = -0.1;
  gripper_y = 0.1;
  gripper_z = 0.9;
  gripper_roll = 0.0;
  gripper_pitch = 0.0;
  gripper_yaw = 0.0;
  kinematic_time = 5;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...
}

void throw_brick(EndEffector &initialPose, string lego_dynamic_link, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_detach){
  double gripper_x;
  double gripper_y;
  double gripper_z;
  double gripper_roll;
  double gripper_pitch;
  double gripper_yaw;
  double kinematic_time;

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Move robotic arm over the basket, and wait for the robotic arm to adjust itself
  gripper_x = 0.0;
  gripper_y = 0.4;
  gripper_z = 0.3;
  gripper_roll = 0.0;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI_2;
  kinematic_time = 5;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...

  //Open the gripper
  gripper_controller(0.0, gripperTopic); sleep(3);
  //...

  //Dynamic detach
  dynamic_link_detach(lego_dynamic_link, client_detach); sleep(3);
  //...

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Raise the robotic arm in straight configuration to avoid problems during the motion of the mir
  gripper_x = -0.1;
  gripper_y = 0.1;
  gripper_z = 0.9;
  gripper_roll = 0.0;
  gripper_pitch = 0.0;
  gripper_yaw = 0.0;
  kinematic_time = 5;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //
}