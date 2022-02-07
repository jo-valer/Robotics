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
    for(int zone=0; zone<4; zone++){
      myfile>>num_rows;
      for(int i=0; i<num_rows; i++){
        myfile>>gazebo_class>>pos_x>>pos_y>>classe;
        input_lego.setName(gazebo_class);
        input_lego.setX(pos_x);
        input_lego.setY(pos_y);
        input_lego.setClasse(classe);
        legos.push_back(input_lego);
      }
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

void pick_brick(EndEffector &initialPose, string lego_dynamic_link, Lego lego, Callback_odometry& pose2d, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6], Callback_jointconfig& joint_data, ros::Publisher& gripperTopic, ros::ServiceClient& client_attach){
  double gripper_x;
  double gripper_y;
  double gripper_z;
  double gripper_roll;
  double gripper_pitch;
  double gripper_yaw;
  double kinematic_time;

  double lego_relative_orientation = lego.getOrientation()-pose2d.theta;  //lego orientation with respect to the mir robot frame

  double ptx = lego.getX() - pose2d.x - 0.3*cos(pose2d.theta);
  double pty = lego.getY() - pose2d.y - 0.3*sin(pose2d.theta);
  double beta = calculate_trajectory(0.0,0.0,ptx, pty);
  double alpha = pose2d.theta - beta;
  double dist = sqrt( pow(ptx, 2) + pow(pty, 2) );
  double lego_relative_x = sin(alpha)*dist;
  double lego_relative_y = cos(alpha)*dist;

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Move robotic arm over the lego, and wait for the robotic arm to adjust itself
  gripper_x = lego_relative_x;
  gripper_y = lego_relative_y;
  gripper_z = lego.getZapproach()+0.12;
  gripper_roll = lego_relative_orientation;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 10.0;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub);
  //...

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Lower the gripper 
  gripper_x = lego_relative_x;
  gripper_y = lego_relative_y;
  gripper_z = lego.getZapproach()+0.02;
  gripper_roll = lego_relative_orientation;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 5.0;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub);
  //...
  
  //Open the gripper accordingly to the lego class to pick up
  gripper_controller(lego.getGrip(), gripperTopic);
  //...

  //Dynamic attach
  dynamic_link_attach(lego_dynamic_link, client_attach);
  //...

  //Update end effector configuration before moving the robotic arm
  ros::spinOnce();
  ur5Direct(joint_data.jnt_pos_start, initialPose);

  //Raise the gripper a little bit
  gripper_x = lego_relative_x;
  gripper_y = lego_relative_y;
  gripper_z = lego.getZapproach()+0.12;
  gripper_roll = lego_relative_orientation;
  gripper_pitch = 0.0;
  gripper_yaw = M_PI;
  kinematic_time = 5.0;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub);
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
  kinematic_time = 5.0;
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
  kinematic_time = 5.0;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //...

  //Open the gripper
  gripper_controller(0.0, gripperTopic);
  //...

  //Dynamic detach
  dynamic_link_detach(lego_dynamic_link, client_detach);
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
  kinematic_time = 5.0;
  point2point_motion_plan(initialPose, gripper_x, gripper_y, gripper_z, gripper_roll, gripper_pitch, gripper_yaw, kinematic_time, loop_rate, joint_com_pub); sleep(3);
  //
}