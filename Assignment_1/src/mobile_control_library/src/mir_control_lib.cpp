#include "mobile_control_library/mir_control_lib.h"
using namespace std;

//===TRAJECTORY CONTROL===

double sinc(double t){
  double s;
  if(t==0.0){
    s = 1.0;
  }else{
    s = sin(t)/t;
  }
  return s;
}

state_type posD(double t){
  //xd=[x0d+(0.5*cos(th0d)*t); y0d+(0.5*sin(th0d)*t); th0d];
  state_type output(3);
  output[0]=x0d+(VEL_LIN*cos(th0d)*t);
  output[1]=y0d+(VEL_LIN*sin(th0d)*t);
  output[2]=th0d;

  return output;
}


std::tuple<double, double> lineControl(const state_type &x, const state_type &xd, double vd, double omegad){
  double v;
  double omega;

  //double xd2=(xd[2]>=0.0) ? xd[2] : 2.0*M_PI+xd[2];
  //double x2=(x[2]>=0.0) ? x[2] : 2.0*M_PI+x[2];
  double xd2=xd[2];
  double x2=x[2];

  double ex = x[0]-xd[0];
  double ey = x[1]-xd[1];
  //double eth = x[2]-xd[2];
  //double eth = x2-xd2;
  double eth = atan2(sin(x2-xd2), cos(x2-xd2));
  
  /*
  double thd = xd[2];
  double th = x[2];
  */
  double thd = xd2;
  double th = x2;

  double psi = atan2(ey, ex);
  double alpha=(th+thd)/2.0;
  double exy = sqrt(ey*ey+ex*ex);
  double dv = -kp*exy*cos(psi-th);

  v = vd+dv;
  double domega =-kth*eth-vd*sinc(eth/2.0)*exy*sin(psi-alpha);
  omega = omegad + domega;

  return std::make_tuple(v, omega);
}

//========================


//===TRAJECTORY PLANNING===

double calculate_trajectory(double from_x, double from_y, double from_th, double to_x, double to_y){

  //cout<<"FROM: x:"<<from_x<<" y: "<<from_y<<endl;
  //cout<<"TO: x:"<<to_x<<" y:"<<to_y<<endl;

  double actual_x=from_x;   //position x read from /base_pose_ground_truth topic
  double actual_y=from_y;   //position y read from /base_pose_ground_truth topic
  double actual_th=from_th; //position th read from /base_pose_ground_truth topic

  double xf=to_x;           //target x position of the mir
  double yf=to_y;           //target y position of the mir

  double th0;       //trajectory angle

  //FIRST STEP: Find the angle (th0) of vector v with respect to x axies.
  //            v is such that:
  //                - direction: line between initialConfig point and finalConfig point
  //                - the vector is directed from initialConfig point to finalConfig point
  bool noLinearMotion=false; //true iff the input moviment is on the spot, there is no need to reach a position far away

  //Support variables
  double dx, dy;

  if(actual_x==xf && actual_y==yf){
    noLinearMotion=true;
  }else if(actual_x==xf){
    if(actual_y>yf){
      th0=-M_PI_2;
    }else{
      th0=M_PI_2;
    }
  }else if(actual_y==yf){
    if(actual_x>xf){
      th0=M_PI;
    }else{
      th0=0.0;
    }
  }else if(xf>actual_x && yf>actual_y){ //first quadrant
    dx=xf-actual_x;
    dy=yf-actual_y;

    th0=acos(dx/sqrt(pow(dx,2)+pow(dy,2)));
  }else if(xf<actual_x && yf>actual_y){ //second quadrant
    dx=actual_x-xf;
    dy=yf-actual_y;

    double alpha=acos(dx/sqrt(pow(dx,2)+pow(dy,2)));
    th0=M_PI-alpha;
  }else if(xf<actual_x && yf<actual_y){ //third quadrant
    dx=actual_x-xf;
    dy=actual_y-yf;

    double gamma=acos(dx/sqrt(pow(dx,2)+pow(dy,2)));
    th0=M_PI+gamma;
  }else if(xf>actual_x && yf<actual_y){ //fourth quadrant
    dx=xf-actual_x;
    dy=actual_y-yf;

    double beta=acos(dx/sqrt(pow(dx,2)+pow(dy,2)));
    th0=-beta;
  }

  return th0;
  //...end of first step
}

//========================

//===TRAVELLING===
void travel(double from_x, double from_y, double from_th, double to_x, double to_y, double to_th, ros::Rate& loop_rate, Callback_odometry& pose2d, ros::Publisher& mirPublisher){
  geometry_msgs::Twist command;

  double angVel;    //angular velocity
  double linVel;    //linear velocity
  double distance;  //linear distance between initial and final point

  double actual_x;   //position x read from /base_pose_ground_truth topic
  double actual_y;   //position y read from /base_pose_ground_truth topic
  double actual_th;  //position th read from /base_pose_ground_truth topic
  double actual_thDeg;

  double desired_actual_x = from_x;   //theoretical position x (where we think the robot is)
  double desired_actual_y = from_y;   //theoretical position y (where we think the robot is)
  double desired_actual_th = from_th; //theoretical position th (where we think the robot is)

  double xf = to_x;           //target x position of the mir
  double yf = to_y;           //target y position of the mir
  double thf = to_th;         //target th position of the mir
  double thfDeg;

  double th0;       //trajectory angle
  double th0Deg;

  //Support variables
  double deltaAng;
  double deltaAngDeg;
  
  //Time
  double t;
  ros::Time beginTime;
  ros::Duration secondsIWantToSendMessagesFor;

  //Trajectory control
  double error;

  //FIRST STEP: Find the angle (th0) of vector v with respect to x axies.
  //            v is such that:
  //                - direction: line between initialConfig point and finalConfig point
  //                - the vector is directed from initialConfig point to finalConfig point

  th0=calculate_trajectory(desired_actual_x, desired_actual_y, desired_actual_th, xf, yf);

  //...end of first step

  //SECOND STEP: given th0 turn on the spot to allign the robot with the straight trajectory from [initial_x0, initial_y0] to [xf, yf]
  ros::spinOnce();
  loop_rate.sleep();
  actual_x=pose2d.x;
  actual_y=pose2d.y;
  actual_th=pose2d.theta;

  //Compute angle distance from actual_th to th0
  th0Deg=th0*(180.0/M_PI);
  actual_thDeg=pose2d.theta*(180.0/M_PI);
  deltaAngDeg=(fmod((th0Deg-actual_thDeg+540.0),360.0))-180.0;
  deltaAng=deltaAngDeg*(M_PI/180.0);
  //...

  angVel=VEL_ANG; //constant angular velocity

  //Decide to turn clockwise or counterclockwise
  if(deltaAng<0){
    angVel = -angVel;
  }
  //...

  t=abs(deltaAng/angVel); //time to apply angular velocity angVel to follow the desired rotation

  //cout<<"th0="<<th0<<" - "<<"deltaAng="<<deltaAng<<" - "<<"t="<<t<<endl;
  
  //Public for t second the angular velocity on the topic /command
  command.angular.z=angVel;

  beginTime = ros::Time::now();
  secondsIWantToSendMessagesFor = ros::Duration(t); 
  ros::Time endTime(secondsIWantToSendMessagesFor.toSec() + beginTime.toSec());
  while(ros::Time::now() < endTime ){
      mirPublisher.publish(command);
      ros::Duration(0.1).sleep();
  }
  command.angular.z=0.0;
  ros::spinOnce();
  loop_rate.sleep();
  //...

  //...end of second step

  //sleep(0.5);

  //Third step: reach the position
  linVel=VEL_LIN; //constant linear velocity

  distance = sqrt(pow(xf-desired_actual_x,2)+pow(yf-desired_actual_y, 2));  //compute distance between two points

  t=distance/linVel;  //compute the time which is necessary to travel for distance [distance] with velocity [linVel]

  //Public the linear velocity on the topic for t seconds
  command.linear.x=linVel;
  
  //desired generic coordinates of the mir
  x0d=desired_actual_x;
  y0d=desired_actual_y;
  th0d=th0;

  //cout<<"DISTANCE: "<<distance<<" TIME:"<<t<<endl;

  double cmdLin;
  double cmdAng;
  state_type xa(3); //actual generic coordinates of the mir

  for(double dt=0.0; dt<t; dt=dt+0.01){
    xa[0]=pose2d.x;
    xa[1]=pose2d.y;
    xa[2]=pose2d.theta;

    tie(cmdLin, cmdAng)=lineControl(xa, posD(dt), VEL_LIN, 0.0);

    //cout<<"DEBUG: velLineare="<<cmdLin<<" - velAngolare="<<cmdAng<<endl;

    command.linear.x = cmdLin;
    command.angular.z = cmdAng;

    mirPublisher.publish(command);

    ros::spinOnce();
    loop_rate.sleep();
  }
  command.linear.x=0.0;
  command.angular.z=0.0;
  mirPublisher.publish(command);
  ros::spinOnce();
  loop_rate.sleep();
  //...end of third step

  //sleep(0.5);

  //Fourth step reach thf
  //Compute the distance between the actual angle and the final angle
  actual_th=pose2d.theta;
  thfDeg=thf*(180.0/M_PI);
  actual_thDeg=actual_th*(180.0/M_PI);
  deltaAngDeg=(fmod((thfDeg-actual_thDeg+540.0),360.0))-180.0;
  deltaAng=deltaAngDeg*(M_PI/180.0);
  //...

  angVel=0.5; //constant angular velocity

  //Decide to turn clockwise or counter-clockwise
  if(deltaAng<0){
    angVel = -angVel;
  }
  //...

  t=abs(deltaAng/angVel); //necessary duration of the motion

  //Public for t seconds on the topic to achive the final configuration
  command.angular.z=angVel;
  beginTime = ros::Time::now();
  secondsIWantToSendMessagesFor = ros::Duration(t); 
  ros::Time endTime2(secondsIWantToSendMessagesFor.toSec() + beginTime.toSec());
  while(ros::Time::now() < endTime2 ){
      mirPublisher.publish(command);
      ros::Duration(0.1).sleep();
  }
  command.angular.z=0.0;
  ros::spinOnce();
  loop_rate.sleep();
  //...

  //Correct the trajectory by checking the actual position from odometry sensors
  actual_thDeg=pose2d.theta*(180.0/M_PI);
  deltaAngDeg=(fmod((thfDeg-actual_thDeg+540.0),360.0))-180.0;
  deltaAng=deltaAngDeg*(M_PI/180.0);

  error=abs(deltaAng);

  angVel=error;
  if(deltaAng<0){
    angVel = -angVel;
  }

  command.angular.z=angVel;
  //cout<<"actual_th="<<actual_th<<" - thf="<<thf<<" - deltaAng:"<<deltaAng<<endl;
  while(error>0.001){

    //cout<<"error: "<<error<<endl;

    mirPublisher.publish(command);
    ros::spinOnce();
    loop_rate.sleep();

    actual_th=pose2d.theta;
    actual_thDeg=actual_th*(180.0/M_PI);
    deltaAngDeg=(fmod((thfDeg-actual_thDeg+540.0),360.0))-180.0;
    deltaAng=deltaAngDeg*(M_PI/180.0);

    error=abs(deltaAng);

    angVel=(error>0.01) ? error*3.0 : 0.03;
    if(deltaAng<0){
      angVel = -angVel;
    }
    command.angular.z=angVel;
  }
  //Publish for 1 second angular velocity 0.0 to avoid strange behaviour of the wheels
  command.angular.z=0.0;
  mirPublisher.publish(command);
  beginTime = ros::Time::now();
  secondsIWantToSendMessagesFor = ros::Duration(1); 
  ros::Time endTimeC1(secondsIWantToSendMessagesFor.toSec() + beginTime.toSec());
  while(ros::Time::now() < endTimeC1 ){
      mirPublisher.publish(command);
      ros::Duration(0.1).sleep();
  }
  //...

  //...end of fourth step
}

//==========================================