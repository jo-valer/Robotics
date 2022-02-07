#include "kinematic_library/ur5Kinematic.h"
#include <complex.h>
using namespace Eigen;
using namespace std;

void eul2rotm(double eul[3], Matrix3d& rotMatrix){
     double ct[3] = {cos(eul[0]), cos(eul[1]), cos(eul[2])};
     double st[3] = {sin(eul[0]), sin(eul[1]), sin(eul[2])};
     rotMatrix(0,0)=ct[1]*ct[0];
     rotMatrix(0,1)=st[2]*st[1]*ct[0]-ct[2]*st[0];
     rotMatrix(0,2)=ct[2]*st[1]*ct[0]+st[2]*st[0];
     rotMatrix(1,0)=ct[1]*st[0];
     rotMatrix(1,1)=st[2]*st[1]*st[0]+ct[2]*ct[0];
     rotMatrix(1,2)=ct[2]*st[1]*st[0]-st[2]*ct[0];
     rotMatrix(2,0)=-st[1];
     rotMatrix(2,1)=st[2]*ct[1];
     rotMatrix(2,2)=ct[2]*ct[1];
}

void rotm2eul(double eul[3], Matrix3d& rotMatrix){
     double sy = sqrt(rotMatrix(0,0)*rotMatrix(0,0)+rotMatrix(1,0)*rotMatrix(1,0));
     bool singular = sy < 1e-6;

     float x,y,z;
     if(!singular){
          x=atan2(rotMatrix(2,1),rotMatrix(2,2));
          y=atan2(-rotMatrix(2,0),sy);
	     z=atan2(rotMatrix(1,0),rotMatrix(0,0));
     }else{
          x=atan2(-rotMatrix(1,2), rotMatrix(1,1));
	     y=atan2(-rotMatrix(2,0), sy);
          z=0;
     }

     //Return roll-pitch-yaw values
     eul[0]=z;
     eul[1]=y;
     eul[2]=x;
}

void ur5Direct(Ur5JointConfiguration &jointConfiguration, EndEffector &endEffector){
    /**DH Parameters*/
    double a1=MyUrd5::a1;
    double a2=MyUrd5::a2;
    double a3=MyUrd5::a3;
    double a4=MyUrd5::a4;
    double a5=MyUrd5::a5;
    double a6=MyUrd5::a6;
    double d1=MyUrd5::d1;
    double d2=MyUrd5::d2;
    double d3=MyUrd5::d3;
    double d4=MyUrd5::d4;
    double d5=MyUrd5::d5;
    double d6=MyUrd5::d6;
    /**---*/

    double th1=jointConfiguration.getTh1();
    double th2=jointConfiguration.getTh2();
    double th3=jointConfiguration.getTh3();
    double th4=jointConfiguration.getTh4();
    double th5=jointConfiguration.getTh5();
    double th6=jointConfiguration.getTh6();

    Matrix4d T10;
    Matrix4d T21;
    Matrix4d T32;
    Matrix4d T43;
    Matrix4d T54;
    Matrix4d T65;

    Matrix4d T06;

    T10<<cos(th1), -sin(th1), 0.0, 0.0,
         sin(th1), cos(th1), 0.0, 0.0,
         0.0, 0.0, 1.0, d1,
         0.0, 0.0, 0.0, 1.0;
    
    T21<<cos(th2), -sin(th2), 0.0, 0.0,
         0.0, 0.0, -1.0, 0.0,
         sin(th2), cos(th2), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    T32<<cos(th3), -sin(th3), 0.0, a2,
         sin(th3), cos(th3), 0.0, 0.0,
         0.0, 0.0, 1.0, d3,
         0.0, 0.0, 0.0, 1.0;

    T43<<cos(th4), -sin(th4), 0.0, a3,
         sin(th4), cos(th4), 0.0, 0.0,
         0.0, 0.0, 1.0, d4,
         0.0, 0.0, 0.0, 1.0;
    
    T54<<cos(th5), -sin(th5), 0.0, 0.0,
         0.0, 0.0, -1.0, -d5,
         sin(th5), cos(th5), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    T65<<cos(th6), -sin(th6), 0.0, 0.0,
         0.0, 0.0, 1.0, d6,
         -sin(th6), -cos(th6), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    T06=T10*T21*T32*T43*T54*T65;

    endEffector.setPostion(T06(0,3), T06(1,3), T06(2,3));
    Matrix3d rotMatrix = T06.block<3,3>(0, 0);
    endEffector.setRotationMatrix(rotMatrix);

    double RPY[3];
    rotm2eul(RPY, rotMatrix);
    endEffector.setOrientation(RPY[0],RPY[1],RPY[2]);
}

void ur5Inverse(Ur5JointConfiguration &jointConfiguration, EndEffector &endEffector){
    complex<double> complexConverter(1.0,0.0);

    /**DH Parameters*/
    double a1=MyUrd5::a1;
    double a2=MyUrd5::a2;
    double a3=MyUrd5::a3;
    double a4=MyUrd5::a4;
    double a5=MyUrd5::a5;
    double a6=MyUrd5::a6;
    double d1=MyUrd5::d1;
    double d2=MyUrd5::d2;
    double d3=MyUrd5::d3;
    double d4=MyUrd5::d4;
    double d5=MyUrd5::d5;
    double d6=MyUrd5::d6;
    /**---*/
    
    
    Matrix3d rotMatrix = endEffector.getRotationMatrix();
    Position pos = endEffector.getPosition();

    Matrix4d T60;
    T60.block<3,3>(0,0)=rotMatrix;
    T60(3, 0)=0;
    T60(3, 1)=0;
    T60(3, 2)=0;
    T60(3, 3)=1;
    T60(0, 3)=pos.getX();
    T60(1, 3)=pos.getY();
    T60(2, 3)=pos.getZ();

    //Finding th1
    Matrix<double, 4, 1> c;   //support variable
    c(0,0)=0.0;c(1,0)=0.0;c(2,0)=-d6;c(3,0)=1.0;
    Matrix<double, 4, 1> p50;
    p50=T60*c;
    double th1=real(atan2(p50(1,0), p50(0,0))*complexConverter + acos( d4/( sqrt( pow(p50(1,0),2)+pow(p50(0,0),2) ) ))*complexConverter + M_PI_2);

    //Finding th5
    double p60[3]={pos.getX(), pos.getY(), pos.getZ()};
    double th5=real(acos( ( p60[0]*sin(th1) - p60[1]*cos(th1) - d4 )/d6*complexConverter));

    //Finding th6
    Matrix4d T06 = T60.inverse();
    Matrix<double, 3, 1> Xhat = T06.block<3,1>(0,0);
    Matrix<double, 3, 1> Yhat = T06.block<3,1>(0,1);
    double th6=real(atan2( ( -Xhat(1,0)*sin(th1)+Yhat(1,0)*cos(th1) )/sin(th5) , (Xhat(0,0)*sin(th1)-Yhat(0,0)*cos(th1))/sin(th5))*complexConverter);

    //Finding th3
    Matrix4d T10;
    Matrix4d T54;
    Matrix4d T65;
    T10<<cos(th1), -sin(th1), 0.0, 0.0,
         sin(th1), cos(th1), 0.0, 0.0,
         0.0, 0.0, 1.0, d1,
         0.0, 0.0, 0.0, 1.0;
    T65<<cos(th6), -sin(th6), 0.0, 0.0,
         0.0, 0.0, 1.0, d6,
         -sin(th6), -cos(th6), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    T54<<cos(th5), -sin(th5), 0.0, 0.0,
         0.0, 0.0, -1.0, -d5,
         sin(th5), cos(th5), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    Matrix4d T41m=T10.inverse()*T60*T65.inverse()*T54.inverse();
    Matrix<double, 3, 1> p41_1 = T41m.block<3,1>(0,3);
    double p41xz_1=sqrt(pow(p41_1(0,0),2)+pow(p41_1(2,0),2));

    double th3=real(acos(( pow(p41xz_1, 2) - pow(a2, 2) - pow(a3, 2) )/( 2*a2*a3 )*complexConverter));

    //Finding th2
    double th2=real(atan2(-p41_1(2,0) , -p41_1(0,0))*complexConverter-asin(( -a3*sin(th3) )/p41xz_1*complexConverter)*complexConverter);

    //Finding th4
    Matrix4d T32;
    Matrix4d T21;
    T32<<cos(th3), -sin(th3), 0.0, a2,
         sin(th3), cos(th3), 0.0, 0.0,
         0.0, 0.0, 1.0, d3,
         0.0, 0.0, 0.0, 1.0;
    T21<<cos(th2), -sin(th2), 0.0, 0.0,
         0.0, 0.0, -1.0, 0.0,
         sin(th2), cos(th2), 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    Matrix4d T43m=T32.inverse()*T21.inverse()*T10.inverse()*T60*T65.inverse()*T54.inverse();
    Matrix<double, 3, 1> Xhat43 = T43m.block<3,1>(0,0);
     
    double th4 = real(atan2(Xhat43(1), Xhat43(0))*complexConverter);

    //Output: Set the joint configuration
    double outputAngles[6]={th1, th2, th3, th4, th5, th6};
    jointConfiguration.setAngles(outputAngles);
}


void point2point_motion_plan(EndEffector &initialPose, double x, double y, double z, double input_roll, double input_pitch, double input_yaw, double t_max, ros::Rate& loop_rate, ros::Publisher (&joint_com_pub) [6]){
  x = -1*x;
  //y = (-1*y)-0.0823;
  y = -1*y;
  z = z;

  //cout<<"Modified: "<<"x: "<<x<<"; y="<<y<<"; z="<<z<<endl;

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
		for(int i=0; i<JOINTS; i++) {
			position[i].data = tmpJointConfigutationAngles[i];
			joint_com_pub[i].publish(position[i]);
		}

    //Using direct kinematic compute the end effector position and orientation (RPY)
    //EndEffector tmpEndEffectorPose;
    ur5Direct(tmpJointConfiguration, tmpEndEffectorPose);
    /*
    cout<<"POINT:"<<endl;
    cout<<"Position: "<<endl;
    cout<<" x: "<<tmpEndEffectorPose.getPosition().getX()<<endl;
    cout<<" y: "<<tmpEndEffectorPose.getPosition().getY()<<endl;
    cout<<" z: "<<tmpEndEffectorPose.getPosition().getZ()<<endl;
    cout<<" R: "<<tmpEndEffectorPose.getOrientation().getR()<<endl;
    cout<<" P: "<<tmpEndEffectorPose.getOrientation().getP()<<endl;
    cout<<" Y: "<<tmpEndEffectorPose.getOrientation().getY()<<endl;
    cout<<"---"<<endl;
    */
     ros::spinOnce();
     loop_rate.sleep();
   }

    //Get new initial pose from the last joint configuration computed
    ur5Direct(tmpJointConfiguration, initialPose);
}

void gripper_controller(double grip, ros::Publisher& gripperTopic){
  std_msgs::Float64 position;
  control_msgs::GripperCommandActionGoal gripperValue;
  gripperValue.goal.command.position=grip;
  gripperTopic.publish(gripperValue);
}
