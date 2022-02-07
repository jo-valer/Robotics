#include "robotic_vision/vision_library.h"

using namespace std;

vector<int> get_yolo_data(Callback_detect& detection_data){   
    vector<int> output;

    if(detection_data.numLegoDetected==0){
        output.push_back(11);
    }else{
        for(DetectedObject dob : detection_data.detected){
            output.push_back(dob.classe);
        }
    }

    return output;
}

std::tuple<int, int, double, double, double, double, double> get_localize_data(){
    ifstream localization(FILE_LOCALIZE_DATA);

    int pixel_x;
    int pixel_y;
    double center_x;
    double center_y;
    double orientation;
    double width;
    double height;
    
    if (localization.is_open()){
        localization>>pixel_x>>pixel_y>>center_x>>center_y>>orientation>>width>>height;
        localization.close();
    }else{
        cout << "ERROR: Unable to open file "<<FILE_LOCALIZE_DATA<<endl;
    }

    return std::make_tuple(pixel_x, pixel_y, center_x, center_y, orientation, width, height);
}

int get_lego_property_x_localized(double w, double h){
    //Values associated to each X measure
    //TODO: riflettere se int o double
    double x1=35.0;
    double x2=72.0;
    
    //Check X2 : X2 is only when the lego is 2x2.
    //For all the other classes X property is equal to 1
    if(abs(x2-w)<5 && abs(x2-h)<5){
        return 2;
    }
    //...the obj is not of X2

    //If the object has not X property equal to 2 so the object has X
    //propery equal to 1
    
    return 1;
}

int get_lego_property_y_localized(double w, double h){
    //Values associated to each Y measure
    //TODO: riflettere se int o double
    double y1=35.0;
    double y2=72.0;
    double y3=108.0;
    double y4=144.0;
    
    //Check Y1: ad hoc case because x and y are both 35 more or less
    if(abs(y1-w)<5 && abs(y1-h)<5){
        return 1;
    }
    //...the obj is not of Y1

    double measures[3]={y2, y3, y4};

    for(int i=0; i<3; i++){
        if(abs(measures[i]-w)<5 || abs(measures[i]-h)<5){
            return i+2;
        }
    }

    return -1;
}

std::tuple<int, int> get_lego_property_z_and_caps(int pixel_x, int pixel_y, PointCloud& rotated_pointcloud){
    double coord_height=0.0;
    int z=1;
    int caps=0;
    double z_limit=0.0500;
    int points_above=0;
    int start_point=((pixel_y-100)*1024)+pixel_x; //l'inizio è 50 righe prima del centro del lego, la fine è un punto a caso , ma dopo aver letto 50 righe
    for(int i = start_point ; i < start_point + (200*1024); ++i){
        geometry_msgs::Point32 point;
        
        point.x = rotated_pointcloud.points[i].x;
        point.y = rotated_pointcloud.points[i].y;
        point.z = rotated_pointcloud.points[i].z;

        if(!isnan(point.z)){
            if(point.z > coord_height){
                coord_height=point.z;
            }
            if(point.z > z_limit){
                points_above++;            }
        }
        
        //logFile<<"POINT: ["<<point.x<<", "<<point.y<<", "<<point.z<<"]"<<endl;
    }
    sleep(2);

    if(coord_height > THRESHOLD_HEIGHT){z=2;}
    if(points_above > (740*1)-100){caps=1;}
    if(points_above > (740*2)-100){caps=2;}
    if(points_above > (740*3)-100){caps=3;}
    if(points_above > (740*4)-100){caps=4;}

    return std::make_tuple(z, caps);
}

int get_lego_property_ramp(int pixel_x, int pixel_y, PointCloud& rotated_pointcloud){
    int centre_point = (pixel_y*1024)+pixel_x; //è il centro del lego (in pixel)
    double centre_height = rotated_pointcloud.points[centre_point].z;

    if(centre_height < Z1_NO_RAMPS){return 0;}
    if(centre_height > TWINFILLET){return 2;}
    return 1;
}

void filter_dim_y(std::vector<int>& yolo_output, int correct_y){
    for(int i=0; i<(yolo_output.size()); i++){
        //Get class property
        int dim_x, dim_y, dim_z;
        tie(dim_x, dim_y, dim_z) = get_lego_properties(i);

        if(correct_y != dim_y){
            yolo_output[i]=0;
        }else{
            yolo_output[i]+=1;
        }
    }
}

std::tuple<int, int, int> get_lego_properties(int lego_int){

    int x_dim, y_dim, z_dim;

    switch(lego_int){
        case 0:
            x_dim=1; y_dim=1; z_dim=2;
            break;
        case 1:
            x_dim=1; y_dim=2; z_dim=1;
            break;
        case 2:
            x_dim=1; y_dim=2; z_dim=2;
            break;
        case 3:
            x_dim=1; y_dim=2; z_dim=2;
            break;
        case 4:
            x_dim=1; y_dim=2; z_dim=2;
            break;
        case 5:
            x_dim=1; y_dim=3; z_dim=2;
            break;
        case 6:
            x_dim=1; y_dim=3; z_dim=2;
            break;
        case 7:
            x_dim=1; y_dim=4; z_dim=1;
            break;
        case 8:
            x_dim=1; y_dim=4; z_dim=2;
            break;
        case 9:
            x_dim=2; y_dim=2; z_dim=2;
            break;
        case 10:
            x_dim=2; y_dim=2; z_dim=2;
            break;
    }

    return std::make_tuple(x_dim, y_dim, z_dim);
}

int yolo_scan(ros::Rate& loop_rate, Callback_odometry& pose2d, Callback_localization& localization_data, Callback_detect& detection_data, ros::Publisher& mirPublisher){

  //Assumption: when this procedure is needed robot's rotation is always pi/2

  int numClasses = 11;
  double ofset = 0.15; //how much to turn left and right in radiants

  //Define sequence of movements:
  double moves[3] = {
    (M_PI/2)-ofset, //go left
    (M_PI/2)+ofset, //go right
    M_PI/2          //reset position
  };

  //Support variables
  geometry_msgs::Twist command;
  double actual_th;
  double actual_thDeg;
  double thf;
  double thfDeg;
  double deltaAng;
  double deltaAngDeg;
  double angVel;
  double t;
  ros::Time beginTime;
  ros::Duration secondsIWantToSendMessagesFor;
  int counter=0;
  int x_dim, y_dim, z_dim;

  //YOLO output
  vector<int> yolo_output;

  //lego_class_frequency[i] count how many times yolo says that the lego belongs to class i
  vector<int> lego_class_frequency(numClasses+1, 0);

  //Read lego x,y,z dimensions from upper camera data
  int pixel_x;
  int pixel_y;
  double center_x;
  double center_y;
  double orientation;
  double width;
  double height;
  //tie(pixel_x, pixel_y, center_x, center_y, orientation, width, height) = get_localize_data();
  pixel_x = localization_data.localized[0].img_x;
  pixel_y = localization_data.localized[0].img_y;
  center_x = localization_data.localized[0].x;
  center_y = localization_data.localized[0].y;
  orientation = localization_data.localized[0].q; 
  width = localization_data.localized[0].w; 
  height = localization_data.localized[0].h;
  //cout<<"DEBUG[localization] center_x: "<<center_x<<" - center_y: "<<center_y<<" - orientation: "<<orientation<<" - width: "<<width<<" - height: "<<height<<endl;

  //Get a comparable lego property [y dim] computed by the localization process
  int actual_y_dim = get_lego_property_y_localized(width, height);
  //cout<<"DEBUG[actual_y_dim]: "<<actual_y_dim<<endl;

  for(int i=0; i<3; i++){

    thf=moves[i]; //next move

    //ROTATION

    //Compute the distance between the actual angle and the final angle
    actual_th=pose2d.theta;
    thfDeg=thf*(180.0/M_PI);
    actual_thDeg=actual_th*(180.0/M_PI);
    deltaAngDeg=(fmod((thfDeg-actual_thDeg+540.0),360.0))-180.0;
    deltaAng=deltaAngDeg*(M_PI/180.0);
    //...

    angVel=2; //constant angular velocity

    //Decide to turn clockwise or counter-clockwise
    if(deltaAng<0){
      angVel = -angVel;
    }
    //...

    command.angular.z=angVel;
    t=abs(deltaAng/angVel); //necessary duration of the motion

    for(double dt=0.0; dt<t; dt=dt+0.01){
      //Read yolo output
      //do{
        yolo_output = get_yolo_data(detection_data);
      //}while(yolo_output.size()==0);  //while yolo doesn't recognize anything, wait

      int prevClass = -1;
      for(int yolo_output_classe : yolo_output){
        if(yolo_output_classe != prevClass){            //#TODO: solve this problem in python 

          //Get properties of lego class yolo_output_classe
          tie(x_dim, y_dim, z_dim) = get_lego_properties(yolo_output_classe);

          if(y_dim == actual_y_dim){  //the object recognized by yolo has the correct y dimension
            lego_class_frequency[yolo_output_classe]+=1;
            counter++;
            //cout<<"CLASSE: "<<yolo_output_classe<<endl;
            prevClass=yolo_output_classe;
          }
        }
      }
      
      mirPublisher.publish(command);
      ros::spinOnce();
      loop_rate.sleep();
    }
    command.angular.z=0.0;
    mirPublisher.publish(command);
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  
  //Take into consideration only lego with correct Y size
  filter_dim_y(lego_class_frequency, actual_y_dim);

  int mostFrequentClass = std::max_element(lego_class_frequency.begin(),lego_class_frequency.end()) - lego_class_frequency.begin();
  double percentage = (100.0*lego_class_frequency[mostFrequentClass])/(counter*1.0);

  //cout<<"YOLO FINAL DECISION: "<<mostFrequentClass<<" PERCENTAGE: "<<percentage<<endl;

  /*
  for(int i=0; i<NUM_CLASSES; i++){
    cout<<"classe["<<i<<"]: "<<lego_class_frequency[i]<<endl;
  }
  */

  return mostFrequentClass;
}