#include "robotic_vision/vision_library.h"

using namespace std;

vector<int> get_yolo_data(){   
    ifstream labels(FILE_YOLO_DATA);
    
    vector<int> output;

    int num_rows;
    int classe;
    double center_x;
    double center_y;
    double width;
    double height;
    double confidence;

    if (labels.is_open()){
        labels>>num_rows;
        if(num_rows>0){
            for(int i=0; i<num_rows; i++){
                labels>>classe>>center_x>>center_y>>width>>height>>confidence;
                output.push_back(classe);
            }
        }else{
            output.push_back(11);
        }
        labels.close();
    }else{
        cout << "ERROR: Unable to open file "<<FILE_YOLO_DATA<<endl;
        output.push_back(11);
    }

    return output;
}

std::tuple<int, int, double, double, double, double, double, int> get_localize_data(){
    ifstream localization(FILE_LOCALIZE_DATA);

    int pixel_x;
    int pixel_y;
    double center_x;
    double center_y;
    double orientation;
    double width;
    double height;
    int pose;
    
    if (localization.is_open()){
        localization>>pixel_x>>pixel_y>>center_x>>center_y>>orientation>>width>>height>>pose;
        localization.close();
    }else{
        cout << "ERROR: Unable to open file "<<FILE_LOCALIZE_DATA<<endl;
    }

    return std::make_tuple(pixel_x, pixel_y, center_x, center_y, orientation, width, height, pose);
}

int get_lego_property_x_localized(double w, double h){
    //Values associated to each X measure
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

std::tuple<int, int> get_lego_property4lying_x_and_surface(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud){
    double coord_height=0.0;
    double min = 100;
    int x=1;
    int surface=0;
    double height_limit=0.047;
    int start_x=pixel_x-80;
    int finish_x=pixel_x+80;
    int start_y=pixel_y-80;
    int finish_y=pixel_y+80;
    
    int i=0;
    int j=0;
    int found_i=0;
    int found_j=0;
    for(i=start_y; i<finish_y; i++){
        for(j=start_x; j<finish_x; j++){
            double point_height=matrixPointcloud[i][j];
            if(point_height>coord_height){
                coord_height=point_height;
                found_i=i;
                found_j=j;
            }
            if(point_height<min){
                min=point_height;
            }
            if(point_height>0.01){
                surface++;
            }
        }
    }

    //Try to read only the height of the center in order to avoid "seeing" other bricks
    coord_height=matrixPointcloud[pixel_y][pixel_x];

    //cout<<"min: "<<min<<"  ALTEZZA: "<<coord_height<<"  PIXEL ["<<found_i<<"]["<<found_j<<"]"<<endl;
    //cout<<"min: "<<min<<"  ALTEZZA: "<<coord_height<<endl;
    if(coord_height>height_limit){x=2;}else{x=1;}
    return std::make_tuple(x, surface);
}

std::tuple<int, int> get_lego_property_z_and_caps(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud){
    double coord_height=0.0;
    int z=1;
    int caps=0;
    double z_limit=0.0500;
    int points_above=0;
    int start_x=pixel_x-80;
    int finish_x=pixel_x+80;
    int start_y=pixel_y-80;
    int finish_y=pixel_y+80;
    for(int i=start_y; i<finish_y; i++){
        for(int j=start_x; j<finish_x; j++){
            double point_height=matrixPointcloud[i][j];
            if(point_height > coord_height){
                coord_height=point_height;
            }
            if(point_height > z_limit){
                points_above++;
            }
        }
    }
    sleep(2);
    //cout<<"HEIGHT BY COORDINATE: "<<coord_height<<endl;
    //cout<<"NUMBER OF POINTS IN CAPS: "<<points_above<<endl;
    if(coord_height > THRESHOLD_HEIGHT){z=2;}
    if(points_above > (740*1)-100){caps=1;}
    if(points_above > (740*2)-100){caps=2;}
    if(points_above > (740*3)-100){caps=3;}
    if(points_above > (740*4)-100){caps=4;}
    //cout<<"NUMBER OF CAPS: "<<caps<<endl;
    return std::make_tuple(z, caps);
}

int get_lego_property_ramp(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud){
    double centre_height = matrixPointcloud[pixel_y][pixel_x];
    //cout<<"HEIGHT FOR RAMP: "<<centre_height<<endl;
    if(centre_height < Z1_NO_RAMPS){return 0;}
    if(centre_height > TWINFILLET){return 2;}
    return 1;
}

void matrix_pointcloud(vector<vector<double>>& matrixPointcloud, PointCloud& pointcloud){
    int matrix_row=POINT_CLOUD_HEIGHT;
    int matrix_column=POINT_CLOUD_WIDTH;
    
    int r=0;
    int c=0;

    //cout<<"DIM PointCloud: "<<pointcloud.points.size()<<endl;

    for(int i=0 ; i<pointcloud.points.size(); ++i){

        if(r<1024){
            geometry_msgs::Point32 point;

            point.z = pointcloud.points[i].z;
            
            if(!isnan(point.z)){
                matrixPointcloud[r][c]=point.z;
            }else{
                matrixPointcloud[r][c]=-0.00001;
            }
            
            c++;

            if(c==1024){
                c=0;
                r++;
            }
        }
    }
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

int compute_class4standing(int actual_x_dim, int actual_y_dim, int lego_height, int lego_caps, int lego_ramps){
    //Class 0: X1_Y1_Z2
    if(actual_y_dim==1)                         {return 0;}
    //Class 9: X2_Y2_Z2
    else if(actual_x_dim==2 && lego_caps>2)     {return 9;}
    //Class 10: X2_Y2_Z2_FILLET
    else if(actual_x_dim==2)                    {return 10;}
    //Class 1: X1_Y2_Z1
    else if(actual_y_dim==2 && lego_height==1)  {return 1;}
    //Class 2: X1_Y2_Z2
    else if(actual_y_dim==2 && lego_caps>1)     {return 2;}
    //Class 3 - 4: X1_Y2_Z2_CHAMFER - X1_Y2_Z2_TWINFILLET
    else if(actual_y_dim==2){
        if(lego_ramps==1)                       {return 3;}
        else                                    {return 4;}
    }

    //Class 5: X1_Y3_Z2
    else if(actual_y_dim==3 && lego_caps>1)     {return 5;}
    //Class 6: X1_Y3_Z2_FILLET
    else if(actual_y_dim==3)                    {return 6;}
    //Class 7: X1_Y4_Z1
    else if(actual_y_dim==4 && lego_height==1)  {return 7;}
    //Class 8: X1_Y4_Z2
    else if(actual_y_dim==4)                    {return 8;}

    else{
        //YOLO is needed
        return 11;
    }
}

int compute_class4lying(int lego_dim_x, int lego_surface){
    //Class 9 - 10: X2_Y2_Z2 - X2_Y2_Z2_FILLET
    if(lego_dim_x==2){
        if(lego_surface>4205)     {return 9;}
        else                      {return 10;}
    }
    //Class 3: X1_Y2_Z2_CHAMFER
    else if(lego_surface<3467)    {return 3;}
    //Class 4: X1_Y2_Z2_TWINFILLET
    else if(lego_surface<3976)    {return 4;}
    //Class 2: X1_Y2_Z2
    else if(lego_surface<4632)    {return 2;}
    //Class 6: X1_Y3_Z2_FILLET
    else if(lego_surface<5765)    {return 6;}
    //Class 5: X1_Y3_Z2
    else if(lego_surface<7736)    {return 5;}
    //Class 8: X1_Y4_Z2
    else                          {return 8;}
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
  int img_x;
  int img_y;
  double center_x;
  double center_y;
  double orientation;
  double width;
  double height;
  int ps;
  tie(img_x, img_y, center_x, center_y, orientation, width, height, ps) = get_localize_data();
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
      yolo_output = get_yolo_data();

      int prevClass = -1;
      for(int yolo_output_classe : yolo_output){
        if(yolo_output_classe != prevClass){

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

  return mostFrequentClass;
}