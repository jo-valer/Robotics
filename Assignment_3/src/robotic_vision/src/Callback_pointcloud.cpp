#include "robotic_vision/Callback_pointcloud.h"

using namespace std;

Callback_pointcloud::Callback_pointcloud(ros::NodeHandle& _nh){
    n = _nh;
    subscriber = n.subscribe("/camera_up/depth/points", 1, &Callback_pointcloud::point_cloud_callback, this);
    enabled = false;
    new_msg = false;
}

void Callback_pointcloud::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  if(enabled){
    //Convert Pointcloud2 to Pointcloud
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pointcloud);
    
    //pcl::fromPCLPointCloud2(*msg, myPointCloud); 

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,myPointCloud);
    new_msg = true;
  }
}

void Callback_pointcloud::disable(){
    this->enabled = false;
}

void Callback_pointcloud::enable(){
    this->enabled = true;
}

bool Callback_pointcloud::is_enable(){
    return enabled;
}

bool Callback_pointcloud::message_ready(){
  return new_msg;
}
        
void Callback_pointcloud::reset_new_message(){
  new_msg = false;
}