#ifndef CALLBACK_POINTCLOUD_H
#define CALLBACK_POINTCLOUD_H

#include "ros/ros.h"
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Callback_pointcloud{
    private:
        ros::NodeHandle n;
        ros::Subscriber subscriber;
        bool enabled;

    public:
        sensor_msgs::PointCloud pointcloud;
        PointCloud myPointCloud;

        Callback_pointcloud(ros::NodeHandle& _nh);
        void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void disable();
        void enable();
        bool is_enable();
};
#endif
