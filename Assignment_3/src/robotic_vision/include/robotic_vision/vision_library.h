#ifndef VISION_LIBRARY_H
#define VISION_LIBRARY_H
#include <iostream>
#include <fstream>
#include <tuple>
#include <vector>

#include "robotic_vision/Callback_detect.h"
#include "robotic_vision/Callback_localization.h"
#include "mobile_control_library/Callback_odometry.h"

//===POINTCLOUD LIBRARIES===
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//==========================

#define FILE_YOLO_DATA "src/mir_controller/src/labels.txt"
#define FILE_LOCALIZE_DATA "src/mir_controller/src/localization.txt"

//Point cloud output size
#define POINT_CLOUD_WIDTH 1024
#define POINT_CLOUD_HEIGHT 1024

#define LOW_LEGO_HEIGHT 0.0386 //approx height of lower lego bricks (i.e. with z1)
#define HIGH_LEGO_HEIGHT 0.0571 //approx height of higher lego bricks (i.e. with z2)
#define THRESHOLD_HEIGHT 0.0478 //height threshold to discriminate between classes (Z1 vs Z2 and also FILLET vs NO_FILLET)

//Height of lego centers
#define Z1_NO_RAMPS 0.0220 //if the lego is lower than this, then is Z1-no-ramps
#define ONE_RAMP 0.0350 //if the lego is lower than this, then has 1 ramp (Y3)
#define TWINFILLET 0.0550 //if the lego is higher than this, then it's twinfillet

/**
 * @brief 
 * Read Yolo output from file FILE_YOLO_DATA
 */
std::vector<int> get_yolo_data();


/**
 * @brief
 * Read localization output from file FILE_LOCALIZE_DATA
 */
std::tuple<int, int, double, double, double, double, double, int> get_localize_data();

/**
 * @brief
 * Get the lego x dimension computed by the localization process
 */
int get_lego_property_x_localized(double w, double h);

/**
 * @brief
 * Get the lego y dimension computed by the localization process
 */
int get_lego_property_y_localized(double w, double h);

/**
 * @brief
 * Get the lego x dimension when the lego is lying, found with the point cloud from the depth camera
 */
std::tuple<int, int> get_lego_property4lying_x_and_surface(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud);

/**
 * @brief
 * Get the lego z dimension and the number of caps the lego has, all found with the point cloud from the depth camera
 */
std::tuple<int, int> get_lego_property_z_and_caps(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud);

/**
 * @brief
 * Get the number of caps that the lego has, found with the point cloud from the depth camera
 */
int get_lego_property_ramp(int pixel_x, int pixel_y, std::vector<std::vector<double>>& matrixPointcloud);

/**
 * @brief 
 * Convert point cloud to a 1024x1024 matrix
 * @param matrixPointcloud output matrix passed as reference
 */
void matrix_pointcloud(std::vector<std::vector<double>>& matrixPointcloud, PointCloud& pointcloud);

/**
 * @brief 
 * Filter on y dimension
 * @param yolo_output classes recognized by yolo
 * @param actual_y_dim actual y_dim computed during the localization process
 * 
 * For each element of yolo_output:
 *  - If actual_y_dim == Y_dim of class yolo_output[i] --> yolo_output[i]+=1
 *  - If actual_y_dim != Y_dim of class yolo_output[i] --> yolo_output[i]=0
 */
void filter_dim_y(std::vector<int>& yolo_output, int correct_y);

/**
 * @brief Get the lego properties object
 * returns main lego THEORETICAL properties:
 *     - X,Y,Z dimensions (for example lego of class 0 is x=1 y=1 z=2)
 */
std::tuple<int, int, int> get_lego_properties(int lego_int);

/**
 * @brief
 * Returns the lego class. It has to be called for bricks standing
 * @param actual_x_dim dimension X of the brick
 * @param actual_y_dim dimension Y of the brick
 * @param lego_height height of the brick
 * @param lego_caps feature "number of caps" of the brick
 * @param lego_ramps feature "number of ramps" of the brick
 */
int compute_class4standing(int actual_x_dim, int actual_y_dim, int lego_height, int lego_caps, int lego_ramps);

/**
 * @brief
 * Returns the lego class. It has to be called for bricks lying
 * @param lego_dim_x dimension X of the brick
 * @param lego_surface of the brick at specified height
 */
int compute_class4lying(int lego_dim_x, int lego_surface);

/**
 * @brief 
 * To achive better results, the procedure of object recognition and classification is done from several point of view
 * Return the class recognized by YOLO
 */
int yolo_scan(ros::Rate& loop_rate, Callback_odometry& pose2d, Callback_localization& localization_data, Callback_detect& detection_data, ros::Publisher& mirPublisher);
/**---*/

#endif
