#ifndef CALLBACK_DETECT_H
#define CALLBACK_DETECT_H

#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include <vector>

//===DETECTION AND LOCALIZATION LIBRARY
#include <robotic_vision/Detect.h>
#include <robotic_vision/DetectArray.h>
#include <robotic_vision/Localize.h>
//=================

//Definition of struct for a single detected obj
typedef struct DetectedObject{
    int64_t classe;
    double center_x;
    double center_y;
    double width;
    double height;
    double confidence;
} DetectedObject;

class Callback_detect{
    private:
        ros::NodeHandle n;
        ros::Subscriber subscriber;
        bool enabled;

    public:
        int numLegoDetected;
        std::vector<DetectedObject> detected;

        Callback_detect(ros::NodeHandle& _nh);
        void detect_callback(const robotic_vision::DetectArray::ConstPtr& msg);
        void disable();
        void enable();
        bool is_enable();
};
#endif
