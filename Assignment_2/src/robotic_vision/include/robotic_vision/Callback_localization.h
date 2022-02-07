#ifndef CALLBACK_LOCALIZATION_H
#define CALLBACK_LOCALIZATION_H

#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include <vector>

//===DETECTION AND LOCALIZATION LIBRARY
#include <robotic_vision/Detect.h>
#include <robotic_vision/DetectArray.h>
#include <robotic_vision/Localize.h>
//=================

//Definition of struct for a single localized obj
typedef struct LocalizedObject{ 
    int64_t img_x;
    int64_t img_y;
    double x;
    double y;
    double q;
    double w;
    double h;
} LocalizedObject;

class Callback_localization{
    private:
        ros::NodeHandle n;
        ros::Subscriber subscriber;
        bool enabled;

    public:
        int numLegoLocalized;
        std::vector<LocalizedObject> localized;

        Callback_localization(ros::NodeHandle& _nh);
        void localize_callback(const robotic_vision::Localize::ConstPtr& msg);
        void disable();
        void enable();
        bool is_enable();
};
#endif
