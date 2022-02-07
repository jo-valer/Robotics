#include "robotic_vision/Callback_detect.h"

using namespace std;

Callback_detect::Callback_detect(ros::NodeHandle& _nh){
    n = _nh;
    subscriber = n.subscribe("/detect", 1, &Callback_detect::detect_callback, this);
    enabled = false;
}

void Callback_detect::detect_callback(const robotic_vision::DetectArray::ConstPtr& msg){
    if(enabled){
        DetectedObject dt;

        //We will clear the vector in case we have already read something from the topic
        detected.clear();

        //Read number of Lego detected
        numLegoDetected=msg->numLego;

        for (int i=0;i<numLegoDetected;i++){
            dt.classe=msg->detectedLego[i].classe;
            dt.center_x=msg->detectedLego[i].center_x;
            dt.center_y=msg->detectedLego[i].center_y;
            dt.width=msg->detectedLego[i].width;
            dt.height=msg->detectedLego[i].height;
            dt.confidence=msg->detectedLego[i].confidence;
            detected.push_back(dt);
            //cout<<"classe: "<<dt.classe<<" center_x: "<<dt.center_x<<" center_y: "<<dt.center_y<<" width: "<<dt.width<<" height: "<<dt.height<<endl;
        }
    }
}

void Callback_detect::disable(){
    this->enabled = false;
}

void Callback_detect::enable(){
    this->enabled = true;
}

bool Callback_detect::is_enable(){
    return enabled;
}