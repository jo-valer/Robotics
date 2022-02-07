#include "robotic_vision/Callback_localization.h"

using namespace std;

Callback_localization::Callback_localization(ros::NodeHandle& _nh){
    n = _nh;
    subscriber = n.subscribe("/localize", 1, &Callback_localization::localize_callback, this);
    enabled = false;
}

void Callback_localization::localize_callback(const robotic_vision::Localize::ConstPtr& msg){
    if(enabled){
        LocalizedObject objL1;
        LocalizedObject objL2;
        LocalizedObject objL3;

        //We will clear the vector in case we have already read something from the topic
        localized.clear();
        
        //Read number of Lego localized
        numLegoLocalized= msg->numLego;

        //Read lego_1 properties from topic localize
        objL1.img_x = msg->lego1_imgx;
        objL1.img_y = msg->lego1_imgy;
        objL1.x = msg->lego1_x;
        objL1.y = msg->lego1_y;
        objL1.q = msg->lego1_q;
        objL1.w = msg->lego1_w;
        objL1.h = msg->lego1_h;
        localized.push_back(objL1);
        
        //Read lego_2 properties from topic localize
        objL2.img_x = msg->lego2_imgx;
        objL2.img_y = msg->lego2_imgy;
        objL2.x = msg->lego2_x;
        objL2.y = msg->lego2_y;
        objL2.q = msg->lego2_q;
        objL2.w = msg->lego2_w;
        objL2.h = msg->lego2_h;
        localized.push_back(objL2);
        
        //Read lego_3 properties from topic localize
        objL3.img_x = msg->lego3_imgx;
        objL3.img_y = msg->lego3_imgy;
        objL3.x = msg->lego3_x;
        objL3.y = msg->lego3_y;
        objL3.q = msg->lego3_q;
        objL3.w = msg->lego3_w;
        objL3.h = msg->lego3_h;
        localized.push_back(objL3);
    }
}


void Callback_localization::disable(){
    this->enabled = false;
}

void Callback_localization::enable(){
    this->enabled = true;
}

bool Callback_localization::is_enable(){
    return enabled;
}