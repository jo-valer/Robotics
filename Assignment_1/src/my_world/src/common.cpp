#include "my_world/common.h"

using namespace std;

string get_lego_string(int lego_int){
    string lego_string;
    switch (lego_int){
        case 0: lego_string = "X1_Y1_Z2"; break;
        case 1: lego_string = "X1_Y2_Z1"; break;
        case 2: lego_string = "X1_Y2_Z2"; break;
        case 3: lego_string = "X1_Y2_Z2_CHAMFER"; break;
        case 4: lego_string = "X1_Y2_Z2_TWINFILLET"; break;
        case 5: lego_string = "X1_Y3_Z2"; break;
        case 6: lego_string = "X1_Y3_Z2_FILLET"; break;
        case 7: lego_string = "X1_Y4_Z1"; break;
        case 8: lego_string = "X1_Y4_Z2"; break;
        case 9: lego_string = "X2_Y2_Z2"; break;
        case 10: lego_string = "X2_Y2_Z2_FILLET"; break;
        default: lego_string = "UNKNOWN";
    }
    return lego_string;
}
