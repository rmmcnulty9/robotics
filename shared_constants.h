/*
 * Some shared Constants/Structs across this Rob
 * */
#include <math.h>

 #ifndef __CONSTANTS__
 #define __CONSTANTS__
typedef struct {
    float x;
    float y;
    float theta;
} pose;


// all for rosie
float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 35.0, 1.0 / 35.0, 1.0 / 45.0};
float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 39.0, 1.0 / 41.0, 1.0 / 35.0, 1.0 / 38.0};
float ns_theta_offset[] = {0.0, 0.0, 0.155946, M_PI_2, -0.101754, 1.488696};
float ns_theta[] = {0.0, 0.0, 1.3554 - M_PI, -1.571900 - M_PI_2, 0.080450 - M_PI_2, -1.430050 - M_PI_2};

const float we_to_rad = M_PI / -120.0;
const float we_to_cm = 1.5; //0.5; 
const float robot_diameter_cm = 29.0;
const char* coef_filename = "fir_coef/s_72";

const float ROOM2 = 1.3554-M_PI;
const float ROOM3 = -0.0019661-M_PI_2;
const float ROOM4 = 1.5953-M_PI_2;
const float ROOM5 = 0.041115-M_PI_2;
//Robot forward velocity in cm/s
const float vel_1 = 33.3;
const float vel_3 = 30.0;
const float vel_5 = 25.0;
const float vel_7 = 20.0;
const float vel_10 = 20.0;
//Robot turn speed in radians/s
const float turn_1 = 3.142;
const float turn_3 = 2.827;
const float turn_5 = 2.094;
const float turn_7 = 1.257;
const float turn_10 = 1.047;
#endif
