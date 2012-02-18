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

const float we_to_rad = M_PI/-120;
const float we_to_cm = 2.1; //1.0/2.0
const float ns_x_to_cm = 1.0/30.0; //1.0/45
const float ns_y_to_cm = 1.0/42.0;
const float robot_diameter_cm = 29.0;
const char* coef_filename = "fir_coef/s_72";

const float ROOM2 = 1.3554-M_PI;
const float ROOM3 = -0.0019661-M_PI_2;
const float ROOM4 = 1.5953-M_PI_2;
const float ROOM5 = 0.041115-M_PI_2;
//Robot forward velocity at speed x in m/s
const float vel_1 = .333;
const float vel_3 = .3;
const float vel_5 = .25;
const float vel_7 = .2;
const float vel_10 = .2;
#endif
