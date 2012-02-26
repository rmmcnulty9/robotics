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


// Constants used for Rosie
//North Star scalings for each room
float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 32.0};
float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 40.0, 1.0 / 41.0, 1.0 / 25.0, 1.0 / 37.0};

//NS theta skew correction constants
float ns_theta_offsets[] = {0.0, 0.0, 0.155946, M_PI_2, -0.101754, 1.488696};

//Rotation for each room, to put the axis in robot frame
float start_pose_thetas[] = {0.0, 0.0, 1.3554 - M_PI, -1.571900 - M_PI_2, 0.080450 - M_PI_2, -1.430050 - M_PI_2};

const float we_to_rad = M_PI / -120.0;
float we_to_cm = 1.5; //0.5; 
const float robot_diameter_cm = 29.0;

const char* coef_filename = "fir_coef/s_72";
const char* ns_theta_coef_filename = "fir_coef/s_73";

const float MOVE_TO_EPSILON = 10.0;
float TURN_TO_EPSILON = 25.0*(M_PI/180);

/*
 * PID Controller constants
 */

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


const double iMax = 10.0;
const double iMin = -10.0;
const double integral = 0.05;
const double proportional = 0.5;
const double derivative = 1.0;

/*
 * Use different uncertainties when the NS signal is weak. No significant difference
 */
//float uncertainty_default [] = {.05, .05, .05, .05, .05, .05, .05, .05, .05};
//float uncertainty_weak_ns [] = {.03, .03, .03, .1, .1, .1, .04, .04, .04};

#endif
