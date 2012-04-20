/*
 * Shared constants header
 *  Used to define constants affecting robot navigation
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */
#include <math.h>
#ifndef __CONSTANTS__
#define __CONSTANTS__
#include <vector>
using namespace std;

typedef struct {
	float x;
	float y;
	float theta;
} pose;

typedef struct {
	vector<int> *moves_x;
	vector<int> *moves_y;
	int curr_x;
	int curr_y;
	int value;
} path;

float *ns_x_to_cm;
float *ns_y_to_cm;
float we_to_cm = -1.0;
float *ns_theta_offsets;

// Constants used for ROSIE
//Fully Charged
float rosie_ns_x_to_cm[] = {0.0, 0.0, 1.0 / 33.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 37.0};
float rosie_ns_y_to_cm[] = {0.0, 0.0, 1.0 / 40.0, 1.0 / 39.0, 1.0 / 25.0, 1.0 / 29.0};
float rosie_we_to_cm = 2.5; //0.5; 
float rosie_ns_theta_offsets[] = {0.0, 0.0,0.0, 85.0*M_PI/180, 0.0, 85.0*M_PI/180};

//Medium to Low Charged
//float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 37.0};
//float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 39.0, 1.0 / 39.0, 1.0 / 25.0, 1.0 / 37.0};
//float we_to_cm = 4.5; //0.5; 


// Constants used for BENDER

//Fully Charged
float bender_ns_x_to_cm[] = {0.0, 0.0, 1.0 / 34.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 37.0};
float bender_ns_y_to_cm[] = {0.0, 0.0, 1.0 / 34.0, 1.0 / 39.0, 1.0 / 25.0, 1.0 / 29.0};
float bender_we_to_cm = 5; //0.5; 
//NS theta skew correction constants
//float bender_ns_theta_offsets[] = {0.0, 0.0, /*0.155946*/ -12.0*M_PI/180 ,  M_PI_2, -0.101754, 1.488696};
float bender_ns_theta_offsets[] = {0.0, 0.0,0.0, 85.0*M_PI/180, 0.0, 85.0*M_PI/180};

// Constants used for JOHNNY5
//Fully Charged
//float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 39.0};
//float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 39.0, 1.0 / 37.0, 1.0 / 25.0, 1.0 / 37.0};
//float we_to_cm = 6.5; //0.5; 

//Low Charged
//float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 39.0};
//float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 39.0, 1.0 / 37.0, 1.0 / 25.0, 1.0 / 43.0};
//float we_to_cm = 6.5; //0.5; 

float uncertainty_normal [] = {.05, .05, .05, .05, .05, .05, .1, .1, .1};
float uncertainty_weak_we [] = {.08, .08, .08, .04, .04, .04, .1, .1, .1};
float uncertainty_weakest_we [] = {.1, .1, .1, .04, .04, .04, .1, .1, .1};
float uncertainty_weak_ns [] = {.06, .06, .06, .07, .07, .07, .12, .12, .12};
float uncertainty_weakest_ns [] = {.06, .06, .06, .1, .1, .1, .12, .12, .12};


// Constants used for GORT
// float ns_x_to_cm[] = {0.0, 0.0, 1.0 / 35.0, 1.0 / 30.0, 1.0 / 34.0, 1.0 / 37.0};
// float ns_y_to_cm[] = {0.0, 0.0, 1.0 / 39.0, 1.0 / 39.0, 1.0 / 25.0, 1.0 / 39.0};
//float we_to_cm = 4.5; //0.5; 

//NS theta skew correction constants
//float ns_theta_offsets[] = {0.0, 0.0, 0.155946, M_PI_2, -0.101754, 1.488696};

//Rotation for each room, to put the axis in robot frame
//float start_pose_thetas[] = {0.0, 0.0, 1.3554 - M_PI, -1.571900 - M_PI_2, 0.080450 - M_PI_2, -1.430050 - M_PI_2};

float start_pose_thetas[] = {0.0, 0.0, 1.1505996 - M_PI , -1.3431533 - M_PI_2, 0.080450 - M_PI_2, -1.5208379 - M_PI_2};

const float we_to_rad = M_PI / -120.0;
const float robot_diameter_cm = 29.0;

//Fir filter constant files
const char* coef_filename = "fir_coef/s_72";
const char* ns_theta_coef_filename = "fir_coef/s_75";



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
/*
float stateTurns[2][7][5] = {{{-M_PI_2, 0.0, -M_PI_2, M_PI, -M_PI_2},
			{-M_PI_2, 0, -M_PI_2, 0, -M_PI_2},
			{-M_PI_2, M_PI, -M_PI_2, 0.0, -M_PI_2},
			{-M_PI_2, 0, -M_PI_2, 0, -M_PI_2},
			{-M_PI_2, M_PI, -M_PI_2, 0.0, -M_PI_2},
			{-M_PI_2, 0, -M_PI_2, 0, -M_PI_2},
			{M_PI, M_PI, M_PI_2, 0.0, 0.0}},
			{{M_PI, M_PI, -M_PI_2, 0.0, 0.0},
			{M_PI_2, 0, M_PI_2, 0, M_PI_2},
			{M_PI_2, M_PI, M_PI_2, 0.0, M_PI_2},
			{M_PI_2, 0, M_PI_2, 0, M_PI_2},
			{M_PI_2, M_PI, M_PI_2, 0.0, M_PI_2},
			{M_PI_2, 0, M_PI_2, 0, M_PI_2},
			{M_PI_2, 0.0, M_PI_2, M_PI, M_PI_2}}};*/
float stateTurns[2][5][7] = 			
	{{{-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,3.141593},
	{0.000000,0.000000,3.141593,0.000000,3.141593,0.000000,3.141593},
	{-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,1.570796},
	{3.141593,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000},
	{-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,-1.570796,0.000000}},
	{{3.141593,1.570796,1.570796,1.570796,1.570796,1.570796,1.570796},
	{3.141593,0.000000,3.141593,0.000000,3.141593,0.000000,0.000000},
	{-1.570796,1.570796,1.570796,1.570796,1.570796,1.570796,1.570796},
	{0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,3.141593},
	{0.000000,1.570796,1.570796,1.570796,1.570796,1.570796,1.570796}}};

			

#endif
