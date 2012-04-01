/*
 * RobotPose Header
 *  Used to define functions and constants 
 *  to collect, filter, and interpret robot NS and WE data
 *  and move robot around maze
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */


#ifndef __ROBOT_POSE__
#define __ROBOT_POSE__

#include "shared_constants.h"
#include "PIDController.h"
#include "CameraPose.h"


extern "C" {
	#include <clapack.h>
	#include "Kalman/rovioKalmanFilter.h"
	#include "Kalman/kalmanFilterDef.h"
}

//Struct for FIR filter
typedef struct {
	float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
	int next_sample;
	float samples[30];
	int TAPS;
} filter;


class RobotPose {
  
public:
	RobotPose(RobotInterface *r, char* player);
	~RobotPose();
  
	void initPose();
	void moveTo(float x, float y, float goal_theta);

	void turnTo(float theta);
	void printRaw();
	void printPoses();
	void updatePosition(bool turning);
	void resetPose(float x, float y, float theta);
	void changeWEScalingConstant(float we);
	void changeUncertainty(float *uc);
  
	// Camera functions
	void moveToCell(int x, int y);
	bool strafeTo(int delta_x, float goal_theta);
	
	//Constant to set strafing threshold
	static const float STRAFE_EPSILON = 35.0;
	
	//Cell size constants
	static const int CELL_DIMENSION_CM = 65;
	static const int CELL_EPSILON_CM = 3;

private:
	bool updateWE(bool turning);
	bool updateNS();
	filter *createFilter(const char *coef, float initval);
	float firFilter(filter* f, float val);
  
	RobotInterface *robot;
	pose pose_start;
	pose pose_we;
	pose pose_ns;
	pose pose_goal;
	pose current_cell;

	PIDController *PID_x;
	PIDController *PID_y;
	PIDController *PID_theta;
	PIDController *PID_camera;
	int room_start;
	int room_cur;
	int player;
	float theta_ns_trans;
  


	//FIR Filters
	filter *fir_left_we;
	filter *fir_right_we;
	filter *fir_rear_we;
	filter *fir_x_ns;
	filter *fir_y_ns;
	filter *fir_theta_ns;
  
	// Kalman Filter
	kalmanFilter kf;
	pose pose_kalman;
  
	//CameraPose
	CameraPose *pose_cam;
};


#endif
