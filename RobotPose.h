/*
 * RobotPose Class
 *  Used to collect, filter, and interpret camera data
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

typedef struct {
  float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
  int next_sample;
  float samples[30];
  int TAPS;
} filter;


class RobotPose {
  
  public:
  RobotPose(RobotInterface *r);
  ~RobotPose();
  
  void initPose();
  void moveTo(float x, float y);

  void turnTo(float theta);
  void printRaw();
  void printPoses();
  void updatePosition(bool turning);
  void resetWEPose(float x, float y, float theta);
  void changeWEScalingConstant(float we);
  
  // Camera functions
  void moveToCell(const int direction);
  bool strafeTo(int delta_x);

	static const int LEFT = 1;
	static const int RIGHT = 2;
	static const int FORWARD = 3;
	static const int BACKWARD = 4;
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
  //pose pose_robot;
  PIDController *PID_x;
  PIDController *PID_y;
  PIDController *PID_theta;
  PIDController *PID_camera;
  int room_start;
  int room_cur;
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
