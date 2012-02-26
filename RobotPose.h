
#ifndef __ROBOT_POSE__
#define __ROBOT_POSE__

#include "shared_constants.h"
#include "PIDController.h"

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
  void printTransformed();
  void updatePosition(bool turning);
  void resetWEPose();
  void changeWEScalingConstant(float we);
  
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
  
};


#endif
