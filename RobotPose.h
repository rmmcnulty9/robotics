
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
  
  void resetCoord();
  void updatePosition(bool turning);
  void moveTo(double x, double y);
  void turnTo(double theta);
  void printRaw();
  void printTransformed();
  bool getPosition(pose& robot);
  
  private:
  bool updateWE(bool turning);
  bool updateNS();
  filter *createFilter(char *coef, float initval);
  float firFilter(filter* f, float val);
  
  RobotInterface *robot;
  pose pose_start;
  pose pose_we;
  pose pose_ns;
  //pose pose_robot;

  int room_start;
  int room_cur;
  double theta_ns_trans;
  
  //FIR Filters
  filter *left_we;
  filter *right_we;
  filter *rear_we;
  filter *x_ns;
  filter *y_ns;
  filter *theta_ns;
  
  // Kalman Filter
  kalmanFilter kf;
  pose pose_kalman;
  
  //PID Controllers
  PIDController *pidX, *pidY, *pidTheta;
};


#endif
