
#ifndef __ROBOT_POSE__
#define __ROBOT_POSE__

#include "shared_constants.h"

typedef struct {
  float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
  int next_sample;
  float samples[30];
  int TAPS;
} filter;

typedef struct {
    float x;
    float y;
    float theta;
} pose;

class RobotPose {
  
  public:
  RobotPose(RobotInterface *r);
  ~RobotPose();
  
  void resetCoord();
  void updatePosition();
  bool getPositionWE(pose& we);
  bool getPositionNS(pose& ns);
  
  private:
  bool updateWE();
  bool updateNS();
  filter *createFilter(char *coef);
  float firFilter(filter *f, float val);
  
  RobotInterface *robot;
  pose pose_start;
  pose pose_we;
  pose pose_ns;

  int room_start;
  int room_cur;
  
  //Filters
  filter left_we;
  filter right_we;
  filter rear_we;
  filter x_ns;
  filter y_ns;
  filter theta_ns;


  
};


#endif
