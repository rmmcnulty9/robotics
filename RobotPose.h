
#ifndef __ROBOT_POSE__
#define __ROBOT_POSE__

#include "shared_constants.h"

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
  
  RobotInterface *robot;
  pose pose_start;
  pose pose_we;
  pose pose_ns;

  int room_start;
  int room_cur;

  
};


#endif
