
#ifndef __ROBOT_POSE__
#define __ROBOT_POSE__

typedef struct robot_pose {
    float x;
    float y;
    float theta;
}pose;


const float we_to_cm = 1/20.0;
const float ns_to_cm = 1/300.0;

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