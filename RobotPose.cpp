#include <robot_if++.h>
#include <iostream>
#include <string>
#include <math.h>

class RobotPose
{
public:
//constructor 
RobotPose(RobotInterface *r){
	robot = r;
	robot->update();
	resetCoord();
};
resetCoord(){
	pose_start[0] = robot->X();
	pose_start[1] = robot->Y();
	pose_start[2] = robot->Theta();
	
	pose_ns[0] = 0;
	pose_ns[1] = 0;
	pose_ns[2] = 0;
	
	pose_we[0] = 0;
	pose_we[1] = 0;
	pose_we[2] = 0;

	room_start = robot->roomID();
	room_cur = room_start;
};
updatePosition(){
	robot->update();
	updateWE();
	updateNS();
};
float[] getPositionWE(){
	return pose_we;
};
float[] getPositionNS(){
	return pose_ns;
};

private:
updateWE(){
  	int left  = robot->getWheelEncoder(RI_WHEEL_LEFT);
	int right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
	int rear  = robot->getWheelEncoder(RI_WHEEL_REAR);
	float dy = ((left * sin(150 * PI/180)) + (right_tot * sin(30 * PI/180)))/2;
	float dx = ((left * cos(150 * PI/180)) + (right_tot * cos(30 * PI/180)))/2;
	float dtheta = rear/(29*PI);
	pose_we[0] = dx*we_to_cm;
	pose_we[1] = dy*we_to_cm;
	pose_we[3] = dtheta*we_to_cm;
};

updateNS(){
  	int x = robot->X();
	int y = robot->Y();
	int theta = robot->Theta();
	int room = robot->roomID();

};

const float PI = 3.14159265;
const float we_to_cm = 1/20;
const float ns_to_cm = 1/300;

RobotInterface *robot;
float pose_start[3];
float pose_we[3];
float pose_ns[3];

int room_start;
int room_cur;
};
