#include <robot_if++.h>
#include <iostream>
#include <string>
#include <math.h>
#include "RobotPose.h"


RobotPose::RobotPose(RobotInterface *r){
  robot = r;
	robot->update();
	resetCoord();
}

 void RobotPose::resetCoord(){
	pose_start.x = robot->X();
	pose_start.y = robot->Y();
	pose_start.theta = robot->Theta();
	
	pose_ns.x = 0;
	pose_ns.y = 0;
	pose_ns.theta = 0;
	
	pose_we.x = 0;
	pose_we.y = 0;
	pose_we.theta = 0;

	room_start = robot->RoomID();
	room_cur = room_start;
}

void RobotPose::updatePosition(){
	robot->update();
	updateWE();
	updateNS();
}
bool RobotPose::getPositionWE(pose& we){
	we.x = pose_we.x;
  we.y = pose_we.y;
  we.theta = pose_we.theta;
  return true;
}
bool RobotPose::getPositionNS(pose& ns){
	return true;
}

bool RobotPose::updateWE(){
  int left  = robot->getWheelEncoder(RI_WHEEL_LEFT);
	int right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
	int rear  = robot->getWheelEncoder(RI_WHEEL_REAR);
	float dy = ((left * sin(150 * M_PI/180)) + (right * sin(30 * M_PI/180)))/2;
	float dx = ((left * cos(150 * M_PI/180)) + (right * cos(30 * M_PI/180)))/2;
	float dtheta = rear/(29*M_PI);
	pose_we.x = dx*we_to_cm;
	pose_we.y = dy*we_to_cm;
	pose_we.theta = dtheta*we_to_cm;
  return true;
}

bool RobotPose::updateNS(){
  	int x = robot->X();
	int y = robot->Y();
	int theta = robot->Theta();
	int room = robot->RoomID();
  /*
   * Conversion
   */
   
   /*
    * Set the NS pose
    */
  return true;
}
