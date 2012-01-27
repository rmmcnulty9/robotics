#include <robot_if++.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include "RobotPose.h"
#include "shared_constants.h"


RobotPose::RobotPose(RobotInterface *r, char* coef_file){
  robot = r;
	robot->update();
	resetCoord();
  //Create all six FIR filters
  
  RobotPose::createFilter(coef_file, x_ns);
  RobotPose::createFilter(coef_file, y_ns);
  RobotPose::createFilter(coef_file, theta_ns);
  
  RobotPose::createFilter(coef_file, left_we);
  RobotPose::createFilter(coef_file, right_we);
  RobotPose::createFilter(coef_file, rear_we);
}
RobotPose::~RobotPose(){
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
	pose_we.x += dx*we_to_cm;
	pose_we.y += dy*we_to_cm;
	pose_we.theta += dtheta*we_to_cm;
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

// firFilterCreate()
// creates, allocates,  and initializes a new firFilter
 
filter *RobotPose::createFilter(char *coef_file, filter *f)
{
  int i;
  f = (filter *)malloc(sizeof(filter));
  //printf("%d\n", sizeof(filter));
  f->TAPS = 0;
  f->next_sample = 0;
  FILE *fp = fopen(coef_file,"r+");
  if(fp==NULL){
    printf("Coefficients could not be loaded from %s\n", coef_file);
    exit(-1);
  }
  
  //Read in coef & count, for TAPS
  for (i = 0; i < 30; i++){
    f->samples[i] = 0;
    if(1!=fscanf(fp,"%e ", &f->coefficients[i])){
      fclose(fp);
      break;
    }
   // printf("%f\n", f->coefficients[i]);
    f->TAPS++;
  }
  
//  printf("Coefficients:\n");
//  for (i = 0; i < f->TAPS; i++) {
//    printf("%d: %f\n", i, f->coefficients[i]);
//  }
  
  return f;
}

// firFilter 
//inputs take a filter (f) and the next sample (val)
//returns the next filtered sample
//incorporates new sample into filter data array
 

float RobotPose::firFilter(filter& f, float val)
{
  float sum =0;
  int i,j;

  // assign  new value to "next" slot 
  f.samples[f.next_sample] = val;

  // calculate a  weighted sum
  //   i tracks the next coeficeint
  //   j tracks the samples w/wrap-around 
  for( i=0,j=f.next_sample; i<f.TAPS; i++) {
    sum += f.coefficients[i]*f.samples[j++];
    if(j == f.TAPS)  j=0;
  }
  if(++(f.next_sample) == f.TAPS) f.next_sample = 0;
  return(sum);
}

