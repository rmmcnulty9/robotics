#include <robot_if++.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cstdio>
#include "RobotPose.h"
#include "shared_constants.h"

extern "C" {
#include "Kalman/kalmanFilterDef.h"
}

RobotPose::RobotPose(RobotInterface *r, char* coef_file){
  robot = r;
    //Create all six FIR filters
  x_ns = RobotPose::createFilter(coef_file,0.0);
  y_ns = RobotPose::createFilter(coef_file,0.0);
  theta_ns = RobotPose::createFilter(coef_file,0.0);
  
  left_we = RobotPose::createFilter(coef_file,0.0);
  right_we = RobotPose::createFilter(coef_file,0.0);
  rear_we = RobotPose::createFilter(coef_file,0.0);

  /*
   * Resets the pose values & initializes start_pose
   * */
  resetCoord();

  /*
   * Initialize Kalman filter
   * */

	float initialPose[3];
	initialPose[0] = 0;
	initialPose[1] = 0;
	initialPose[2] = 0;
	
	float Velocity[3];
	Velocity[0] = 0;
	Velocity[1] = 0;
	Velocity[2] = 0;
	int deltat = 1;
	//void initKalmanFilter(kalmanFilter *kf, float * initialPose, float *Velocity, int deltat) 
	//initKalmanFilter(&kf, initialPose, Velocity, deltat);

  /*  Is this need anymore - ryan
   * 
   * int x = robot->X();
   * int y = robot->Y();
   * robot->Move(RI_MOVE_FORWARD, RI_FASTEST);
   * robot->update();
   * x = robot->X() - x;
   * y = robot->Y() - y;
   * // theta = (x, y) * (0, 1) / (len( (x, y) ) * len( (0, 1) ))
   * printf("%d %d %f\n", x, y, sqrt((double)(x * x + y * y)));
   * theta_ns_trans = acos((double)y / (sqrt((double)(x * x + y * y))));
   * updateWE();
   * updateNS();
* */
  
  /*
   * Is this needed - ryan
   * */
  //double len = sqrt((double)(x * x + y * y));
  //double x_1 = (double)x / len;
  //double y_1 = (double)y / len;
  //printf("len: %f\n", len);
  //printf("theta_orig: %f\n", pose_ns.theta_orig);
  //printf("x1 y1: %f %f\n", x_1, y_1);
  
  //double x_2 = -sin(pose_ns.theta_orig);
  //double y_2 = cos(pose_ns.theta_orig);
  
  //printf("x2 y2: %f %f\n", x_2, y_2);
  

}
RobotPose::~RobotPose(){
}

void RobotPose::resetCoord() {
/*
 * Read in constant for current room for the start pose
 * Room 2 = 1.3554
 * Room 3 = -0.0019661
 * Room 4 =  1.5953
 * Room 5 = 0.041115
 * */

// Will always start in Room 2
pose_start.theta = 1.3554; //2
//pose_start.theta = -0.0019661; //3
//pose_start.theta = 1.5953; //4
//pose_start.theta = 0.041115; //5

robot->update();
pose_start.x =robot->X();
pose_start.y = robot->Y();
 
pose_ns.theta = robot->Theta()-pose_start.theta;
pose_ns.x = 0;
pose_ns.y = 0;


//std::cout << "Start NS: " << pose_start.x << "," << pose_start.y << "," << pose_ns.theta * (180/M_PI) << "\n";

pose_we.x = 0;
pose_we.y = 0;
pose_we.theta = 0;

room_start = robot->RoomID();
room_cur = room_start;
}

void RobotPose::printRaw(){
	int d_left = robot->getWheelEncoder(RI_WHEEL_LEFT);
	int d_right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
	int d_rear = robot->getWheelEncoder(RI_WHEEL_REAR);
	int t_left = robot->getWheelEncoderTotals(RI_WHEEL_LEFT);
	int t_right = robot->getWheelEncoderTotals(RI_WHEEL_RIGHT);
	int t_rear = robot->getWheelEncoderTotals(RI_WHEEL_REAR);
               
 	int x = robot->X();
	int y = robot->Y();
	float theta = robot->Theta();
               
	printf("%d %d %f %d %d %d %d %d %d\n", x, y, theta, d_left, d_right, d_rear, t_left, t_right, t_rear);
}
void RobotPose::printTransformed(){
	//Prints wheel encoder and then north star transformed data
	printf("%f %f %f %f %f %f\n",pose_we.x, pose_we.y, pose_we.theta*(180/M_PI), pose_ns.x, pose_ns.y, pose_ns.theta*(180/M_PI));
}

bool RobotPose::getPosition(pose& bot){
//Update the position
//updatePosition();

//Pass through Kalman filter
float NSdata[3], WEdata[3], track[9];
NSdata[0] = pose_ns.x;
NSdata[1] = pose_ns.y;
NSdata[2] = pose_ns.theta;

WEdata[0] = pose_we.x;
WEdata[1] = pose_we.y;
WEdata[2] = pose_we.theta;
//rovioKalmanFilter(&kf,NSdata, WEdata, track);

//return the filtered robot pose
bot.x = track[0];
bot.y = track[1];
bot.theta = track[2];

return true;
}

//TODO This should probably be private
void RobotPose::updatePosition(bool turning=false){
robot->update();

updateWE(turning);
updateNS();
//printRaw();
printTransformed();
}

//TODO This should probably just be moved into updatePosition()
bool RobotPose::updateWE(bool turning){
  double dx_2, dy_2;
  
int left = robot->getWheelEncoder(RI_WHEEL_LEFT);
int right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
int rear = robot->getWheelEncoder(RI_WHEEL_REAR);
//std::cout << "[" << left << ",\t\t" << right << ",\t\t" << rear << "]\n";
left = firFilter(left_we, left);
right = firFilter(right_we, right);
rear = firFilter(rear_we, rear);
//std::cout << "{" << left << ",\t" << right << ",\t" << rear << "}\n";
float dy = ((left * sin(150 * M_PI/180)) + (right * sin(30 * M_PI/180)))/2;
float dx = ((left * cos(150 * M_PI/180)) + (right * cos(30 * M_PI/180)))/2;
float dtheta = rear/(robot_diameter_cm*M_PI);

pose_we.theta -= dtheta;

if(!turning){
dx_2 = dx * cos(pose_we.theta) - dy * sin(pose_we.theta);
dy_2 = dx * sin(pose_we.theta) + dy * cos(pose_we.theta);
pose_we.x += dx_2*we_to_cm;
pose_we.y += dy_2*we_to_cm;
}
return true;
}

//TODO This should also probably be merged with updatePosition()
bool RobotPose::updateNS(){
 
/*
* Check for a new room - if different change the way we do conversion:
* 
* Read in constant for current room for the start pose
* Room 2 = 1.3554
* Room 3 = -0.0019661
* Room 4 =  1.5953
* Room 5 = 0.041115
* 
* */
int room = robot->RoomID();
double x, y, theta, x_2, y_2; 
/* 
  if(room != room_cur){
    switch(room){
      case 2: pose_start.theta = 1.3554; break;
      case 3: pose_start.theta = -0.0019661; break;
      case 4: pose_start.theta = 1.5953; break;
      case 5: pose_start.theta = 0.041115; break;
      default: printf("Error changing rooms!!!\n"); exit(-1); 
    }
    
    //Set the new pose_start to current reading
    pose_start.x = robot->X();
    pose_start.y = robot->Y();
    
    //Simple send the last value back in to FIR for this room change cycle
    x = firFilter(x_ns,(robot->X() - pose_start.x));
    y = firFilter(y_ns,(robot->Y() - pose_start.y));
    theta = firFilter(theta_ns,(robot->Theta() - pose_start.theta));
    
  }else{
    */
    x = (robot->X()- pose_start.x);
    y = (robot->Y()- pose_start.y);
    theta = (robot->Theta() - pose_start.theta);
  //}
  
  
  x_2 = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
  y_2 = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);

/*
* Set the NS pose
*/

 //printf("%f %f\n", x_2, y_2);

  pose_ns.x = firFilter(x_ns,-x_2 * ns_to_cm);
  pose_ns.y = firFilter(y_ns,-y_2 * ns_to_cm);
  pose_ns.theta = firFilter(theta_ns, theta);
  //printf("Room: %d ", room);
 // std::cout << std::setw(6) << pose_ns.x << ",\t" << std::setw(6)<< pose_ns.y << ",\t"
 //   << std::setw(6)<< pose_ns.theta * (180/M_PI)<< ",\t Room: " << room_cur << ", Nav Strength:" << robot->NavStrengthRaw() << "\n";
  return true;
}

// firFilterCreate()
// creates, allocates, and initializes a new firFilter
 
filter *RobotPose::createFilter(char *coef_file, float initval)
{
int i;
filter* f = (filter *)malloc(sizeof(filter));
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
//f->samples[i] = 0;
f->samples[i] = initval;
if(1!=fscanf(fp,"%e ", &f->coefficients[i])){
fclose(fp);
break;
}
// printf("%f\n", f->coefficients[i]);
f->TAPS++;
}
  
//printf("Coefficients:\n");
//for (i = 0; i < f->TAPS; i++) {
//printf("%d: %f\n", i, f->coefficients[i]);
//}
  // std::cout << "Test:" << f->next_sample << "\n";

return f;
}

// firFilter
//inputs take a filter (f) and the next sample (val)
//returns the next filtered sample
//incorporates new sample into filter data array
 

float RobotPose::firFilter(filter* f, float val)
{
float sum =0;
int i,j;

// assign new value to "next" slot
f->samples[f->next_sample] = val;

// calculate a weighted sum
// i tracks the next coeficeint
// j tracks the samples w/wrap-around
for( i=0,j=f->next_sample; i<f->TAPS; i++) {
sum += f->coefficients[i]*f->samples[j++];
if(j == f->TAPS) j=0;
}
if(++(f->next_sample) == f->TAPS) f->next_sample = 0;
return(sum);
}
