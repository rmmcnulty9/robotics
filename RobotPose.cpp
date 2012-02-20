#include <robot_if++.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cstdio>
#include "RobotPose.h"
#include "PIDController.h"
#include "shared_constants.h"

extern "C" {
#include "Kalman/kalmanFilterDef.h"
}


RobotPose::RobotPose(RobotInterface *r){
	robot = r;
	//std::cout << "Battery:" << r->getBattery() >> "\n";
	
	//Create all six FIR filters
	fir_x_ns = RobotPose::createFilter(coef_filename,0.0);
	fir_y_ns = RobotPose::createFilter(coef_filename,0.0);
	fir_theta_ns = RobotPose::createFilter(coef_filename,0.0);
  
	fir_left_we = RobotPose::createFilter(coef_filename,0.0);
	fir_right_we = RobotPose::createFilter(coef_filename,0.0);
	fir_rear_we = RobotPose::createFilter(coef_filename,0.0);

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
	initialPose[2] = M_PI_2;
	
	float Velocity[3];
	Velocity[0] = 0;
	Velocity[1] = 0;
	Velocity[2] = 0;
	int deltat = 1;

	initKalmanFilter(&kf, initialPose, Velocity, deltat);
	
	//Initialize PID controllers
	PID_x = new PIDController(10.0,-10.0,0.05,0.5,1.0);
	PID_y = new PIDController(10.0,-10.0,0.05,0.5,1.0);
	PID_theta = new PIDController(10.0,-10.0,0.05,0.5,1.0);
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
	pose_start.theta = ROOM2;
	//pose_start.theta = ROOM3
	//pose_start.theta = ROOM4
	//pose_start.theta = ROOM5

	robot->update();
	
	room_start = robot->RoomID();
	room_cur = room_start;
	pose_start.theta = ns_theta[room_start];

	float x = robot->X();
	float y = robot->Y();

	pose_start.x = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
	pose_start.y = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);

	//Fill theta fir filter with legitimate values
	for(int i=0;i<30;i++){
		pose_ns.theta = firFilter(fir_theta_ns,(robot->Theta()-pose_start.theta));
	}
	pose_ns.x = 0.0;
	pose_ns.y = 0.0;

	pose_we.x = 0.0;
	pose_we.y = 0.0;
	pose_we.theta = M_PI_2;
}

void RobotPose::moveTo(float x, float y) {
	updatePosition(false);
	//Determine how much robot needs to turn to reach goal
	//float theta = atan((y-pose_kalman.y)/(x-pose_kalman.x)); 
	float goal_theta = acos((x-pose_kalman.x)/sqrt((x-pose_kalman.x)*(x-pose_kalman.x)+(y-pose_kalman.y)*(y-pose_kalman.y)));

	if((y-pose_kalman.y)<=0.0) {
		goal_theta = -goal_theta;
	}
   
	printf(" %f %f %f \n",x, y, goal_theta * 180/M_PI);
 	//printf("kalman %f %f %f\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta); 
	printf("K: %f %f %f \t NS: %f %f %f \t WE: %f %f %f \t Signal: %d\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta*180/M_PI,
	pose_ns.x, pose_ns.y, pose_ns.theta*180/M_PI, pose_we.x, pose_we.y, pose_we.theta*180/M_PI, robot->NavStrengthRaw());
    
	turnTo(goal_theta);  
	
	//Determine errors in pose
	float error_distance_x = pose_kalman.x - x;
	float error_distance_y = pose_kalman.y - y;
	float error_distance = sqrt(error_distance_x * error_distance_x + error_distance_y * error_distance_y);

	//Get PID
	float PID_xres = PID_x->UpdatePID(error_distance_x, pose_kalman.x);
	float PID_yres = PID_y->UpdatePID(error_distance_y, pose_kalman.y);
  
	//Calculate speed based on PID
	float total_PID = abs(sqrt(PID_xres * PID_xres + PID_yres * PID_yres));
	int robot_speed;
	float velocity[3];
	//Set robot's current speed and set new kalman velocity
	if(total_PID > 50.0){
		robot_speed = 1;
		velocity[0] = vel_1 * cos(pose_kalman.theta);
		velocity[1] = vel_1 * sin(pose_kalman.theta);
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	else if(total_PID < 50.0 && total_PID > 25.0){
		robot_speed = 3;
		float velocity [3];
		velocity[0] = vel_3 * cos(pose_kalman.theta);
		velocity[1] = vel_3 * sin(pose_kalman.theta);
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	else {
		robot_speed = 5;
		velocity[0] = vel_5 * cos(pose_kalman.theta);
		velocity[1] = vel_5 * sin(pose_kalman.theta);
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	//Wall check
	if (robot->IR_Detected() && x != -180.0 && x != -354.0) {
		printf("wall!\n");
		exit(0);
	}
	//Move unless within range of base
	else if (error_distance > 10.0) {
		robot->Move(RI_MOVE_FORWARD, robot_speed);
		moveTo(x, y);
	}
	else {
		printf("ARRIVED! %f %f\n", x, y);
		//robot->Move(RI_HEAD_UP, RI_FASTEST);
		//robot->Move(RI_HEAD_DOWN, RI_FASTEST);
	}
}

void RobotPose::turnTo(float goal_theta) {


	updatePosition(true);
	//Determine error in theta
	float error_theta1 = goal_theta-pose_kalman.theta;
	float error_theta2 = pose_kalman.theta-goal_theta;

	if(error_theta1<0.0) error_theta1+=(2.0*M_PI);
	if(error_theta2<0.0) error_theta2+=(2.0*M_PI);

	float error_theta = error_theta1<error_theta2?error_theta1:error_theta2;
	//Don't turn if error theta not large enough
	if(error_theta>=(-25.0*(M_PI/180)) && error_theta<= (25.0*(M_PI/180))){
		 printf("Theta too small\n");
		 return;
	}

	printf("K: %f %f %f \t NS: %f %f %f \t WE: %f %f %f\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta*180/M_PI,
	pose_ns.x, pose_ns.y, pose_ns.theta*180/M_PI, pose_we.x, pose_we.y, pose_we.theta*180/M_PI);
  
	//Call PID for Theta
	float PID_res = abs(PID_theta->UpdatePID(error_theta, pose_kalman.theta));
	//Determine speed
	printf("Theta PID: %f\n", PID_res);

	int robot_speed; 
	float velocity[3];
	if(PID_res > 1.0){
		robot_speed = 5;
		velocity[0] = 0.0;
		velocity[1] = 0.0;
		velocity[2] = vel_5;
		//rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	else if(PID_res < 1.0 && PID_res > 0.25){
		robot_speed = 7;
		velocity[0] = 0.0;
		velocity[1] = 0.0;
		velocity[2] = vel_7;
		//rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	else {
		robot_speed = 10;
		velocity[0] = 0.0;
		velocity[1] = 0.0;
		velocity[2] = vel_10;
		//rovioKalmanFilterSetVelocity(&kf,velocity);
	}
	//Turn depending on error 
	if(error_theta==error_theta1){
		printf("Turning Left \n", error_theta1*(180/M_PI), error_theta2*(180/M_PI));
 		robot->Move(RI_TURN_LEFT, robot_speed);
	}
	else if(error_theta==error_theta2){
		printf("Turning Right \n", error_theta1*(180/M_PI), error_theta2*(180/M_PI));
		robot->Move(RI_TURN_RIGHT, robot_speed);
	}

	printf("Recursing: at K: %f NS: %f WE: %f not %f\n",pose_kalman.theta*(180/M_PI), pose_ns.theta*(180/M_PI), pose_we.theta*(180/M_PI), goal_theta*(180/M_PI));
	turnTo(goal_theta);

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

void RobotPose::updatePosition(bool turning=false){
	robot->update();
	updateNS();
	pose_we.theta = pose_ns.theta;
	updateWE(turning);

	//If there is a weaker NS signal change the uncertainty - higher NS lower WE and lower PRE
	if(robot->NavStrengthRaw() < 5000){
		rovioKalmanFilterSetUncertainty(&kf,uncertainty_weak_ns);
	}else{
		rovioKalmanFilterSetUncertainty(&kf,uncertainty_default);
	}

	//Pass through Kalman filter
	float NSdata[3], WEdata[3], track[9];
	NSdata[0] = pose_ns.x;
	NSdata[1] = pose_ns.y;
	NSdata[2] = pose_ns.theta;

	WEdata[0] = pose_we.x;
	WEdata[1] = pose_we.y;
	WEdata[2] = pose_we.theta;
	rovioKalmanFilter(&kf,NSdata, WEdata, track);

	//return the filtered robot pose
	pose_kalman.x = track[0];
	pose_kalman.y = track[1];
	pose_kalman.theta = track[2];
	//printf("K: %f %f %f \t NS: %f %f %f \t WE: %f %f %f\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta*180/M_PI,
	//pose_ns.x, pose_ns.y, pose_ns.theta*180/M_PI, pose_we.x, pose_we.y, pose_we.theta*180/M_PI);

}

bool RobotPose::updateWE(bool turning){
	float dx_2, dy_2;
  
	int left = robot->getWheelEncoder(RI_WHEEL_LEFT);
	int right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
	int rear = robot->getWheelEncoder(RI_WHEEL_REAR);
	//Filter wheel encoder data
	if(!turning){
		left = firFilter(fir_left_we, left);
		right = firFilter(fir_right_we, right);
	}
	rear = firFilter(fir_rear_we, rear);
	//Transform data
	float dy = ((left * sin(150.0 * M_PI/180.0)) + (right * sin(30.0 * M_PI/180.0)) + (rear * sin(90.0 * M_PI/180.0)))/3.0;
	float dx = ((left * cos(150.0 * M_PI/180.0)) + (right * cos(30.0 * M_PI/180.0)))/2.0;
	dx = 0.0; // I don't think we are supposed to move in this direction.

	float dtheta = (rear*we_to_rad);

//pose_we.theta += dtheta;

//Normalizing the theta between PI and -PI
/*if(pose_we.theta>M_PI){
 pose_we.theta-=(2*M_PI);
}else if(pose_we.theta<-M_PI){
  pose_we.theta+=(2*M_PI);
}*/
	//Rotate delta x and y data to align with global coordinates
	if(!turning){
		dx_2 = dx * cos(pose_we.theta - M_PI_2) - dy * sin(pose_we.theta - M_PI_2);
		dy_2 = dx * sin(pose_we.theta - M_PI_2) + dy * cos(pose_we.theta - M_PI_2);
		pose_we.x += dx_2*we_to_cm;
		pose_we.y += dy_2*we_to_cm;
	}
	return true;
}

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
	//Get initial data
	static float prev_theta = robot->Theta() - pose_start.theta; 
	static float total_theta = robot->Theta() - pose_start.theta;
	float delta_theta;
	static int jump_ctr = 0;
	int new_room = robot->RoomID();
	float x, y, theta, x_2, y_2; 
 
	//Change room coordinates
	if(new_room != room_cur){
		printf("ROOM CHANGED %d\n", new_room);

		pose_start.theta = ns_theta[new_room];
    
		//Set the new pose_start to current reading
		pose_start.x = pose_kalman.x / ns_x_to_cm[room_cur];
		pose_start.y = pose_kalman.y / ns_y_to_cm[room_cur];
    
		//Simple send the last value back in to FIR for this room change cycle
		//x = firFilter(x_ns,(robot->X() - pose_start.x));
		//y = firFilter(y_ns,(robot->Y() - pose_start.y));
		//theta = firFilter(theta_ns,(robot->Theta() - pose_start.theta));
		
		x = robot->X();
		y = robot->Y();
		
		//Rotate
		x_2 = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
		y_2 = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);
		
		//Translate
		pose_start.x = x_2 - pose_start.x;
		pose_start.y = y_2 - pose_start.y;
		
		room_cur = new_room;
	}
    
	x = robot->X();
	y = robot->Y();
	theta = robot->Theta();
    
	//Rotate
	x_2 = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
	y_2 = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);
	
	//Translate
	x = x_2 - pose_start.x;
	y = y_2 - pose_start.y;
  
	//Scale
	x = x * ns_x_to_cm[room_cur];
	y = y * ns_y_to_cm[room_cur];
    
	//Transform theta
	theta = (theta - pose_start.theta);
    
	delta_theta = theta-prev_theta;
      
	if(abs(delta_theta)>(3*M_PI_2) && prev_theta>0 && theta<0) {
		jump_ctr+=1;
	}
	else if(abs(delta_theta)>(3*M_PI_2) && theta>0 && prev_theta<0) {
		jump_ctr-=1;
	}
      
	prev_theta=theta;
	pose_ns.theta = theta;

	//Apply FIR filter
	pose_ns.x = firFilter(fir_x_ns,x);
	pose_ns.y = firFilter(fir_y_ns,y);
	//pose_ns.theta = firFilter(theta_ns, theta+(jump_ctr*2*M_PI)) - (jump_ctr*2*M_PI);
  
	/*float avg_theta = 0.0;
	for (int i = 0; i < 3; i++) {
		robot->update();
		avg_theta += robot->Theta() + 0.34906585; //0.436332313 = 25 degrees
		//printf("theta.... %f\n", (robot->Theta() + 0.34906585) * 180.0/M_PI);
	}
	pose_ns.theta = avg_theta / 3.0;
	//printf("avg_theta %f\n", pose_ns.theta * 180.0/M_PI);*/
	//Rotate theta
	pose_ns.theta = robot->Theta() + ns_theta_offset[room_cur];
	return true;
}

void RobotPose::resetPose() {
	/*pose_ns.x = pose_kalman.x;
	pose_ns.y = pose_kalman.y;
	pose_ns.theta = pose_kalman.theta;*/
  
	pose_we.x = pose_kalman.x;
	pose_we.y = pose_kalman.y;
	pose_we.theta = pose_kalman.theta;
}

void RobotPose::changeWE(float we) {
	we_to_cm = we;
}

// firFilterCreate()
// creates, allocates, and initializes a new firFilter
 
filter *RobotPose::createFilter(const char *coef_file, float initval)
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
		f->TAPS++;
	}
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
