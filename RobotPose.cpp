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
	//void initKalmanFilter(kalmanFilter *kf, float * initialPose, float *Velocity, int deltat) 
	initKalmanFilter(&kf, initialPose, Velocity, deltat);
	//PIDController PID_x(10.0,-10.0,1.0,1.0,1.0);
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

	double x = robot->X();
	double y = robot->Y();

	pose_start.x = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
	pose_start.y = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);

	
	for(int i=0;i<30;i++){
		pose_ns.theta = firFilter(fir_theta_ns,(robot->Theta()-pose_start.theta));
	}
	pose_ns.x = 0.0;
	pose_ns.y = 0.0;

	pose_we.x = 0.0;
	pose_we.y = 0.0;
	pose_we.theta = M_PI_2;

	room_start = robot->RoomID();
	room_cur = room_start;
}

void RobotPose::moveTo(double x, double y) {
	updatePosition(false);
	//Turn to
	//double theta = atan((y-pose_kalman.y)/(x-pose_kalman.x)); 
	double goal_theta = acos((x-pose_kalman.x)/sqrt((x-pose_kalman.x)*(x-pose_kalman.x)+(y-pose_kalman.y)*(y-pose_kalman.y)));

	if((y-pose_kalman.y)<=0.0) {
		goal_theta = -goal_theta;
	}
   
	printf(" %f %f goal_theta %f \t %f %f\n",x, y, goal_theta * 180/M_PI, pose_kalman.x, pose_kalman.y);
  
	printf("kalman %f %f %f \t north star %f %f %f \t wheel encoder %f %f %f\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta*180/M_PI,
	pose_ns.x, pose_ns.y, pose_ns.theta*180/M_PI, pose_we.x, pose_we.y, pose_we.theta*180/M_PI);
    
	turnTo(goal_theta);  

	double error_distance_x = pose_kalman.x - x;
	double error_distance_y = pose_kalman.y - y;
	double error_distance = sqrt(error_distance_x * error_distance_x + error_distance_y * error_distance_y);

	double PID_xres = PID_x->UpdatePID(error_distance_x, pose_kalman.x);
	double PID_yres = PID_y->UpdatePID(error_distance_y, pose_kalman.y);
	printf("Err x: %f\tErr y: %f\n", error_distance_x, error_distance_y);
  
	//Calculate speed based on PID
	double total_PID = sqrt(PID_xres * PID_xres + PID_yres * PID_yres);
	printf("PID x: %f\tPID y: %f\tTotal PID: %f\n", PID_xres, PID_yres, total_PID);
	int robot_speed;
	if(total_PID > 50.0)
		robot_speed = 1;
		velocity[0] = 0.0;
		velocity[1] = vel_1;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	else if(total_PID < 50.0 && total_PID > 25.0)
		robot_speed = 3;
		float velocity [3];
		velocity[0] = 0.0;
		velocity[1] = vel_3;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	else {
		robot_speed = 5;
		velocity[0] = 0.0;
		velocity[1] = vel_5;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	}
	if (robot->IR_Detected()) {
		printf("wall!\n");
		exit(0);
	}
	else if (error_distance > 10.0) {
		robot->Move(RI_MOVE_FORWARD, robot_speed);
		moveTo(x, y);
	}
	else {
		printf("arrived!\n");
	}
}

void RobotPose::turnTo(double goal_theta) {


	updatePosition(true);
	double error_theta1 = goal_theta-pose_kalman.theta;
	double error_theta2 = pose_kalman.theta-goal_theta;

	if(error_theta1<0.0) error_theta1+=(2.0*M_PI);
	if(error_theta2<0.0) error_theta2+=(2.0*M_PI);

	double error_theta = error_theta1<error_theta2?error_theta1:error_theta2;

	if(error_theta>=(-30.0*(M_PI/180)) && error_theta<= (30.0*(M_PI/180))){
		 printf("Theta too small\n");
		 return;
	}

	printf("K: %f %f %f \t NS: %f %f %f \t WE: %f %f %f\n", pose_kalman.x, pose_kalman.y, pose_kalman.theta*180/M_PI,
	pose_ns.x, pose_ns.y, pose_ns.theta*180/M_PI, pose_we.x, pose_we.y, pose_we.theta*180/M_PI);
  
	//Call PID for Theta
	double PID_res = PID_theta->UpdatePID(error_theta, pose_kalman.theta);
	//Determine speed
	printf("Theta PID: %f\n", PID_res);

	int robot_speed; 
	if(PID_res > 1.0)
		robot_speed = 3;
		velocity[0] = 0.0;
		velocity[1] = vel_3;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	else if(PID_res < 1.0 && PID_res > 0.25)
		robot_speed = 5;
		velocity[0] = 0.0;
		velocity[1] = vel_5;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	else {
		robot_speed = 7;
		velocity[0] = 0.0;
		velocity[1] = vel_7;
		velocity[2] = 0.0;
		rovioKalmanFilterSetVelocity(kf,velocity);
	}

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

//TODO This should probably be private
void RobotPose::updatePosition(bool turning=false){
	robot->update();
	updateNS();
	pose_we.theta = pose_ns.theta;
	updateWE(turning);


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

}

//TODO This should probably just be moved into updatePosition()
	bool RobotPose::updateWE(bool turning){
	double dx_2, dy_2;
  
	int left = robot->getWheelEncoder(RI_WHEEL_LEFT);
	int right = robot->getWheelEncoder(RI_WHEEL_RIGHT);
	int rear = robot->getWheelEncoder(RI_WHEEL_REAR);

	if(!turning){
		left = firFilter(fir_left_we, left);
		right = firFilter(fir_right_we, right);
	}
	rear = firFilter(fir_rear_we, rear);

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

	if(!turning){
		dx_2 = dx * cos(pose_we.theta - M_PI_2) - dy * sin(pose_we.theta - M_PI_2);
		dy_2 = dx * sin(pose_we.theta - M_PI_2) + dy * cos(pose_we.theta - M_PI_2);
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
	static double prev_theta = robot->Theta() - pose_start.theta; 
	static double total_theta = robot->Theta() - pose_start.theta;
	double delta_theta;
	static int jump_ctr = 0;
	int room = robot->RoomID();
	double x, y, theta, x_2, y_2; 
 
	if(room != room_cur){
		//printf("No room changing yet\n");
		//exit(-1);
	//}
		switch(room){
			case 2: pose_start.theta = ROOM2; break;
			case 3: pose_start.theta = ROOM3; break;
			case 4: pose_start.theta = ROOM4; break;
			case 5: pose_start.theta = ROOM5; break;
			default: printf("Error changing rooms!!!\n"); exit(-1); 
		}
    
		//Set the new pose_start to current reading
		pose_start.x = pose_kalman.x * 1.0 / ns_x_to_cm;
		pose_start.y = pose_kalman.y * 1.0 / ns_y_to_cm;
    
		//Simple send the last value back in to FIR for this room change cycle
		//x = firFilter(x_ns,(robot->X() - pose_start.x));
		//y = firFilter(y_ns,(robot->Y() - pose_start.y));
		//theta = firFilter(theta_ns,(robot->Theta() - pose_start.theta));
	}
    
	x = robot->X();
	y = robot->Y();
	theta = robot->Theta();
    
	// rotate
	x_2 = x * cos(-pose_start.theta) - y * sin(-pose_start.theta);
	y_2 = x * sin(-pose_start.theta) + y * cos(-pose_start.theta);
    
	// translate
	x = x_2 - pose_start.x;
	y = y_2 - pose_start.y;
  
	// scale
	x = x * ns_x_to_cm;
	y = y * ns_y_to_cm;
	//printf("y: %f\n", y);
    
	// transform theta
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
  



/*
* Set the NS pose
*/

 //printf("%f %f\n", x_2, y_2);



	// apply FIR filter
	pose_ns.x = firFilter(fir_x_ns,x);
	pose_ns.y = firFilter(fir_y_ns,y);
  //pose_ns.theta = firFilter(theta_ns, theta+(jump_ctr*2*M_PI)) - (jump_ctr*2*M_PI);
  
	/*double avg_theta = 0.0;
	for (int i = 0; i < 3; i++) {
		robot->update();
		avg_theta += robot->Theta() + 0.34906585; //0.436332313 = 25 degrees
		//printf("theta.... %f\n", (robot->Theta() + 0.34906585) * 180.0/M_PI);
	}
	pose_ns.theta = avg_theta / 3.0;
	//printf("avg_theta %f\n", pose_ns.theta * 180.0/M_PI);*/
	
	pose_ns.theta = robot->Theta() + 0.34906585;
	return true;
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
