/*
 * CameraPose Class
 *  Used to collect, filter, and interpret camera data
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

#include <robot_if++.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cstdio>
#include "CameraPose.h"
#include "shared_constants.h"
#include "robot_color.h"
#include <list>

using namespace std;


/*
 * Initializes the CameraPose to handle camera based navigation
 * Configures camera and initializes images and display windows
 */

CameraPose::CameraPose(RobotInterface *r){
	robot = r;
	//Initializes images for storing most recent camera data
	cameraImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	hsvImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	filteredImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

	//Setup display windows
	cvNamedWindow("Unfiltered", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Filtered", CV_WINDOW_AUTOSIZE);

	// Setup the camera
        if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
                std::cout << "Failed to configure the camera!" << std::endl;
                exit(-1);
        }
}
CameraPose::~CameraPose(){}

/*
 * Function that will strafe a delta Y value positive = right, negative = left
 * delta_x's range is -320 to +320
 */
void CameraPose::strafeTo(int delta_x){
	int robot_speed = 5;

	//PID Controller code here


	//move the robot left or right
	if((delta_x+STRAFE_EPSILON)<0){
		robot->Move(RI_MOVE_LEFT, robot_speed);
	}else if((delta_x-STRAFE_EPSILON)>0){
		robot->Move(RI_MOVE_RIGHT, robot_speed);
	}else{
		//Base case
		return;
	}
	//Call to updateCamera() & getError()
	//strafeTo(getCenterError);
}

/*
 * Move forward to the delta_y
 */
void CameraPose::moveTo(int delta_y){
	int robot_speed = 3;

	//PID Controller goes here

	if(delta_y > MOVE_TO_EPSILON) {
			robot->Move(RI_MOVE_FORWARD, robot_speed);

			/*
			 * If not done move
			 *
			 * For not just decrement delta_y
			 */
			moveTo(delta_y-5);

	}else{
		printf("ARRIVED\n");
		return;
	}
}

/*
 * Updates robot and gets current image
 */
void CameraPose::updateCamera(){
	// Update the robot's sensor information
	robot->update();
	// Get the current camera image
	robot->getImage(cameraImage);
	// Convert to hsv
	cvCvtColor(cameraImage, hsvImage, CV_BGR2HSV);
	
	//Find and match yellow squares
	squares_t * currentSquares;
	list<squarePair> pairs;
	cvInRangeS(hsvImage, RC_YELLOW_LOW, RC_YELLOW_HIGH, filteredImage);
	currentSquares = robot->findSquares(filteredImage, MIN_SQUARE);
	drawSquares(currentSquares, CV_RGB(0,255,0));
	pairs = matchSquares(currentSquares);
	printCenters(pairs);
	strafeTo(getCenterError(pairs));
	
	//Find and match pink squares
	cvInRangeS(hsvImage, RC_PINK_LOW, RC_PINK_HIGH, filteredImage);
	currentSquares = robot->findSquares(filteredImage, MIN_SQUARE);
	drawSquares(currentSquares, CV_RGB(255,0,0));
	pairs = matchSquares(currentSquares);
	printCenters(pairs);
	displayImages();
}
/*
 * Displays current images
 */
void CameraPose::displayImages(){
	cvShowImage("Unfiltered", cameraImage);
	cvShowImage("Filtered", filteredImage);
	cvWaitKey(500);
}

/*
 * Draw boxes around squares found in robot->findSquares
 *  draw in color displayColor
 */
void CameraPose::drawSquares(squares_t *squares, CvScalar displayColor){
	CvPoint pt1, pt2;
	
	while(squares != NULL) {
        
		int sq_amt = (int) (sqrt(squares->area) / 2);

		// Upper Left to Lower Left
		pt1.x = squares->center.x - sq_amt;
		pt1.y = squares->center.y - sq_amt;
		pt2.x = squares->center.x - sq_amt;
		pt2.y = squares->center.y + sq_amt;
		cvLine(cameraImage, pt1, pt2, displayColor, 2, CV_AA, 0);

		// Lower Left to Lower Right
		pt1.x = squares->center.x - sq_amt;
		pt1.y = squares->center.y + sq_amt;
		pt2.x = squares->center.x + sq_amt;
		pt2.y = squares->center.y + sq_amt;
		cvLine(cameraImage, pt1, pt2, displayColor, 2, CV_AA, 0);                        
		// Upper Left to Upper Right
		pt1.x = squares->center.x - sq_amt;
		pt1.y = squares->center.y - sq_amt;
		pt2.x = squares->center.x + sq_amt;
		pt2.y = squares->center.y - sq_amt;
		cvLine(cameraImage, pt1, pt2, displayColor, 2, CV_AA, 0);

		// Lower Right to Upper Right
		pt1.x = squares->center.x + sq_amt;
		pt1.y = squares->center.y + sq_amt;
		pt2.x = squares->center.x + sq_amt;
		pt2.y = squares->center.y - sq_amt;
		cvLine(cameraImage, pt1, pt2, displayColor, 2, CV_AA, 0);
		squares = squares->next;
	}
}

/*
 * Find squares of same height and draw lines between them
 */
list<squarePair> CameraPose::matchSquares(squares_t *squares){
	squares_t *tempSquares;
	list <squarePair> pair_list;
	squarePair temp_pair;
	temp_pair.left = NULL; temp_pair.right = NULL;
	CvPoint pt1, pt2;
	while(squares != NULL){
		tempSquares = squares->next;
		while(tempSquares != NULL){
			
			if(abs(squares->center.y - tempSquares->center.y) < 20 && abs(squares->center.x - tempSquares->center.x) > 20){
				//Draw line
				pt1 = cvPoint(squares->center.x, squares->center.y);
				pt2 = cvPoint(tempSquares->center.x, tempSquares->center.y);
				cvLine(cameraImage, pt1, pt2, CV_RGB(0,0,255), 2, CV_AA, 0);
				
				//Record squares
				int newArea = (squares->area + tempSquares->area)/2;
				if(squares->center.x < tempSquares->center.x){
					temp_pair.left = squares;
					temp_pair.right = tempSquares;
				}
				else{
					temp_pair.left = tempSquares;
					temp_pair.right = squares;
				}
				pair_list.push_back(temp_pair);
				break;
				
			}
			tempSquares = tempSquares->next;
		}
		squares = squares->next;
	}
	return pair_list;
}
void CameraPose::printCenters(list<squarePair> pairs){
	list<squarePair>::iterator it;
	cvLine(cameraImage, cvPoint(320,0), cvPoint(320,480), CV_RGB(128,128,128), 2, CV_AA, 0);
	int center = 0; 
	int height = 0;
	for(it=pairs.begin(); it!=pairs.end(); it++){
		center = (it->left->center.x + it->right->center.x)/2;
		height = (it->left->center.y + it->right->center.y)/2;
		cvLine(cameraImage, cvPoint(center,height-5), cvPoint(center,height+5), CV_RGB(255,0,0), 2, CV_AA, 0);
	}
}
int CameraPose::getCenterError(list<squarePair> pairs){
	list<squarePair>::iterator it;
	int centers = 0; 
	int size = 0;
	for(it=pairs.begin(); it!=pairs.end(); it++){
		centers += (it->left->center.x + it->right->center.x)/2;
		size++;
	}	
	if(size == 0)
		return 0;
	else
		return centers/size - 320;
	
}


