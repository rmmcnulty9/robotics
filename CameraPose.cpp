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
#include "robot_if++.h"
#include "robot_color.h"

/*
 * Initializes the CameraPose to handle camera based navigation
 * Configures camera and intilizes images and display windows
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
 * Updates robot and gets current image
 */
void CameraPose::updateCamera(){
	// Update the robot's sensor information
	robot->update();
	// Get the current camera image
	robot->getImage(cameraImage);
	// Convert to hsv
	cvCvtColor(cameraImage, hsvImage, CV_BGR2HSV);

	cvInRangeS(hsvImage, RC_YELLOW_LOW, RC_YELLOW_HIGH, filteredImage);
	findSquares(filteredImage, CV_RGB(0,255,0));
	
	cvInRangeS(hsvImage, RC_PINK_LOW, RC_PINK_HIGH, filteredImage);
	findSquares(filteredImage, CV_RGB(255,0,0));	
	
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
 * Finds squares in a prefiltered image *f 
 *  and draws boxes around them of color displayColor
 */
void CameraPose::findSquares(IplImage *f, CvScalar displayColor){
	squares_t *squares;
	CvPoint pt1, pt2;
	squares = robot->findSquares(f, 250);
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


