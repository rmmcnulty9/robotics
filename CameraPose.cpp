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

CameraPose::CameraPose(RobotInterface *r){
	robot = r;
	image = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
	hsv = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
	filtered = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1);

	cvNamedWindow("Unfiltered", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Filtered", CV_WINDOW_AUTOSIZE);

	// Setup the camera
        if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_320, RI_CAMERA_QUALITY_HIGH)) {
                std::cout << "Failed to configure the camera!" << std::endl;
                exit(-1);
        }
}
CameraPose::~CameraPose(){}

void CameraPose::updateCamera(){
	// Update the robot's sensor information
	robot->update();
	// Get the current camera image
	robot->getImage(image);
	// Convert to hsv
	cvCvtColor(image, hsv, CV_BGR2HSV);
	cvInRangeS(hsv, RC_PINK_LOW, RC_PINK_HIGH, filtered);
	findSquares(filtered, CV_RGB(255,0,0));	
	cvInRangeS(hsv, RC_YELLOW_LOW, RC_YELLOW_HIGH, filtered);
	findSquares(filtered, CV_RGB(0,255,0));
	
	displayImage();
}

void CameraPose::displayImage(){
	cvShowImage("Unfiltered", image);
	cvShowImage("HSV", hsv);
	cvShowImage("Filtered", filtered);
	cvWaitKey(25);
}

void CameraPose::findSquares(IplImage *f, CvScalar color){
	squares_t *squares;
	CvPoint pt1, pt2;
	squares = robot->findSquares(filtered, 10);
	while(squares != NULL) {
        
			int sq_amt = (int) (sqrt(squares->area) / 2);

                        // Upper Left to Lower Left
                        pt1.x = squares->center.x - sq_amt;
                        pt1.y = squares->center.y - sq_amt;
                        pt2.x = squares->center.x - sq_amt;
                        pt2.y = squares->center.y + sq_amt;
                        cvLine(image, pt1, pt2, color, 2, CV_AA, 0);

                        // Lower Left to Lower Right
                        pt1.x = squares->center.x - sq_amt;
                        pt1.y = squares->center.y + sq_amt;
                        pt2.x = squares->center.x + sq_amt;
                        pt2.y = squares->center.y + sq_amt;
                        cvLine(image, pt1, pt2, color, 2, CV_AA, 0);                        
			// Upper Left to Upper Right
                        pt1.x = squares->center.x - sq_amt;
                        pt1.y = squares->center.y - sq_amt;
                        pt2.x = squares->center.x + sq_amt;
                        pt2.y = squares->center.y - sq_amt;
                        cvLine(image, pt1, pt2, color, 2, CV_AA, 0);

                        // Lower Right to Upper Right
                        pt1.x = squares->center.x + sq_amt;
                        pt1.y = squares->center.y + sq_amt;
                        pt2.x = squares->center.x + sq_amt;
                        pt2.y = squares->center.y - sq_amt;
                        cvLine(image, pt1, pt2, color, 2, CV_AA, 0);
		squares = squares->next;
	}
}


