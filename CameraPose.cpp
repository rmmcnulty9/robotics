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
#include "modified_color.h"
#include <list>

using namespace std;


/*
 * Initializes the CameraPose to handle camera based navigation
 * Configures camera and initializes images and display windows
 */

CameraPose::CameraPose(RobotInterface *r){
	robot = r;
	//Initializes images for storing most recent camera data
	cameraImage = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 3);
	hsvImage = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 3);
	filteredImage = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 1);
	pinkLow = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 1);
	pinkHigh = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 1);
	yellow = cvCreateImage(cvSize(SCREEN_WIDTH, SCREEN_HEIGHT), IPL_DEPTH_8U, 1);

	//Setup display windows
	cvNamedWindow("Unfiltered", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Filtered Pink High", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Filtered Pink Low", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Filtered Yellow", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Filtered", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("All Filtered", CV_WINDOW_AUTOSIZE);

	// Setup the camera
        if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
                std::cout << "Failed to configure the camera!" << std::endl;
                exit(-1);
        }
}
CameraPose::~CameraPose(){}



/*
 * Updates robot and gets current image & manipulated them
 */
list<squarePair> CameraPose::updateCamera(){
	// Update the robot's sensor information
	robot->update();
	// Get the current camera image
	robot->getImage(cameraImage);
	// Convert to hsv
	cvCvtColor(cameraImage, hsvImage, CV_BGR2HSV);
	
	//Find and match yellow squares
	squares_t * currentSquares;
	list<squarePair> yellowPairs, pinkPairs;
	
	cvInRangeS(hsvImage, RC_YELLOW_LOW, RC_YELLOW_HIGH, yellow);
	currentSquares = robot->findSquares(yellow, MIN_SQUARE);
	drawSquares(currentSquares, CV_RGB(0,255,0));
	yellowPairs = matchSquares(currentSquares, YELLOW);
	printCenters(yellowPairs);


	//Find and match pink squares
	cvInRangeS(hsvImage, RC_PINK1_LOW, RC_PINK1_HIGH, pinkLow);
	cvInRangeS(hsvImage, RC_PINK2_LOW, RC_PINK2_HIGH, pinkHigh);
	cvOr(pinkLow, pinkHigh, filteredImage, NULL);
	
	currentSquares = robot->findSquares(filteredImage, MIN_SQUARE);
	drawSquares(currentSquares, CV_RGB(255,0,0));
	pinkPairs = matchSquares(currentSquares, PINK);
	printCenters(pinkPairs);

	displayImages();
	
	//Save image
	char file_name [256];
	sprintf(file_name,"camera.%04d.jpg", image_ctr);
	cvSaveImage(file_name,cameraImage);

	sprintf(file_name,"filtered.%04d.jpg", image_ctr);
	cvSaveImage(file_name,filteredImage);

	sprintf(file_name,"yellow.%04d.jpg",image_ctr);
	cvSaveImage(file_name,yellow);

	sprintf(file_name,"pinklow.%04d.jpg", image_ctr);
	cvSaveImage(file_name,pinkLow);

	sprintf(file_name,"pinkhigh.%04d.jpg", image_ctr);
	cvSaveImage(file_name,pinkHigh);
	image_ctr+=1;
	
	//Return both yellow and pink pairs
	yellowPairs.splice(yellowPairs.end(), pinkPairs);
	return yellowPairs;
  
}
/*
 * Displays current images - filtered and unfiltered
 */
void CameraPose::displayImages(){
	cvShowImage("Unfiltered", cameraImage);
	cvShowImage("Filtered", filteredImage);
	//cvShowImage("Filtered Pink High", pinkHigh);
	//cvShowImage("Filtered Pink Low", pinkLow);
	cvShowImage("Filtered Yellow", yellow);
	//cvShowImage("Filtered All", filteredImage);
	cvWaitKey(200);
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
 * Comparison function for list sort to use
 */
bool compare_areas (squarePair first, squarePair second){

	if((first.left->area + first.right->area) > (second.left->area + second.right->area)) return true;
	else return false;
}


/*
 * Find squares of same height and draw lines between them
 */
list<squarePair> CameraPose::matchSquares(squares_t *squares, int color){
	squares_t *tempSquares;
	list <squarePair> pair_list;
	squarePair temp_pair;
	temp_pair.left = NULL; temp_pair.right = NULL;
	CvPoint pt1, pt2;
	while(squares != NULL){
		tempSquares = squares->next;
		while(tempSquares != NULL){
			
			//Test if y values are close, if x values are far, and if centers in top half of screen
			if(abs(squares->center.y - tempSquares->center.y) < SCREEN_HEIGHT/15 
				&& abs(squares->center.x - tempSquares->center.x) > SCREEN_WIDTH/6
				&& (squares->center.y) < SCREEN_HEIGHT * (3.0/5.0)
				&& (tempSquares->center.y) < SCREEN_HEIGHT * (3.0/5.0)){
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
				temp_pair.color = color;
				pair_list.push_back(temp_pair);
				break;
				
			}
			tempSquares = tempSquares->next;
		}
		squares = squares->next;
	}
	//Sort the squarePairs largest area to smallest
	pair_list.sort(compare_areas);
	return pair_list;
}
/*
 * Draws a line between the squarePairs & puts a tick at the line's midpoint
 */
void CameraPose::printCenters(list<squarePair> pairs){
	list<squarePair>::iterator it;
	cvLine(cameraImage, cvPoint(SCREEN_WIDTH/2,0), cvPoint(SCREEN_WIDTH/2,SCREEN_HEIGHT), CV_RGB(128,128,128), 2, CV_AA, 0);
	int center = 0; 
	int height = 0;
	for(it=pairs.begin(); it!=pairs.end(); it++){
		center = (it->left->center.x + it->right->center.x)/2;
		height = (it->left->center.y + it->right->center.y)/2;
		cvLine(cameraImage, cvPoint(center,height-5), cvPoint(center,height+5), CV_RGB(255,0,0), 2, CV_AA, 0);
	}
}
/*
 * Finds the distance the midpoint between squares in from the middle of the image itself
 */
int CameraPose::getCenterError(list<squarePair> pairs){
	list<squarePair>::iterator it;
	int centers = 0; 
	//for(it=pairs.begin(); it!=pairs.end(); it++){
	//	centers += (it->left->center.x + it->right->center.x)/2;
	//}	
	if(pairs.size() == 0)
		return 0;
	else{
		list<squarePair>::iterator it = pairs.begin();
		centers = (it->left->center.x + it->right->center.x)/2;
		//printf("Best Center L(%d,%d) R(%d,%d) Color:%d\n",it->left->center.x,
		//	it->left->center.y, it->right->center.x, it->right->center.y, it->color);
		if(pairs.size() > 1){
			it++;
		//	printf("Best Center L(%d,%d) R(%d,%d) Color:%d\n",it->left->center.x,
		//		it->left->center.y, it->right->center.x, it->right->center.y, it->color);
		}
		return centers - (SCREEN_WIDTH/2);
	}
	//	return (centers/pairs.size()) - (SCREEN_WIDTH/2);
	
}
int CameraPose::getTurnError(list<squarePair> pairs){
	list<squarePair>::iterator it;
	if(pairs.size() == 0)
		return 0;
	else{
		list<squarePair>::iterator it = pairs.begin();
		int left_error = abs(it->left->center.x - SCREEN_WIDTH/2);
		int right_error = abs(it->right->center.x - SCREEN_WIDTH/2);
		return left_error - right_error;
	}
}
/*
 * Find the distance to the next cell based on where it finds squarePairs in the image
 * -based onthe location and size of squarePairs
 */
int CameraPose::getCellError(list<squarePair> pairs){
	list<squarePair>::iterator it = pairs.begin();
	int error = 0;
	if(it->color == PINK){
		error += it->left->center.y - Y_CELL_CENTER_PINK;
		error += it->right->center.y - Y_CELL_CENTER_PINK;
	}else if(it->color == YELLOW){
		error += it->left->center.y - Y_CELL_CENTER_YELLOW;
		error += it->right->center.y - Y_CELL_CENTER_YELLOW;

	}
	
	return error;
}


