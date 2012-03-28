/*
 * CameraPose Header
 *  Used to define functions and constants to collect, filter, and interpret camera data
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */
#ifndef __CAMERA_POSE__
#define __CAMERA_POSE__

#include "robot_if++.h"
#include "modified_color.h"
#include <list>

using namespace std;
/*
 * Holds pair of horizontally matching squares and their color
 */
typedef struct {
	squares_t* left;
	squares_t* right;
	int color;
} squarePair;


class CameraPose {
  
	public:
	CameraPose(RobotInterface *r);
	~CameraPose();
  
	list<squarePair> updateCamera();
	void displayImages();
	list<squarePair> matchSquares(squares_t *squares, int color);
	void drawSquares(squares_t *squares, CvScalar displayColor);


	void printCenters(list<squarePair> pairs);
	int getCenterError(list<squarePair> pairs);
	int getCellError(list<squarePair> pairs);
	int getTurnError(list<squarePair> pairs);
	void removeOverlap(squares_t *squares);
	int getSquareSide(squares_t *squares);
	
	//Current image number for saving purposes
	unsigned int image_ctr;

	private:
  
	RobotInterface *robot;
	
	//Images for camera data and display
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *pinkHigh;
	IplImage *pinkLow;
	IplImage *yellow;
	IplImage *filteredImage;
	
	int turnError;
	static const int MIN_SQUARE = 400;
	
	//Constants for screen resolution
	static const int SCREEN_WIDTH = 640;
	static const int SCREEN_HEIGHT = 480;
	
	//Constants for squarePair color
	static const int PINK = 0;
	static const int YELLOW = 1;

	//Constants for square locations when at center of cell
	static const int Y_CELL_CENTER_PINK = 168;
	static const int Y_CELL_CENTER_YELLOW = 135;
	static const int X_CELL_CENTER_RIGHT = 530;
	static const int X_CELL_CENTER_LEFT = 120;
  
};

#endif
