
#ifndef __CAMERA_POSE__
#define __CAMERA_POSE__

#include "shared_constants.h"
#include "robot_if++.h"
#include "modified_color.h"
#include <list>

using namespace std;
/*
 * Holds pair of matching squares
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
//NOT USED RIGHT NOW
//	void strafeTo(int delta_y);
//	void moveTo();

	void strafeTo(float delta_y);
	void printCenters(list<squarePair> pairs);
	int getCenterError(list<squarePair> pairs);
	int getCellError(list<squarePair> pairs);

	static const float STRAFE_EPSILON = 15.0;
	unsigned int image_ctr;

	private:
  
	RobotInterface *robot;
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *pinkHigh;
	IplImage *pinkLow;
	IplImage *yellow;
	IplImage *filteredImage;
	static const int MIN_SQUARE = 400;
	static const int SCREEN_WIDTH = 640;
	static const int SCREEN_HEIGHT = 480;
	static const int PINK = 0;
	static const int YELLOW = 1;

	static const int Y_CELL_CENTER_PINK = 168;
	static const int Y_CELL_CENTER_YELLOW = 135;
  
};

#endif
