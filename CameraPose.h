
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
} squarePair;


class CameraPose {
  
	public:
	CameraPose(RobotInterface *r);
	~CameraPose();
  
	list<squarePair> updateCamera();
	void displayImages();
	list<squarePair> matchSquares(squares_t *squares);
	void drawSquares(squares_t *squares, CvScalar displayColor);
//NOT USED RIGHT NOW
//	void strafeTo(int delta_y);
//	void moveTo();

	void strafeTo(float delta_y);
	void printCenters(list<squarePair> pairs);
	int getCenterError(list<squarePair> pairs);
	int getCellError(list<squarePair> pairs);

	static const float STRAFE_EPSILON = 5.0;

	private:
  
	RobotInterface *robot;
	IplImage *cameraImage;
	IplImage *hsvImage;
	IplImage *filteredImage;
	static const int MIN_SQUARE = 200;
	static const int SCREEN_WIDTH = 640;
	static const int SCREEN_HEIGHT = 480;
  
};


#endif
