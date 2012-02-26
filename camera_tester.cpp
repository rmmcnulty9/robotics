#include "robot_if++.h"
#include "robot_color.h"
#include <iostream>
#include <string>

int main(int argv, char **argc) {
        int major, minor;
        IplImage *image = NULL, *hsv = NULL, *threshold = NULL;
        squares_t *squares, *biggest, *sq_idx;
        CvPoint pt1, pt2;
        int sq_amt;

        // Make sure we have a valid command line argument
        if(argv <= 1) {
                std::cout << "Usage: robot_test <address of robot>" << std::endl;
                exit(-1);
        }

        // Setup the robot interface
        RobotInterface *robot = new RobotInterface(argc[1],0);

        // Print the API Version
        robot->API_Version(&major, &minor);
        std::cout << "Robot API Test: API Version v" << major << "." << minor << std::endl;

        // Setup the camera
        if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_320, RI_CAMERA_QUALITY_HIGH)) {
                std::cout << "Failed to configure the camera!" << std::endl;
                exit(-1);
        }

        // Create a window to display the output
        cvNamedWindow("Rovio Camera", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("Biggest Square", CV_WINDOW_AUTOSIZE);

        // Create an image to store the image from the camera
        image = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);

        // Create an image to store the HSV version in
        // We configured the camera for 320x240 above, so use that size here
        hsv = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);

        // And an image for the thresholded version
        threshold = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1);

        // Move the head up to the middle position
        robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);

        // Action loop
        do {
                // Update the robot's sensor information
                if(robot->update() != RI_RESP_SUCCESS) {
                        std::cout << "Failed to update sensor information!" << std::endl;
                        continue;
                }

                // Get the current camera image and display it

                if(robot->getImage(image) != RI_RESP_SUCCESS) {
                        std::cout << "Unable to capture an image!" << std::endl;
                        continue;
                }
                cvShowImage("Rovio Camera", image);

                // Convert the image from RGB to HSV
                cvCvtColor(image, hsv, CV_BGR2HSV);

                // Pick out only the yellow color from the image
                cvInRangeS(hsv, RC_YELLOW_LOW, RC_YELLOW_HIGH, threshold);

                // Find the squares in the image
                squares = robot->findSquares(threshold, RI_DEFAULT_SQUARE_SIZE);

                // Loop over the squares and find the biggest one
                biggest = squares;
                sq_idx = squares;
                while(sq_idx != NULL) {
                        if(sq_idx->area > biggest->area)
                                biggest = sq_idx;
                        sq_idx = sq_idx->next;
                }

                // Only draw if we have squares
                if(biggest != NULL) {
                        // Draw an X marker on the image
                        sq_amt = (int) (sqrt(biggest->area) / 2);

                        // Upper Left to Lower Right
                        pt1.x = biggest->center.x - sq_amt;
                        pt1.y = biggest->center.y - sq_amt;
                        pt2.x = biggest->center.x + sq_amt;
                        pt2.y = biggest->center.y + sq_amt;
                        cvLine(image, pt1, pt2, CV_RGB(0, 255, 0), 3, CV_AA, 0);

                        // Lower Left to Upper Right
                        pt1.x = biggest->center.x - sq_amt;
                        pt1.y = biggest->center.y + sq_amt;
                        pt2.x = biggest->center.x + sq_amt;
                        pt2.y = biggest->center.y - sq_amt;
                        cvLine(image, pt1, pt2, CV_RGB(0, 255, 0), 3, CV_AA, 0);
                }

                // Display the image with the drawing on it
                cvShowImage("Biggest Square", image);

                // Update the UI (10ms wait)
                cvWaitKey(10);

                // Release the square data
                while(squares != NULL) {
                        sq_idx = squares->next;
                        delete(squares);
                        squares = sq_idx;
                }

                // Move forward unless there's something in front of the robot
                if(!robot->IR_Detected())
                        robot->Move(RI_MOVE_FORWARD, RI_SLOWEST);
        } while(1);

        // Clean up (although we'll never get here...)
        delete(robot);

        cvDestroyWindow("Rovio Camera");
        cvDestroyWindow("Biggest Square");

        // Free the images
        cvReleaseImage(&hsv);
        cvReleaseImage(&threshold);
        cvReleaseImage(&image);


        return 0;
}
