/** This program will display a screen to visually allow the user to verify
 *  the correctness of heading direction calculation.
 */

// -- C++ imports
#include <stdio.h>
#include <iostream>
#include <vector>

// -- OPENCV imports
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// GLOBALS
const char *WINDOW_NAME = "Testing Window";
const int WINDOW_SIZE = 500;
const char *IRATIMGLOCATION = "irat.pgm";
int x_pos = -1;
int y_pos = -1;
double direction = 0;
int history_pacer = 15;
int histCount = 0;
vector<Point> iRatHistory;
const int maxHistCount = 5;
Point change;
Point iRatHeading;
float angle;

// Function definitions
void callback(int, int, int, int, void *);
void usage(void);
void calculate_direction(void);
Mat update_display(Mat);

/* Lets go main lets go */
int main(int argc, char *argv[]){
	namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	setMouseCallback(WINDOW_NAME, callback, NULL);
	Mat original(WINDOW_SIZE, WINDOW_SIZE, CV_8UC3);
	int radius = 10;
	circle(original, Point(WINDOW_SIZE/2, WINDOW_SIZE/2), radius, 
			Scalar(0, 0, 255), 5);
	Mat edited;

	// init the history 
	iRatHistory.push_back(Point(-1,-1));

	// Might bother to put this in idk
	//Mat irat_img = imread(IRATIMAGELOCATION, CV_LOAD_IMAGE_COLOR);
	int key = -1;
	while(1){
		calculate_direction();
		edited = update_display(original);
		imshow(WINDOW_NAME, edited);
		key = waitKey(10);
		if (key == 'q' || key == 'Q'){
			break;
		}
	}
	return 0;
}

/** Given an image, draw the current state of the system to the image
 */
Mat update_display(Mat orig) {
	Mat result = orig.clone();
	circle(result, Point(x_pos, y_pos), 4, Scalar(255, 0, 0), 3);


	stringstream ss;
	ss << x_pos << " " << y_pos;

	int offset_x = 100;
	int offset_y = 40;
	Point test_pos(WINDOW_SIZE - offset_x, WINDOW_SIZE - offset_y);
	putText(result, ss.str(), test_pos, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5,
			Scalar::all(255), 2, 2);

	// Display the final direction
	stringstream nss;
	nss << direction;
	Point dir_pos(offset_x, offset_y);
	putText(result, nss.str(), dir_pos, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5,
			Scalar::all(255), 2, 2);
	// Draw the change in green	
	line(result, Point(x_pos, y_pos), Point(x_pos, y_pos) + change, 
			Scalar(20, 230, 20), 3);
	// Draw the iRats heading in purple
	line(result, Point(x_pos, y_pos), Point(x_pos, y_pos) + iRatHeading, 
			Scalar(120, 30, 170), 3);

	// Draw angle
	stringstream ass;
	ass << angle;
	putText(result, ass.str(), Point(2*offset_x, offset_y), 
			FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar::all(255), 2, 2);

	return result;
}


/** This function implements the tracking to be trialed
 */
void calculate_direction(void) {
	// TODO code to test
	// Uses variables x_pos and y_pos as the rat location 
	// TODO maybe heading direction should be set dynamically by cmd line args
	
	// Josh's ported implementation
	/* things needed:
	 * 	- Point iRatLoc
	 *	- Point iRatHeading
	 *	- Point ratLoc 
	 *	- double mag 
	 * 	- Also need access to a global "histCount" - last x positions of irat
	 */
	Point iRatLoc(x_pos, y_pos);//WINDOW_SIZE/2, WINDOW_SIZE/2);
	// Point iRatHeading is set as a global TODO
	Point ratLoc(WINDOW_SIZE/2, WINDOW_SIZE/2);//x_pos, y_pos);
	double mag = 0.5;

	// Calculate iRat heading diretion
	int histSize = iRatHistory.size();
	// if histsize is > maxSize then just go a few back else use oldest
	Point oldPoint = histSize > maxHistCount ? 
			iRatHistory[histSize - maxHistCount] : iRatHistory[0];
	
	iRatHeading = iRatHistory[histSize-1] - oldPoint;




	// arguments set, begin implementation
	Point diff = ratLoc - iRatLoc;
	Point sum;
	
	// calculate angle
	float len1 = sqrt(iRatHeading.x * iRatHeading.x + 
			iRatHeading.y * iRatHeading.y);
	float len2 = sqrt(diff.x * diff.x + diff.y * diff.y);
	float dot = iRatHeading.x * diff.x + iRatHeading.y * diff.y;
	float a = dot / (len1 * len2);

	if (a >= 1.0) {
		angle = 0.0;
	} else if (a <= -1.0) {
		angle = 3.14;
	} else {
		angle = acos(a);
	}
	/*
	if (histCount == maxHistCount-1) {
		// This implementation will just store a vector of Points
		iRatHeading = iRatLoc - iRatHistory[maxHistCount-1][0];
	}
	histCount++;
	if (histCount == maxHistCount) {
		histCount = 0;
	}
	*/	
	change = diff - iRatHeading;
	double turn = iRatHeading.y*change.x < 0 ? -1 : 1;
	
	// Setting direction is the 'return' statement here
	direction = turn * mag;
	

}


/** Mouse callback to update the position variables
 */ 
void callback(int event, int x, int y, int flags, void *userdata) {
	if (history_pacer++ > 15){
		x_pos = x;
		y_pos = y;
		iRatHistory.push_back(Point(x, y));
		history_pacer = 0;
	}
}

/** Displays the usage information
 */
void usage(void){
	cout << "Usage: simulate" << endl;
}
