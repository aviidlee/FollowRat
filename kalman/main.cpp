/**
 * This file will implement surf object detection combine with a kalman filter
 * to identify and track an object in a video file.
 */

//Usual suspects
#include <string>
#include <stdio.h>
#include <iostream>
#include <vector>

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

// headers and misc.
#include "main.h" 

using namespace std;
using namespace cv;

// function definitions
void usage(void);
void mouse_movement(int, int, int, int, void *);
void init_kalman(KalmanFilter *, Point *);
Point find_obj(SurfFeatureDetector *, Mat *, Mat *);
void here(int);
//vector<Point> find_object(Mat, Mat);

// GLOBALS
//Point mouse_pos;
vector<Point> kalman_estmte; // The kalman predictions of location
vector<Point> measured_location;


// CONSTANTS
const char *WINDOW_NAME = "Kalman window";
const int MINHESSIAN = 400;
const int ESC_KEY = 27;
//TODO Move this define to header file
#define CAMERANOTOPEN -2 
#define SURFCREATIONFAILED -3

int main(int argc, char *argv[]) {
	if (argc != 3) {
		usage();
		return INPUTARGSINVALID; 
	}

	cvNamedWindow(WINDOW_NAME, 1);
	//setMouseCallback(WINDOW_NAME, mouse_movement, NULL); //Set callback

	VideoCapture cap("play5.avi");//atoi(argv[2]));
	if (!cap.isOpened()) {
		cout << "Failed to open camera" << endl;
		return CAMERANOTOPEN;
	}
	Mat frame;
	// Set up SURF and obj feature vector
	Mat obj_img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	if (!obj_img.data) {
		cout << "-- (!) Failed to read obj_image" << endl;
		return SURFCREATIONFAILED;
	}

	// Detect features
	SurfFeatureDetector detector(MINHESSIAN);
	vector<KeyPoint> keypoints_obj; 
	detector.detect(obj_img, keypoints_obj);

	// Extract descriptors
	SurfDescriptorExtractor extractor;
	Mat obj_desc;
	extractor.compute( obj_img, keypoints_obj, obj_desc );


	/*
	if (!detector) {
		cout << "Failed to load object to detect image." << endl;
		return SURFCREATIONFAILED;
	}
	*/


	// Find initial location of object
	Point init_location;
	do {
		cap >> frame;
		imshow(WINDOW_NAME, frame);
		init_location = find_obj(&detector, &frame, &obj_desc);
		waitKey(10);
	} while(init_location.x == -1 && init_location.y == -1);
	measured_location.push_back(init_location);

	// Create Kalman filter and initialise
	KalmanFilter KF(6, 2, 0); // 6 variables, 2 meaurements
	init_kalman(&KF, &init_location);
	Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

	Mat gray;	
	int key = -1;
	while(key != 'q' && key != 'Q' && key != ESC_KEY) {
		// Do movement prediction
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
		Point measPt(-1, -1);
		do {
			// Take a measurement
			cap >> frame;
			cvtColor(frame, gray, CV_RGB2GRAY);
			measPt = find_obj(&detector, &gray, &obj_desc);
		} while (measPt.x == -1 && measPt.y == -1);
		measured_location.push_back(measPt);


		// incorporate measurement into model and do correction
		Mat estimated = KF.correct(measurement);
		Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		kalman_estmte.push_back(statePt);
			
		for (int i = 0; i < kalman_estmte.size()-1; i++) {
			line(frame, kalman_estmte[i], kalman_estmte[i+1], 
					Scalar(0,0,255), 1);
					here(kalman_estmte[i].y);
		}

		for (int i = 0; i < measured_location.size()-1; i++) {
			line(frame, measured_location[i], measured_location[i+1], 
					Scalar(255,0,0), 1);
		}

		circle(frame, measPt, 5, Scalar(0,255,0), 2);

		imshow(WINDOW_NAME, frame);
		imshow("gray", gray);
		// and around we go again!
		key = waitKey(10);
	}
	
	return 0;
}

/** Accesses the global OBJ_FEATURES to track and searches the 
 * global, LAST_IMAGE, to find the object, returns a point representing
 * the center of the object. Takes no arguments. Returns NULL if object
 * is not found. //TODO update this comment to reflect args
 */
Point find_obj(SurfFeatureDetector *det, Mat *scene, Mat *obj_desc){
	//Point result = NULL;
	// Detect features in scene
	vector<KeyPoint> keypoints_scene;
	(*det).detect((*scene), keypoints_scene);

	// Extract descriptors
	SurfDescriptorExtractor extractor;
	Mat descriptors_scene;
	extractor.compute( *scene, keypoints_scene, descriptors_scene );

	// check descriptors exist
	if (descriptors_scene.empty()) {
		Point bad(-1, -1);
		return bad;
	}

	// Now do matching using FLANN
	FlannBasedMatcher matcher;
	vector< DMatch > matches;
	matcher.match( *obj_desc, descriptors_scene, matches );

	// Calcuate distances between keypoints (to remove far ones later)
	double max_dist = 0; double min_dist = 100;
	for( int i = 0; i < obj_desc->rows; i++ ) { 
		double dist = matches[i].distance;
		if( dist < min_dist ) {
			min_dist = dist;
		}
		if( dist > max_dist ) { 
			max_dist = dist;
		}
	}

	// filter matches to only keep good match 
  vector< DMatch > good_matches;
	for(int i = 0; i < obj_desc->rows; i++) { 
		if( matches[i].distance < 3*min_dist ) { 
			good_matches.push_back( matches[i]); \
		}
	}
	
	// Find middle of keypoints
	int x = 0;
	int y = 0;
  for( int i = 0; i < good_matches.size(); i++ ){
		x += keypoints_scene[good_matches[i].trainIdx].pt.x;
		y += keypoints_scene[good_matches[i].trainIdx].pt.y;
		circle(*scene, Point(keypoints_scene[good_matches[i].trainIdx].pt.x,
				keypoints_scene[good_matches[i].trainIdx].pt.y), 4, Scalar(0,0,0), 4);
	}

	Point result(x/good_matches.size(), y/good_matches.size());

	//TODO consider the case when no object is found

	return result;
}


/** initialise the kalman filter with the given point as the initial position.
 * Takes a pointer to a Kalman filter to fill in and a pointer to a Point
 * representing the initial position of the object to be tracked.
*/
void init_kalman(KalmanFilter *KF, Point *init_pos) {
	//Set up kalman filter
	//KalmanFilter KF(6, 2, 0); 
	
	Mat_<float> state(6, 1);
	Mat processNoise(6, 1, CV_32F);
	
	KF->statePre.at<float>(0) = init_pos->x;
	KF->statePre.at<float>(1) = init_pos->y;
	KF->statePre.at<float>(2) = 0;
	KF->statePre.at<float>(3) = 0;
	KF->statePre.at<float>(4) = 0;
	KF->statePre.at<float>(5) = 0;
	// Create transition matrix which relates how meausrements are used
	KF->transitionMatrix = *(Mat_<float>(6, 6) << 
			1,0,1,0,0.5,0,   
			0,1,0,1,0,0.5,  
			0,0,1,0,1,0, 
			0,0,0,1,0,1,  
			0,0,0,0,1,0,  
			0,0,0,0,0,1);
	
	KF->measurementMatrix = *(Mat_<float>(2,6)<<
			1,0,1,0,0.5,0,
			0,1,0,1,0,0.5);

	setIdentity(KF->measurementMatrix);
	setIdentity(KF->processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF->errorCovPost, Scalar::all(.1));
	
	//-----------------------------------------------------------------------
	
	//KalmanFilter KF(4, 2, 0);
	/*
	KF->transitionMatrix = *(Mat_<float>(4, 4) << 
			1,0,1,0,   
			0,1,0,1,  
			0,0,1,0,  
			0,0,0,1);

	Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
	 
	 // init...
	 KF->statePre.at<float>(0) = init_pos->x;
	 KF->statePre.at<float>(1) = init_pos->y;
	 KF->statePre.at<float>(2) = 0;
	 KF->statePre.at<float>(3) = 0;
	 setIdentity(KF->measurementMatrix);
	 setIdentity(KF->processNoiseCov, Scalar::all(1e-4));
	 setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
	 setIdentity(KF->errorCovPost, Scalar::all(.1));
	*/

}

/** Display usage information to std out.
	* Takes nothing, returns nothing.
	*/
void usage() {
	cout << "Usage: iTrack obj_img cam_num" << endl;
}

/** Debuging function
 *
 */
void here(int num) {
	cout << "here " << num << endl;
}
