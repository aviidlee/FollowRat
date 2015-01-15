// iRat-related includes 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <irat_msgs/IRatVelocity.h>

// OpenCV and C++ related headers 
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string> 
#include <irat_msgs/IRatDistance.h>

// Custom message for publishing optical flow data
#include <stuck/IRatOpticalFlow.h>

// Maximum number of features to track 
#define MAX_FEATURES 500
// The rat must get MAX_COUNT consecutive instances in which it has nonzero command velocity 
// and zero (or low, should probably set threshold) optical flow before it decides it is stuck.
#define MAX_COUNT 10
// Whether or not we should display the camera feed and the features being tracked
#define VISUALISE true

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

// Stopping condition for iterative algorithm optical flow
// Termination criteria are:
// 		CV_TERMCRIT_ITER: terminate after iterating twice the number of max_iter
//		CV_TERMCRIT_EPS: terminate if accuracy falls below epsilon.
TermCriteria termCrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
// Size of search window at each pyramid level for optical flow algorithm
Size winSize(31, 31);
// Vectors to store tracked features 
vector<Point2f> oldFeatures, newFeatures;
// Vectors to pass into optical flow algorithm 
vector<uchar> status;
vector<float> err;
// Objects to store images
Mat frame, greyFrame, prevFrame;
// True if we have a set of features to compare current ones to
bool prevExists = false;
// Name for the window that displays the camera feed
static const char iRatView[] = "iRat's View";
// Message to publish 
stuck::IRatOpticalFlow flowMsg;
// Publisher for optical flow 
ros::Publisher pub_flow;

// Store the translational and rotational command velocity 
double vtrans, vrot;

/** 
 * Store the number of times we've had stuckness detected consecutively
 * i.e., command velocity is not zero but optical flow is. 
 *
 * If we say the iRat is stuck as soon as we get an instance of 
 * command velocity != 0 && optical flow == 0, then we will get a lot of 
 * false positives, e.g., when the iRat is changing directions or being 
 * issued a velocity command from a stand still. 
 */
int stuckCount = 0;

/**
 * Callback for command velocities; updates the global variables 
 * vtrans and vrot. 
 * 
 * @param vel the command velocity message. 
 */
void cmdvel_callback(irat_msgs::IRatVelocityConstPtr vel) {
    vtrans = vel->magnitude;
    vrot = vel->angle;
    return;
}

/** 
 * Callback for images sent from the iRat's camera. Chooses features 
 * to track from the images, perform optical flow calculation, determine 
 * whether or not the iRat is stuck and publishes the information to the 
 * flow topic. 
 * 
 * @param image the image from the iRat's camera
 */
void image_callback(sensor_msgs::ImageConstPtr image) {
	// Some tolerance because command velocity is always nonzero.
	double eta = 0.01;

	// Copy image into mutable variable 
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Do optical flow! 
	// Convert to greyscale 
	cvtColor(cv_ptr->image, greyFrame, CV_BGR2GRAY);
	// Need to see if we have any previous images
	if(!prevExists) {
		/*
		 * Find some good features to track (OpenCV function).
		 * Tends to choose edges and points of high contrast; if the iRat receives a very bland,
		 * low-contrast image such as a white wall, the feature selection becomes essentially random. 
		 */
		goodFeaturesToTrack(greyFrame, newFeatures, MAX_FEATURES, 0.01, 5, Mat(), 3, 0, 0.04);
		prevExists = true;
	} else if (!oldFeatures.empty()) { // see if we have anything to compare to
		// Do optical flow. newFeatures is output vector containing calculated new positions
		// This is an OpenCV function. 
		calcOpticalFlowPyrLK(prevFrame, greyFrame, oldFeatures, newFeatures, status, err, winSize, 3, termCrit, 0, 0.001);

		// Variables to store the differences in position of the tracked features
		int xDiff, yDiff;
		// The sum of the magnitudes of the vectors 
		unsigned int totalDiff = 0;
		// The sums of x and y components, signs retained. 
		int xSum = 0;
		int ySum = 0;

		// Calculate how much the features have moved. 
		for(vector<Point2f>::size_type i = 0; i < oldFeatures.size(); i++) {
			xDiff = int(newFeatures[i].x - oldFeatures[i].x);
			yDiff = int(newFeatures[i].y - oldFeatures[i].y);
			totalDiff += (fabs(xDiff) + fabs(yDiff));
			xSum += xDiff;
			ySum += yDiff; 

			if(VISUALISE) {
				// Draw circles around tracked features and lines to indicate flow.
				if((newFeatures[i].x - oldFeatures[i].x) > 0) {
					line(cv_ptr->image, newFeatures[i], oldFeatures[i], 
						Scalar(0, 0, 255), 1, 1, 0);

					circle(cv_ptr->image, newFeatures[i], 2, Scalar(255, 0, 0), 1, 1, 0);
				} else {
					line(cv_ptr->image, newFeatures[i], oldFeatures[i], Scalar(0, 255, 0), 1, 1, 0);
					circle(cv_ptr->image, newFeatures[i], 2, Scalar(255, 0, 0), 1, 1, 0);
				}
			}
		}
		
		// Prepare message for publication
		flowMsg.header.stamp = ros::Time::now();
		flowMsg.flowSum = totalDiff;
		flowMsg.xSum = xSum;
		flowMsg.ySum = ySum;

        // See if we are stuck 
        if(totalDiff==0 && (fabs(vtrans) > eta || fabs(vrot) > eta)) {
        	stuckCount++;
        	if(stuckCount > MAX_COUNT) { 
        		std::cout << "We are stuck!!" << endl;
        		flowMsg.stuck = true;
        	}  	
        } else { // we are not stuck
        	// Reset stuckCount 
        	stuckCount = 0;
        	flowMsg.stuck = false;
        }

		pub_flow.publish(flowMsg);
		flowMsg.header.seq++;

		// Get some new good features to track
		goodFeaturesToTrack(greyFrame, newFeatures, MAX_FEATURES, 0.01, 10, Mat(), 3, 0, 0.04);
	}

	swap(oldFeatures, newFeatures);
	newFeatures.clear();
	// Display what the iRat is seeing
	imshow(iRatView, cv_ptr->image);
	// Make current frame the old frame. 
	greyFrame.copyTo(prevFrame);
}

int main(int argc, char** argv) {

	// topic root, e.g., irat_red; can be specified from commandline with _topic:/irat_[colour]
	string topic;

	// initialise node
	ros::init(argc, argv, "DetectStuckness");
	// Make node private 
	ros::NodeHandle node("~");
	// Make topic root parameter; if not specified from cmd/launch file, default to red rat.
	node.param("topic", topic, string("/irat_red"));

	//window code change
	//namedWindow(iRatView, CV_WINDOW_AUTOSIZE);
	cvNamedWindow( iRatView );
	// cvStartWindowThread( );
	// waitKey(5);

	pub_flow = node.advertise<stuck::IRatOpticalFlow>(topic + "/flow", 1);
	
	image_transport::ImageTransport it(node);
	// subscribe to image feed
	image_transport::Subscriber sub = it.subscribe(topic + "/camera/images", 1, image_callback);
    // subscribe to command velocity 
    ros::Subscriber cmdvel_sub = node.subscribe(topic + "/serial/cmdvel", 1, cmdvel_callback);
	ros::spin();
	while(ros::ok()) {
		waitKey(5);
	}
	return 0;
}

