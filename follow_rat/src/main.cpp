/**************** General C++ includes ***********/
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

/**************** ROS includes *******************/
#include <sensor_msgs/image_encodings.h>
#include <irat_msgs/IRatVelocity.h>
#include <ros/ros.h>

/**************** OpenCV Stuff *******************/
#include <opencv2/opencv.hpp>
// For blob detection 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

/**************** Blobs *************************/
#include <BlobResult.h>

// Misc
int DEBUG = 1;

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  // topic root, e.g., irat_red; can be specified from commandline with _topic:/irat_[colour]
  string topic;
  // initialise node
  ros::init(argc, argv, "DetectStuckness");
  // Make node private 
  ros::NodeHandle node("~");
  // Make topic root parameter; if not specified from cmd/launch file, default to red rat.
  node.param("topic", topic, string("/irat_red")); 

  CBlobResult blobs;
  
  cout << "Ros initialised!" << endl;

  while(ros::ok()) {
      ros::spinOnce();
      waitKey(5);
    } 
	
  return 0;
}
