/**
 * FILE: ros.cpp 
 * AUTHOR: avadendas
 * DESCRIPTION: The most vanilla ros template. 
 */

// C++ Library Includes
#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>

// OpenCV Includes 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS Includes 
#include <ros/ros.h>
// Provides interface between ROS images and OpenCV images
#include <cv_bridge/cv_bridge.h>

// iRat messages
#include <irat_msgs/IRatVelocity.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// for keyboard input 
#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>

// Whether or not to display camera feed
#define VISUALISE true

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

// Name for window that displays camera feed
static const char iRatView[] = "iRat's View";


/**
 * Split the image into NxN grids. 
 *
 * @param N the dimensions of the grid 
 * @param orig the original Mat image to be split up. 
 *
 * @return vector<cv::Mat> 
 */
vector<Mat> split(int N, Mat orig) {
  vector<Mat> grids;
  int numGridRows = orig.rows/N;
  int numGridCols = orig.cols/N;
  int leftOverRowPix = orig.rows % N;
  int leftOverColPix = orig.cols % N;
  int topBuffer = leftOverRowPix/2;
  int leftBuffer = leftOverColPix/2;
 
  /*
  cout << "numGridRows: " << numGridRows << endl;
  cout << "numGridCols: " << numGridCols << endl;
  cout << "topBuffer: " << topBuffer << endl;
  cout << "leftBuffer: " << leftBuffer << endl;
  cout << "rows: " << orig.rows << endl;
  cout << "cols: " << orig.cols << endl;
  */
  
  // If we can't evenly divide up the grid we'll try to 
  // centre the image, and cut off the left-over pixels on
  // the edges. 
  for(int i=0; i < numGridRows; i++) {
    int rowStart = topBuffer + N*i;
    for(int j=0; j < numGridCols; j++) {
      int colStart = leftBuffer + N*j;
      // Make and add the grid 
      /*
      cout << "rowStart: " << rowStart << endl;
      cout << "colStart: " << colStart << endl;
      */
      grids.push_back(Mat (orig, Rect(colStart, rowStart, N, N)));
    }
  }

  return grids;
}

/**
 * Callback for images sent from iRat's camera.
 */
void image_callback(sensor_msgs::ImageConstPtr image) {
  // copy image 
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // CvImagePtr has a cv::Mat image member.
  imshow(iRatView, cvPtr->image);

  Mat cvImage = cvPtr->image;
  vector<Mat> grids = split(4, cvImage);
  /*
  for(int i = 0; i < grids.size(); i++) {
    cout << grids[i] << endl;
  }
  */
}

int main(int argc, char** argv) {
  // Get the iRat name 
  const char * irat_id_const_char_star = getenv("IRAT_ID");
  string irat_id;
  string topic;

  if(!irat_id_const_char_star) {
    std::cout << "IRAT_ID not set, setting to" << " irat_red" << endl;
    irat_id = "irat_red";
  } else {
    irat_id = irat_id_const_char_star;
  }
  
  // Initialise node 
  ros::init(argc, argv, irat_id+"motion_detection");
  ros::NodeHandle node("~");
  
  // set topic root from launch file/command line
  node.param("topic", topic, string("/" + irat_id));
  
  cout << "Creating window" << endl;

  // create window to display image.
  cvNamedWindow(iRatView);
  
  image_transport::ImageTransport it(node);
  // subscribe to image feed 
  image_transport::Subscriber sub = it.subscribe(topic + "/camera/images", 1, image_callback);

  // Rate limiter 
  ros::Rate rate(30.0);

  while(ros::ok()) {
    ros::spinOnce();
    waitKey(5);
  }

  return 0;
}
