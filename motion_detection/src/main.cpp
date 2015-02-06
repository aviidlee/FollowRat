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

// OpenCV Includes 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS Includes 
#include <ros/ros.h>

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

  // show image
  cout << "Showing image..." << endl;
  imshow(iRatView, cvPtr->image);
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
  node.param("topic", topic, string("/irat_red"));
  
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
