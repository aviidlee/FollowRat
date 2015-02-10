/**************** General C++ includes ***********/
#include <iostream>
#include <string>
#include <vector>

/**************** OpenCV Stuff *******************/
#include <opencv2/opencv.hpp>
// For blob detection 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

/**************** Constants *********************/
#define RADIUS 2
#define EVER ;;
#define HISTORY 100

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  int deviceNo;
  cout << "Commencing blob detection from video feed..." << endl;
  if(argc == 2) {
    deviceNo = atoi(argv[1]);
  } else {
    cout << "Usage: overhead <deviceNumber>" << endl;
    return -1;
  }

	VideoCapture cap(deviceNo); // hopefully open the webcam; 0 is default.
	if(!cap.isOpened()) {
    cout << "Something wrong with video capture..." << endl;
		return -1;
	}
  
  cout << "YAY" << endl;
	cvNamedWindow("Webcam", 1);
  cvNamedWindow("Background");

  vector<vector<Point> > contours;
  Mat frame, back, fore;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  // I have no idea what this number does. 
  // mixture components, protected, can't seem to set it via constructor.
  // bg.nmixtures = 3;
  // bg.bShadowDetection = false;

	for(EVER) {
		cap >> frame; // get new frame from camera
    
    bg.operator () (frame, fore);
    bg.getBackgroundImage(back);
    erode(fore, fore, Mat());
    dilate(fore, fore, Mat());
    findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    drawContours(frame, contours, -1, Scalar(0, 0, 255), 2);

		imshow("Webcam", frame);
    imshow("Background", back);

		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
