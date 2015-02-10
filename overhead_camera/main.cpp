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
		return -1;
	}
  
  cout << "YAY" << endl;
	cvNamedWindow("Webcam", 1);
  
  // blob detector parameters 
  SimpleBlobDetector::Params params;
  // for now use defaults 
  // params.minDistBetweenBlobs = 40.0f;
  // etc. 
  
  // Create blob detector 
  SimpleBlobDetector blobDet(params);
  // Vector for storing blob stuff 
  vector<KeyPoint> keyPoints;

	for(EVER) {
		Mat frame, origFrame;
		cap >> origFrame; // get new frame from camera

    // Scale image down for faster processing 
    resize(origFrame, frame, frame.size(), 0.5, 0.5);
    // frame = origFrame;
    
    // Detect blobs 
    blobDet.detect(frame, keyPoints);

    // Extract x, y coordinates from detected blobs, draw circle
    for(int i = 0; i < keyPoints.size(); i++) {
      KeyPoint p = keyPoints[i];
      circle(frame, p.pt, p.size, Scalar(255, 0, 0), 1, 8);
      // circle(frame, p.pt, 50, Scalar(255, 0, 0), 1, 8);
		  imshow("Webcam", frame);
    }

		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
