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

// kernel size has to be odd.
#define MAX_KERNEL_LENGTH 31
#define MAX_SIGMA_X 10
#define MAX_SIGMA_Y 10

using namespace cv;
using namespace std;

RNG rng(12345);

string trackbarWindowName = "Trackbars";

// the std deviations for Gaussian blur
int blurSigmaX, blurSigmaY;
// Gaussian blur kernel size
int kernelSize;

void createTrackbars() {
  cvNamedWindow("Trackbars");
  createTrackbar("kernel size", trackbarWindowName, &kernelSize,
      MAX_KERNEL_LENGTH, NULL); 
  createTrackbar("Sigma X", trackbarWindowName, &blurSigmaX,
      MAX_SIGMA_X, NULL); 
  createTrackbar("Sigma Y", trackbarWindowName, &blurSigmaY,
      MAX_SIGMA_Y, NULL); 
}

int main(int argc, char** argv) {
  int deviceNo;
  cout << "Commencing blob detection from video feed..." << endl;
  if(argc == 2) {
    deviceNo = atoi(argv[1]);
  } else {
    cout << "Usage: bg_subtractor <deviceNumber>" << endl;
    cout << "NB: Your default device is 0." << endl;
    return -1;
  }

	VideoCapture cap(deviceNo); // hopefully open the webcam; 0 is default.
	if(!cap.isOpened()) {
    cout << "Something wrong with video capture..." << endl;
		return -1;
	}
  
  cout << "YAY" << endl;
	cvNamedWindow("Webcam", 1);
  cvNamedWindow("Foreground");
  cvNamedWindow("Background");
  createTrackbars();

  vector<vector<Point> > contours;
  Mat frame, back, fore, origFrame;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  // I have no idea what this number does. 
  // mixture components, protected, can't seem to set it via constructor.
  // bg.nmixtures = 3;
  // bg.bShadowDetection = false;

  Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
      rng.uniform(0, 255));

	for(EVER) {
		cap >> origFrame; // get new frame from camera
    // make the kernel size odd. Else crash.  
    int kernel = kernelSize % 2 == 0 ? kernelSize + 1 : kernelSize;
    GaussianBlur(origFrame, frame, Size(kernel, kernel), blurSigmaX, blurSigmaY);
    
    // Convert to greyscale
    cvtColor(frame, frame, CV_BGR2GRAY);
  
    // Do background subtraction
    bg.operator () (frame, fore);
    bg.getBackgroundImage(back);
    erode(fore, fore, Mat());
    dilate(fore, fore, Mat());
    findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  
    // Get bounding box 
    vector<vector<Point> > contoursPoly(contours.size());
    vector<Rect> boundingRects(contours.size());
    for(int i = 0; i < contours.size(); i++) {
      approxPolyDP(Mat(contours[i]), contoursPoly[i], 3, true);
      boundingRects[i] = boundingRect(Mat(contoursPoly[i]));
    }

    drawContours(frame, contours, -1, Scalar(0, 0, 255), 2);

    // Draw the bounding box
    for(int i = 0; i < contours.size(); i++) {
      rectangle(frame, boundingRects[i].tl(), boundingRects[i].br(),
          colour, 2, 8, 0);
    }

		imshow("Webcam", frame);
    imshow("Background", back);
    imshow("Foreground", fore);

		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
