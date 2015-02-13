/**************** General C++ includes ***********/
#include <iostream>
#include <string>
#include <vector>

/**************** OpenCV Stuff *******************/
#include <opencv2/opencv.hpp>
// For blob detection 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

/**************** Blobs *************************/
#include <BlobResult.h>

/**************** Constants *********************/
#define RADIUS 2
#define EVER ;;
#define HISTORY 100

// kernel size has to be odd.
#define MAX_KERNEL_LENGTH 31
#define MAX_SIGMA_X 10
#define MAX_SIGMA_Y 10
#define MAX_AREA_THRESH 300

using namespace cv;
using namespace std;

RNG rng(12345);

string trackbarWindowName = "Trackbars";

/*************** Dynamically adjustable params ****/
// the std deviations for Gaussian blur
int blurSigmaX, blurSigmaY;
// Gaussian blur kernel size
int kernelSize;
// the area threshold for excluding very small blobs. 
int areaThresh = 100;

struct Track {
  // the id of the blob
  int id;
  // bounding rectangle of the blob 
  Rect boundingRect;
  // number of frames since object became visible
  int age;
  // total number of frames for which the object has been visible
  int totalVisibleFrames;
  // total CONSECUTIVE frames for which object has been invisible
  int consecutiveInvisibleFrames;
  // Something for the Kalman filter.
};

void createTrackbars() {
  cvNamedWindow("Trackbars");
  createTrackbar("kernel size", trackbarWindowName, &kernelSize,
      MAX_KERNEL_LENGTH, NULL); 
  createTrackbar("Sigma X", trackbarWindowName, &blurSigmaX,
      MAX_SIGMA_X, NULL); 
  createTrackbar("Sigma Y", trackbarWindowName, &blurSigmaY,
      MAX_SIGMA_Y, NULL); 
  createTrackbar("Minimum blob area", trackbarWindowName, &areaThresh,
      MAX_AREA_THRESH, NULL);
}

void printUsageMessage() {
  cout << "Usage: ./bg_subtractor <deviceNo or file name>" << endl; 
}

int main(int argc, char** argv) {
  int deviceNo;
  int check;
  VideoCapture cap;

  if(argc == 2) {
    
    check = sscanf(argv[1], "%d", &deviceNo);
    if(check) {
      // One thing (hopefully device number) matched, open video.
      cap.open(deviceNo); 
    } else { 
      // argument was not an integer; assume it was filename.
      cap.open(argv[1]);
    }
  } else {
    printUsageMessage();
    cout << "NB: Your default device is 0." << endl;
    return -1;
  }

	if(!cap.isOpened()) {
    cout << "Something wrong with video capture..." << endl;
		return -1;
	}
  
  cout << "Commencing blob detection..." << endl;
	cvNamedWindow("Webcam", 1);
  cvNamedWindow("Foreground");
  // cvNamedWindow("Background");
  cvNamedWindow("Blobs");

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
  
  CBlob currentBlob;

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

    // Do blob detection 
    Mat blobFrame(frame.size(), frame.type());
    CBlobResult blobs(fore);
    // Filter blobs by size to get rid of little bits 
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, areaThresh);
    int numBlobs = blobs.GetNumBlobs();
    cout << "numBlobs" << numBlobs << endl;
    
    /*
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
    */
    
    // Get bounding boxes for blobs
    vector<Rect> boundingBoxes;
    for(int i = 0; i < numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      currentBlob.FillBlob(blobFrame, Scalar(0, 255, 0));
      boundingBoxes.push_back(currentBlob.GetBoundingBox());
      rectangle(frame, boundingBoxes[i].tl(), boundingBoxes[i].br(),
          colour, 2, 8, 0);
    }

		imshow("Webcam", frame);
    // imshow("Background", back);
    imshow("Foreground", fore);
    imshow("Blobs", blobFrame);

		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
