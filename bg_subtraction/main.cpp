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
#define MAX_AREA_THRESH 500
#define MAX_WC 200
#define MAX_WH 200
#define MIN_OBJ_AREA 100
#define MAX_OBJ_AREA (640*480/1.5)

// codes for keyboard presses 
#define keyP 1048688
#define keyEsc 1048603

// Parameters for colour filtering.
int bmin = 0;
int gmin = 54;
int rmin = 0;
int bmax = 66;
int gmax = 255;
int rmax = 50;

// For ROI enlargement
int WC = 40; // amount to increase width by
int WH = 30; // amount to increase height by

// Misc
int DEBUG = 0;

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
int areaThresh = 150;
// pause program 
bool paused;

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
  createTrackbar("Debug mode", trackbarWindowName, &DEBUG,
      1, NULL);
  createTrackbar("Increase ROI width", trackbarWindowName, &WC,
      MAX_WC, NULL);
  createTrackbar("Increase ROI height", trackbarWindowName, &WH,
      MAX_WH, NULL);
  createTrackbar("kernel size", trackbarWindowName, &kernelSize,
      MAX_KERNEL_LENGTH, NULL); 
  createTrackbar("Sigma X", trackbarWindowName, &blurSigmaX,
      MAX_SIGMA_X, NULL); 
  createTrackbar("Sigma Y", trackbarWindowName, &blurSigmaY,
      MAX_SIGMA_Y, NULL); 
  createTrackbar("Minimum blob area", trackbarWindowName, &areaThresh,
      MAX_AREA_THRESH, NULL);
  
  createTrackbar("Red min", trackbarWindowName, &rmin,
      255, NULL);
  createTrackbar("Red max", trackbarWindowName, &rmax,
      255, NULL);
  createTrackbar("Green min", trackbarWindowName, &gmin,
      255, NULL);
  createTrackbar("Green max", trackbarWindowName, &gmax,
      255, NULL);
  createTrackbar("Blue min", trackbarWindowName, &bmin,
      255, NULL);
  createTrackbar("Blue max", trackbarWindowName, &bmax,
      255, NULL);

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
	cvNamedWindow("Processed", 1);
  // cvNamedWindow("Foreground");
  // cvNamedWindow("Filtered");
  // cvNamedWindow("Background");
  // cvNamedWindow("Blobs");
  cvNamedWindow("Original frame");
  cvNamedWindow("Filtered frame");

  createTrackbars();

  vector<vector<Point> > contours;
  Mat frame, back, fore, origFrame, filteredFrame;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
      rng.uniform(0, 255));
  
  CBlob currentBlob;
  bool readFrame;

	for(EVER) {
		readFrame = cap.read(origFrame); // get new frame from camera
    // check if it's end of the video
    if(!readFrame) {
      cout << "End of video file/video stream disrupted." << endl;
      // Pause until user ends program by pressing a key.
      if(waitKey(0)) {
        break;
      }
    }
    // make the kernel size odd. Else crash.  
    int kernel = kernelSize % 2 == 0 ? kernelSize + 1 : kernelSize;

    /* Pre-process the image */
    GaussianBlur(origFrame, frame, Size(kernel, kernel), blurSigmaX, blurSigmaY);
    // Convert frame to greyscale
    cvtColor(frame, frame, CV_BGR2GRAY);
  
    /* Do background subtraction */
    bg.operator () (frame, fore);
    bg.getBackgroundImage(back);

    // Morph ops to get rid of noise
    Mat eroder = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat dilater = getStructuringElement(MORPH_ELLIPSE, Size(6, 6));
    // Can just supply Mat() instead of eroder/dilator.
    erode(fore, fore, eroder);
    dilate(fore, fore, dilater);

    /* Do blob detection on the foreground */
    Mat blobFrame(origFrame.size(), origFrame.type());
    CBlobResult blobs(fore);
    // Filter blobs by size to get rid of little bits 
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, areaThresh);
    int numBlobs = blobs.GetNumBlobs();
    if(DEBUG) {
      cout << "numBlobs" << numBlobs << endl;
    }

    // Coordinates of top left of enlarged rectangle 
    int x, y; 
    // width and height of enlarged rectangle. bad names.
    int right, bottom;
    int rows = origFrame.rows;
    int cols = origFrame.cols;

    // Get bounding boxes for blobs
    // Don't need them stored to draw them, but we will need them later.
    vector<Rect> boundingBoxes;
    for(int i = 0; i < numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      currentBlob.FillBlob(frame, Scalar(0, 255, 0));
      Rect currentRect = currentBlob.GetBoundingBox();
      boundingBoxes.push_back(currentBlob.GetBoundingBox());
      rectangle(frame, boundingBoxes[i].tl(), boundingBoxes[i].br(),
          colour, 2, 8, 0);

      /* Now check the colour of the original image within this ROI
         to see if it's the iRat
      */
      inRange(origFrame, Scalar(bmin, gmin, rmin), Scalar(bmax, gmax, rmax),
          filteredFrame);

      // Add 2*WC and 2*WH to width and height of ROI 
      // Make the bounding box of blob bigger
      x = currentRect.tl().x-WC > 0 ? currentRect.tl().x-WC : 0;
      y = currentRect.tl().y-WH > 0 ? currentRect.tl().y-WH : 0;
      int width = currentRect.width;
      int height = currentRect.height;
      right = currentRect.width+WC < cols ? currentRect.width+WC : cols-currentRect.br().x + width;
      bottom = height+WH < rows ? height+WH : rows - currentRect.br().y + height;
      
      if(DEBUG) {
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        cout << "right: " << right << endl;
        cout << "bottom: " << bottom << endl;
      }
      
      Rect enlarged(x, y, right, bottom);
      // Make a new mat from the original frame clipping out the 
      // enlarged ROI 
      Mat bigMat(origFrame, enlarged);
      Mat bigMatRes(bigMat);
      inRange(bigMat, Scalar(bmin, gmin, rmin), Scalar(bmax, gmax, rmax),
            bigMatRes);
      
      /* Erode and dilate the filtered ROI, then calculate its area.
       * if it is a reasonable size, we say we found the rat.
       */
      erode(bigMatRes, bigMatRes, eroder);
      dilate(bigMatRes, bigMatRes, dilater);
      vector<vector<Point> > contours;
      vector<Vec4i> hierachy;

      findContours(bigMatRes, contours, hierachy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
      if(hierachy.size() > 0) {
        int numObj = hierachy.size();
        for(int j = 0; j >= 0; j = hierachy[j][0]) {
          Moments moment = moments((Mat) contours[j]);
          double area = moment.m00;
          if(area > MIN_OBJ_AREA && area < MAX_OBJ_AREA) {
            // Draw a circle on the filtered blob. 
            circle(origFrame, currentBlob.getCenter(), 30, Scalar(0, 0, 255));
          }
        }
      }

      // Draw enlarged rectangle on original frame.
      rectangle(origFrame, enlarged.tl(), enlarged.br(), colour,
          2, 8, 0);
    }
    
    imshow("Filtered frame", filteredFrame);
		imshow("Processed", frame);
    // imshow("Background", back);
    // imshow("Foreground", fore);
    // imshow("Blobs", blobFrame);
    imshow("Original frame", origFrame);
    
    int keyPressed = waitKey(30);
    switch(keyPressed) {
      case keyEsc: // Esc key - quit program
        return 0;
      case keyP: // p - for pause 
        paused = !paused;
        if(paused) {
          cout << "Program paused." << endl;
          while(paused) {
            switch(waitKey(0)) {
              case keyP:
                paused = false;
                break;
            }
          }
        }
      case -1: // no key pressed 
        break;
      default:
        cout << "Key pressed: " << keyPressed << endl;
		}
	}

	return 0;
}
