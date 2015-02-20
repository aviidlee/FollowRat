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
#include <opencv2/imgproc/imgproc.hpp>

/**************** Blobs *************************/
#include <BlobResult.h>

/**************** Constants *********************/
// Syntactic sugar
#define EVER ;;
// Parameter for background subtraction
#define HISTORY 100
// Maximum distance possible (squared) in a standard webcam frame.
#define MAX_DIST 640*640
// Shorthand for font used for displaying text labels on frames
#define FONT FONT_HERSHEY_SIMPLEX

/****** Parameters for pre-processing images ****/
// kernel size has to be odd.
#define MAX_KERNEL_LENGTH 31
#define MAX_SIGMA_X 10
#define MAX_SIGMA_Y 10
// trackbar slider maximum for area filtering blobs. 
#define MAX_AREA_THRESH 500
// maximum amount by which to pad width of bounding rectangles
#define MAX_WC 200
// maximum amount by which to pad height of bounding rectangles
#define MAX_WH 200
// minimum area required for coloure-filtered object to be treated 
// as an object and not as noise.
#define MIN_OBJ_AREA 100
/// minimum area allowed for coloure-filtered object to be treated 
// as an object and not as noise.
#define MAX_OBJ_AREA (640*480/1.5)
// video feed framerate in Hz
#define FRAMERATE 15

// codes for keyboard presses 
#define keyP 1048688
#define keyEsc 1048603

// number of previous locations of blobs to keep track of.
#define MAX_AGE
#define VISUALISE true

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
int DEBUG = 1;

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
int areaThresh = 180;
// pause program 
bool paused;

/**
 * Represents an object identified in the frame. 
 */
class Track {
  public:
    // the id of the blob
    int id;
    // The type of object we think it is. 
    string type;
    // bounding rectangle of the blob 
    Rect boundingRect;
    // centroid 
    Point centroid;
    // number of frames since object became visible
    int age;
    // total number of frames for which the object has been visible
    int totalVisibleFrames;
    // total CONSECUTIVE frames for which object has been invisible
    int consecutiveInvisibleFrames;

    Track(int objNo) : id(objNo) {}
    
};

/******************** Other stuff...********************/
// Number of rats expected to be in frame. 
int numRats;
// Number of iRats expected to be in frame. 
int numiRats;
// The current tracks for the iRats
vector<Track> nowTrack;
// Tracks from last frame for the iRats  
vector<Track> prevTracks;
// Previous frame's tracks for rats 
vector<Track> prevRatTracks;
// Current tracks for rats 
vector<Track> nowRatTrack;
// Stores the mouse click positions; required for updating the rats' positions.
vector<Point> mouseClickRats;
// Whether or not the mouse has been clicked during this frame.
bool mouseClicked;

/** 
 * Optical flow related 
 */
// Maximum number of features to track 
#define MAX_FEATURES 500
// Stopping condition for iterative algorithm optical flow 
// Termination criteria are:
//    CV_TERMCRIT_ITER: terminate after iterating twice the number of max_iter
//    CV_TERMCRIT_EPS: terminate if accuracy falls below epsilon.
TermCriteria termCrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
// Size of search window at each pyramid level for optical flow algorithm
Size winSize(31, 31);
// Vectors to pass into optical flow algorithm 
vector<uchar> status;
vector<float> err;
vector<Point2f> oldFeatures;
vector<Point2f> newFeatures;


/**
 * Return the vrot required to follow the rat. 
 * Note negative vrot is right. 
 * 
 * @param  iRatLoc the location of the iRat; coordinates of blob centre.
 * @param  iRatHeading the vector describing the heading direction of the iRat.
 * @param  ratLoc      the location of the rat; coordinates of blob centre. 
 * @param  mag         the maximum magnitude of the rotational velocity.
 *                      ... was what it's supposed to be. Now it's just some 
 *                      magnitude multiplier. Oops.
 * @return             the required rotational velocity of the iRat.
 */
double getDir(Point iRatLoc, Point iRatHeading, Point ratLoc, double mag) {
  Point diff = ratLoc - iRatLoc;
  Point change = diff - iRatHeading;
  double turn = -1*iRatHeading.y*change.x < 0 ? -1 : 1;
  // Adjust turning velocity depending on how far to the left/right the rat is
  // of the iRat.
  double vrot = mag*turn*(change.x/50) < mag ? mag*turn*(change.x/50) : mag;
  return vrot;
}

/**
 * Click to specify where the rats are. Left mouse button specifies where 
 * rat 1 is, and right mouse button specifies where rat 2 is. 
 */
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
  mouseClicked = true;
  // Click to specify position of rat 1
  if(event == EVENT_LBUTTONDOWN && numRats > 0) {
    if(DEBUG) {
      cout << "Left mouse button clicked at (x, y) = " 
                   << x << "," << y << endl;
    }
    mouseClickRats[0] = Point(x, y);
  }
  
  // Click to specify position of rat 2
  if(event == EVENT_RBUTTONDOWN && numRats > 1) {
    if(DEBUG) {
      cout << "Right mouse button clicked at (x, y) = " 
                    << x << "," << y << endl;
    }
    mouseClickRats[1] = Point(x, y);
  }

}

/**
 * Create the trackbars for adjusting processing parameters 
 * such as Gaussian blur parameters and colour filtering 
 * parameters.
 */
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
  cout << "Usage: ./bg_subtractor <deviceNo or file name> <num expected iRats>" 
       << "<num expected rats> [<output filename>]" << endl; 
}

/**
 * Dumb dumb update. Use previous known position of objects 
 * to guess which of the current blobs they are. Loop through 
 * all the current blobs; the one closest to the previous known
 * location of a tracked object (i.e., rat or iRat) is updated 
 * as the new positions. 
 * 
 * @param blobs the list of blobs identified in the current blob.
 *        the blobs are expected to have gone through all filtering.
 * @required this function is called after the iRat's position has been 
 *           updated.
 */
void updatePositions(CBlobResult blobs) {
  int numBlobs = blobs.GetNumBlobs();

  for(int i = 0; i < numRats; i++) {
    Point candidate;
    double minDist = MAX_DIST;
    double dist;
    for(int j = 0; j < numBlobs; j++) {
      Point blobCentre = blobs.GetBlob(j)->getCenter();
      Point diff = prevRatTracks[i].centroid - blobCentre;
      dist = diff.ddot(diff);
      if(dist < minDist) { 
        // Check if candidate is iRat; if so, ignore it. 
        if(blobCentre == nowTrack[0].centroid) {
          continue;
        } else {
          candidate = blobCentre;
          minDist = dist;
        }
      }
    }
    nowRatTrack[i].centroid = candidate;
  }
  
  return;
}

/**
 * Perform dilation and erosion to get rid of noise and fill holes.
 * 
 * @param src the source frame to perform morph ops on.
 * @param dest the destination frame to put morphed image in.
 * @param eroder the structuring element used in erode operation 
 * @param dilater the structuring element used in dilate operation.
 */
void morphOps(Mat& src, Mat& dest, const Mat& eroder=getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), 
  const Mat dilater=getStructuringElement(MORPH_ELLIPSE, Size(6, 6))) {
  erode(src, dest, eroder);
  dilate(src, dest, dilater);
  return;
}

/**
 * Preprocess the image by doing Gaussian blur and turning it into greyscale. 
 * Parameters come from trackbar sliders. I.e., dynamically adjustable at runtime.
 */
void preprocess(Mat& src, Mat& dest) {
  // make the kernel size odd. Else crash.  
  int kernel = kernelSize % 2 == 0 ? kernelSize + 1 : kernelSize;
  GaussianBlur(src, dest, Size(kernel, kernel), blurSigmaX, blurSigmaY);
  // Convert frame to greyscale
  cvtColor(dest, dest, CV_BGR2GRAY);
  return;
}


/**
 * Enlarge the given rectangle by increasing its width and height by a 
 * given amount in a manner such that it still fits the given frame,
 * and return a Mat corresponding to that enlarged rectangle. 
 *
 *
 * @param rect the original rectangle
 * @param origFrame the frame from which to clip out the enlarged ROI.
 * 
 * @return the region clipped out from origFrame 
 */
Rect enlargeRect(Mat& origFrame, Rect& rect) {
    int rows = origFrame.rows;
    int cols = origFrame.cols;
    // Coordinates of top left of enlarged rectangle 
    int x, y; 
    // width and height of enlarged rectangle. 
    int width, height;
    // Add 2*WC and 2*WH to width and height of ROI 
    // Make the bounding box of blob bigger
    x = rect.tl().x-WC > 0 ? rect.tl().x-WC : 0;
    y = rect.tl().y-WH > 0 ? rect.tl().y-WH : 0;
    int origWidth = rect.width;
    int origHeight = rect.height;
    width = origWidth+WC < cols ? origWidth+WC : cols - rect.br().x + origWidth;
    height = origHeight+WH < rows ? origHeight+WH : rows - rect.br().y + origHeight;

    Rect enlarged(x, y, width, height);
    return enlarged;
}

void opticalFlow(Mat& greyFrame, Mat& prevFrame, Mat& displayFrame) {
  // Check if we have a previous frame
  if(!prevFrame.empty()) {
    goodFeaturesToTrack(greyFrame, newFeatures, MAX_FEATURES, 0.01, 5, Mat(), 3, 
                        0, 0.04);
    if(newFeatures.empty()) {
      cout << "Can't find any good features do detect!!!" << endl;
    }
  } else if(!oldFeatures.empty()) {
    calcOpticalFlowPyrLK(prevFrame, greyFrame, oldFeatures, newFeatures, status, 
                         err, winSize, 3, termCrit, 0, 0.001);

    // Variables to store the differences in position of the tracked features
    int xDiff, yDiff;
    // The sum of the magnitudes of the vectors 
    unsigned int totalDiff = 0;
    // The sums of x and y components, signs retained. 
    int xSum = 0;
    int ySum = 0;
    cout << "Doing optical flow..." << endl;
    // Calculate how much the features have moved. 
    for(vector<Point2f>::size_type i = 0; i < oldFeatures.size(); i++) {
      xDiff = int(newFeatures[i].x - oldFeatures[i].x);
      yDiff = int(newFeatures[i].y - oldFeatures[i].y);
      totalDiff += (fabs(xDiff) + fabs(yDiff));
      xSum += xDiff;
      ySum += yDiff; 

      cout << "Drawing optical flow stuf..." << endl;
      // Draw circles around tracked features and lines to indicate flow.
      if((newFeatures[i].x - oldFeatures[i].x) > 0) {
        line(displayFrame, newFeatures[i], oldFeatures[i], 
          Scalar(0, 0, 255), 1, 1, 0);

        circle(displayFrame, newFeatures[i], 2, Scalar(255, 0, 0), 1, 1, 0);
      } else {
        line(displayFrame, newFeatures[i], oldFeatures[i], Scalar(0, 255, 0), 1, 1, 0);
        circle(displayFrame, newFeatures[i], 2, Scalar(255, 0, 0), 1, 1, 0);
      }

    } 
    // Get some new good features to track
    goodFeaturesToTrack(greyFrame, newFeatures, MAX_FEATURES, 0.01, 10, Mat(), 3, 0, 0.04);  
  }
  swap(oldFeatures, newFeatures);
  newFeatures.clear();
  if(oldFeatures.empty()) {
    cout << "what?! no old features???" << endl;
  }
  return;
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
  string iRatVelTopic = topic + "/serial/cmdvel";
  cout << "Publishing to: " << iRatVelTopic << endl;
  // register a publisher for the iRat’s command velocity
  // ros on the iRat would subscribe to this
  ros::Publisher pub_cmdvel;
  // message to publish
  irat_msgs::IRatVelocity cmdvel_msg;// instantiate once only because of header squence number
  pub_cmdvel = node.advertise<irat_msgs::IRatVelocity>(iRatVelTopic, 1);

  cout << "Ros initialised!" << endl;
  // the video device number 
  int deviceNo;
  // the video feed we're reading frames from. 
  VideoCapture cap;
  // optionally able to write the frame, with bounding boxes etc. added, to file.
  VideoWriter writer;
  // indexing variable used for initialising Tracks; do not remove.
  int k;

  // colours...
  Scalar green = Scalar(0, 255, 0);
  Scalar blue = Scalar(255, 0, 0);
  Scalar black = Scalar(0, 0, 0);
  Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
      rng.uniform(0, 255));

  // Objects to store images
  Mat prevFrame;

  if(argc >= 4) {

    if(sscanf(argv[1], "%d", &deviceNo)) {
      // One thing (hopefully device number) matched, open video.
      cap.open(deviceNo); 
    } else { 
      // argument was not an integer; assume it was filename.
      cap.open(argv[1]);
    }

    if(!sscanf(argv[2], "%d", &numiRats)) {
      cout << "Something wrong with provided number of expected iRats" << endl;
      return -1;
    }
    if(!sscanf(argv[3], "%d", &numRats)) {
      cout << "Something wrong with provided number of expected rats" << endl;
      return -1;
    }
    
    cout << "Initialising tracks..." << endl;

    // Initialise the tracks
    for(k = 0; k < numiRats; k++) {
      nowTrack.push_back(Track(k));
    }
    for(int j = k; j < k + numRats; j++) {
      nowRatTrack.push_back(Track(j));
      mouseClickRats.push_back(Point(0, 0));
    }

    cout << "Number of iRats: " << nowTrack.size() << endl;
    cout << "Number of rats: " << nowRatTrack.size() << endl;

  } else {
    printUsageMessage();
    cout << "NB: Your default device is 0." << endl;
    return -1;
  }

	if(!cap.isOpened()) {
    cout << "Something wrong with video capture..." << endl;
		return -1;
	}

  // Optional command - write video to file. 
  if(argc == 5) {
    int frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    writer.open(argv[4], CV_FOURCC('M', 'J', 'P', 'G'), FRAMERATE, 
        Size(frameWidth, frameHeight));
  }
  
  cout << "Commencing blob detection..." << endl;
	cvNamedWindow("Processed", 1);
  cvNamedWindow("Original frame");
  cvNamedWindow("Filtered frame");

  createTrackbars();
  setMouseCallback("Original frame", mouseCallback, NULL);

  vector<vector<Point> > contours;
  Mat frame, back, fore, origFrame, filteredFrame;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  
  CBlob currentBlob;
  bool readFrame;

	while(ros::ok()) {
    ros::spinOnce();
		readFrame = cap.read(origFrame); // get new frame from camera
    // check if it's end of the video
    if(!readFrame) {
      cout << "End of video file/video stream disrupted." << endl;
      // Pause until user ends program by pressing a key.
      if(waitKey(0)) {
        break;
      }
    }

    preprocess(origFrame, frame);
  
    /* Do background subtraction */
    bg.operator () (frame, fore);
    bg.getBackgroundImage(back);

    // Morph ops to get rid of noise; erode and dilate.
    morphOps(fore, fore);

    /* Do blob detection on the foreground */
    Mat blobFrame(origFrame.size(), origFrame.type());
    CBlobResult blobs(fore);
    // Filter blobs by size to get rid of little bits 
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, areaThresh);
    int numBlobs = blobs.GetNumBlobs();
   
    // Number of coloured objects seen so far. 
    int colouredCount = 0;
    // list of locations of coloured objects 
    vector<Point> coloured;
    // The minimum squared distance between a coloured blob and previous iRat location.
    double minDist = 640*640;

    // Get bounding boxes for blobs
    // Don't need them stored to draw them, but we will need them later.
    vector<Rect> boundingBoxes;
    for(int i = 0; i < numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      currentBlob.FillBlob(frame, Scalar(0, 255, 0));
      Rect currentRect = currentBlob.GetBoundingBox();
      boundingBoxes.push_back(currentBlob.GetBoundingBox());
      // Draw the unmodified bounding box onto the processed images frame.
      rectangle(frame, boundingBoxes[i].tl(), boundingBoxes[i].br(),
          colour, 2, 8, 0);

      /* 
      * Colour filter the entire original frame so that we know what 
      * the colour filtering is doing.
      */
      inRange(origFrame, Scalar(bmin, gmin, rmin), Scalar(bmax, gmax, rmax),
          filteredFrame);

      // Make a new mat from the original frame clipping out the 
      // enlarged ROI 
      Rect enlarged = enlargeRect(origFrame, currentRect);
      Mat bigMat(origFrame, enlarged);
      Mat bigMatRes(bigMat);
      
      inRange(bigMat, Scalar(bmin, gmin, rmin), Scalar(bmax, gmax, rmax),
            bigMatRes);
      
      /* Erode and dilate the filtered ROI, then calculate its area.
       * if it is a reasonable size, we say we found the rat.
       */
      morphOps(bigMatRes, bigMatRes);
      vector<vector<Point> > contours;
      vector<Vec4i> hierachy;
      
      // cout << "Finding contours for filtered blobs..." << endl;
      findContours(bigMatRes, contours, hierachy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
      if(hierachy.size() > 0) {
        int numObj = hierachy.size();
        for(int j = 0; j >= 0; j = hierachy[j][0]) {
          Moments moment = moments((Mat) contours[j]);
          double area = moment.m00;
          if(area > MIN_OBJ_AREA && area < MAX_OBJ_AREA) {
            Point centre = currentBlob.getCenter();
            // Draw a circle on the filtered blob. 
            circle(origFrame, centre, 30, Scalar(0, 0, 255));
            // cout << "Coloured count is " << colouredCount << endl;
            if(colouredCount && !prevTracks.empty()) { 
              // cout << "Already seen something coloured... calculating distances" << endl;
              // We've already seen a coloured object... which one is the iRat?
              // See how far the current blob is from the previous iRat's location.
              // cout << "Size of coloured " << coloured.size() << endl;
              Point diff = coloured[colouredCount-1] - prevTracks[0].centroid;
              double distSq = diff.ddot(diff); 
              if(distSq < minDist) {
                minDist = distSq;
                // This one is closer to previous location, update current location of iRat.
                nowTrack[0].centroid = centre;
              }

            } else { // haven't seen anything colossred yet
              // This is our guess for where iRat is
              // cout << "Seeing something coloured for first time. It's the iRat." << endl;
              colouredCount++;
              nowTrack[0].centroid = centre; 
              nowTrack[0].boundingRect = enlarged;
              coloured.push_back(centre);
              // Calculate (squared) distance from previous known location of iRat
              if(!prevTracks.empty()) {
                Point diff = centre - prevTracks[0].centroid;
                minDist = diff.ddot(diff);
              }
            }
          }
        }
      }

      // Draw enlarged rectangle on original frame.
      rectangle(origFrame, enlarged.tl(), enlarged.br(), colour,
          2, 8, 0);
    }
    
    // Do optical flow
    opticalFlow(frame, prevFrame, origFrame); 
    
    // Update positions of rats.    
    if(!prevTracks.empty() && !prevRatTracks.empty()) {
      updatePositions(blobs);
    }
    
    // Override deduced position with manual input if there is any.
    if(mouseClicked) {
      for(int i = 0; i < numRats; i++) {
        nowRatTrack[i].centroid = mouseClickRats[i];
      }
    }

    // We have been through all the blobs. Finalise loc of iRat
    
    for(int i = 0; i < numiRats; i++) {
      putText(origFrame, "iRat", nowTrack[i].centroid, FONT_HERSHEY_SIMPLEX, 0.5, green, 2);
    }
    
    for(int i = 0; i < numRats; i++) {
      stringstream ss;
      ss << "Rat " << i;
      putText(origFrame, ss.str(), nowRatTrack[i].centroid, FONT, 0.5, blue, 2);
      // putText(origFrame, "clicked here", mouseClickRats[i], FONT, 0.5, black, 2);
    }

    // Work out which way the iRat should turn 
    double vrot;
    if(!prevTracks.empty()) {
      vrot = getDir(nowTrack[0].centroid, nowTrack[0].centroid - prevTracks[0].centroid, 
                           nowRatTrack[0].centroid, 0.5);  
      stringstream ss;
      ss << vrot;
      putText(origFrame, ss.str(), Point(30, 30), FONT, 0.5, blue, 2);    
      line(origFrame, prevTracks[0].centroid, nowTrack[0].centroid, green);
    }

    imshow("Filtered frame", filteredFrame);
		imshow("Processed", frame);
    // imshow("Background", back);
    // imshow("Foreground", fore);
    // imshow("Blobs", blobFrame);
    imshow("Original frame", origFrame);
    writer.write(origFrame);
    
    // Update track 
    prevTracks = nowTrack;
    prevRatTracks = nowRatTrack;
    frame.copyTo(prevFrame);
    colouredCount = 0;
    mouseClicked = false;

    /**** Issue velocity commands ****/
    cmdvel_msg.header.stamp = ros::Time::now();
    // keep moving forward
    cmdvel_msg.magnitude = 0.05;
    cmdvel_msg.angle = vrot;
    pub_cmdvel.publish(cmdvel_msg);
    cmdvel_msg.header.seq++;

    int keyPressed = waitKey(1);
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
