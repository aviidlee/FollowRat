/**************** General C++ includes ***********/
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <climits>

/**************** ROS includes *******************/
#include <sensor_msgs/image_encodings.h>
#include <irat_msgs/IRatVelocity.h>
#include <ros/ros.h>
#include <irat_msgs/IRatRangers.h>

/**************** OpenCV Stuff *******************/
#include <opencv2/opencv.hpp>
// For blob detection 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**************** Blobs *************************/
#include <BlobResult.h>

/**************** Constants *********************/
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
// the maximum distance an two objects can be apart between frames for them
// to be identified as the same object.
//#define DIST_THRESH (640/4)*(640/4)
#define DIST_THRESH ((640*640)/2)
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
// Need to keep history longer than just 1 frame... 
vector<vector<Track> > iRatHistory;
int histCount = 0;
int maxHistCount = 5;
// whether or not velocity command should be decided by ranger callback or auton. prog
bool rangersDecide = false;
// The rotational velocity to be issued to the iRat 
double vrot; 
// The translation velocity to be issued to the iRat
double vtrans;

// Previous frame's tracks for rats 
vector<Track> prevRatTracks;
// Current tracks for rats 
vector<Track> nowRatTrack;
// Stores the mouse click positions; required for updating the rats' positions.
vector<Point> mouseClickRats;
// Whether or not the mouse has been clicked during this frame.
bool mouseClicked = false;
// The indices for the ranger values
enum Wall {RIGHT = 0, CENTRE = 1, LEFT = 2};
#define MAX_VTRANS 0.1
// Issues same vrot for CONSEC frames before changing.
#define CONSEC 5

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

/***** Kalman *****/
const int MINHESSIAN = 400;


/**
 * If the iRat is very close to a wall, then set the global boolean 
 * rangersDecide to true and update the velocity command. If not, 
 * set the variable to false and return so that the tracking program can
 * decide the velocity. 
 * 
 * @param rangers the pointer to the ranger message. 
 */
void rangersCallback(irat_msgs::IRatRangersConstPtr rangers) {
  // Check if we are heading straight into something.
  // Use 10cm for now  
  rangersDecide = true;
  if(rangers->rangers[CENTRE].range < 0.1) {
		// vrot = rng.uniform(0, 1) < 0.5	? 0.5 : -0.5;
		vtrans = 0;
	} else if (rangers->rangers[RIGHT].range < 0.05) {
    vrot = 0.5;
		vtrans = 0;
  } else if(rangers->rangers[LEFT].range < 0.05) {
    vrot = -0.5;
		vtrans = 0;
  } else { // let the tracking program decide the velocity.
    rangersDecide = false;
  }

  return;
}

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
double getDir(Point iRatLoc, Point iRatHeading, Point ratLoc, double mag, Mat& img) {
  Point diff = ratLoc - iRatLoc;
	// work out iRat's heading direction by averaging its last histCount
	// directions. 
	/*
	if(histCount == maxHistCount-1) {
		iRatHeading = iRatLoc - iRatHistory[maxHistCount-1][0].centroid;
	} 
  histCount++;
  if(histCount == maxHistCount) {
    histCount = 0;
  }
  */

  Point change = diff - iRatHeading;
  double turn = iRatHeading.y*change.x < 0 ? -1 : 1;
  // Adjust turning velocity depending on how far to the left/right the rat is
  // of the iRat.
  //double vrot = mag*turn*(change.x/50) < mag ? mag*turn*(change.x/50) : mag;
  
  // Draw stuff	
  // Scalar blue = Scalar(255, 0, 0);
  // Draw the vector from iRat to rat
  // line(img, ratLoc, iRatLoc, 3);
  // Draw heading direction 

  return turn*mag;
}

/**
 * Click to specify where the rats are. Left mouse button specifies where 
 * rat 1 is, and right mouse button specifies where rat 2 is. 
 */
void mouseCallback(int event, int x, int y, int flags, void* userdata) {

  // Click to specify position of rat 1
  if(event == EVENT_LBUTTONDOWN && numRats > 0) {
    if(DEBUG) {
      cout << "Left mouse button clicked at (x, y) = " 
                   << x << "," << y << endl;
    }
    mouseClickRats[0] = Point(x, y);
    mouseClicked = true; 
  }
  
  // Click to specify position of rat 2
  if(event == EVENT_RBUTTONDOWN && numRats > 1) {
    if(DEBUG) {
      cout << "Right mouse button clicked at (x, y) = " 
                    << x << "," << y << endl;
    }
    mouseClickRats[1] = Point(x, y);
    // It may look awful that we are doing mouseClicked = true twice but it 
    // is necessary; if we put it as the first thing, it will set true on 
    // ANY mouse movement, not just clicking, because this callback is called 
    // on mouse MOVEMENTS of any kind and not just clicks.
    mouseClicked = true; 
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

/**
 * Exception to be thrown when 
 */
class TooFar: public exception {
  virtual const char* what() const throw() {
    return "This is a cryptic exception message. Ehehehe.";
  }
};

/**
 * return the square of the distance between p and q. 
 * 
 * @param  p [description]
 * @param  q [description]
 * @return   [description]
 */
int distSq(Point p, Point q) {
  Point diff = p - q;
  return diff.ddot(diff);
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

/** initialise the kalman filter with the given point as the initial position.
 * Takes a pointer to a Kalman filter to fill in and a pointer to a Point
 * representing the initial position of the object to be tracked.
*/
void init_kalman(KalmanFilter *KF, Point *init_pos) {
  //Set up kalman filter
  //KalmanFilter KF(6, 2, 0); 
  
  /* 
  Mat_<float> state(6, 1);
  Mat processNoise(6, 1, CV_32F);
  
  KF->statePre.at<float>(0) = init_pos->x;
  KF->statePre.at<float>(1) = init_pos->y;
  KF->statePre.at<float>(2) = 0;
  KF->statePre.at<float>(3) = 0;
  KF->statePre.at<float>(4) = 0;
  KF->statePre.at<float>(5) = 0;
  // Create transition matrix which relates how meausrements are used
  KF->transitionMatrix = *(Mat_<float>(6, 6) << 
      1,0,1,0,0.5,0,   
      0,1,0,1,0,0.5,  
      0,0,1,0,1,0, 
      0,0,0,1,0,1,  
      0,0,0,0,1,0,  
      0,0,0,0,0,1);
  
  KF->measurementMatrix = *(Mat_<float>(2,6)<<
      1,0,1,0,0.5,0,
      0,1,0,1,0,0.5);

  setIdentity(KF->measurementMatrix);
  setIdentity(KF->processNoiseCov, Scalar::all(1e-4));
  setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
  setIdentity(KF->errorCovPost, Scalar::all(.1));
  */
  //-----------------------------------------------------------------------
  
  //KalmanFilter KF(4, 2, 0);
  
  KF->transitionMatrix = *(Mat_<float>(4, 4) << 
      1,0,1,0,   
      0,1,0,1,  
      0,0,1,0,  
      0,0,0,1);

  Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
   
   // init...
   KF->statePre.at<float>(0) = init_pos->x;
   KF->statePre.at<float>(1) = init_pos->y;
   KF->statePre.at<float>(2) = 0;
   KF->statePre.at<float>(3) = 0;
   setIdentity(KF->measurementMatrix);
   setIdentity(KF->processNoiseCov, Scalar::all(1e-4));
   setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
   setIdentity(KF->errorCovPost, Scalar::all(.1));
   
}

/**
 * [findClosest description]
 * @param  points [description]
 * @param  point  [description]
 * @return        [description]
 */
Point findClosest(const vector<Point>& points, const Point& point) {
  Point diff;
  Point closest;
  int dist;
  int minDist = INT_MAX;
  for(int i = 0; i < points.size(); i++) {
    diff = point - points[i];
    dist = diff.ddot(diff);
    if(dist < minDist) {
      closest = points[i];
      minDist = dist;
    }
  }

  return closest;
}

Point kalmanPredict(KalmanFilter& kf, const vector<Point>& possibleLocs, bool override) {
  Mat prediction = kf.predict();
  Point predictedPt(prediction.at<float>(0), prediction.at<float>(1));
  if(override) {
    // use mouse position if available 
  } else {

  }

  Point correct = override && mouseClicked ? mouseClickRats[0] : findClosest(possibleLocs, predictedPt);

  // if the closest thing is still really far away, don't update; it's 
  // probably the green bucket thing moving. 
  Point statePt;
  if(distSq(predictedPt, correct) < DIST_THRESH || override) {
    Mat_<float> meas(2, 1);
    meas.setTo(Scalar(0));
    meas(0) = correct.x;
    meas(1) = correct.y;
    Mat estimated = kf.correct(meas); 
    statePt = Point(estimated.at<float>(0), estimated.at<float>(1));
  } else {
    throw TooFar();
  }
  return statePt;
}

/**
 * Takes a list of points which could represent a rat, and the Kalman
 * Filter which has been set up to predict that rat. It gets a prediction
 * of the rat's location from the Kalman filter, and then chooses the point
 * from the list which is closest to the predicted location. It updates the
 * Kalman filter with this chosen location. The 'corrected' prediction is 
 * retrieved from the filter and returned.  
 * 
 * @param  kf           [description]
 * @param  possibleLocs [description]
 * @return              [description]
 */
Point kalmanRatPredict(KalmanFilter& kf, const vector<Point>& possibleLocs) {
  Mat prediction = kf.predict();
  Point predictedPt(prediction.at<float>(0), prediction.at<float>(1));
  // If the mouse button has not been pressed (to identify the rat manually)
  // then guess that the closest moving thing is the rat; otherwise use the 
  // mouse click. 
  Point ratLoc = mouseClicked ? mouseClickRats[0] : findClosest(possibleLocs, predictedPt);

  Mat_<float> meas(2, 1);
  meas.setTo(Scalar(0));
  meas(0) = ratLoc.x;
  meas(1) = ratLoc.y;
  Mat estimated = kf.correct(meas); 
  Point statePt(estimated.at<float>(0), estimated.at<float>(1));
  return statePt;
}



/**
 * Get Kalman filter to predict location of iRat, then feeds in the location of the 
 * iRat as determined by colouring filtering to 'correct' the Kalman filter. 
 * 
 * @param  kf      [description]
 * @param  correct [description]
 * @return         [description]
 */
Point kalmanRobotPredict(KalmanFilter& kf, const vector<Point>& possible) {
  Mat prediction = kf.predict();
  Point predictedPt(prediction.at<float>(0), prediction.at<float>(1));
  Point correct = findClosest(possible, predictedPt);
  // if the closest thing is still really far away, don't update; it's 
  // probably the green bucket thing moving. 
  Point statePt;
  if(distSq(predictedPt, correct) < DIST_THRESH) {
    Mat_<float> meas(2, 1);
    meas.setTo(Scalar(0));
    meas(0) = correct.x;
    meas(1) = correct.y;
    Mat estimated = kf.correct(meas); 
    statePt = Point(estimated.at<float>(0), estimated.at<float>(1));
  } else {
    throw TooFar();
  }
  return statePt;
}

/**
 * Removes the Point exclude from the vector points 
 * 
 * @param points  [description]
 * @param exclude [description]
 */
void removePoint(vector<Point>& points, const Point& exclude) {
  vector<Point>::iterator iter = find(points.begin(), points.end(), exclude);
  if(iter != points.end()) {
    points.erase(iter);
  } 

  return;
}

/**
 * Calculate the angle between two points in radians
 */
float angleBetween(Point v1, Point v2) {
	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

	float dot = v1.x * v2.x + v1.y * v2.y;

	float a = dot / (len1 * len2);

	if (a >= 1.0)
		return 0.0;
	else if (a <= -1.0) {
		return 3.14;
	}
	return acos(a);
}

void debugMsg(string msg) {
  if(DEBUG) {
    cout << "DEBUG: " << msg << endl;
  }
  return;
}

int main(int argc, char** argv) {
  // topic root, e.g., irat_red; can be specified from commandline with _topic:/irat_[colour]
  string topic;
  // initialise node
  ros::init(argc, argv, "FollowRat");
  // Make node private 
  ros::NodeHandle node("~");
  // Make topic root parameter; if not specified from cmd/launch file, default to red rat.
  node.param("topic", topic, string("/irat_green")); 
  string iRatVelTopic = topic + "/serial/cmdvel";
  cout << "Publishing to: " << iRatVelTopic << endl;
  // register a publisher for the iRat’s command velocity
  // ros on the iRat would subscribe to this
  ros::Publisher pub_cmdvel;
  // message to publish
  irat_msgs::IRatVelocity cmdvel_msg;// instantiate once only because of header squence number
  pub_cmdvel = node.advertise<irat_msgs::IRatVelocity>(iRatVelTopic, 1);
	// number of times we've consecutively published this command
	int pubCount = 0;
	double oldVrot;
	
  iRatHistory.resize(maxHistCount);
	cout << "iRatHistorySize: " << iRatHistory.size() << endl;
  
  string iRatRangersTopic = topic + "/serial/rangers";
  ros::Subscriber sub_rangers =
    node.subscribe<irat_msgs::IRatRangers>(iRatRangersTopic, 1, rangersCallback,
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  
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
  Scalar red = Scalar(0, 0, 255);
  Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
      rng.uniform(0, 255));

  // Store last frame for optical flow. 
  Mat prevFrame;

  // Create Kalman filter and initialise
  KalmanFilter KF(4, 2, 0); // 4 variables, 2 meaurements
 
  // For storing measurements to feed back into KF. 
  Mat_<float> measurement(2,1); 
  measurement.setTo(Scalar(0));
  vector<Point> kalmanEstim;
  bool kalmanInitialised = false;

  // Kalman filter for keeping track of iRat's position 
  // Being lazy. Probably better to put in a vector. 
  KalmanFilter iRatKF(4, 2, 0);
  Mat_<float> iRatMeas(2, 1);
  vector<Point> iRatKalmanEstim;
  bool iRatKalmanInit = false;

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
  if(argc >= 5) {
    int frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    writer.open(argv[4], CV_FOURCC('M', 'J', 'P', 'G'), FRAMERATE, 
        Size(frameWidth, frameHeight));
  }
  
  int playSpeed = 10;
  if(argc == 6) {
    if(!sscanf(argv[5], "%d", &playSpeed)) {
      cout << "Bad playback speed" << endl;
      return -1;
    } 
    cout << "Pausing " << playSpeed << "ms each frame." << endl;
  } 

  cout << "Commencing blob detection..." << endl;
	//cvNamedWindow("Processed", 1);
  cvNamedWindow("Original frame");
  //cvNamedWindow("Filtered frame");

  createTrackbars();
  setMouseCallback("Original frame", mouseCallback, NULL);

  vector<vector<Point> > contours;
  Mat frame, back, fore, origFrame, filteredFrame;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  
  CBlob currentBlob;
  bool readFrame;
  vector<Point> ratMotionHist;
  vector<Point> colouredBlobs;

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
    // bg.getBackgroundImage(back);

    // Morph ops to get rid of noise; erode and dilate.
    morphOps(fore, fore);

    /* Do blob detection on the foreground */
    Mat blobFrame(origFrame.size(), origFrame.type());
    CBlobResult blobs(fore);
    // Filter blobs by size to get rid of little bits 
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, areaThresh);
    int numBlobs = blobs.GetNumBlobs();
   
    // Get bounding boxes for blobs
    // Don't need them stored to draw them, but we will need them later.
    vector<Rect> boundingBoxes;
    // Centroids of blobs 
    vector<Point> centroids;

    debugMsg("Starting the for looppp");

    for(int i = 0; i < numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      centroids.push_back(currentBlob.getCenter());
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
        // int numObj = hierachy.size();
        for(int j = 0; j >= 0; j = hierachy[j][0]) {
          Moments moment = moments((Mat) contours[j]);
          double area = moment.m00;
          if(area > MIN_OBJ_AREA && area < MAX_OBJ_AREA) {
            Point centre = currentBlob.getCenter();
            colouredBlobs.push_back(centre);
            // Draw a circle on the filtered blob. 
            circle(origFrame, centre, 30, Scalar(0, 0, 255));
            debugMsg("FOUND SOMETHING COLOURED YAYA BEANS");
          }
        }
      }

      // Draw enlarged rectangle on original frame.
      rectangle(origFrame, enlarged.tl(), enlarged.br(), colour,
          2, 8, 0);
    }
     
    // old dumb way for comparison; find closest thing to previous 
    if(!colouredBlobs.empty() && !prevTracks.empty()) {
      nowTrack[0].centroid = findClosest(colouredBlobs, prevTracks[0].centroid);
      debugMsg("Did dumb way calculation");
    } else if(!colouredBlobs.empty()) {
      // Rather dangerously choose one of the coloured blobs. First one sounds good. 
      nowTrack[0].centroid = colouredBlobs[0];
      debugMsg("Guessed Green's location");
    } else {
      // Oh gosh, nothing green in the scene?!!
      // Then we simply do not update nowTrack. 
      debugMsg("Didn't see anything of the right colour");
    } 

    if(iRatKalmanInit) {
      Point kfPred;
      /**
       * Update the iRat's Kalman filter. if no objects of correct
       * colour have been spotted, do not update; keep old guess.
       */
      if(!colouredBlobs.empty()) {
        try {
          kfPred = kalmanPredict(iRatKF, colouredBlobs, false);
          iRatKalmanEstim.push_back(kfPred);
          debugMsg("Did Kalman prediction for robot");
        } catch(TooFar& e) {
          debugMsg("WE HAVE CAUGHT AN EXCEPTIONNNN BETTER THAN POKEMONNN");
          // The closest thing is really far away. 
          // DO not update; draw old circle
          if(!iRatKalmanEstim.empty()) {
            kfPred = iRatKalmanEstim[iRatKalmanEstim.size()-1];
          }
          debugMsg("Exception successfully CAUGHT");
        } 
      } else {
          if(!iRatKalmanEstim.empty()) {
            kfPred = iRatKalmanEstim[iRatKalmanEstim.size()-1];
          }
      }

      circle(origFrame, kfPred, 5, green, 2);
      colouredBlobs.clear();
      
      // Draw the Kalman predictions 
      if(!iRatKalmanEstim.empty()) {
        for (int i = 0; i < iRatKalmanEstim.size()-1; i++) {
          line(origFrame, iRatKalmanEstim[i], iRatKalmanEstim[i+1], 
              green, 1);
        } 
      }
      // Draw Kalman's predicted point.
      circle(origFrame, kfPred, 5, green, 2);
      debugMsg("Found next location of iRat!");
    } else {
      init_kalman(&iRatKF, &nowTrack[0].centroid);
      iRatKalmanInit = true;
      cout << "Kalman filter initialised for iRat" << endl;

    }

    // Remove position of iRat from list we pass to kalman filter
    // so that it does not confuse the iRat and the rat.
    // @pre - the iRat's position has been finalised. 
    removePoint(centroids, nowTrack[0].centroid);
   
    // Initialise Kalman Filter when the rat is identified with mouse click.
    if(!kalmanInitialised && mouseClicked) {
      init_kalman(&KF, &mouseClickRats[0]);
      kalmanInitialised = true;
      cout << "Kalman Filter has been initialised" << endl;
    }

    // Do Kalman filter stuff. 
    if(kalmanInitialised) {
      Point kfPred;
      if(!centroids.empty()) {
        try {
          kfPred = kalmanPredict(KF, centroids, true);
          kalmanEstim.push_back(kfPred);
        } catch(TooFar& e) {
          if(!kalmanEstim.empty()) {
            kfPred = kalmanEstim[kalmanEstim.size()-1];
          }
        }
      } else {
        // Keep the old prediction
        if(!kalmanEstim.empty()) {
            kfPred = kalmanEstim[kalmanEstim.size()-1];
        }
      }
      
      // Draw the Kalman predictions 
      for (int i = 0; i < kalmanEstim.size()-1; i++) {
        line(origFrame, kalmanEstim[i], kalmanEstim[i+1], 
            red, 1);
      } 
      // Draw Kalman's predicted point.
      circle(origFrame, kfPred, 5, red, 2);
      // Draw the rat history 
      // I dont' know why but I need the explicit check; somehow it magically 
      // gets into the loop even when size = 0; it magically thinks 0 < -1.
      /*
      if(!ratMotionHist.empty()) {
        for(unsigned int j = 0; j < ratMotionHist.size()-1; j++) {
          line(origFrame, ratMotionHist[j], ratMotionHist[j+1], blue);
        } 
      }
      */
    }

    // Do optical flow
    // opticalFlow(frame, prevFrame, origFrame); 
    
    // Update positions of rats.    
    if(!prevTracks.empty() && !prevRatTracks.empty()) {
      updatePositions(blobs);
      debugMsg("Updated rat positions the old way");
    }
    
    // Override deduced position of rats with manual input if there is any.
    if(mouseClicked) {
      for(int i = 0; i < numRats; i++) {
        nowRatTrack[i].centroid = mouseClickRats[i];
      }
    }  

    // Draw iRat position
    for(int i = 0; i < numiRats; i++) {
      putText(origFrame, "iRat", nowTrack[i].centroid, FONT_HERSHEY_SIMPLEX, 0.5, green, 2);
      debugMsg("Labelled iRat's position, as calculated by old code");
    }

    // Label position of rats 
    for(int i = 0; i < numRats; i++) {
      stringstream ss;
      ss << "Rat " << i;
      if(!kalmanEstim.empty()) {
        putText(origFrame, ss.str(), kalmanEstim[kalmanEstim.size()-1], FONT, 0.5, red, 2);
      }
      // putText(origFrame, "clicked here", mouseClickRats[i], FONT, 0.5, black, 2);
      debugMsg("Labelled rat positions");
    }

    // Keep history all the way from beginning of program to now. 
    // NB this is what the *nearest-blob method* detected. It is here for 
    // comparison with the KF predictions.
    if(kalmanInitialised) {
      ratMotionHist.push_back(nowRatTrack[0].centroid);
      debugMsg("Pushed to history");
    }

    // Work out which way the iRat should turn 
    if(!prevTracks.empty() && !rangersDecide) {
      debugMsg("Yoyoyo");
			if(kalmanInitialised && iRatKalmanInit) {
        debugMsg("Adventure time!");
        // Get heading direction of iRat by using the predicted point from 5 frames ago, 
        // or the oldest frame you can get. 
        int histSize = iRatKalmanEstim.size();
        Point oldPoint = histSize > maxHistCount ? 
                        iRatKalmanEstim[histSize-maxHistCount] : iRatKalmanEstim[0];

        Point heading = iRatKalmanEstim[histSize-1] - oldPoint;
        // iRat location, heading, rat location, magnitude, frame to draw vis on. 
				vrot = getDir(iRatKalmanEstim[histSize-1], heading, kalmanEstim[kalmanEstim.size()-1], 0.5, origFrame);
      } else {
        debugMsg("yay beans");
        // Use the old dumb code to find position of rat.  
      	vrot = getDir(nowTrack[0].centroid, nowTrack[0].centroid - prevTracks[0].centroid, 
                           nowRatTrack[0].centroid, 0.5, origFrame);  
			}
			stringstream ss;
      ss << vrot;
      putText(origFrame, ss.str(), Point(30, 30), FONT, 0.5, blue, 2);    
      line(origFrame, prevTracks[0].centroid, nowTrack[0].centroid, green);
      debugMsg("Worked out which way to turn");
    }

    debugMsg("Doing the harmless stuff now.");

    //imshow("Filtered frame", filteredFrame);
		//imshow("Processed", frame);
    // imshow("Background", back);
    // imshow("Foreground", fore);
    // imshow("Blobs", blobFrame);
    imshow("Original frame", origFrame);
    writer.write(origFrame);
    
    // Update track 
    prevTracks = nowTrack;
    prevRatTracks = nowRatTrack;
    frame.copyTo(prevFrame);
    mouseClicked = false;
    debugMsg("Updated stuff");

		/**** Issue velocity commands ****/
    cmdvel_msg.header.stamp = ros::Time::now();

				vrot = getDir(iRatKalmanEstim[histSize-1], heading, kalmanEstim[kalmanEstim.size()-1], 0.5, origFrame);
		// Limit speed forward based on angle to Rat
		float speed = MAX_VTRANS;
		// vector between = iratLocation - ratLocation
		Point iratRatVec = iRatKalmanEstim[histSize-1] - 
				kalmanEstim[kalmanDestim.size()-1]; 
		
		float angle = angleBetween(iratRatVec, heading);
		speed = -MAX_VTRANS * ((angle / 3.14) - 1);


    // keep moving forward unless obstacle
    cmdvel_msg.magnitude = rangersDecide? vtrans : speed;
		// Continue to publish the same rotational velocity command
		// for CONSEC consecutive frames unless in obstacle avoidance mode.
		if(pubCount < CONSEC && !rangersDecide) {
    	cmdvel_msg.angle = oldVrot;
    } else { // update to new vrot, reset pubCount 
			cmdvel_msg.angle = vrot;
			oldVrot = vrot;
			pubCount = 0;
		}
		pubCount++;
		pub_cmdvel.publish(cmdvel_msg);
    cmdvel_msg.header.seq++;

    int keyPressed = waitKey(playSpeed);
    if(keyPressed == 27 || keyPressed == keyEsc) {
      return 0;
    } else if(keyPressed == 'p' || keyPressed == keyP) {
      paused = !paused;
      if(paused) {
        cout << "Program paused." << endl;
        while(paused) {
          int key = waitKey(0);
          if(key == 'p' || key == keyP) {
            paused = false;
          }
        }
      }
    } else {
      // No key pressed, do nothing.
    }  

	}

	return 0;
}
