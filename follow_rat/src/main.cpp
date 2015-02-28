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
#define DIST_THRESH ((640*640)/2)
// codes for keyboard presses 
#define keyP 1048688
#define keyEsc 1048603
// What people eat. Also a mathematical constant of minor importance. 
#define PI 3.14

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

/******************** Other stuff...********************/
// Number of rats expected to be in frame. 
int numRats;
// Number of iRats expected to be in frame. 
int numiRats;
// How far to go back in history when averaging the iRat's heading vector.
int maxHistCount = 5;

// Stores the mouse click positions; required for updating the rats' positions.
vector<Point> mouseClickRats(2);
// Whether or not the mouse has been clicked during this frame.
bool mouseClicked = false;
// The indices for the ranger values
enum Wall {RIGHT = 0, CENTRE = 1, LEFT = 2};
#define MAX_VTRANS 0.1
#define MAX_VROT 0.5
// Issues same vrot for CONSEC frames before changing.
#define CONSEC 5
// For storing distance to front, left and side objects as reported by
// IR rangers.
vector<double> rangerVals(3);

/***** Kalman *****/
const int MINHESSIAN = 400;

/**
 * Update the ranger data, which is stored in the global variable
 * rangerVals. 
 * 
 * @param rangers the pointer to the ranger message. 
 */
void rangersCallback(irat_msgs::IRatRangersConstPtr rangers) {
  rangerVals[RIGHT] = rangers->rangers[RIGHT].range;
  rangerVals[LEFT] = rangers->rangers[LEFT].range;
  rangerVals[CENTRE] = rangers->rangers[CENTRE].range;

  return;
}

/**
 * Return the turning direction required to follow the rat. 
 * 
 * @param  iRatLoc the location of the iRat; coordinates of blob centre.
 * @param  iRatHeading the vector describing the heading direction of the iRat.
 * @param  ratLoc      the location of the rat; coordinates of blob centre. 
 * 
 * @return             the required turing direction of the iRat; -1 is right,
 *                     1 is left.
 */
double getDir(Point iRatLoc, Point iRatHeading, Point ratLoc) {
  Point diff = ratLoc - iRatLoc;
  Point change = diff - iRatHeading;
  double turn = iRatHeading.y*change.x < 0 ? -1 : 1;
  return turn;
}

/**
 * Function called whenever there is a mouse event in the frame. 
 * Left click updates the position of rat 1 and right click updates 
 * the position of rat 2 (although the rest of the code does not currently
 * support two rats)
 * 
 * @param event    the mouse event; movement, clicks etc. 
 * @param x        the x coordinate of the mouse event 
 * @param y        the y coordinate of the mouse event 
 * @param flags    flags
 * @param userdata custom user data, unused. 
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
  return;
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
 * When updating the Kalman filter, the correct measurement is considered
 * to be the nearest moving object to what the Kalman filter predicted. 
 * But if the nearest moving object is very far away, then chances are that 
 * the object we were tracking has stopped moving rather than jumped across 
 * the entire arena in one frame. Thus if the closest moving object is too
 * far away (as determined by a globally-defined threshold), we throw this
 * exception. 
 */
class TooFar: public exception {
  virtual const char* what() const throw() {
    return "The nearest moving object is too far away!";
  }
};

/**
 * Return the square of the distance between p and q. 
 * 
 * @param  p the first point 
 * @param  q the second point 
 * @return   the square of the distance between p and q.
 */
int distSq(Point p, Point q) {
  Point diff = p - q;
  return diff.ddot(diff);
}

/**
 * Print the usage message.
 */
void printUsageMessage() {
  cout << "Usage: rosrun follow_rat follow [_topic:=/<irat name>] <deviceNo or file name> <num expected iRats>" 
       << "<num expected rats> [<output filename>] [<time to wait between frames>]" << endl; 
  cout << "Please refer to USER_README.txt for further usage instructions." << endl;
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
 *
 * @param src  the source Mat 
 * @param dest the destination Mat to put the processed frame into.
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
 * The enlargement parameters are #defined. 
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

/** initialise the kalman filter with the given point as the initial position.
 * Takes a pointer to a Kalman filter to fill in and a pointer to a Point
 * representing the initial position of the object to be tracked.
 *
 * @require the Kalman filter has been initialised with the arguments (4, 2, 0)
 * @require kf has not yet been initialised. 
 * 
 * @param KF pointer to the kalman filter to be initialised. 
 * @param init_pos the pointer to the initial position of the kalman filter.
*/
void init_kalman(KalmanFilter *KF, Point *init_pos) {
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
  
  return; 
}

/**
 * Find the closest (Euclidean distance) point in points 
 * which is closest to the given point.
 * 
 * @param  points the list of points to search for 
 * @param  point  the target point which we want to find the closest point to.
 * @return        the closest point to point from points. 
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

/**
 * Predict the next location of the object which kf is tracking. 
 *
 * The object in possibleLocs which is closest to the kalman filter's 
 * prediction is fed into the filter as the correction.
 * 
 * @param  kf           the Kalman filter 
 * @param  possibleLocs the list of possible locations of the actual object;
 *                      for the iRat, this will be all moving objects of the right colour
 *                      (default green), and for the rat, it will be all the moving objects
 *                      minus the one identified as the iRat.
 * @param  override     if set to true, allows mouse clicks (which manually identify
 *                      location of rat in the frame) to be used as the correction for
 *                      kf, instead of the closest point in possibleLocs
 * @return              the Kalman estimation produced after the correction.
 *
 * @throw  TooFar       Thrown when the 'correct' location of the object is unreasonably far away
 *                      (as determined by a global threshold); it's not possible for an object to
 *                      move across the entire frame in one frame, and in reality 
 *                      the object we are tracking has probably stopped moving. 
 */
Point kalmanPredict(KalmanFilter& kf, const vector<Point>& possibleLocs, bool override) {
  Mat prediction = kf.predict();
  Point predictedPt(prediction.at<float>(0), prediction.at<float>(1));
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
 * Removes the Point exclude from the vector points. 
 * 
 * @param points  the vector of points 
 * @param exclude the point to exclude 
 */
void removePoint(vector<Point>& points, const Point& exclude) {
  vector<Point>::iterator iter = find(points.begin(), points.end(), exclude);
  if(iter != points.end()) {
    points.erase(iter);
  } 

  return;
}

/**
 * Calculate the angle between two vectors in radians.
 * 
 * @param  v1 the first vector
 * @param  v2 the second vector 
 * @return    the angle between v1 and v2 in radians, value in [0, Pi].
 */
double angleBetween(Point v1, Point v2) {
	double len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	double len2 = sqrt(v2.x * v2.x + v2.y * v2.y);
	double dot = v1.x * v2.x + v1.y * v2.y;

	double a = dot / (len1 * len2);

	if (a >= 1.0) {
		return 0.0;
  } else if (a <= -1.0) {
		return PI;
	}
	return acos(a);
}

/**
 * Print msg if the global DEBUG flag is on. 
 * 
 * @param msg the debug message to print. 
 */
void debugMsg(string msg) {
  if(DEBUG) {
    cout << "DEBUG: " << msg << endl;
  }
  return;
}

/**
 * Write img to a file named name-timestamp-jpg. 
 *
 * Used mainly to output intermediate steps in the computation
 * to file for debugging/explaining the code. 
 * 
 * @param img       the image to output
 * @param name      the name of the processing stage; e.g., foreground.
 * @param timestamp the current time, usually as returned by ros::Time::now().
 *                  used to keep intermediate steps from the same original 
 *                  frame together.
 */
void outputImg(const Mat& img, string name, ros::Time timestamp) {
  stringstream ss;
  ss << name << "-" << timestamp << ".jpg";
  imwrite(ss.str(), img);
  return;
}

/**
 * Return true IFF the BOUNDING BOX of the 
 * circle at centre centre of radius radius intersects
 * with rect. 
 * 
 * @param  centre the centre of the circle
 * @param  radius the radius of the circle
 * @param  rect   the rectangle to test for intersection. 
 * @return        true if the bounding square of the circle 
 *                with centre centre and radius radius intersects 
 *                rect, false otherwise. 
 */
bool intersects(const Point& centre, int radius, const Rect& rect) {
  Point tl = Point(centre.x - radius, centre.y - radius);
  int x = tl.x >= 0 ? tl.x : 0;
  int y = tl.y >= 0 ? tl.y : 0;
  Rect bounding(x, y, 2*radius, 2*radius);
  Rect intersect = bounding & rect;
  return intersect.width > 0 || intersect.height > 0;
}

/**
 * Find and return the largest circle in circles, 
 * where the returned circle is specified by the x, y 
 * coordinates of the centre, then radius. 
 *
 * NB if circles is empty, will return a circle centred
 * at (0, 0) with a radius of 0.
 *
 * @param circles   the vector of circles 
 * @return          the circle of largest radius in circles
 */
vector<int> findMax(const vector<Vec3f> circles) {
  int max = 0;
  int radius, x, y; 
  for(int i = 0; i < circles.size(); i++) {
    radius = cvRound(circles[i][2]);
    if(max < radius) {
      max = radius;
      x = cvRound(circles[i][0]);
      y = cvRound(circles[i][1]);
    }
  }

  vector<int> circle;
  circle.push_back(x);
  circle.push_back(y);
  circle.push_back(max);
  return circle;
}

/**
 * Updates the position of the iRat/rat using the old method of simply
 * finding the coloured blob that is closest to the last known location
 * of the iRat. 
 * 
 * @param  possibleLocs    the possible locations of the object being tracked
 * @param  lastLocs        the last known location of the object being tracked
 * @param  nowLocs         the vector to put the new position of objects in.
 * @return                 0 if the closest object is successfully found, 
 *                         1 if there is no known last location, and the new location
 *                         is updated as the first location in possibleLocs
 *                         -1 if there are no possible locations; in this case 
 *                         nothing is done and nowLocs is not updated.
 */
int oldPositionUpdate(const vector<Point>& possibleLocs, const vector<Point>& lastLocs, 
  vector<Point>& nowLocs, bool override) {
  // Use the non-Kalman filter way to update. 
  if(!possibleLocs.empty()) {
    if(!lastLocs.empty()) {
      nowLocs[0] = findClosest(possibleLocs, lastLocs[0]); 
      return 0;
    } else {
      // No known previous location. 
      // Rather dangerously choose one of the possible. First one sounds good. 
      nowLocs[0] = possibleLocs[0];
      return 1;
    }
  } 

  if(override) {
    nowLocs[0] = mouseClickRats[0];
  }

  // No possible locations; do nothing.
  return -1;
} 

/**
 * Get the angle by which the iRat would have to turn in order to be 
 * facing the rat. 
 * 
 * @param  iRatLoc     the current location of the iRat on screen
 * @param  iRatHeading the current heading vector of the iRat
 * @param  ratLoc      the current location of the rat 
 * @return             the angle the iRat would have to turn to face the rat directly.
 */
double getTurningAngle(const Point& iRatLoc, const Point& iRatHeading, const Point& ratLoc) {
  Point iratRatVec = ratLoc - iRatLoc;
  return angleBetween(iratRatVec, iRatHeading);
}

/**
 * Return the rotational velocity required for the iRat to follow the rat.
 * 
 * @param  iRatLoc     the current location of the iRat on screen
 * @param  iRatHeading the current heading vector of the iRat
 * @param  ratLoc      the current location of the rat 
 * @return             the rotational velocity required for the iRat to follow the rat
 */
double getVrot(const Point& iRatLoc, const Point& iRatHeading, const Point& ratLoc) {
  double dir = getDir(iRatLoc, iRatHeading, ratLoc);
  double angle = getTurningAngle(iRatLoc, iRatHeading, ratLoc);
  double vrot = dir*MAX_VROT;
  if(angle < PI/6) {
    // if turning angle is small, then just turn a little bit :)
    vrot = vrot/5;
  } 
  return vrot;
}

/**
 * Return the translational velocity required for the iRat to follow the rat.
 * 
 * @param  iRatLoc     the current location of the iRat on screen
 * @param  iRatHeading the current heading vector of the iRat
 * @param  ratLoc      the current location of the rat 
 * @return             the translational velocity required for the iRat to follow the rat
 */
double getVtrans(const Point& iRatLoc, const Point& iRatHeading, const Point& ratLoc) {
  double angle = getTurningAngle(iRatLoc, iRatHeading, ratLoc);
  double vtrans = MAX_VTRANS*sqrt(1-((angle*angle)/(PI*PI)));
	return vtrans;
}

/**
 * Get the current heading vector of the iRat, as determined by looking at the 
 * iRat's current position and the iRat's position goBack frames ago. 
 *
 * @param  history the previous locations of the iRat, in chronological order
 * @param  goBack  the number of frames to go back through. 
 * @return         the iRat's heading vector.
 */
Point getHeading(const vector<Point>& history, int goBack) {
  int histSize = history.size();
  Point oldPoint = histSize > goBack ? history[histSize-goBack] : history[0];
  return history[histSize-1] - oldPoint;
}

/**
 * Update the kalman filter and the kalman filter's estimates (see
 * kalmanPredict), and return the new estimate.
 * 
 * If possibleCorrections is empty, the filter is not updated, and 
 * the last estimate is returned and added to estimates.
 * If possibleCorrections is empty but there are no previous estimates,
 * does nothing and returns the point at (0, 0).
 * 
 * @param  kf                  the kalman filter
 * @param  estimates           the kalman filter's estimates to date
 * @param  possibleCorrections the possible locations of the tracked object
 * @param  override            whether or not user mouse input can override 
 *                             the correction
 * @return                     the new estimate of the kalman filter.
 */
Point updateKalman(KalmanFilter& kf, vector<Point>& estimates, 
  const vector<Point>& possibleCorrections, bool override) {

  Point kfPred;
  if(!possibleCorrections.empty()) {
    try {
      kfPred = kalmanPredict(kf, possibleCorrections, override);
    } catch(TooFar& e) {
      if(!estimates.empty()) {
        // The closest thing is really far away. 
        // Do not update; keep the old Kalman estimate. 
        kfPred = estimates[estimates.size()-1];
      }
    }
  } else {
    if(!estimates.empty()) {
        // The closest thing is really far away. 
        // Do not update; keep the old Kalman estimate. 
        kfPred = estimates[estimates.size()-1];
    }
  }
  estimates.push_back(kfPred);

  return kfPred;
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

  // Subscribe to rangers
  string iRatRangersTopic = topic + "/serial/rangers";
  ros::Subscriber sub_rangers =
    node.subscribe<irat_msgs::IRatRangers>(iRatRangersTopic, 1, rangersCallback,
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

	// number of times we've consecutively published this command
	int pubCount = 0;
  // The old rotational velocity, for publishing same one repeatedly. 
	double oldVrot;
  // The rotational and translational velocity to be issued to the iRat.
	double vrot, vtrans;
  // The current locations for the iRats
  vector<Point> nowRobotLocs;
  // Locations from last frame for the iRats  
  vector<Point> lastRobotLocs;
  // The current locations for the rats
  vector<Point> nowRatLocs;
  // Locations from last frame for the rats  
  vector<Point> lastRatLocs; 
  
  cout << "Ros initialised!" << endl;

  // the video device number 
  int deviceNo;
  // the video feed we're reading frames from. 
  VideoCapture cap;
  // optionally able to write the frame, with bounding boxes etc. added, to file.
  VideoWriter writer;

  // colours...
  Scalar green = Scalar(0, 255, 0);
  Scalar blue = Scalar(255, 0, 0);
  Scalar red = Scalar(0, 0, 255);
  Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
      rng.uniform(0, 255));

  // Kalman filter for tracking rat's position. 
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
	
	Point lastHeading;

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

    // Initialise the location trackers 
    nowRobotLocs.resize(numiRats);
    nowRatLocs.resize(numRats);

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
  
  // Optional command - change video playback speed. Only really
  // applies if code is run on recorded video file.  
  int playSpeed = 10;
  if(argc == 6) {
    if(!sscanf(argv[5], "%d", &playSpeed)) {
      cout << "Bad playback speed" << endl;
      return -1;
    } 
    cout << "Pausing " << playSpeed << "ms each frame." << endl;
  } 

  cout << "Commencing blob detection..." << endl;
  cvNamedWindow("Original frame");
  createTrackbars();
  setMouseCallback("Original frame", mouseCallback, NULL);

  vector<vector<Point> > contours;
  Mat frame, back, fore, origFrame, filteredFrame;
  // According to docs, 9 is default for varThreadhold.
  BackgroundSubtractorMOG2 bg(HISTORY, 9, false);
  
  CBlob currentBlob;
  bool readFrame;
  vector<Point> colouredBlobs;

  while(ros::ok()) {
    ros::Time timestamp = ros::Time::now();
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
    // Find Hough transform - detect the circle of the arena. 
    vector<Vec3f> circles;
    HoughCircles(frame, circles, CV_HOUGH_GRADIENT, 1, frame.rows/8, 
      200, 100, 0, 0);
    // Find the largest circle and assume that it is the arena. 
    vector<int> arena = findMax(circles);
  
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

    // Centroids of blobs 
    vector<Point> centroids;

    for(int i = 0; i < numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      centroids.push_back(currentBlob.getCenter());
      currentBlob.FillBlob(frame, Scalar(0, 255, 0));
      Rect currentRect = currentBlob.GetBoundingBox();
      // If this rect is not intersecting the arena, then don't bother processing it.
      // Unless of course something failed and no circles were found 
      if(!intersects(Point(arena[0], arena[1]), arena[2], currentRect) &&
          arena[2] != 0) { // will get circle of zero radius is no circles found.
        continue;
      } 

      // Make a new mat from the original frame clipping out the 
      // enlarged ROI 
      Rect enlarged = enlargeRect(origFrame, currentRect);
      // Draw enlarged rectangle on original frame.
      rectangle(origFrame, enlarged.tl(), enlarged.br(), colour,
          2, 8, 0);
      // Mat holding the enlarged ROI
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
    }
    
    /******************** iRat Position Update ********************/
    // Use the non-Kalman way to update tracks for use when Kalman not initialised. 
    oldPositionUpdate(colouredBlobs, lastRobotLocs, nowRobotLocs, false);
    debugMsg("Did old position update");

    // Use Kalman filter to update robot location 
    if(iRatKalmanInit) {
      Point ratLoc = updateKalman(iRatKF, iRatKalmanEstim, colouredBlobs, false);
      circle(origFrame, ratLoc, 5, green, 2);
      putText(origFrame, "iRat", ratLoc, FONT_HERSHEY_SIMPLEX, 0.5, green, 2);
      colouredBlobs.clear();
      
      // Draw trajectory
      for (unsigned int i = 1; i < iRatKalmanEstim.size(); i++) {
        line(origFrame, iRatKalmanEstim[i-1], iRatKalmanEstim[i], green, 1);
      } 
      
      debugMsg("Found next location of iRat!");
    } else if(!colouredBlobs.empty()) { // Kalman not initialised, coloured blobs available so initalise.
      init_kalman(&iRatKF, &nowRobotLocs[0]);
      iRatKalmanInit = true;
      cout << "Kalman filter initialised for iRat" << endl;
    } else { // Kalman NOT initialised but coloured blobs not available.
			debugMsg("No coloured object detected, unable to initialise iRat kf");
		}

    // Remove position of iRat from list we pass to kalman filter
    // so that it does not confuse the iRat and the rat.
    // @require- the iRat's position has been finalised. 
    removePoint(centroids, nowRobotLocs[0]);
    
    /******************** Rat Position Update ********************/
    // Update positions of rats using the old method; 
    oldPositionUpdate(centroids, lastRatLocs, nowRatLocs, true); 
    debugMsg("Old position update for rat");
    // Initialise Kalman Filter when the rat is identified with mouse click.
    if(!kalmanInitialised && mouseClicked) {
      init_kalman(&KF, &mouseClickRats[0]);
      kalmanInitialised = true;
      cout << "Kalman Filter has been initialised" << endl;
    }

    // Update the rat's Kalman filter 
    if(kalmanInitialised) {
      Point ratLoc = updateKalman(KF, kalmanEstim, centroids, true);
      debugMsg("Updated Kalman for rat");
      circle(origFrame, ratLoc, 5, red, 2);
      putText(origFrame, "Rat", ratLoc, FONT, 0.5, red, 2);

      // Draw trajectory 
      for (unsigned int i = 1; i < kalmanEstim.size(); i++) {
        debugMsg("Drawing rat trajectory");
        line(origFrame, kalmanEstim[i-1], kalmanEstim[i], red, 1);
      } 
    }
    
    /******************** Velocity command update ********************/
		Point heading; // the iRat's heading 
    // Work out which way the iRat should turn 
    if(!lastRobotLocs.empty()) {
      debugMsg("Yoyoyo");
			if(!kalmanEstim.empty() && !iRatKalmanEstim.empty()) {
        debugMsg("Finding velocity with both Kalmans initialised!");
        int size = iRatKalmanEstim.size();
        heading = getHeading(iRatKalmanEstim, maxHistCount);
				if(heading == Point(0, 0)) {
					heading = lastHeading;
				} else {
					lastHeading = heading;
				}
				vrot = getVrot(iRatKalmanEstim[size-1], heading, kalmanEstim[kalmanEstim.size()-1]);
        vtrans = getVtrans(iRatKalmanEstim[size-1], heading, kalmanEstim[kalmanEstim.size()-1]);
			} else { // Kalman not initialised
        debugMsg("Kalman filters not initialised, using old method.");
        heading = lastRobotLocs[0] - nowRobotLocs[0];
      	vrot = getVrot(nowRobotLocs[0], heading, nowRatLocs[0]); 
        vtrans = getVtrans(nowRobotLocs[0], heading, nowRatLocs[0]);
      }

      debugMsg("Worked out which way to turn");
    }

    debugMsg("Doing the harmless stuff now.");
		
		if(rangerVals[CENTRE] < 0.15) {
			vtrans = 0;
		}

		stringstream ss;
		ss << "vrot: " << vrot << ", vtrans: " << vtrans;
		putText(origFrame, ss.str(), Point(30, 30), FONT, 0.5, blue, 2);    
    imshow("Original frame", origFrame);
    writer.write(origFrame);
    
    // Update track 
    lastRobotLocs = nowRobotLocs;
    lastRatLocs = nowRatLocs;
    mouseClicked = false;
    debugMsg("Updated stuff");

		/**** Issue velocity commands ****/
    cmdvel_msg.header.stamp = ros::Time::now();
    // keep moving forward unless obstacle
    cmdvel_msg.magnitude = vtrans;
		// Continue to publish the same rotational velocity command
		// for CONSEC consecutive frames unless in obstacle avoidance mode.
		if(pubCount < CONSEC) {
			cmdvel_msg.angle = vrot; 
		} else {
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
