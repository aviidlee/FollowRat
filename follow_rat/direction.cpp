/** This file will be used to test correcting the heading direction of 
 * the iRat and give an environment in which testing can be done.
 */
#include <vector>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cmath>
#include <string>

// To be clear when accessing vector (which simulates irat_msg)
#define MAG 0
#define VROT 1
#define ARBMAG 10 // Define an arbitrary magnitude for heading direction
#define PI 3.14

//defines for visuliser
#define SCREENX 640
#define SCREENY 480
#define MIDSCREENX SCREENX/2
#define MIDSCREENY SCREENY/2
#define DELAY 50
#define XPOS 0
#define YPOS 1


using namespace std;
using namespace cv;

struct test_t {
	vector<float> msg; 			// Previous msg sent to iRat
	vector<float> expt_out; 	// expected output
	Point iRatLocation;  				
	Point ratLocation;
	Point iRatHeading;
};

//GLOBAL
int debug = 1;
Point ratPos(-1, -1);

// HEADERS 
/*
void set_msg(vector<float>, Point, Point, Point);
bool vecs_equal(vector<float>, vector<float>);
void create_tests(vector<test_t>);
void print_failed(int, test_t);
*/

/** Debug function for printing log messages
 * @params 		s - error string to print
 */
int here(string s) {
	if (debug) {
		cout << s << endl;
	}
}



/** print an error message detail the failed test
 *	@params		test_num - the number of the test that failed
 *  @params 	test - details of the test that failed
 */
void print_failed(int test_num, test_t& test) {
	cout << "  Test " << test_num << " got " << test.msg[MAG] << endl;
}

/**
 * Calculate the angle between two points in radians
 * @params 		v1 - First vector
 * @parmas 		v2 - second vector
 */
float angleBetween(Point v1, Point v2) {
	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

	float dot = v1.x * v2.x + v1.y * v2.y;

	float a = dot / (len1 * len2);

	if (a >= 1.0)
		return 0.0;
	else if (a <= -1.0) {
		return PI;
	}
	return acos(a);
}


/**
 * Using details of the rats/iRats location decide on the best movement
 * for the iRat to chase the rat.
 * @params		msg - The cmdVel msg to fill out with the correct movement
 * @params 		iRatLocation - The iRats location in the arena
 * @params		ratLocation - The rats location in the arena
 * @params 		iRatHeading - A vector of the iRats current direction
 */
void set_msg(vector<float>& msg, Point& iRatLocation, 
							Point& ratLocation, Point& iRatHeading) {

	Point diff = ratLocation - iRatLocation;
	float angle = angleBetween(iRatHeading, diff);
	
	// Which side is the rat on? (left or right)
	//Point change = diff - 
	
  msg[MAG] = 0.1;
  msg[VROT] = 0.0;
	

}


/** Given two vectors, compare the first two elements return
 * true if equal or false otherwise. Return false if vectors are not
 * of size() 2. 
 * @params 		v1 - First vector to compare
 * @params 		v2 - Second vector to compare
 */
bool vecs_equal(vector<float>& v1, vector<float>& v2){
	if( (v1.size() != v2.size()) || (v1.size() != 2) ){
		return false;
	}
	if( (v1[MAG] != v2[MAG]) || (v1[VROT] != v2[VROT]) ){
		return false;
	}
	return true;
}


/** Deprecated - was to be used to create a series of standard tests
 * but different methods of testing are going to be pursued for now
 */
void createBasicEight(vector<test_t>& tests) {
	// directly in front of iRat facing up, should just go forward
	tests.push_back(test_t{
			vector<float>{0.0, 0.0}, // original msg
			vector<float>{0.1, 0.0}, // output msg
			Point(MIDSCREENX, MIDSCREENY), 
			Point(MIDSCREENX, MIDSCREENY - 100), // directly above
			Point(0, ARBMAG)  // TODO where is the x-axis in this system?
									// Assume (1, 1) points downwards to the right
		});
	/*
	// 0.1 radians to right of iRats heading
	tests.push_back(test_t{
			vector<float>{0.0, 0.0}, // original msg
			vector<float>{0.1, 0.0}, // output msg
			Point(MIDSCREENX, MIDSCREENY), 
			Point(MIDSCREENX, MIDSCREENY - 100), // directly above
			Point(0, ARBMAG)  // TODO where is the x-axis in this system?
									// Assume (1, 1) points downwards to the right
		});
	*/




	/* Was going to do this automatically butgive up for time
	int radius = 100;
	Point iratLoc(radius, radius);

	for(int i = 0; i < 8; i++) {
		//rat_x = 
	}
	*/
}


/** 
 * Fill in the given vector with test cases to be used
 * @params 		tests - empty vector to be filled with tests to run
 */
void create_tests(vector<test_t>& tests) {
	// directly in front of iRat facing up, should just go forward
	tests.push_back(test_t{
			vector<float>{0.0, 0.0}, // original msg
			vector<float>{0.1, 0.0}, // output msg
			Point(MIDSCREENX, MIDSCREENY), 
			Point(MIDSCREENX, MIDSCREENY - 100), // directly above
			Point(0, ARBMAG)  // TODO where is the x-axis in this system?
									// Assume (1, 1) points downwards to the right
		});







	// TEST 1 - iRat directly above rat, should go directly straight
	/*
	tests.push_back(test_t{
			vector<float>{0.0, 0.0}, // original msg
			vector<float>{0.1, 0.0}, // Expected output in msg after func call
			Point(100, 100), // iRatLocation
			Point(100, 200), // ratLocation (directly below iRat)
			Point(0, 30)  // iRatHeading (iRat facing directly towards rat)
										// Does 30 need to be negative? TODO 
										// Does 30 need to be normalised to zero? TODO
		});

	// TEST 2 - 
	tests.push_back(test_t{
			vector<float>{0.0, 0.0}, // original msg
			vector<float>{0.0, 0.0}, // Expected output in msg after func call
			Point(100, 100), // iRatLocation
			Point(100, 200), // ratLocation (directly below iRat)
			Point(0, 30)  // iRatHeading 
		});
		*/

	
}



struct polar_t {
	float mag;
	float angle;
};



/**
 * Given a cartesian point return it's polar eqivalance
 * @params 		p - Point to convert
 */
polar_t cvtCart2Polar(Point p){
	float r = sqrt( pow(p.x, 2)  + pow(p.y,2));  
	float theta = atan(p.y/p.x);
	polar_t result{r,theta};
	return result;
}

/**
 * Convert given polar point to cartesian
 * @params 		p - polar point to convert
 */
Point cvtPolar2Cart(polar_t p){
	int x = (int) (p.mag * cos(p.angle));
	int y = (int) (p.mag * sin(p.angle));
	return Point(x, y);
}

/**
 * flip cartesian co-ords so we can think without y being backwards
 * @params		p - catesian point to convert
 */
Point cvtPointFlipy(Point p){
	return Point(p.x, p.y - SCREENY);
}

/**
 * convert from flippy world to normal cartesian world
 * @params    p - Point in flippy world to convert
 */
Point cvtFlipyPoint(Point p) {
  return Point(p.x, SCREENY + p.y);
}

/**
 * Add two polar points together
 * @params 		p1 - first polar point
 * @params 		p2 - second polar point
 */
polar_t addVecs(polar_t p1, polar_t p2) {
	Point rc = cvtPolar2Cart(p1) + cvtPolar2Cart(p2);
	return cvtCart2Polar(rc);
}


void callback(int event, int x, int y, int flags, void *userdata){
  ratPos = cvtFlipyPoint(Point(x, y)); // remember to change to normal cart
}


void run_visual(void){
	Mat bg(SCREENY, SCREENX, CV_8UC3);
	Mat img;
	polar_t iratPos = cvtCart2Polar(cvtFlipyPoint(Point(MIDSCREENX, MIDSCREENY)));
  polar_t iratHeading_p{10.0, 0.0}; // have him point whatever flat is
  

	string WINDOW_NAME = "Simulation";
	namedWindow(WINDOW_NAME);
  setMouseCallback(WINDOW_NAME, callback, 0);

	imshow(WINDOW_NAME, bg);

  vector<float> msg{0.0, 0.0};
  polar_t deltaHeading;
	while(1) {
		img = bg.clone();
    Point headingPoint = cvtPointFlipy(cvtPolar2Cart(iratHeading_p));
    Point iratloc = cvtPointFlipy(cvtPolar2Cart(iratPos)); // iratLoc 
    Point ratloc = cvtPointFlipy(ratPos);                // ratLoc
	  set_msg(msg, iratloc, ratloc, headingPoint);
    // Now convert the msg sent into polar so we can use it
    deltaHeading.mag = msg[MAG];
    deltaHeading.angle = msg[VROT];

    // Now update iRat position based on new movement
    iratPos = addVecs(iratPos, deltaHeading);

    cout << "delta " << cvtPointFlipy(cvtPolar2Cart(iratPos)) << endl;
    // Draw iRat
    int iratSize = 5;
    circle(img, 
        cvtPointFlipy(cvtPolar2Cart(iratPos)),
        iratSize,
        Scalar(255, 0 ,0)); 

		imshow(WINDOW_NAME, img);
		waitKey(DELAY);
	}
}



int main(int argc, char *argv[]) {
	if (argc == 2 && argv[1][0] == 'v') {
		run_visual();
		return 0;
	}
	
	// Vector of tests to run and expected outputs
	vector<test_t> tests;
	vector<int> failed;
	
	create_tests(tests);
	cout << endl << "Starting " << tests.size() << " tests..." << endl;

	int test_num = 0;
	while( test_num < tests.size() ) {
		test_t cur = tests[test_num];

		// Actually test the method, cur.msg should be updated after call
		set_msg(cur.msg, cur.iRatLocation, cur.ratLocation, cur.iRatHeading);	

		if (!vecs_equal(cur.msg, cur.expt_out)) {
			//Failed test
			cout << "FAILED " << test_num + 1 << endl;
			failed.push_back(test_num);
		} else {
			cout << "Passed " << test_num + 1 << endl;
		}

		test_num++;
	}
	
	if (failed.size() == 0) {
		cout << endl << "ALL TESTS PASSED" << endl;
		return 0;
	}

	// Loop through and print out the failed tests
	cout << endl << "The following tests failed:" << endl;
	test_num = 0;
	while (test_num < failed.size()){
		print_failed(test_num + 1, tests[test_num]);
		test_num++;
	}
	return 0;
 }


//TODO Thoughts about the system to check with alex
/*
 * 1 - The heading direction, does (1, 1) point downwards to the right?
 * 2 - 
 */


