/** This file takes a list of files (as stdin) and will display the images
  * for the user to select the region in which there is an object or skip the
	* image. If the user selects and object that image name and object location
	* are saved to file (output.txt).
	* key q skips the image, key w signifys the image has an object and the user
	* will select its region(s) containing the object(s).
	*/


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
//Don't need core as highgui already has core
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <sstream>
#include <fstream>

// waitKey() return values for each key
#define Q_KEY 113
#define W_KEY 119
#define E_KEY 101
#define R_KEY 114
#define INTERNAL_ERROR -2

using namespace std;
using namespace cv;

// GLOBLES
int fx;
int fy;
int sx;
int sy;
int BOX_W = 20;
int BOX_H = 20;
Mat img; // Img to display
string WINDOW_NAME = "Window";
char mode;
int clip_num = 0;

void reset_globals(){
	fx = -1;
	fy = -1;
	sx = -1;
	sy = -1;
}

Rect get_rect(int x, int y) {
	return Rect( x - BOX_W, y - BOX_H, BOX_W * 2, BOX_H * 2);
}

void callback(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN){
		cout << "Click (" << x << ", " << y << ")" << endl;
		if (fx != -1 && sx != -1) {  // if both are already set, then clear them
			reset_globals();
		}

		if (fx == -1){
			fx = x;
			fy = y;
		} else if (sx == -1) {
			sx = x;
			sy = y;
		}
		
		if (mode == 'c') {
			//Save a copy of clicked region
			Rect rect = get_rect(x, y);
			Mat dst;
			stringstream ss;
			ss << "pos/sample-" << clip_num++ << ".pgm";
			img(rect).copyTo(dst);
			imwrite(ss.str(), dst);
			cout << "Saved: " << ss.str() << endl;
		}

	} else if ( event == EVENT_MOUSEMOVE ) {
		//Draw retangle
		Scalar colour = Scalar(0, 255, 0);
		Mat new_img = img.clone();
		Rect rect = get_rect(x, y);
		//rectangle( new_img, Point(x-BOX_W, y-BOX_H), Point(x+BOX_W, y+BOX_H), 
		//		colour, 2, 8, 0);
		rectangle( new_img, rect.tl(), rect.br(), colour);
		imshow(WINDOW_NAME, new_img);
	}
}


string get_current_info(string filename){
	stringstream ss;
	ss << filename;
	int fox = fx - BOX_W;
	int foy = fy - BOX_H;
	int sox = sx - BOX_W;
	int soy = sy - BOX_H;
	int width = BOX_W * 2;
	int height = BOX_H * 2;
	if (fx != -1 && sx != -1) { // both have positions
		ss << "  2  " << fox << " " << foy << " " << width << " " << height;
		ss << "   " << sox << " " << soy << " " << width << " " << height;
	} else if (fx != -1) {
		ss << "  1  " << fox << " " << foy << " " << width << " " << height;
	} else {
		throw INTERNAL_ERROR; // Should never get here
	}
	ss << endl;
	return ss.str();

}


int main(int argc, char** argv)
{
	mode = 'a'; // Default to annotate mode
	if (argc == 3) {
		mode = argv[1][0]; // could also be select (s) mode
		BOX_W = atoi(argv[2]) / 2; // Box_x represents half the width
		BOX_H = atoi(argv[2]) / 2;
	} else {
		cout << "Usage: imageSelect <mode (s/a/c)> box_size" << endl;
	}

 	cout << "Start" << endl;
	vector<string> output_strs;
	int key;

	namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	setMouseCallback(WINDOW_NAME, callback, NULL);
	reset_globals(); // Set globals to -1;

	string path = ".";
	string filename = "";
	while (getline(cin, filename) && !filename.empty()) {
		//have filename, now display it... gonna need a window
		// Need to read first
		img = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
		imshow(WINDOW_NAME, img);
		do {
			key = waitKey(0);
			
			if (mode == 'c') {
				cout << "Current image: " << filename << endl;
				continue;
			} else if (key == Q_KEY){
				cout << filename << " was skipped"<< endl;
				continue;
			} else if (key == W_KEY && mode == 'a'){ //a for annotate
				output_strs.push_back(get_current_info(filename));
				cout << get_current_info(filename);
				reset_globals();
			} else if (key == W_KEY && mode == 's') { // select mode
				output_strs.push_back(filename + "\n");
			} else {
				cout << "Please use only 'q' and 'w' keys" << endl;
			}

		} while (key != Q_KEY && key != W_KEY);

	}

	ofstream output_file;
	string output_filename = "output.txt";
	output_file.open(output_filename);
	for (string s : output_strs) {
		output_file << s;
	}
	output_file.close();

	cout << "Done";

	return 0;


}
