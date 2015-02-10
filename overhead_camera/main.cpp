#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int, char**) {
	VideoCapture cap(1); // hopefully open the webcam; 0 is default.
	if(!cap.isOpened()) {
		return -1;
	}

	cvNamedWindow("Webcam", 1);

	for(;;) {
		Mat frame;
		cap >> frame; // get new frame from camera
		imshow("Webcam", frame);
		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
