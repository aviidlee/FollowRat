#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#define EVER ;;

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  int deviceNo;

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

	for(EVER) {
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
