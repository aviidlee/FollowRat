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
#define FRAMERATE 15

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  int deviceNo;
  string fileName;

  cout << "Commencing reading from video feed..." << endl;
  if(argc == 3) {
    deviceNo = atoi(argv[1]);
    fileName = argv[2];
    cout << "fileName: " << fileName << endl;

  } else {
    cout << "Usage: record <deviceNumber> <fileName>" << endl;
    return -1;
  }

	VideoCapture cap(deviceNo); // hopefully open the webcam; 0 is default.
	if(!cap.isOpened()) {
		return -1;
	}
  
  cout << "YAY" << endl;
	cvNamedWindow("Webcam", 1);
  
  int frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  VideoWriter writer(fileName, CV_FOURCC('M', 'J', 'P', 'G'), FRAMERATE, Size(frameWidth, frameHeight));
	Mat frame;

	for(EVER) {
		cap >> frame; // get new frame from camera
    writer.write(frame);
    imshow("Webcam", frame);

		if(waitKey(30) >= 0) {
			// terminate program on key press 
			break;
		}
	}

	return 0;
}
