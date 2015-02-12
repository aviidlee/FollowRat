#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <vector>

#define EVER ;;

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  int deviceNo;
  
  if(argc == 3) {
    deviceNo = atoi(argv[1]);
  } else {
    cout << "Usage: overhead <deviceNumber> exp_num" << endl;
    return -1;
  }
  
	VideoCapture cap(deviceNo); // hopefully open the webcam; 0 is default.
	if(!cap.isOpened()) {
		return -1;
	}
  
  cout << "YAY" << endl;
	cvNamedWindow("Webcam", 1);
	int counter = 1;

	int timer = 0;

	for(EVER) {
		Mat frame;
		cap >> frame; // get new frame from camera
		imshow("Webcam", frame);

		//Create a vector telling opencv to save file as pgm
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PXM_BINARY);
		compression_params.push_back(0);
		
		if (timer % 15 == 0) {
		//if (waitKey(30) != -1){
			cout << "Saving image: " << counter << endl;
			stringstream filename;
			filename << "img/img-" << argv[2] << "-" << counter++ << ".pgm";
			imwrite(filename.str(), frame, compression_params);
		}
		timer += 1;
		//if(waitKey(30) >= 0) {
			// terminate program on key press 
		//	break;
		//}
	}

	return 0;
}
