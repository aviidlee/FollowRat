/**
 * FILE: std.cpp
 * AUTHOR: avadendas
 * DESCRIPTION: The most vanilla cpp template. 
 */

#include <iostream>
#include <string>
#include <algorithm>

// Blobs lib
#include <BlobResult.h>

// OpenCV
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  int deviceNo;

  if(argc == 2) {
    deviceNo = atoi(argv[1]);
  } else {
    cout << "Usage: ./blobslib_test <deviceNumber>" << endl;
    cout << "Hint: use 0 as deviceNumber for default device." << endl;
    return -1;
  }

  cout << "Commencing blob detection from web cam stream..." << endl;

  VideoCapture cap(deviceNo);
  CBlob currentBlob;

  if(!cap.isOpened()) {
    cout << "Something wrong with video capture." << endl;
    return -2;
  }

  cvNamedWindow("Webcam", 1);
  cvNamedWindow("Blobs");
  
  Mat frame;

  while(1) {
    cap >> frame;
    Mat blobFrame(frame.size(), frame.type());

    CBlobResult blobs(frame);
    int numBlobs = blobs.GetNumBlobs();
    // cout << "numBlobs " << numBlobs << endl;

    for(int i = 0; i < numBlobs; i++) {
      try {
        // currentBlob = new CBlob(blobs.GetBlob(i));
        // Mat, colour, offset x, offset y, bool contours, Mat srcImg
        currentBlob = blobs.GetBlob(i);
        currentBlob.FillBlob(blobFrame, Scalar(255, 0, 0));
        // currentBlob.FillBlob(frame, Scalar(255, 0, 0));
      } catch (Exception& e) {
        cout << "BLOODY EXCEPTIONS!!!" << endl;
        cout << e.what() << endl;
        break;
      }
    }

    imshow("Webcam", frame);
    imshow("Blobs", blobFrame);

    if(waitKey(10) >= 0) {
      break;
    }
  }

  return 0;
}
