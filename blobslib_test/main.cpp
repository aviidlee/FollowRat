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
  cout << "Commencing blob detection from web cam stream..." << endl;
  int deviceNo;

  if(argc == 2) {
    deviceNo = atoi(argv[1]);
  } else {
    cout << "Usage: ./blobslib_test <deviceNumber>" << endl;
    cout << "Hint: use 0 as deviceNumber for default device." << endl;
    return -1;
  }

  VideoCapture cap(deviceNo);
  CBlob currentBlob;

  if(!cap.isOpened()) {
    cout << "Something wrong with video capture." << endl;
    return -2;
  }

  cvNamedWindow("Webcam", 1);
  cvNamedWindow("Blobs");
  
  Mat frame;
  Mat blobFrame;

  while(1) {
    cap >> frame;
    CBlobResult blobs(frame);
    int numBlobs = blobs.GetNumBlobs();

    for(int i = 0; numBlobs; i++) {
      currentBlob = blobs.GetBlob(i);
      currentBlob.FillBlob(blobFrame, CV_RGB(255, 0, 0));
    }

    imshow("Webcame", frame);
    imshow("Blobs", blobFrame);

    if(waitKey(10) >= 0) {
      break;
    }
  }

  return 0;
}
