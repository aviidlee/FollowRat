#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
//Don't need core as highgui already has core
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	string WINDOW_NAME = "Window";
    cout << "Start" << endl;

	Mat img = imread(argv[1], CV_LOAD_IMAGE_UNCHANGED);

	if (img.empty())
	{
		cout << "Error, image empty" << endl;
		//system("pause");
		return -1;
	}

	SimpleBlobDetector::Params params;
	
	params.minThreshold = 40;
	params.maxThreshold = 60;
	params.thresholdStep = 5;
	params.minDistBetweenBlobs = 70.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.blobColor = 200.0f;
	params.filterByCircularity = false;
	//params.minCircularity = 1.0f;
	//params.maxCircularity = 200.0f;
	params.filterByArea = true;
	params.minArea = 10.0f;
	params.maxArea = 500.0f;
	
	SimpleBlobDetector blob_detector(params);

	vector<KeyPoint> keypoints;
	blob_detector.detect(img, keypoints);

	for (int i=0; i<keypoints.size(); i++){
		KeyPoint p = keypoints[i];
		cout << keypoints[i].pt.x << "," << keypoints[i].pt.y << endl;
		circle( img, p.pt, p.size, Scalar(0, 0, 255), 1, 8);

	}

	namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	imshow(WINDOW_NAME, img);

	imwrite("test.jpg", img);

	waitKey(0);

	destroyWindow(WINDOW_NAME);

	cout << "Done";

	return 0;


}
