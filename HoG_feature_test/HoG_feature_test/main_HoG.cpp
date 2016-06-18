#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <array>
#include "Config.h"
#include "feature.h"


using namespace std;
using namespace cv;

vector<Point> door_head_points;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		door_head_points.push_back(Point(x, y));
	}
}



int main(int argc, char** argv)
{
	bool initialized = false;
	Rect search_range;
	// Read image from file 
	VideoCapture cap("C270_2.wmv");
	Mat img;
	Mat search_img;
	//if fail to read the image
	if (!cap.isOpened())
	{
		cout << "Error Can not open video" << endl;
		return -1;
	}
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> feature_hog = { 0, 0};
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < HoG_GRAD_BIN_SIZE * 9; ++j)
		{
			cout << feature_hog[i][j] << ',';
		}
		cout << endl;
	}
	cap >> img;
	//Create a window
	namedWindow("My Window", 1);
	//set the callback function for any mouse event
	setMouseCallback("My Window", CallBackFunc, NULL);
	while (1){
		//show the image
		imshow("My Window", img);
		char c = (char)waitKey(10);
		switch (c)
		{
		case 'n':
			cap >> img;
			door_head_points.clear();
			initialized = false;
			break;
		case 's':
			cap >> search_img;
			imshow("match image", search_img);
			break;
		case 'y':
			if (initialized) {
				Point match_point1, match_point2;
				find_match(search_img, search_range, feature_hog, match_point1, match_point2, door_head_points);
				circle(search_img, match_point1, 3, Scalar(255, 0, 0));
				circle(search_img, match_point2, 3, Scalar(0, 0, 255));
				rectangle(search_img, search_range, Scalar(0, 255, 0));
				imshow("match image", search_img);
			}
			else {
				cout << "WARNING: you should select the two head points before press 's'" << endl;
			}
			break;
		default:
			break;
		}
		if (initialized == false && door_head_points.size() == 2) {
			//init_track(door_head_points, img, search_range, feature_hog);
			init_hog(door_head_points, img, search_range, feature_hog, 0.8, 1.5);
			rectangle(img, search_range, Scalar(0, 255, 0));
			//circle(img, door_head_points[0], 3, Scalar(255, 0, 0));
			//circle(img, door_head_points[1], 3, Scalar(255, 0, 0));

			for (int i = 0; i < 2; ++i) {
				for (int j = 0; j < HoG_GRAD_BIN_SIZE * 9; ++j)
				{
					cout << feature_hog[i][j] << ',';
				}
				cout << endl;
			}
			cout << "press \'s\' to select the dst image, or press \'y\' to start matcing..." << endl;
			initialized = true;
		}
	}
	return 0;

}