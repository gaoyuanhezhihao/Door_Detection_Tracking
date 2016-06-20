#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>
#include "track_point.h"
#include "time.h"
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
	bool tracking = false;
	double duration_ms;
	clock_t start_time, end_time;
	Rect line_search_rect, feature_search_rect;
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> feature;
	double start = double(getTickCount());
	// Read image from file 
	VideoCapture cap("C270_2.wmv");
	Mat img, search_img;
	search_range_state track_state;
	//if fail to read the image
	if (!cap.isOpened())
	{
		cout << "Error Can not open video" << endl;
		return -1;
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
			tracking = false;
			break;
		case 's':
			cap >> search_img;
			start_time = clock();
			start = double(getTickCount());
			track_state = track_door(door_head_points, search_img, line_search_rect, feature_search_rect, feature, ALPHA, BETA, DETA);
			end_time = clock();
			cout << "clickes: " << end_time - start_time << "seconds: " << ((float)(end_time - start_time) / CLOCKS_PER_SEC) << endl;
			duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
			cout << "It took " << duration_ms << " ms." << endl;
			circle(search_img, door_head_points[0], 3, Scalar(255, 0, 0));
			circle(search_img, door_head_points[1], 3, Scalar(255, 0, 0));
			rectangle(search_img, feature_search_rect, Scalar(0, 100, 0));
			rectangle(search_img, line_search_rect, Scalar(0, 0, 100));
			imshow("match image", search_img);
			break;
		default:
			break;
		}
		if (initialized == false && door_head_points.size() == 2) {
			init_tracking(door_head_points, img, line_search_rect,feature_search_rect, feature, ALPHA, BETA);
			rectangle(img, feature_search_rect, Scalar(0, 100, 0));
			rectangle(img, line_search_rect, Scalar(100, 0, 0));

			//circle(img, door_head_points[0], 3, Scalar(255, 0, 0));
			//circle(img, door_head_points[1], 3, Scalar(255, 0, 0));

			//for (int i = 0; i < 2; ++i) {
			//	for (int j = 0; j < HoG_GRAD_BIN_SIZE * 9; ++j)
			//	{
			//		cout << feature[i][j] << ',';
			//	}
			//	cout << endl;
			//}
			cout << "press \'s\' to select the dst image, or press \'y\' to start matcing..." << endl;
			initialized = true;
		}
		if (tracking) {
			cap >> search_img;
			start_time = clock();
			start = double(getTickCount());
			track_state = track_door(door_head_points, search_img, line_search_rect, feature_search_rect, feature, ALPHA, BETA, DETA);
			end_time = clock();
			cout << "clickes: " << end_time - start_time << "seconds: " << ((float)(end_time - start_time) / CLOCKS_PER_SEC) << endl;
			duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
			cout << "It took " << duration_ms << " ms." << endl;
			imshow("match image", search_img);
			if (track_state != search_range_state::safe) {
				tracking = false;
				cout << "got to the edge, stop tracking." << endl;
			}
			else {
				cout << "updated..." << endl;
			}
		}
	}


	// Wait until user press some key
	waitKey(0);

	return 0;

}