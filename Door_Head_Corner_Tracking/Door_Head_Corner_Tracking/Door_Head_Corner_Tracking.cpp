#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>
#include "track_point.h"
#include "time.h"
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
	bool processed = false;
	clock_t start_time, end_time;
	double start = double(getTickCount());
	// Read image from file 
	VideoCapture cap("C270_2.wmv");
	Mat img;
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
			processed = false;
			break;
		default:
			break;
		}
		if (processed == false && door_head_points.size() == 2) {
			cap >> img;
			start_time = clock();
			start = double(getTickCount());
			track_door(door_head_points, img);
			end_time = clock();
			cout << "clickes: " << end_time - start_time<< "seconds: "<<((float)(end_time - start_time)/CLOCKS_PER_SEC)  << endl;
			double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
			cout << "It took " << duration_ms << " ms." << endl;
			processed = true;
		}
	}


	// Wait until user press some key
	waitKey(0);

	return 0;

}