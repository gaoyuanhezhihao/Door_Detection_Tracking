#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

#include "main_func.h"
#include "vanish_point_door_detection.h"
#include "communication.h"
#include "Compile_Order.h"
#include "FileLogger.hpp"
#include "Camshift.h"
#include "verification.h"

#define PI 3.1415926
#define CENTER_RANGE 0.3

using namespace cv;
using namespace std;

int main()
{
	// Init system.
	init_file_logger("0.1", "test_log.txt");
	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA data;
	if (WSAStartup(sockVersion, &data) != 0)
	{
		return -1;
	}
	SOCKET sclient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	Init_Socket(sclient);
	//send(sclient, "hello", strlen("hello"), 0);

	Point mark_point, current_vp;
	int center_x_min, center_x_max, center_y_min, center_y_max;
	vp_states current_vp_state;
	door_states current_door_state;
	vector<std::array<Point, 4>> doors_points;
	object_target target;
	vector<Vec2f> vertical_lines;
	Mat image, edge_img;
	Mat trush_img;
	Mat dst, cdst;
	double start = double(getTickCount());
	string cmd_2_car;
	long socket_data_amount = 0;
	char sock_recv_buf[100] = { 0 };
	u_long iMode = 1;
	/*detection*/
	double duration_ms = 0;

	/*verify door*/
	char side_of_door = '\0';

	VideoCapture cap("C270_2.wmv");
	system_mode current_mode = system_mode::detect_vp_dp;
	//VideoCapture cap(0);
	//cap.set(CV_CAP_PROP_FPS, 5);
	if (!cap.isOpened())
	{
		cout << "Capture could not be opened successfully" << endl;
		return -1;
	}



	//Init_Car(sclient);
	namedWindow("Video");
	char order = 0;

	//get the initial position image.

	#ifdef MY_DEBUG_VIDEO_OUT
		Mat debug_frame = Mat::zeros(image.rows, image.cols * 3, CV_8UC3);
		Mat left_debug_frame(debug_frame, Rect(0, 0, image.cols, image.rows));
		Mat center_debug_frame(debug_frame, Rect(image.cols, 0, image.cols, image.rows));
		Mat right_debug_frame(debug_frame, Rect(image.cols * 2, 0, image.cols, image.rows));
		VideoWriter vd_writer("debug_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5.0, Size(image.cols * 3, image.rows));
	#endif
	cap >> image;
	if (image.empty())
	{
		cout << "Video over" << endl;
		//break;
	}
	mark_point.x = image.cols / 2;
	mark_point.y = image.rows / 2;
	center_x_max = (1 + CENTER_RANGE) * mark_point.x;
	center_x_min = (1 - CENTER_RANGE) * mark_point.x;
	center_y_max = (1 + CENTER_RANGE) * mark_point.y;
	center_y_min = (1 - CENTER_RANGE) * mark_point.y;
	while (char(waitKey(1)) != 'q' && cap.isOpened())
	{
		start = double(getTickCount());
		cap >> image;
		//omp_unset_lock(&lck);
		if (image.empty())
		{
			cout << "Video over" << endl;
			//break;
		}
		else
		{
			switch(current_mode) {
			case system_mode::detect_vp_dp:
				#ifdef MY_DEBUG_VIDEO_OUT
				image.copyTo(left_debug_frame);
				#endif
				start = double(getTickCount());
				current_vp_state = vanish_point_detection(image, cdst, edge_img, vertical_lines, current_vp);
				cout << "vanishi_point_detection time:" << (double(getTickCount()) - start) * 1000 / getTickFrequency() << endl;
				start = double(getTickCount());
				if (judge_vanish_point(current_vp, current_vp_state, center_x_min, center_x_max, center_y_min, center_y_max, cap, cdst, sclient)) {
					/*detect doors*/
					doors_points.clear();
					current_door_state = Detect_door(cdst, edge_img, vertical_lines, doors_points, current_vp);
					judge_door(current_mode, target, image, current_door_state, doors_points);
				}
				duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
				#ifdef MY_DEBUG_TIME
				cout << "It took " << duration_ms << " ms." << endl;
				#endif // MY_DEBUG_TIME
				#ifdef MY_DEBUG_SHOW_IM
				line(cdst, Point(mark_point.x, 0), Point(mark_point.x, image.rows), Scalar(100, 100, 100), 1, CV_AA);
				imshow("detected lines", cdst);

				#endif //MY_DEBUG_SHOW_IM
				#ifdef MY_DEBUG_VIDEO_OUT
				cdst.copyTo(right_debug_frame);
				cvtColor(dst, center_debug_frame, CV_GRAY2BGR);
				vd_writer << debug_frame;
				#endif // MY_DEBUG_VIDEO_OUT
				break;
			case system_mode::tracking_door:
				update_target_window(target, image);
				judge_tracking(current_mode, target, image.size(), sclient);
				break;
			case system_mode::verifing_door:
				verify_door(image, target.side_in_im, current_mode, sclient);
				break;
			default:
				throw std::invalid_argument("invalid line state");
				break;
			}
		}
	}
	cout << "Press any key to exit:" << endl;
	std::cin >> order;



	cv::waitKey();
	return 0;
}