#ifndef VANISH_POINT_DOOR_DETECTION_H
#define VANISH_POINT_DOOR_DETECTION_H

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <winsock2.h>
#include "enum_class.h"
#include "communication.h"
#include "Compile_Order.h"
#include <numeric>
#include <array>

#define TURN_DEGREE 1
#define PI 3.1415926
#define CENTER_RANGE 0.3
#define DOOR_MERGE_MAX_DIST 0.05

using namespace std;
using namespace cv;


bool hough_line_detect(Mat & image, Mat & cdst, Mat & dst, vector<Vec2f> & left_lines, vector<Vec2f> & right_lines, vector<Vec2f> &vertical_lines, vector<Vec2f> & other_lines);
//vp_states vanish_point_detection(Mat & image, Mat & cdst);
bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color);
door_states Detect_door(Mat & cdst, Mat & edge_im, vector<Vec2f> &vertical_lines, vector<std::array<Point, 4>> & doors_points, Point & vp);
door_states find_next_vline(vector<pair<Point, Point>> & vertical_lines, vector<std::array<Point, 4>> & doors_points, \
	size_t & next_id, size_t start_id, const Mat & edge_im, Mat & cdst, Point & vp);
bool get_door_head_lines(vector<array<int, 4>> &door_head_line_segment, const Mat & edge_im, int left_x, int right_x, Point &vp);
bool init_vanishing_point(cv::VideoCapture &cap, int & center_x_min,
	int &center_x_max, int &center_y_min, int &center_y_max);
vp_states vanish_point_detection(Mat & image, Mat & cdst,
	Mat &edge_im, vector<Vec2f> &vertical_lines, Point &vp);
#endif // VANISH_POINT_DOOR_DETECTION_H