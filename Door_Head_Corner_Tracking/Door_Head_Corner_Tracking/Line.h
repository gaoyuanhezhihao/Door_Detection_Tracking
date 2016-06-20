#ifndef MAIN_FUNC_H
#define MAIN_FUNC_H
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <array>

#define PI 3.1415926

using namespace std;
using namespace cv;
enum ret_code{
	Get_Door,
	No_Door,
	Found_a_door,
	Not_a_door
};
int select_lines(vector<Vec2f> & lines, vector<Vec2f> & selected_lines, array<array<double, 2>, 2> &theta_range);
bool hough_line_detect(Mat & image, Mat & cdst, Mat & dst, double thres, vector<Vec2f> & lines);
char vanish_point_detection(Mat & image, Mat & cdst);
bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color);
ret_code Detect_door(Mat & cdst, Mat & edge_im, vector<Vec2f> &vertical_lines, vector<std::array<Point, 4>> & doors_points, Point & vp);
ret_code find_next_vline(vector<pair<Point, Point>> & vertical_lines, vector<std::array<Point, 4>> & doors_points, \
	size_t & next_id, size_t start_id, const Mat & edge_im, Mat & cdst, Point & vp);
bool get_door_head_lines(vector<array<int, 4>> &door_head_line_segment, const Mat & edge_im, int left_x, int right_x, Point &vp);
void get_neigbor_theta_range(double theta, double deta, array<array<double, 2>, 2> & neib_theta_range);
double get_theta_from_2point(Point & p1, Point & p2);
#endif MAIN_FUNC_H