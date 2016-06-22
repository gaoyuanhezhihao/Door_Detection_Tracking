#ifndef TRACK_POINT_H
#define TRACK_POINT_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include "Line.h"
#include "feature.h"

#define FEATURE_MATCH_THRES 1500
#define RETRY_MAX 8
#define ENLARGE_RATIO 1.2

using namespace std;
using namespace cv;

enum decision {
	retry,
	keep_going,
	stop_car,
	turn_lef,
	turn_right,
	switch_to_camShift,
};
enum search_range_state {
	near_lr_edge,
	near_ud_edge,
	near_lr_ud_edge, // very dangerous.
	search_range_too_small,
	safe
};

//struct track_state {
//	bool near_lr_edge = false;
//	bool near_ud_edge = false;
//	bool ok = false;
//	bool bad_match = false;
//	bool bad_line = false;
//};
enum track_state
{
	near_lr,
	near_ud,
	ok,
	bad_match,
	bad_line,
};

enum key_point_state {
	good,
	bad_p1,
	bad_p2,
	bad_two,
	range_too_small
};

decision decide(track_state state, int times_tried,
	Point & p1, Point&p2, Point&old_p1, Point&old_p2, Size img_size);
bool init_tracking(vector<Point> & door_head_points, Point & old_p1,
	Point & old_p2, Mat &img, Rect & line_rect,
	Rect & feature_rect,
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog,
	double alpha, double beta);
bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p);
key_point_state select_best_point(vector<Vec2f> &v_lines, vector<Vec2f> & h_lines, Rect & feature_search_rect, Rect & line_search_rect,
	vector<Point> & door_head_points, Mat &ori_img,
	const array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog,
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & best_features,
	double & min_error1, double & min_error2);
search_range_state get_hough_rect(const Point & old_p1, const Point & old_p2,
	Size img_size, const Point & p1,
	const Point & p2, Rect &hough_rect,
	double alpha, double beta);
search_range_state get_hog_rect(const Point & old_p1, const Point & old_p2,
	Size img_size, const Point &p1,
	const Point &p2, Rect & hog_rect,
	double tau);
track_state track_door(Point &old_p1, Point & old_p2,
	vector<Point> &door_head_points, Mat &ori_img, Rect & line_rect,
	Rect & feature_rect,
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog,
	double alpha, double beta, double deta);
#endif //TRACK_POINT_H