#ifndef FEATURE_H
#define FEATURE_H
//-----------------------------------------------------

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <array>
#include <iostream>
#include <limits>
using namespace std;
using namespace cv;
#define HoG_GRAD_BIN_SIZE 8
#define STD_CELL_WIDTH 8
#define STD_CELL_HEIGHT 8
#define STD_CELL_PER_BLOCK_ROW 3
#define STD_CELL_PER_BLOCK_COLOMN 3
#define PI 3.1415927
#define ALPHA 0.8
#define BETA 3
#define DETA (8 * PI / 180)
#define TAU 2.0
#define CHI 1.0 // vertical line threshold ratio.
#define ETA 0.7 // door head line lenth threshold ratio.
#define RHO 1 // rho param of the hough line detection function.
#define THETA (CV_PI / 180) // theta param of the hough line detection function.

enum search_range_state {
	near_lr_edge,
	near_ud_edge,
	near_lr_ud_edge, // very dangerous.
	search_range_too_small,
	safe
};

int get_gradient_id(int bin_size, double bin_range, double dx, double dy);
double compare_feature(array<double, HoG_GRAD_BIN_SIZE * 9> & f1, array<double, HoG_GRAD_BIN_SIZE * 9> & f2);
//bool find_match(Mat & search_img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, Point &match_point1, Point &match_point2);
search_range_state find_match(Mat & search_img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, Point &match_point1, Point &match_point2, vector<Point> & door_head_points);
bool get_grad_img(Mat & im_Ix, Mat & im_Iy, Mat & grad_angle_id, Mat & grad_mag);
bool get_hog_from_map(array<double, HoG_GRAD_BIN_SIZE * 9> & feature, const Mat & grad_angle_id,
						const Mat & grad_mag, const Rect & feature_rect, int center_r, int center_c);
//bool get_hog_from_local_map(array<double, HoG_GRAD_BIN_SIZE * 9> & feature, Mat & grad_angle_id, Mat & grad_mag, int center_r, int center_c);
bool init_tracking(vector<Point> & door_head_points, Mat &img, Rect & line_rect, Rect & feature_rect, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog, double alpha, double beta);
//search_range_state get_search_range(const Point & p1, const Point & p2, Size img_size, Rect & search_rect, double alpha, double beta);
search_range_state get_hough_rect(Size img_size, const Point & p1, const Point & p2, Rect &hough_rect,
	double alpha, double beta);
search_range_state get_hog_rect(Size &img_size, const Point &p1, const Point &p2, Rect & hog_rect, double tau);
//*********************************************************
#endif //FEATURE_H