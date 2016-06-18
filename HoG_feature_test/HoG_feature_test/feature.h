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
#define PI 3.1415926
bool init_track(vector<Point> & door_head_points, Mat &img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2>  &feature_hog, double alpha = 0.8, double beta = 1.5);
bool get_hog(Mat & img, Point & patch_center, array<double, HoG_GRAD_BIN_SIZE * 9> &descr);
int get_gradient_id(int bin_size, double bin_range, double dx, double dy);
double compare_feature(array<double, HoG_GRAD_BIN_SIZE * 9> & f1, array<double, HoG_GRAD_BIN_SIZE * 9> & f2);
//bool find_match(Mat & search_img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, Point &match_point1, Point &match_point2);
bool find_match(Mat & search_img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, Point &match_point1, Point &match_point2, vector<Point> & door_head_points);
bool get_grad_img(Mat & im_Ix, Mat & im_Iy, Mat & grad_angle_id, Mat & grad_mag);
bool get_hog_from_map(array<double, HoG_GRAD_BIN_SIZE * 9> & feature, Mat & grad_angle_id, Mat & grad_mag, int center_r, int center_c);
bool init_hog(vector<Point> & door_head_points, Mat &img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog, double alpha, double beta);
//*********************************************************
#endif //FEATURE_H