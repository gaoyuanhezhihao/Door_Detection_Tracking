#ifndef TRACK_POINT_H
#define TRACK_POINT_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include "Line.h"
#include "feature.h"

using namespace std;
using namespace cv;

search_range_state track_door(vector<Point> &door_head_points, Mat &ori_img, Rect & line_rect, Rect & feature_rect, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, double alpha, double beta, double deta);
bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p);
search_range_state select_best_point(vector<Vec2f> &v_lines, vector<Vec2f> & h_lines, Rect & feature_search_rect, 
										Rect & line_search_rect, vector<Point> & door_head_points, 
										Mat &ori_img, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, 
										double & min_error1, double & min_error2);
#endif //TRACK_POINT_H