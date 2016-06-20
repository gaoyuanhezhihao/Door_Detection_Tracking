#ifndef TRACK_POINT_H
#define TRACK_POINT_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include "Line.h"

using namespace std;
using namespace cv;

bool track_door(vector<Point> &door_head_points, Mat &img, double alpha = 0.5, double beta = 3, double deta= 5*PI/180);
bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p);
#endif //TRACK_POINT_H