#ifndef MAIN_FUNC_H
#define MAIN_FUNC_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Compile_Order.h"
#include "vanish_point_door_detection.h"
#include "Camshift.h"

#define TRACK_LR_EDGE_THRES 5
#define TRACK_END_FORWARD_DIST 1//meter


using namespace cv;
using namespace std;

bool judge_vanish_point(Point &current_point, vp_states & current_state,
	int center_x_min, int center_x_max, int center_y_min,
	int center_y_max, VideoCapture & cap, Mat & cdst, SOCKET & sclient);
bool judge_door(system_mode & current_mode, object_target & target,
	Mat & image, door_states &curr_door_states,
	const vector<std::array<Point, 4>> & doors_points);
bool verify_bias(const char last_state, cv::VideoCapture & cap, cv::Mat & cdst, cv::Point & current_point, SOCKET & sclient);
bool judge_tracking(system_mode & current_mode, object_target & target,
	Size & view_size, SOCKET & sclient);

#endif //MAIN_FUNC_H