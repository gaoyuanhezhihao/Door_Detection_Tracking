#include "Line.h"
#define DOOR_MERGE_MAX_DIST 0.05
size_t global_img_cols = 0;
size_t global_img_rows = 0;
bool _transform_door_vline(vector<Vec2f> &origin_vlines, vector<pair<Point, Point>> &door_line_candidate, \
	size_t rows, size_t cols);
Point get_vanish_point(vector<Point> & points, Mat & cdst);
double _calc_dist_of_point_line(Point &vp, double x1, double y1, double x2, double y2);

bool hough_line_detect(Mat & image, Mat & cdst, Mat & dst, double thres, vector<Vec2f> & lines)
{
	//Mat dst;
	Canny(image, dst, 30, 70, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	// detect lines
	HoughLines(dst, lines, 1, CV_PI / 180, thres, 0, 0);
	return true;
}

int select_lines(vector<Vec2f> & lines, vector<Vec2f> & selected_lines, array<array<double, 2>, 2> &theta_range) {
	int count_selection = 0;
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		if (theta_range[0][0] <= theta && theta < theta_range[0][1] )
		{
			selected_lines.push_back(lines[i]);
			++count_selection;
		}
		else if (theta_range[1][0] <= theta && theta < theta_range[1][1]) {
			selected_lines.push_back(lines[i]);
			++count_selection;
		}
	}
	return count_selection;
}
bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color)
{
	for (size_t i = 0; i < vec_lines.size(); ++i)
	{
		float rho = vec_lines[i][0], theta = vec_lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		pt1.x = rho / a;
		pt1.y = 0;
		pt2.x = pt1.x - image.rows * tan(theta);
		pt2.y = image.rows;
		//double x0 = a*rho, y0 = b*rho;
		//pt1.x = cvRound(x0 + 1000 * (-b));
		//pt1.y = cvRound(y0 + 1000 * (a));
		//pt2.x = cvRound(x0 - 1000 * (-b));
		//pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(image, pt1, pt2, color, 1, CV_AA);
	}
	return true;
}

/*
* Transform the vertical line from "(rho, theta)" mode to "(p0, p1)"--two point model. 
* p0, p1 should be in the y=0 line and that line will be rejected if its po or p1 is not.
* ----------------------------------------------------------------------------
* @vertical_lines: the original vertical lines in (rho, theta) form.
* @door_line_candidate: the candidate of the vertical lines of the doors.
* @rows: the rows of the image.
* @cols: the column count of the image.
*-----------------------------------------------------------------------------
* return: transformed successfully or failed.
*/
bool _transform_door_vline(vector<Vec2f> &origin_vlines, vector<pair<Point, Point>> &door_line_candidate, \
							size_t rows, size_t cols) {
	int i = 0;
	float rho, theta;
	Point pt1, pt2;
	for (i = 0; i < origin_vlines.size(); ++i) {
		rho = origin_vlines[i][0];
		theta = origin_vlines[i][1];
		double a = cos(theta), b = sin(theta);
		pt1.x = rho / a;
		pt1.y = 0;
		pt2.x = pt1.x - rows * tan(theta);
		pt2.y = rows;
		if (pt1.x < 0 || pt1.x > cols || pt2.x < 0 || pt2.x > cols) {
			continue; // reject this candidate.
		}
		else {
			door_line_candidate.push_back({ pt1, pt2 });
		}
	}
	return true;
}

ret_code find_next_vline(vector<pair<Point, Point>> & vertical_lines, vector<std::array<Point, 4>> & doors_points, \
	size_t & next_id, size_t start_id, const Mat & edge_im, Mat & cdst, Point & vp) {
	const int min_width_of_door = edge_im.cols * 0.05;
	const int max_width_of_door = edge_im.cols * 0.2;
	const double DOOR_HEIGHT = 0.2;
	size_t i = 0;
	size_t j = 0;
	Mat door_figure;
	Point pt1_right, pt2_right;
	Point pt1_left, pt2_left;
	vector<array<int, 4>> door_head_lines;
	int left_x = 0, right_x = 0;

	pt1_right = vertical_lines[start_id].first;
	for (i = start_id + 1; i < vertical_lines.size(); ++i) {
		pt1_left = vertical_lines[i].first;
		assert(pt1_left.x <= pt1_right.x);
		if (pt1_right.x - pt1_left.x > max_width_of_door) {
			return ret_code::Not_a_door;
		}
		else if (pt1_right.x - pt1_left.x < min_width_of_door) {
			continue;
		}
		else {
			door_head_lines.clear();
			if (true == get_door_head_lines(door_head_lines, edge_im, pt1_left.x, pt1_right.x, vp)) {
				for (size_t k = 0; k < door_head_lines.size(); ++k) {
					cv::line(cdst, { door_head_lines[k][0], door_head_lines[k][1] }, \
							{ door_head_lines[k][2], door_head_lines[k][3] }, {255, 255, 0}, 3, CV_AA);
					cv::circle(cdst, Point(door_head_lines[k][0], door_head_lines[k][1]), 5, {255, 0, 255});
					cv::circle(cdst, Point(door_head_lines[k][2], door_head_lines[k][3]), 5, { 255, 0, 255 });
					doors_points.push_back(std::array < Point, 4 > {{ Point{ door_head_lines[k][0], int(door_head_lines[k][1] + DOOR_HEIGHT * cdst.rows) }, \
						Point{ door_head_lines[k][0], door_head_lines[k][1] }, Point(door_head_lines[k][2], door_head_lines[k][3]), \
						Point(door_head_lines[k][2], door_head_lines[k][3] + DOOR_HEIGHT * cdst.rows) }});
				}
				return ret_code::Found_a_door;
			}
		}
	}
	return ret_code::Not_a_door;
}

bool get_door_head_lines(vector<array<int, 4>> &door_head_line_segment, const Mat & edge_im, int left_x, int right_x, Point &vp) {
	vector<Vec2f> door_head_lines;
	const double point_in_line_thres = 0.01;
	Mat door_figure = edge_im(Rect(left_x, 0, right_x - left_x, edge_im.rows));
	bool get_valid_head_line = false;
	HoughLines(door_figure, door_head_lines, 1, CV_PI / 180, (right_x - left_x) * 0.8, 0, 0);
	if (door_head_lines.empty()) {
		return false;
	}
	else {
		for (size_t i = 0; i != door_head_lines.size(); ++i) {
			//if (30 * PI / 180 > door_head_lines[i][1] || door_head_lines[i][1] > 60 * PI / 180)
			//{
			//	if (300 * PI / 180 > door_head_lines[i][1] || door_head_lines[i][1] > 360 * PI / 180)
			//	{
			//		continue;
			//	}
			//}
			float rho = door_head_lines[i][0];
			float theta = door_head_lines[i][1];
			int y1 = rho / sin(theta);
			int y2 = y1 - door_figure.cols / tan(theta);
			if (y1 < 0 || y2 < 0) {
				continue;
			}
			if (y1 > door_figure.rows / 2 || y2 > door_figure.rows / 2) {
				continue;
			}
			if (point_in_line_thres * door_figure.cols < _calc_dist_of_point_line(vp, left_x, y1, right_x, y2)) {
				continue;
			}
			array<int, 4> new_head_line = {left_x, y1, right_x, y2 };
			door_head_line_segment.push_back(new_head_line);
			get_valid_head_line = true;
		}
		if (get_valid_head_line){
			return true;
		}
	}
	return false;
}

double _calc_dist_of_point_line(Point &vp, double x1, double y1, double x2, double y2) {
	double num = abs((y2 - y1)*vp.x - (x2 - x1)*vp.y + x2*y1 - y2*x1);
	double denom = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
	return num / denom;
}

double get_theta_from_2point(Point & p1, Point & p2) {
	return PI / 2.0 + atan((double)(p1.y - p2.y)/(double)(p1.x - p2.x));
}

void get_neigbor_theta_range(double theta, double deta, array<array<double, 2>, 2> & neib_theta_range) {
	double down_limit = theta - deta;
	double up_limit = theta + deta;
	if (down_limit < 0) {
		neib_theta_range[0][0] = 0.0;
		neib_theta_range[0][1] = up_limit;
		neib_theta_range[1][0] = PI + down_limit;
		neib_theta_range[1][1] = PI;
	}
	else if (up_limit > PI) {
		neib_theta_range[0][0] = 0.0;
		neib_theta_range[0][1] = up_limit - PI;
		neib_theta_range[1][0] = down_limit;
		neib_theta_range[1][1] = PI;
	}
	else {
		neib_theta_range[0][0] = down_limit;
		neib_theta_range[0][1] = up_limit;
		neib_theta_range[1][0] = 0;
		neib_theta_range[1][1] = 0;
	}
}