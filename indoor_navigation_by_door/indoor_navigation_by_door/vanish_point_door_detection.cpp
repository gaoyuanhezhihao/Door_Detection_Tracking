#include "vanish_point_door_detection.h"



size_t global_img_cols = 0;
size_t global_img_rows = 0;
bool _transform_door_vline(vector<Vec2f> &origin_vlines, vector<pair<Point, Point>> &door_line_candidate, \
	size_t rows, size_t cols);
Point get_vanish_point(vector<Point> & points, Mat & cdst);
double _calc_dist_of_point_line(Point &vp, double x1, double y1, double x2, double y2);



vp_states vanish_point_detection(Mat & image, Mat & cdst, Mat &edge_im, vector<Vec2f> &vertical_lines, Point &vp)
{
	//static object_target target;
	//static bool is_track_task_full = false;
	global_img_cols = cdst.cols;
	global_img_rows = cdst.rows;
	//Point vp;// vanishing point.
	vector<Vec2f> left_lines;
	vector<Vec2f> right_lines;
	vertical_lines.clear();
	//vector<Vec2f> vertical_lines;
	vector<Vec2f> other_lines;
	vector<Point> Intersection;
	vector<std::array<Point, 4>> doors_points;
	vector<Vec2f> door_vertical_lines;
	//Mat edge_im;
	bool vp_detected = false;
	hough_line_detect(image, cdst, edge_im, left_lines, right_lines, vertical_lines, other_lines);
	draw_line(cdst, left_lines, Scalar(0, 255, 0));
	draw_line(cdst, right_lines, Scalar(255, 0, 0));
	draw_line(cdst, vertical_lines, Scalar(0, 0, 100));
	//draw_line(cdst, door_vertical_lines, Scalar(0, 0, 255));
	//draw_line(cdst, other_lines, Scalar(255, 0, 255));
	//size_t i = 0, j = 0;
	//long sum_x = 0;
	//long sum_y = 0;
	//double x = 0;
	//double y = 0;
	//for (i = 0; i < left_lines.size(); ++i)
	//{
	//	for (j = 0; j < right_lines.size(); ++j)
	//	{
	//		float rho_l = left_lines[i][0], theta_l = left_lines[i][1];
	//		float rho_r = right_lines[j][0], theta_r = right_lines[j][1];
	//		double denom = (sin(theta_l)*cos(theta_r) - cos(theta_l)*sin(theta_r));
	//		x = (rho_r*sin(theta_l) - rho_l*sin(theta_r)) / denom;
	//		y = (rho_l*cos(theta_r) - rho_r*cos(theta_l)) / denom;
	//		Point pt(x, y);
	//		circle(cdst, pt, 5, Scalar(0, 0, 255));
	//	}
	//}
	if (left_lines.size() == 0 && right_lines.size() == 0)
	{
		// neighter of the two side line is detected.
		vp.x = 0;
		vp.y = 0;
		return vp_states::no_line;
	}
	else if (left_lines.size() == 0)
	{
		// only lines of right side are detected.
		vp.x = 1;
		vp.y = 0;
		return vp_states::only_right_line;
	}
	else if (right_lines.size() == 0)
	{
		// only lines of left side are detected.
		vp.x = cdst.rows;
		vp.y = 0;
		return vp_states::only_left_line;
	}
	size_t i = 0, j = 0;
	double x = 0;
	double y = 0;
	for (i = 0; i < left_lines.size(); ++i)
	{
		for (j = 0; j < right_lines.size(); ++j)
		{
			float rho_l = left_lines[i][0], theta_l = left_lines[i][1];
			float rho_r = right_lines[j][0], theta_r = right_lines[j][1];
			double denom = (sin(theta_l)*cos(theta_r) - cos(theta_l)*sin(theta_r));
			x = (rho_r*sin(theta_l) - rho_l*sin(theta_r)) / denom;
			y = (rho_l*cos(theta_r) - rho_r*cos(theta_l)) / denom;
			Point pt(x, y);
			Intersection.push_back(pt);
			circle(cdst, pt, 5, Scalar(0, 150, 150));
		}
	}

	vp = get_vanish_point(Intersection, cdst);
	return vp_states::got_vp;
	//Detect_door(cdst, edge_im, vertical_lines, doors_points, vp);
	//if (false == is_track_task_full) {
	//	if (door_states::Get_Door == Detect_door(cdst, edge_im, vertical_lines, doors_points, vp)) {
	//		Rect target_window(doors_points[0][1].x + 5, doors_points[0][2].y + 10, \
	//			doors_points[0][2].x - doors_points[0][1].x - 10, doors_points[0][3].y - doors_points[0][2].y - 10);
	//		init_cam_shift(target, image, target_window);
	//		is_track_task_full = true;
	//	}
	//}
	//else {
	//	update_target_window(target, image);
	//}
	//return '\0';
}

Point get_vanish_point(vector<Point> & points, Mat & cdst)
{
	long x = 0, y = 0;
	if (points.size() == 0)
	{
		return Point(0, 0);
	}
	for (int i = 0; i < points.size(); ++i)
	{
		x += points[i].x;
		y += points[i].y;
	}
	x /= points.size();
	y /= points.size();
	Point vp(x, y);
	circle(cdst, vp, 5, Scalar(0, 0, 255));
	return vp;
}

bool hough_line_detect(Mat & image, Mat & cdst, Mat & dst, vector<Vec2f> & left_lines, vector<Vec2f> & right_lines, vector<Vec2f> &vertical_lines, vector<Vec2f> & other_lines)
{
	//Mat dst;
	Canny(image, dst, 30, 70, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	vector<Vec2f> lines;
	// detect lines
	HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		if (rho < 0) {
			cout << "debug" << endl;
		}
		if (10 * PI / 180 < theta && theta < 60 * PI / 180)
		{
			left_lines.push_back(lines[i]);
		}
		else if (110 * PI / 180 < theta && theta < 170 * PI / 180)
		{
			right_lines.push_back(lines[i]);
		}
		else if (theta < 5 * PI / 180 || theta > 175 * PI / 180)
		{
			vertical_lines.push_back(lines[i]);
		}
		else
		{
			other_lines.push_back(lines[i]);
		}
	}
	return true;
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
		cv::line(image, pt1, pt2, color, 3, CV_AA);
	}
	return true;
}

door_states Detect_door(Mat & cdst, Mat & edge_im, vector<Vec2f> &vertical_lines, vector<std::array<Point, 4>> & doors_points, Point & vp) {
	// Sort the lines by the rho param.
	struct sort_match{
		bool operator()(const pair<Point, Point> & left, const pair<Point, Point> & right)
		{

			return left.first.x > right.first.x;
		}
	};
	bool found_door = false;
	vector<pair<Point, Point>> door_line_candidate;
	vector<pair<Point, Point> > merged_door_line_candidate;
	_transform_door_vline(vertical_lines, door_line_candidate, cdst.rows, cdst.cols);
	sort(door_line_candidate.begin(), door_line_candidate.end(), sort_match());

	for (size_t k = 0; k < door_line_candidate.size(); ++k) {
		cv::line(cdst, door_line_candidate[k].first, door_line_candidate[k].second, { 0, 0, 255 }, 3, CV_AA);
	}
	// Merge the lines that are too close. 
	//vector<Vec2f> merged_vertical_lines;
	size_t i = 0;
	size_t j = 0;
	int x_left = 0;
	int x_right = 0;
	size_t thres = cdst.cols * DOOR_MERGE_MAX_DIST;
	for (i = 0; i < door_line_candidate.size();) {
		x_right = door_line_candidate[i].first.x;
		for (j = i + 1; j < door_line_candidate.size(); ++j) {
			x_right = door_line_candidate[j].first.x;
			assert(x_right >= x_left);
			if (x_right - x_left > thres) {
				break;
			}
		}
		merged_door_line_candidate.push_back(door_line_candidate[i]);
		i = j;
	}
	//	vertical_lines = merged_vertical_lines;
	//	return door_states::No_Door;
	//  match pairs of lines in its left side whose distance is in valid range.
	size_t next_id = 0;
	for (i = 0; i < merged_door_line_candidate.size(); ++i) {
		if (find_next_vline(merged_door_line_candidate, doors_points, next_id, i, edge_im, cdst, vp) == door_states::Found_a_door) {
			found_door = true;
		}
	}
	if (found_door){
		return door_states::Get_Door;
	}
	return door_states::No_Door;
}

bool init_vanishing_point(cv::VideoCapture &cap, int & center_x_min,
	int &center_x_max, int &center_y_min, int &center_y_max) {
	Mat image, cdst, edge_img;
	vector<Vec2f> vlines;
	Point mark_point;
	vp_states line_state;

	char order;
	while (1)
	{
		cap >> image;
		if (image.rows == 0 || image.cols == 0)
		{
			continue;
		}
		line_state = vanish_point_detection(image, cdst, edge_img, vlines, mark_point);
		imshow("Video", cdst);
		waitKey(1);
		cout << "what about this ?" << endl;
		std::cin >> order;
		if (order == 'y')
		{
			break;
		}
	}
	center_x_max = (1 + CENTER_RANGE) * mark_point.x;
	center_x_min = (1 - CENTER_RANGE) * mark_point.x;
	center_y_max = (1 + CENTER_RANGE) * mark_point.y;
	center_y_min = (1 - CENTER_RANGE) * mark_point.y;
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

door_states find_next_vline(vector<pair<Point, Point>> & vertical_lines, vector<std::array<Point, 4>> & doors_points, \
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
			return door_states::Not_a_door;
		}
		else if (pt1_right.x - pt1_left.x < min_width_of_door) {
			continue;
		}
		else {
			door_head_lines.clear();
			if (true == get_door_head_lines(door_head_lines, edge_im, pt1_left.x, pt1_right.x, vp)) {
				for (size_t k = 0; k < door_head_lines.size(); ++k) {
					cv::line(cdst, { door_head_lines[k][0], door_head_lines[k][1] }, \
					{ door_head_lines[k][2], door_head_lines[k][3] }, { 255, 255, 0 }, 3, CV_AA);
					cv::circle(cdst, Point(door_head_lines[k][0], door_head_lines[k][1]), 5, { 255, 0, 255 });
					cv::circle(cdst, Point(door_head_lines[k][2], door_head_lines[k][3]), 5, { 255, 0, 255 });
					doors_points.push_back(std::array < Point, 4 > {{ Point{ door_head_lines[k][0], int(door_head_lines[k][1] + DOOR_HEIGHT * cdst.rows) }, \
						Point{ door_head_lines[k][0], door_head_lines[k][1] }, Point(door_head_lines[k][2], door_head_lines[k][3]), \
						Point(door_head_lines[k][2], door_head_lines[k][3] + DOOR_HEIGHT * cdst.rows) }});
				}
				return door_states::Found_a_door;
			}
		}
	}
	return door_states::Not_a_door;
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
			array<int, 4> new_head_line = { left_x, y1, right_x, y2 };
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