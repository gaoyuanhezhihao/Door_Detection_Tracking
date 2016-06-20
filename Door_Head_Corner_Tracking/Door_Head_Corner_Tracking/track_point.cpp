#include "track_point.h"
bool track_door(vector<Point> &door_head_points, Mat &img, double alpha, double beta, double deta) {
	// calculate the ROI.
	//double L = cv::norm(door_head_points[0]-door_head_points[1]);
	double L = abs(door_head_points[0].x - door_head_points[1].x);
	Point up_left;
	Point down_right;
	up_left.x = min(door_head_points[0].x, door_head_points[1].x) - alpha * L;
	up_left.y = min(door_head_points[0].y, door_head_points[1].y) - alpha * L;
	up_left.x = up_left.x >= 0 ? up_left.x : 0;
	up_left.y = up_left.y >= 0 ? up_left.y : 0;
	down_right.x = max(door_head_points[0].x, door_head_points[1].x) + alpha* L;
	down_right.y = max(door_head_points[0].y, door_head_points[1].y) + (alpha + beta) * L;
	down_right.x = down_right.x < img.cols ? down_right.x : img.cols;
	down_right.y = down_right.y < img.rows ? down_right.y : img.rows;
	Rect search_rect(up_left, down_right);
	Mat search_im = img(search_rect);
	
	double head_line_theta = get_theta_from_2point(door_head_points[0], door_head_points[1]);
	array<array<double, 2>, 2> head_line_neib_theta;
	array<array<double, 2>, 2> v_line_neib_theta;
	get_neigbor_theta_range(head_line_theta, deta*2, head_line_neib_theta);
	get_neigbor_theta_range(0.0, deta, v_line_neib_theta);
	vector<Vec2f> vertical_lines;
	vector<Vec2f> head_lines;
	vector<Vec2f> long_lines;
	vector<Vec2f> short_lines;
	vector<Point> Intersection;
	Mat edge_im;
	Mat cdst;
	Mat dst;
	bool vp_detected = false;
	vector<Point> inter_points;

	blur(search_im, search_im, Size(3, 3));
	Canny(search_im, dst, 30, 70, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	// detect lines
	HoughLines(dst, long_lines, 1, CV_PI / 180, 1.5*L, 0, 0);
	HoughLines(dst, short_lines, 1, CV_PI / 180, 0.5*L, 0, 0);


	select_lines(long_lines, vertical_lines, v_line_neib_theta);
	draw_line(cdst, vertical_lines, Scalar(0, 0, 100));
	select_lines(short_lines, head_lines, head_line_neib_theta);
	if (head_lines.size() == 0) {
		draw_line(cdst, short_lines, Scalar(0, 100, 100));
	}
	else {
		draw_line(cdst, head_lines, Scalar(0, 255, 0));
	}
	rectangle(img, search_rect, Scalar(255, 0, 0));
	// get intersection point of the lines.
	Point inter_point;
	for (int i = 0; i < vertical_lines.size(); ++i) {
		for (int j = 0; j < head_lines.size(); ++j) {
			get_public_point(vertical_lines[i], head_lines[j], inter_point);
			inter_points.push_back(inter_point);
			//circle(cdst, inter_point, 2, Scalar(100, 0, 0));
		}
	}
	//select_best_point(inter_points, door_head_points, search_im);
	imshow("line image", cdst);
	imshow("edge image", dst);
	cout << "detected long line count:" << long_lines.size() << '\n' <<
		"vertical line count: " << vertical_lines.size() << '\n' <<
		"detected short line count: " << short_lines.size() << '\n'<<
		"head line count: " << head_lines.size() << '\n' <<
		"cdst size:" << cdst.size() << "search rect" << search_rect.width << ',' << search_rect.height
		<< endl;

	return true;
}


bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p){
	double rho1 = line1[0], theta1 = line1[1];
	double rho2 = line2[0], theta2 = line2[1];
	double denom = (sin(theta1)*cos(theta2) - cos(theta1)*sin(theta2));
	inter_p.x = (rho2*sin(theta1) - rho1*sin(theta2)) / denom;
	inter_p.y = (rho1*cos(theta2) - rho2*cos(theta1)) / denom;
	return true;
}