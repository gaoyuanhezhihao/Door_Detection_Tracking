#include "track_point.h"
search_range_state track_door(vector<Point> &door_head_points, 
								Mat &ori_img, Rect & line_rect, 
								Rect & feature_rect, 
								array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, 
								double alpha, double beta, double deta) {
	// calculate the ROI.
	//double L = cv::norm(door_head_points[0]-door_head_points[1]);
	//double L = abs(door_head_points[0].x - door_head_points[1].x);
	//Point up_left;
	//Point down_right;
	//up_left.x = min(door_head_points[0].x, door_head_points[1].x) - alpha * L;
	//up_left.y = min(door_head_points[0].y, door_head_points[1].y) - alpha * L;
	//up_left.x = up_left.x >= 0 ? up_left.x : 0;
	//up_left.y = up_left.y >= 0 ? up_left.y : 0;
	//down_right.x = max(door_head_points[0].x, door_head_points[1].x) + alpha* L;
	//down_right.y = max(door_head_points[0].y, door_head_points[1].y) + (alpha + beta) * L;
	//down_right.x = down_right.x < img.cols ? down_right.x : img.cols;
	//down_right.y = down_right.y < img.rows ? down_right.y : img.rows;
	//Rect search_rect(up_left, down_right);

	/*select lines.*/
	search_range_state track_feature_state;
	search_range_state line_search_range_state;
	Mat search_im = ori_img(line_rect);
	double L = abs(door_head_points[0].x - door_head_points[1].x);
	
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
	Canny(search_im, dst, 20, 70, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	HoughLines(dst, long_lines, RHO, THETA, (int)(L*CHI), 0, 0);
	HoughLines(dst, short_lines, RHO, THETA, (int)(L*ETA), 0, 0);


	select_lines(long_lines, vertical_lines, v_line_neib_theta);
	draw_line(cdst, vertical_lines, Scalar(0, 0, 100));
	select_lines(short_lines, head_lines, head_line_neib_theta);

	if (head_lines.size() == 0) {
		draw_line(cdst, short_lines, Scalar(0, 100, 100));
	}
	else {
		draw_line(cdst, head_lines, Scalar(0, 255, 0));
	}
	//rectangle(ori_img, line_rect, Scalar(255, 0, 0));
	imshow("line image", cdst);
	imshow("edge image", dst);
	cout << "detected long line count:" << long_lines.size() << '\n' <<
		"vertical line count: " << vertical_lines.size() << '\n' <<
		"detected short line count: " << short_lines.size() << '\n' <<
		"head line count: " << head_lines.size() << '\n' <<
		"cdst size:" << cdst.size() << "search rect" << line_rect.width <<
		',' << line_rect.height
		<< endl;
	/* feature track. */
	double min_error1, min_error2;
	track_feature_state = select_best_point(vertical_lines, head_lines, feature_rect, line_rect, door_head_points, ori_img, feature_hog, min_error1, min_error2);
	cout << "min errors" << min_error1 << ',' << min_error2 << endl;
	line_search_range_state = get_hough_rect(ori_img.size(), door_head_points[0], door_head_points[1], line_rect, ALPHA, BETA);
	return track_feature_state;
}

search_range_state select_best_point(vector<Vec2f> &v_lines, vector<Vec2f> & h_lines, Rect & feature_search_rect, Rect & line_search_rect,
										vector<Point> & door_head_points, Mat &ori_img, 
										array<array<double, HoG_GRAD_BIN_SIZE * 9>,2> & feature_hog, 
										double & min_error1, double & min_error2) {
	Mat search_im = ori_img(feature_search_rect);
	Mat  search_im_gray_64;
	Mat search_im_gray;
	cvtColor(search_im, search_im_gray, COLOR_BGR2GRAY);
	search_im_gray.convertTo(search_im_gray_64, CV_64FC1);
	Mat Ix_kernel = (Mat_<double>(1, 3) << -1, 0, 1);
	Mat Iy_kernel = (Mat_<double>(3, 1) << -1, 0, 1);
	Mat im_Ix, im_Iy;
	filter2D(search_im_gray_64, im_Ix, -1, Ix_kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(search_im_gray_64, im_Iy, -1, Iy_kernel, Point(-1, -1), 0, BORDER_DEFAULT);

	Mat grad_angle_id = Mat::zeros(im_Ix.rows, im_Ix.cols, CV_8UC1);
	Mat grad_mag = Mat::zeros(im_Ix.rows, im_Ix.cols, CV_64FC1);

	get_grad_img(im_Ix, im_Iy, grad_angle_id, grad_mag);

	int half_patch_width = STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW / 2;
	int half_patch_height = STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_COLOMN / 2;
	array<double, HoG_GRAD_BIN_SIZE * 9> tmp_feature = { 0 };
	array<double, HoG_GRAD_BIN_SIZE * 9> best_feature1 = { 0 }, best_feature2 = {0};
	min_error1 = std::numeric_limits<double>::infinity();
	min_error2 = std::numeric_limits<double>::infinity();
	double tmp_error = 0;
	if (feature_search_rect.height <= STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW || feature_search_rect.width <= STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW) {
		return search_range_state::search_range_too_small;
	}
	// get intersection point of the lines.
	Point inter_point;
	for (int i = 0; i < v_lines.size(); ++i) {
		for (int j = 0; j < h_lines.size(); ++j) {
			get_public_point(v_lines[i], h_lines[j], inter_point);
			if (false == get_hog_from_map(tmp_feature, grad_angle_id, grad_mag, feature_search_rect, inter_point.y + line_search_rect.y, inter_point.x + line_search_rect.x) ){
				continue;// feature out of range.
			}
			else {
				circle(ori_img, Point(inter_point.x + line_search_rect.x, inter_point.y + line_search_rect.y), 3, Scalar(100, 100, 0));
			}
			tmp_error = compare_feature(feature_hog[0], tmp_feature);
			if (tmp_error < min_error1) {
				door_head_points[0] = inter_point;
				min_error1 = tmp_error;
				best_feature1 = tmp_feature;
			}
			tmp_error = compare_feature(feature_hog[1], tmp_feature);
			if (tmp_error < min_error2) {
				door_head_points[1] = inter_point;
				min_error2 = tmp_error;
				best_feature2 = tmp_feature;
			}
		}
	}
	// update feature.
	feature_hog[0] = best_feature1;
	feature_hog[1] = best_feature2;
	// update door head point.
	door_head_points[0].x += line_search_rect.x;
	door_head_points[0].y += line_search_rect.y;
	door_head_points[1].x += line_search_rect.x;
	door_head_points[1].y += line_search_rect.y;
	// calculate next feature_search_rect.
	return get_hog_rect(ori_img.size(), door_head_points[0], door_head_points[1], feature_search_rect, TAU);
}


bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p){
	double rho1 = line1[0], theta1 = line1[1];
	double rho2 = line2[0], theta2 = line2[1];
	double denom = (sin(theta1)*cos(theta2) - cos(theta1)*sin(theta2));
	inter_p.x = (int)((rho2*sin(theta1) - rho1*sin(theta2)) / denom);
	inter_p.y = (int)((rho1*cos(theta2) - rho2*cos(theta1)) / denom);
	return true;
}