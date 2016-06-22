#include "track_point.h"
decision decide(track_state state, int times_tried,
		Point & p1, Point&p2, Point&old_p1, Point&old_p2, Size img_size) {
	decision judge;
	switch (state)
	{
	case track_state::bad_line:
		judge = decision::retry;
		break;
	case track_state::bad_match:
		judge = decision::retry;
		break;
	case track_state::near_lr:
		if (p1.x + p2.x < img_size.width *2)
		{
			judge = decision::turn_lef;
		} 
		else {
			judge = decision::turn_right;
		}
		break;
	case track_state::near_ud:
		judge = decision::switch_to_camShift;
		break;
	case track_state::ok:
		/*check if two points' relative position is right*/
		if ((p2.x - p1.x) * (old_p2.x - old_p1.x) > 0) {
			judge = decision::keep_going;
		}
		else {
			judge = decision::retry;
		}

		break;
	default:
		break;
	}
	if (judge == decision::retry && times_tried >= RETRY_MAX) {
		judge = decision::stop_car;// track failed.
	}
	return judge;
}
track_state track_door(Point &old_p1, Point & old_p2,
	vector<Point> &door_head_points, Mat &ori_img, Rect & line_rect, 
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
	key_point_state track_feature_state;
	search_range_state line_search_range_state, feature_search_range_state;
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
	if (vertical_lines.size() == 0) {
		return track_state::bad_line;
	}
	draw_line(cdst, vertical_lines, Scalar(0, 0, 100));
	select_lines(short_lines, head_lines, head_line_neib_theta);

	if (head_lines.size() == 0) {
		draw_line(cdst, short_lines, Scalar(0, 100, 100));
		return track_state::bad_line;
	}
	else {
		draw_line(cdst, head_lines, Scalar(0, 255, 0));
	}
	//rectangle(ori_img, line_rect, Scalar(255, 0, 0));
	imshow("line image", cdst);
	imshow("edge image", dst);
	//cout << "detected long line count:" << long_lines.size() << '\n' <<
	//	"vertical line count: " << vertical_lines.size() << '\n' <<
	//	"detected short line count: " << short_lines.size() << '\n' <<
	//	"head line count: " << head_lines.size() << '\n' <<
	//	"cdst size:" << cdst.size() << "search rect" << line_rect.width <<
	//	',' << line_rect.height
	//	<< endl;
	/* feature track. */
	double min_error1, min_error2;
	array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> best_features;
	track_feature_state = select_best_point(vertical_lines, head_lines,
		feature_rect, line_rect, door_head_points, ori_img, feature_hog,
		best_features, min_error1, min_error2);
	cout << "min errors" << min_error1 << ',' << min_error2 << endl;
	if (track_feature_state != key_point_state::good) {
		door_head_points[0] = old_p1;
		door_head_points[1] = old_p2;
		return track_state::bad_match;
	}
	/*update feature.*/
	feature_hog = best_features;
	/*calculate next line_search_rect.*/
	line_search_range_state = get_hough_rect(door_head_points[0]- old_p1,
		door_head_points[1] - old_p2, ori_img.size(), door_head_points[0],
		door_head_points[1], line_rect, ALPHA, BETA);
	/*calculate next feature_search_rect.*/
	feature_search_range_state = get_hog_rect(door_head_points[0] - old_p1,
		door_head_points[1] - old_p2, ori_img.size(), door_head_points[0],
		door_head_points[1], feature_rect, TAU);
	/*check search range states.*/
	if (line_search_range_state == search_range_state::near_lr_edge ||
		feature_search_range_state == search_range_state::near_lr_edge) {
		return track_state::near_lr;
	}
	else if (line_search_range_state == search_range_state::near_lr_ud_edge) {
		return track_state::near_ud;
	}
	return track_state::ok;
}

key_point_state select_best_point(vector<Vec2f> &v_lines, vector<Vec2f> & h_lines, Rect & feature_search_rect, Rect & line_search_rect,
										vector<Point> & door_head_points, Mat &ori_img, 
										const array<array<double, HoG_GRAD_BIN_SIZE * 9>,2> & feature_hog, 
										array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & best_features,
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
	min_error1 = std::numeric_limits<double>::infinity();
	min_error2 = std::numeric_limits<double>::infinity();
	double tmp_error = 0;
	if (feature_search_rect.height <= STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW || feature_search_rect.width <= STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW) {
		return key_point_state::range_too_small;
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
				best_features[0] = tmp_feature;
			}
			tmp_error = compare_feature(feature_hog[1], tmp_feature);
			if (tmp_error < min_error2) {
				door_head_points[1] = inter_point;
				min_error2 = tmp_error;
				best_features[1] = tmp_feature;
			}
		}
	}
	// update door head point.
	door_head_points[0].x += line_search_rect.x;
	door_head_points[0].y += line_search_rect.y;
	door_head_points[1].x += line_search_rect.x;
	door_head_points[1].y += line_search_rect.y;
	short good_match1 = (int)(min_error1 < FEATURE_MATCH_THRES);
	short good_match2 = (int)(min_error2 < FEATURE_MATCH_THRES);
	switch (good_match1 << 1 | good_match2) {
	case 0:
		return key_point_state::bad_two;
		break;
	case 1:
		return key_point_state::bad_p2;
		break;
	case 2:
		return key_point_state::bad_p1;
		break;
	case 3:
		return key_point_state::good;
	default:
		throw range_error("ERROR, switch_condition out of range");
	}
}


bool get_public_point(Vec2f & line1, Vec2f & line2, Point &inter_p){
	double rho1 = line1[0], theta1 = line1[1];
	double rho2 = line2[0], theta2 = line2[1];
	double denom = (sin(theta1)*cos(theta2) - cos(theta1)*sin(theta2));
	inter_p.x = (int)((rho2*sin(theta1) - rho1*sin(theta2)) / denom);
	inter_p.y = (int)((rho1*cos(theta2) - rho2*cos(theta1)) / denom);
	return true;
}

/* get the search rectangle area for hough line.
*-----------------------------------------------
@img_size: the size of the original image(vision size of the camera).
@p1: first point of the door head line.
@p2: second point of the door head line.
@hough_rect: result saved there.
---------------------------------------------------------
return: state of the search range.(near left/right/up/down edge, or save.)
*/
search_range_state get_hough_rect(const Point & p1_move, const Point & p2_move, 
								  Size img_size, const Point & p1,
								  const Point & p2, Rect &hough_rect,
								  double alpha, double beta) {
	/*predict next point position*/
	Point prd_p1 = p1 + p1_move;
	Point prd_p2 = p2 + p2_move;
	/*calculate search rectangle*/
	double L = abs(prd_p1.x - prd_p2.x);
	Point up_left;
	Point down_right;
	short near_ud_edge = 0;
	short near_lr_edge = 0;
	up_left.x = (int)(min(prd_p1.x, prd_p2.x) - alpha * L);
	up_left.y = (int)(min(prd_p1.y, prd_p2.y) - alpha * L);
	up_left.x = (int)(up_left.x >= 0 ? up_left.x : 0);
	up_left.y = (int)(up_left.y >= 0 ? up_left.y : 0);
	down_right.x = (int)(max(prd_p1.x, prd_p2.x) + alpha* L);
	down_right.y = (int)(max(prd_p1.y, prd_p2.y) + (alpha + beta) * L);
	down_right.x = (int)(down_right.x < img_size.width - 1 ? down_right.x : img_size.width - 1);
	down_right.y = (int)(down_right.y < img_size.height - 1 ? down_right.y : img_size.height - 1);

	if (up_left.x == 0 || down_right.x == img_size.width - 1){
		near_lr_edge = 1;
	}
	if (up_left.y == 0 || down_right.y == img_size.height - 1) {
		near_ud_edge = 1;
	}
	hough_rect.x = up_left.x;
	hough_rect.y = up_left.y;
	hough_rect.width = down_right.x - hough_rect.x;
	hough_rect.height = down_right.y - hough_rect.y;
	switch (near_lr_edge << 1 | near_ud_edge)
	{
	case 0:
		return search_range_state::safe;
		break;
	case 1:
		return search_range_state::near_ud_edge;
		break;
	case 2:
		return search_range_state::near_lr_edge;
		break;
	case 3:
		return search_range_state::near_lr_ud_edge;
		break;
	default:
		break;
	}
	throw range_error("ERROR, switch_condition out of range");
}

search_range_state get_hog_rect(const Point & p1_move, const Point & p2_move,
								Size img_size, const Point &p1,
								const Point &p2, Rect & hog_rect,
								double tau) {
	/*predict next point position*/
	Point prd_p1 = p1 + p1_move;
	Point prd_p2 = p2 + p2_move;
	/*calculate search rectangle*/
	double L = abs(prd_p1.x - prd_p2.x);
	int step = (int)(tau * L);
	int ul_x = min(prd_p1.x, prd_p2.x) - step;
	int ul_y = min(prd_p1.y, prd_p2.y) - step;
	int dr_x = max(prd_p1.x, prd_p2.x) + step;
	int dr_y = max(prd_p1.y, prd_p2.y) + step;
	short near_ud_edge = 0;
	short near_lr_edge = 0;
	if (ul_x < 0 || dr_x >= img_size.width) {
		near_lr_edge = 1;
	}
	if (ul_y < 0 || dr_y >= img_size.height) {
		near_ud_edge = 1;
	}

	hog_rect.x = ul_x >= 0 ? ul_x : 0;
	hog_rect.y = ul_y >= 0 ? ul_y : 0;
	dr_x = dr_x < img_size.width ? dr_x : img_size.width;
	dr_y = dr_y < img_size.height ? dr_y : img_size.height;

	hog_rect.width = dr_x - hog_rect.x;
	hog_rect.height = dr_y - hog_rect.y;
	switch (near_lr_edge << 1 | near_ud_edge)
	{
	case 0:
		return search_range_state::safe;
		break;
	case 1:
		return search_range_state::near_ud_edge;
		break;
	case 2:
		return search_range_state::near_lr_edge;
		break;
	case 3:
		return search_range_state::near_lr_ud_edge;
		break;
	default:
		break;
	}
	throw range_error("ERROR, switch_condition out of range");
}

bool init_tracking(vector<Point> & door_head_points, Point & old_p1,
					Point & old_p2, Mat &img, Rect & line_rect,
					Rect & feature_rect,
					array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog,
					double alpha, double beta) {
	//// calculate the ROI.
	////double L = cv::norm(door_head_points[0]-door_head_points[1]);
	//double L = abs(door_head_points[0].x - door_head_points[1].x);
	//Point up_left;
	//Point down_right;
	//up_left.x = (int)(min(door_head_points[0].x, door_head_points[1].x) - alpha * L);
	//up_left.y = (int)(min(door_head_points[0].y, door_head_points[1].y) - alpha * L);
	//up_left.x = (int)(up_left.x >= 0 ? up_left.x : 0);
	//up_left.y = (int)(up_left.y >= 0 ? up_left.y : 0);
	//down_right.x = (int)(max(door_head_points[0].x, door_head_points[1].x) + alpha* L);
	//down_right.y = (int)(max(door_head_points[0].y, door_head_points[1].y) + (alpha + beta) * L);
	//down_right.x = (int)(down_right.x < img.cols ? down_right.x : img.cols);
	//down_right.y = (int)(down_right.y < img.rows ? down_right.y : img.rows);
	//search_range.x = up_left.x;
	//search_range.y = up_left.y;
	//search_range.width = down_right.x - search_range.x;
	//search_range.height = down_right.y - search_range.y;

	/*record old point*/
	old_p1 = door_head_points[0];
	old_p2 = door_head_points[1];
	/*calculate ROI.*/
	get_hough_rect(Point(0,0), Point(0,0), img.size(), door_head_points[0], door_head_points[1], line_rect, ALPHA, BETA);
	get_hog_rect(Point(0,0), Point(0,0), img.size(), door_head_points[0], door_head_points[1], feature_rect, TAU);
	// feature;
	Mat search_im_gray;
	Mat ckech_im_gray_64bit;
	cvtColor(img(feature_rect), search_im_gray, COLOR_BGR2GRAY);
	search_im_gray.convertTo(ckech_im_gray_64bit, CV_64FC1);


	Mat Ix_kernel = (Mat_<double>(1, 3) << -1, 0, 1);
	Mat Iy_kernel = (Mat_<double>(3, 1) << -1, 0, 1);
	blur(ckech_im_gray_64bit, ckech_im_gray_64bit, Size(3, 3));
	Mat im_Ix, im_Iy;
	filter2D(ckech_im_gray_64bit, im_Ix, -1, Ix_kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(ckech_im_gray_64bit, im_Iy, -1, Iy_kernel, Point(-1, -1), 0, BORDER_DEFAULT);

	Mat grad_angle_id = Mat::zeros(im_Ix.rows, im_Ix.cols, CV_8UC1);
	Mat grad_mag = Mat::zeros(im_Ix.rows, im_Ix.cols, CV_64FC1);

	get_grad_img(im_Ix, im_Iy, grad_angle_id, grad_mag);

	int half_patch_width = STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW / 2;
	int half_patch_height = STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_COLOMN / 2;

	if (false == get_hog_from_map(feature_hog[0], grad_angle_id, grad_mag, feature_rect, door_head_points[0].y, door_head_points[0].x)) {
		return false;
	}
	if (false == get_hog_from_map(feature_hog[1], grad_angle_id, grad_mag, feature_rect, door_head_points[1].y, door_head_points[1].x)) {
		return false;
	}
	return true;
}