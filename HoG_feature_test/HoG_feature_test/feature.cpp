#include "feature.h"

bool init_track(vector<Point> & door_head_points, Mat &img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog, double alpha, double beta) {
	// calculate the ROI.
	//double L = cv::norm(door_head_points[0]-door_head_points[1]);
	double L = abs(door_head_points[0].x - door_head_points[1].x);
	Point up_left;
	Point down_right;
	up_left.x = (int)( min(door_head_points[0].x, door_head_points[1].x) - alpha * L);
	up_left.y = (int)( min(door_head_points[0].y, door_head_points[1].y) - alpha * L);
	up_left.x = (int)(up_left.x >= 0 ? up_left.x : 0);
	up_left.y = (int)(up_left.y >= 0 ? up_left.y : 0);
	down_right.x = (int)(max(door_head_points[0].x, door_head_points[1].x) + alpha* L);
	down_right.y = (int)(max(door_head_points[0].y, door_head_points[1].y) + (alpha + beta) * L);
	down_right.x = (int)(down_right.x < img.cols ? down_right.x : img.cols);
	down_right.y = (int)(down_right.y < img.rows ? down_right.y : img.rows);
	search_range.x = up_left.x;
	search_range.y = up_left.y;
	search_range.width = down_right.x - search_range.x;
	search_range.height = down_right.y - search_range.y;
	// get feature.
	get_hog(img, door_head_points[0], feature_hog[0]);
	get_hog(img, door_head_points[1], feature_hog[1]);
	return true;
}

bool init_hog(vector<Point> & door_head_points, Mat &img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> &feature_hog, double alpha, double beta) {
	// calculate the ROI.
	//double L = cv::norm(door_head_points[0]-door_head_points[1]);
	double L = abs(door_head_points[0].x - door_head_points[1].x);
	Point up_left;
	Point down_right;
	up_left.x = (int)(min(door_head_points[0].x, door_head_points[1].x) - alpha * L);
	up_left.y = (int)(min(door_head_points[0].y, door_head_points[1].y) - alpha * L);
	up_left.x = (int)(up_left.x >= 0 ? up_left.x : 0);
	up_left.y = (int)(up_left.y >= 0 ? up_left.y : 0);
	down_right.x = (int)(max(door_head_points[0].x, door_head_points[1].x) + alpha* L);
	down_right.y = (int)(max(door_head_points[0].y, door_head_points[1].y) + (alpha + beta) * L);
	down_right.x = (int)(down_right.x < img.cols ? down_right.x : img.cols);
	down_right.y = (int)(down_right.y < img.rows ? down_right.y : img.rows);
	search_range.x = up_left.x;
	search_range.y = up_left.y;
	search_range.width = down_right.x - search_range.x;
	search_range.height = down_right.y - search_range.y;
	// feature;
	Mat search_im_gray;
	Mat ckech_im_gray_64bit;
	cvtColor(img(search_range), search_im_gray, COLOR_BGR2GRAY);
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

	get_hog_from_map(feature_hog[0], grad_angle_id, grad_mag, door_head_points[0].y- search_range.y, door_head_points[0].x - search_range.x);
	get_hog_from_map(feature_hog[1], grad_angle_id, grad_mag, door_head_points[1].y- search_range.y, door_head_points[1].x - search_range.x);
	return true;
}

bool get_hog(Mat & img, Point & patch_center, array<double, HoG_GRAD_BIN_SIZE * 9> &descr) {
	Point patch_lt;
	double dx_limit = min(img.cols - patch_center.x, patch_center.x);
	int width = (int)min(dx_limit*2, double(STD_CELL_WIDTH*STD_CELL_PER_BLOCK_ROW+2));// plus 2 columns for gradient calculation.
	int cell_w = width / STD_CELL_PER_BLOCK_ROW;
	int half_cell_w = cell_w / 2;
	cell_w = half_cell_w * 2;
	double dy_limit = min(img.rows - patch_center.y, patch_center.y);
	int height = (int)min(dy_limit*2, double(STD_CELL_HEIGHT*STD_CELL_PER_BLOCK_COLOMN+2));// plus 2 rows for gradient calculation.
	int cell_h = height / STD_CELL_PER_BLOCK_COLOMN;
	int half_cell_h = cell_h / 2;
	cell_h = half_cell_h * 2;
	cout << "cell_w: " << cell_w << ", " << "cell_h: " << cell_h << endl;
	patch_lt.x = patch_center.x - cell_w * 3 / 2;
	patch_lt.y = patch_center.y - cell_h * 3 / 2;

	// create gradient image.
	Rect search_rect(patch_lt.x - 1, patch_lt.y - 1, width, height);
	Mat search_im_gray;
	Mat search_im;
	cvtColor(img(search_rect), search_im_gray, COLOR_BGR2GRAY);
	search_im_gray.convertTo(search_im, CV_64FC1);

	Mat Ix_kernel = (Mat_<double>(1, 3) << -1, 0, 1);
	Mat Iy_kernel = (Mat_<double>(3, 1) << -1, 0, 1);

	blur(search_im, search_im, Size(3, 3));
	Mat im_Ix, im_Iy;
	filter2D(search_im, im_Ix, -1, Ix_kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(search_im, im_Iy, -1, Iy_kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	int r = 0;
	int c = 0;
	int block_id = 0;
	double angle = 0;
	double mag = 0;
	int angle_bin_id = 0;
	double angle_bin_size = 2* PI / HoG_GRAD_BIN_SIZE;
	for (r = 0; r < cell_h*STD_CELL_PER_BLOCK_COLOMN; ++r) {
		for (c = 0; c < cell_w * STD_CELL_PER_BLOCK_ROW; ++c) {
			block_id = r / cell_h * STD_CELL_PER_BLOCK_ROW + c / cell_w;
			angle_bin_id = get_gradient_id(HoG_GRAD_BIN_SIZE, angle_bin_size, im_Ix.at<double>(r + 1, c + 1), im_Iy.at<double>(r + 1, c + 1));
			mag = sqrt(pow(im_Iy.at<double>(r + 1, c + 1), 2) + pow(im_Ix.at<double>(r + 1, c + 1), 2));
			descr[block_id * HoG_GRAD_BIN_SIZE + angle_bin_id] += mag;
		}
	}
	return true;
}

int get_gradient_id(int bin_size, double bin_range, double dx, double dy) {
	short sign_dx = dx>0.0 ? 2: 0;
	short sign_dy = dy > 0.0 ? 1 : 0;
	int bin_id = 0;
	double angle = 0;
	dx += 0.00001;
	switch (sign_dx & sign_dy) {
	case 0:// both are negative.
		angle = atan(dy / dx) + PI;
		break;
	case 1:// dy>0, dx <0.
		angle = atan(dy / dx) + PI;
		break;
	case 2:// dx>0, dy <0
		angle = atan(dy / dx) + 2 * PI;
		break;
	case 3:// dx>0, dy>0.
		angle = atan(dy / dx);
		break;
	default:
		break;
	}
	bin_id = int(angle / bin_range);
	assert(0 <= bin_id && bin_id < bin_size);
	//cout << bin_id<< ", "<< angle<< ", "<< bin_range<< ", "<< dx<< ", "<< dy << endl;
	return bin_id;
}


bool find_match(Mat & search_img, Rect & search_range, array<array<double, HoG_GRAD_BIN_SIZE * 9>, 2> & feature_hog, Point &match_point1, Point &match_point2, vector<Point> & door_head_points) {
	Mat search_im_gray;
	Mat ckech_im_gray_64bit;
	cvtColor(search_img(search_range), search_im_gray, COLOR_BGR2GRAY);
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

	int half_patch_width = STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW  /2;
	int half_patch_height = STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_COLOMN /2;
	int r, c;
	array<double, HoG_GRAD_BIN_SIZE * 9> tmp_feature = {0};
	double min_error1 = std::numeric_limits<double>::infinity();
	double min_error2 = std::numeric_limits<double>::infinity();
	double tmp_error = 0;
	for (r = half_patch_height; r < (search_range.height - half_patch_height); ++r) {
		for (c = half_patch_width; c < (search_range.width - half_patch_width); ++c) {
			if (r + search_range.y == door_head_points[0].y && c + search_range.x == door_head_points[0].x) {
				cout << "debug" << endl;
			}
			circle(search_im_gray, Point(c, r), 3,  Scalar(255, 0, 0));
			get_hog_from_map(tmp_feature, grad_angle_id, grad_mag, r, c);
			tmp_error = compare_feature(feature_hog[0], tmp_feature);
			if (tmp_error < min_error1) {
				match_point1.x = c;
				match_point1.y = r;
				min_error1 = tmp_error;
			}

		}
	}
	match_point1.x += search_range.x;
	match_point1.y += search_range.y;
	return true;
}

double compare_feature(array<double, HoG_GRAD_BIN_SIZE * 9> & f1, array<double, HoG_GRAD_BIN_SIZE * 9> & f2) {
	double error = 0.0;
	int i = 0;
	for (i = 0; i < HoG_GRAD_BIN_SIZE * 9; ++i) {
		error += abs(f1[i] - f2[i]);
	}
	return error;
}


bool get_hog_from_map(array<double, HoG_GRAD_BIN_SIZE * 9> & feature, Mat & grad_angle_id, Mat & grad_mag, int center_r, int center_c) {
	int half_patch_width = STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW / 2;
	int half_patch_height = STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_COLOMN / 2;
	Point ul(center_c - half_patch_width, center_r - half_patch_height);
	int r = 0, c = 0;
	int block_id = 0;
	int angle_bin_id = 0;
	int i = 0;
	for (i = 0; i < HoG_GRAD_BIN_SIZE * 9; ++i) {
		feature[i] =0;
	}
	for (r = 0; r < STD_CELL_WIDTH * STD_CELL_PER_BLOCK_ROW; ++r) {
		for (c = 0; c < STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_COLOMN; ++c) {
			block_id = r / STD_CELL_HEIGHT * STD_CELL_PER_BLOCK_ROW + c / STD_CELL_WIDTH;
			angle_bin_id = grad_angle_id.at<uchar>(ul.y + r, ul.x + c);
			feature[block_id * HoG_GRAD_BIN_SIZE + angle_bin_id] += grad_mag.at<double>(ul.y + r, ul.x + c);
		}
	}
	return true;
}

bool get_grad_img(Mat & im_Ix, Mat & im_Iy, Mat & grad_angle_id, Mat & grad_mag) {
	int r = 0, c = 0;
	double bin_angle_gap = 2 * PI / HoG_GRAD_BIN_SIZE;
	for (r = 0; r < im_Ix.rows; ++r) {
		for (c = 0; c < im_Ix.cols; ++c) {
			grad_angle_id.at<uchar>(r,c) = get_gradient_id(HoG_GRAD_BIN_SIZE, bin_angle_gap, im_Ix.at<double>(r, c), im_Iy.at<double>(r, c));
			grad_mag.at<double>(r, c) = sqrt(pow(im_Iy.at<double>(r, c), 2) + pow(im_Ix.at<double>(r, c), 2));
		}
	}
	return true;
}