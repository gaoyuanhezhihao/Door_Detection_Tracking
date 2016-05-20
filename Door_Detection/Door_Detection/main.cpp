#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "main_func.h"

#define PI 3.1415926

using namespace cv;
using namespace std;

int main()
{
	Mat image;

	Mat dst, cdst;
	double start = double(getTickCount());

	VideoCapture cap("C270_2.mp4");
	if (!cap.isOpened())
	{
		cout << "Capture could not be opened successfully" << endl;
		return -1;
	}
	namedWindow("Video");
	char order = 0;
	cap >> image;
	VideoWriter vd_writer("debug_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5.0, Size(image.cols, image.rows));
	VideoWriter camshift_vd_writer("debug_video_camshift.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5.0, Size(image.cols, image.rows));
	while (char(waitKey(1)) != 'q' && cap.isOpened())
	{
		start = double(getTickCount());
		cap >> image;
		if (image.empty())
		{
			cout << "Video over" << endl;
			break;
		}
		else
		{
			vanish_point_detection(image, cdst);
			//Canny(image, dst, 30, 70, 3);
			//cvtColor(dst, cdst, CV_GRAY2BGR);

			//vector<Vec2f> lines;
			//// detect lines
			//HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);

			//// draw lines
			//for (size_t i = 0; i < lines.size(); i++)
			//{
			//	float rho = lines[i][0], theta = lines[i][1];
			//	Point pt1, pt2;
			//	double a = cos(theta), b = sin(theta);
			//	double x0 = a*rho, y0 = b*rho;
			//	pt1.x = cvRound(x0 + 1000 * (-b));
			//	pt1.y = cvRound(y0 + 1000 * (a));
			//	pt2.x = cvRound(x0 - 1000 * (-b));
			//	pt2.y = cvRound(y0 - 1000 * (a));
			//	if (10 * PI / 180 < theta && theta < 60 * PI / 180)
			//	{
			//		line(cdst, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
			//	}
			//	else if (110 * PI / 180 < theta && theta < 170 * PI / 180)
			//	{
			//		line(cdst, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
			//	}
			//	else
			//	{
			//		line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
			//	}
			//}
			double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
			cout << "It took " << duration_ms << " ms." << endl;
			imshow("detected lines", cdst);
			vd_writer << cdst;
			camshift_vd_writer << image;

		}

	}
	cout << "Press any key to exit:" << endl;
	cin >> order;



	waitKey();
	return 0;
}