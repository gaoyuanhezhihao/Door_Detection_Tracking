#include "Camshift.h"
#include <vector>

using namespace cv;
using namespace std;
bool init_cam_shift(object_target & target, Mat & image, Rect &trackWindow, int vmin, int vmax, int smin) {
	Mat hsv, mask, hue;
	//Mat histimg = Mat::zeros(image.size(), CV_8UC3);
	int hsize = 16;
	float hranges[] = { 0, 180 };//hranges在后面的计算直方图函数中要用到
	const float* phranges = hranges;
	target.trackWindow = trackWindow;
	cvtColor(image, hsv, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的
	inRange(hsv, Scalar(0, smin, MIN(vmin, vmax)),
		Scalar(180, 256, MAX(vmin, vmax)), mask);
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
	mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组
	Mat roi(hue, trackWindow), maskroi(mask, trackWindow);//mask保存的hsv的最小值
	calcHist(&roi, 1, 0, maskroi, target.hist, 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
	normalize(target.hist, target.hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255
	return true;
}

bool update_target_window(object_target & target, Mat & image, int vmin, int vmax, int smin) {
	Mat hsv, hue, mask, backproj;
	float hranges[] = { 0, 180 };//hranges在后面的计算直方图函数中要用到
	const float* phranges = hranges;
	int _vmin = vmin, _vmax = vmax;
	cvtColor(image, hsv, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的
	//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
	//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
	//mask对应的那个点的值全为1(0xff)，否则为0(0x00).
	inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
		Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
	mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组

	calcBackProject(&hue, 1, 0, target.hist, backproj, &phranges);//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
	backproj &= mask;

	//opencv2.0以后的版本函数命名前没有cv两字了，并且如果函数名是由2个意思的单词片段组成的话，且前面那个片段不够成单词，则第一个字母要
	//大写，比如Camshift，如果第一个字母是个单词，则小写，比如meanShift，但是第二个字母一定要大写
	target.trackBox = CamShift(backproj, target.trackWindow,               //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
		TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));//CV_TERMCRIT_EPS是通过forest_accuracy,CV_TERMCRIT_ITER
	//trackWindow = trackBox.boundingRect();
	if (target.trackWindow.area() <= 1)                                                  //是通过max_num_of_trees_in_the_forest  
	{
		int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		target.trackWindow = Rect(target.trackWindow.x - r, target.trackWindow.y - r,
			target.trackWindow.x + r, target.trackWindow.y + r) &
			Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
	}
	rectangle(image, target.trackBox.boundingRect(), Scalar(0, 0, 255), 3);//跟踪的时候以椭圆为代表目标
	imshow("CamShift", image);
	return true;
}