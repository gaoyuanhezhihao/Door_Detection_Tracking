#include "Camshift.h"
#include <vector>

using namespace cv;
using namespace std;
bool init_cam_shift(object_target & target, Mat & image, Rect &trackWindow, int vmin, int vmax, int smin) {
	Mat hsv, mask, hue;
	//Mat histimg = Mat::zeros(image.size(), CV_8UC3);
	int hsize = 16;
	float hranges[] = { 0, 180 };//hranges�ں���ļ���ֱ��ͼ������Ҫ�õ�
	const float* phranges = hranges;
	target.trackWindow = trackWindow;
	cvtColor(image, hsv, CV_BGR2HSV);//��rgb����ͷ֡ת����hsv�ռ��
	inRange(hsv, Scalar(0, smin, MIN(vmin, vmax)),
		Scalar(180, 256, MAX(vmin, vmax)), mask);
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());//hue��ʼ��Ϊ��hsv��С���һ���ľ���ɫ���Ķ������ýǶȱ�ʾ�ģ�������֮�����120�ȣ���ɫ���180��
	mixChannels(&hsv, 1, &hue, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue�У�0��������
	Mat roi(hue, trackWindow), maskroi(mask, trackWindow);//mask�����hsv����Сֵ
	calcHist(&roi, 1, 0, maskroi, target.hist, 1, &hsize, &phranges);//��roi��0ͨ������ֱ��ͼ��ͨ��mask����hist�У�hsizeΪÿһάֱ��ͼ�Ĵ�С
	normalize(target.hist, target.hist, 0, 255, CV_MINMAX);//��hist����������鷶Χ��һ��������һ����0~255
	return true;
}

bool update_target_window(object_target & target, Mat & image, int vmin, int vmax, int smin) {
	Mat hsv, hue, mask, backproj;
	float hranges[] = { 0, 180 };//hranges�ں���ļ���ֱ��ͼ������Ҫ�õ�
	const float* phranges = hranges;
	int _vmin = vmin, _vmax = vmax;
	cvtColor(image, hsv, CV_BGR2HSV);//��rgb����ͷ֡ת����hsv�ռ��
	//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask����0ͨ������Сֵ��Ҳ����h����
	//����������hsv��3��ͨ�����Ƚ�h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)�����3��ͨ�����ڶ�Ӧ�ķ�Χ�ڣ���
	//mask��Ӧ���Ǹ����ֵȫΪ1(0xff)������Ϊ0(0x00).
	inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
		Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());//hue��ʼ��Ϊ��hsv��С���һ���ľ���ɫ���Ķ������ýǶȱ�ʾ�ģ�������֮�����120�ȣ���ɫ���180��
	mixChannels(&hsv, 1, &hue, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue�У�0��������

	calcBackProject(&hue, 1, 0, target.hist, backproj, &phranges);//����ֱ��ͼ�ķ���ͶӰ������hueͼ��0ͨ��ֱ��ͼhist�ķ���ͶӰ��������backproj��
	backproj &= mask;

	//opencv2.0�Ժ�İ汾��������ǰû��cv�����ˣ������������������2����˼�ĵ���Ƭ����ɵĻ�����ǰ���Ǹ�Ƭ�β����ɵ��ʣ����һ����ĸҪ
	//��д������Camshift�������һ����ĸ�Ǹ����ʣ���Сд������meanShift�����ǵڶ�����ĸһ��Ҫ��д
	target.trackBox = CamShift(backproj, target.trackWindow,               //trackWindowΪ���ѡ�������TermCriteriaΪȷ��������ֹ��׼��
		TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));//CV_TERMCRIT_EPS��ͨ��forest_accuracy,CV_TERMCRIT_ITER
	//trackWindow = trackBox.boundingRect();
	if (target.trackWindow.area() <= 1)                                                  //��ͨ��max_num_of_trees_in_the_forest  
	{
		int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		target.trackWindow = Rect(target.trackWindow.x - r, target.trackWindow.y - r,
			target.trackWindow.x + r, target.trackWindow.y + r) &
			Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
	}
	rectangle(image, target.trackBox.boundingRect(), Scalar(0, 0, 255), 3);//���ٵ�ʱ������ԲΪ����Ŀ��
	imshow("CamShift", image);
	return true;
}