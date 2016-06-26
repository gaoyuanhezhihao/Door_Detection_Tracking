#ifndef VERIFICATION_H	
#define VERIFICATION_H
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <winsock2.h>

#include "communication.h"
#include "main_func.h"

using namespace std;
using namespace cv;
bool verify_door(const Mat &img, char side_of_door, system_mode & current_mode,
	SOCKET & sclient);
#endif //VERIFICATION_H