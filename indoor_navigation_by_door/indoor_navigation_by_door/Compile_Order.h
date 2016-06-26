#ifndef _COMPILE_ORDER_H
#define _COMPILE_ORDER_H

	#include "FileLogger.hpp"
	#define MY_DEBUG_SHOW_IM
	//#define MY_DEBUG
	//#define MY_DEBUG_VIDEO_OUT
	//#define MY_DEBUG_SOCKET
	#define MY_DEBUG_TIME
	#ifdef MY_DEBUG
		//#define DEBUG_COUT(a) cout << a << endl;
		#define DEBUG_COUT(a) write_global_logger(a, 'i');
	#else
		#define DEBUG_COUT(a)
	#endif
#endif