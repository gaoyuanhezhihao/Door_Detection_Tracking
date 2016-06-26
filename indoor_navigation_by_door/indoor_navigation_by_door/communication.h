#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <chrono>
#include <string>
#include "Compile_Order.h"
#include <iostream>
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")


#define TRACK_END_FORWARD_TIME 2 // seconds

using namespace std;

bool go_forward_dist(SOCKET & sclient, int dist);
bool make_a_turn(char side, int degree, SOCKET & sclient);
bool send_forward_order(SOCKET & sclient);
bool send_stop_order(SOCKET & sclient);
void adjust_pwm(char bias_side, int current_x, int center_x, SOCKET & sclient);
void Init_Car(SOCKET & sclient);
int Init_Socket(SOCKET & sclient);
#endif COMMUNICATION_H