#include "communication.h"


#define ORI_L_PWM  100
#define ORI_R_PWM  100
unsigned int l_pwm = ORI_L_PWM;
unsigned int r_pwm = ORI_R_PWM;
unsigned int f_l_pwm = ORI_L_PWM;
unsigned int f_r_pwm = ORI_R_PWM;

/*go forward a distance.
*---------------------------------------
* @sclient: socket connection to car.
* @dist: distance needed to go forward.
*---------------------------------------
* return: succeeded or failed.
*/
bool go_forward_dist(SOCKET & sclient, int dist) {
	send_forward_order(sclient);
	chrono::system_clock::time_point t_start = chrono::system_clock::now();
	long long time_passed= 0;
	do{
		time_passed = chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - t_start).count();
	} while (time_passed < TRACK_END_FORWARD_TIME);
	send_stop_order(sclient);
	return true;
}


void send_pwm_order(SOCKET & sclient)
{
	int result = 0;
	string cmd_2_car;
	char recv_buf[256] = { 0 };
	// left pwm.
	cmd_2_car = "p\n";
	cmd_2_car += to_string(l_pwm);
	cmd_2_car += "\n";
	while (strcmp(recv_buf, "ok\n"))
	{
		result = send(sclient, cmd_2_car.c_str(), cmd_2_car.size(), 0);
		result = recv(sclient, recv_buf, 256, 0);
		if (result > 0)
		{
			recv_buf[result] = 0;
		}
	}
	DEBUG_COUT(cmd_2_car);
	// right pwm.
	cmd_2_car = "q\n";
	cmd_2_car += to_string(r_pwm);
	cmd_2_car += "\n";
	recv_buf[0] = 0;
	while (strcmp(recv_buf, "ok\n"))
	{
		result = send(sclient, cmd_2_car.c_str(), cmd_2_car.size(), 0);
		result = recv(sclient, recv_buf, 256, 0);
		if (result > 0)
		{
			recv_buf[result] = 0;
		}
	}
	DEBUG_COUT(cmd_2_car);
}
void Init_Car(SOCKET & sclient)
{
	l_pwm = ORI_L_PWM;
	r_pwm = ORI_R_PWM;
	send_pwm_order(sclient);
}
void adjust_pwm(char bias_side, int current_x, int center_x, SOCKET & sclient)
{
	double percent = 0;
	if (bias_side == 'l')
	{
		assert(current_x > center_x);
		percent = (double)(current_x - center_x) / (double)center_x;
		r_pwm = (1 - percent) * ORI_R_PWM;
		l_pwm = (1 + percent) * ORI_L_PWM;
		send_pwm_order(sclient);
	}
	else if (bias_side == 'r')
	{
		assert(current_x < center_x);
		percent = (double)(center_x - current_x) / (double)center_x;
		r_pwm = (1 + percent) * ORI_R_PWM;
		l_pwm = (1 - percent) * ORI_L_PWM;
		send_pwm_order(sclient);
	}
	else
	{
		throw(std::invalid_argument("adjust_pwm()::invalid bias_side"));
	}
}
bool make_a_turn(char side, int degree, SOCKET & sclient)
{
	cv::Mat img_trush;
	string cmd_2_car;
	string ack_msg;
	ack_msg += side;
	ack_msg += "_ok\n";
	int sock_data_count = 0;
	char sock_recv_buf[100] = {0};
	u_long iMode = 1;
	cmd_2_car += side;
	cmd_2_car += '\n';
	cmd_2_car += to_string(degree);
	cmd_2_car += '\n';

	do
	{
		sock_recv_buf[0] = 0;
		send(sclient, cmd_2_car.c_str(), cmd_2_car.size(), 0);
#ifdef MY_DEBUG_SOCKET
		cout << "waiting the ack completed..." << endl;
#endif //MY_DEBUG_SOCKET
		sock_data_count = recv(sclient, sock_recv_buf, 100, 0);
		sock_recv_buf[sock_data_count] = 0;
	} while (strncmp(sock_recv_buf, "ok\n", sock_data_count));
#ifdef MY_DEBUG_SOCKET
	cout << "acked" << endl;
#endif //MY_DEBUG_SOCKET
	sock_data_count = ioctlsocket(sclient, FIONBIO, &iMode);
	while (0 != strcmp(sock_recv_buf, ack_msg.c_str()))
	{
		//cout << "waiting the turning completed" << endl;
		sock_data_count = recv(sclient, sock_recv_buf, 100, 0);
		//cout << "waiting..." << endl;
		/*cap >> img_trush;*/
		if (sock_data_count > 0)
		{
			//sock_data_count = recv(sclient, sock_recv_buf, 100, 0);
			sock_recv_buf[sock_data_count] = '\0';
#ifdef MY_DEBUG_SOCKET
			cout << "recv:" << sock_recv_buf << endl;
#endif // MY_DEBUG_SOCKET
		}
	}
	iMode = 0;
	sock_data_count = ioctlsocket(sclient, FIONBIO, &iMode);
	//DEBUG_COUT("pwm:");
	//DEBUG_COUT(l_pwm);
	//DEBUG_COUT(r_pwm);
	DEBUG_COUT(cmd_2_car);
	return true;
}

bool send_forward_order(SOCKET & sclient)
{
	int result = 0;
	string cmd_2_car;
	char recv_buf[256] = { 0 };
	cmd_2_car = "f\n";
	while (strcmp(recv_buf, "ok\n"))
	{
		result = send(sclient, cmd_2_car.c_str(), cmd_2_car.size(), 0);
		result = recv(sclient, recv_buf, 256, 0);
		if (result > 0)
		{
			recv_buf[result] = 0;
		}
	}
	return true;
	
}

bool send_stop_order(SOCKET & sclient)
{
	int result = 0;
	string cmd_2_car;
	char recv_buf[256] = { 0 };
	cmd_2_car = "s\n";
	while (strcmp(recv_buf, "ok\n"))
	{
		result = send(sclient, cmd_2_car.c_str(), cmd_2_car.size(), 0);
		result = recv(sclient, recv_buf, 256, 0);
		if (result > 0)
		{
			recv_buf[result] = 0;
		}
	}
	return true;
}
int Init_Socket(SOCKET & sclient)
{
	if (sclient == INVALID_SOCKET)
	{
		printf("invalid socket !");
		return -1;
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(6790);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	//serAddr.sin_addr.S_un.S_addr = InetPton();
	if (connect(sclient, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("connect error !");
		closesocket(sclient);
		return -1;
	}
}