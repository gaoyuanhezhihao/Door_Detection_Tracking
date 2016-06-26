#include "main_func.h"


/* judge the result of door detection.
*------------------------------------------------------------------------
@current_mode: the mode of this system. set it to "tracking" if we need
to switch to tracking mode.
@target: tracked target. updated for every frame. 
@curr_door_states: got door or not.
@doors_points: doors detected.
*------------------------------------------------------------------------
return: 
	false: stop door detection, mode changed.
	true: continue door detection.
*/
bool judge_door(system_mode & current_mode, object_target & target, Mat & image,
		door_states & curr_door_states, const vector<std::array<Point, 4>> & doors_points) {
	Rect target_window;
	switch (curr_door_states) {
	case door_states::Get_Door:
		/*init door tracking*/
		target_window.x = doors_points[0][1].x + 5;
		target_window.y = doors_points[0][2].y + 10;
		target_window.width = doors_points[0][2].x - doors_points[0][1].x - 10;
		target_window.height = doors_points[0][3].y - doors_points[0][2].y - 10;
		init_cam_shift(target, image, target_window);
		/*switch to tracking mode.*/
		current_mode = system_mode::tracking_door;
		break;
	case door_states::No_Door:
		return true;
		break;
	default:
		return true;
		break;
	}
}

/*judge the track result
* ----------------------------------------------
	@current_mode: the mode of this system. set it to "tracking" if we need
	to switch to tracking mode.
	@target: tracked target. updated for every frame.
	@view_size: image size of the camera.
	@sclient: socket connection to the car proxy.
* -------------------------------------------------
* return: 
	true: continue tracking.
	false: switch to another mode.
*/
bool judge_tracking(system_mode & current_mode, object_target & target,
		Size & view_size, SOCKET & sclient) {
	if (target.trackWindow.x < TRACK_LR_EDGE_THRES) {
		/*near left edge.*/
		/*go forward a little distance.*/
		go_forward_dist(sclient, TRACK_END_FORWARD_DIST);
		/*turn left.*/
		make_a_turn('l', 90, sclient);
		/*switch mode to verify door.*/
		target.side_in_im = 'l';
		current_mode = system_mode::verifing_door;
		return false;
	}
	else if(target.trackWindow.x + target.trackWindow.width > \
			view_size.width - TRACK_LR_EDGE_THRES){
		/*near right edge.*/
		/*go forward a little distance.*/
		go_forward_dist(sclient, TRACK_END_FORWARD_DIST);
		/*turn left.*/
		make_a_turn('r', 90, sclient);
		/*switch mode to verify door.*/
		target.side_in_im = 'r';
		current_mode = system_mode::verifing_door;
		return false;
	}
	return true;
}

bool verify_bias(const char last_state, cv::VideoCapture & cap, cv::Mat & cdst, cv::Point & current_point, SOCKET & sclient)
{
	vp_states line_state;
	Mat im;
	Mat edge_im;
	vector<Vec2f> v_lines;
	Point vp;
	send_stop_order(sclient);
	//DEBUG_COUT("verify_bias start:");
	double start = double(getTickCount());
	while (getTickCount() - start < 0.5)
	{
		cap >> im;
	}
	cap >> im;

	line_state = vanish_point_detection(im, cdst, edge_im,v_lines, vp);
	switch (line_state)
	{
	case vp_states::only_left_line:
		//current bias is left.
		if (last_state == 'l')
		{
			//DEBUG_COUT("verify: true");
			Init_Car(sclient);
			make_a_turn('r', TURN_DEGREE, sclient);
			return true;
		}
		else
		{
			//DEBUG_COUT("verify: true");
			return false;
		}
		break;
	case vp_states::only_right_line:
		// current bias is right.
		if (last_state == 'r')
		{
			//DEBUG_COUT("verify: true");
			Init_Car(sclient);
			make_a_turn('l', TURN_DEGREE, sclient);
			return true;
		}
		else
		{
			//DEBUG_COUT("verify: false");
			return false;
		}
		break;
	case vp_states::no_line:
		// current state is "deteced nothing"
		if (last_state == 'n')
		{
			//DEBUG_COUT("verify: false");
			return true;
		}
		else
		{
			//DEBUG_COUT("verify: false");
			return false;
		}
		break;
	case vp_states::got_vp:
		//DEBUG_COUT("verify: true");
		return false;
		break;
	default:
		throw std::invalid_argument("invalid line state");
		break;
	}
}

bool judge_vanish_point(Point &current_point, vp_states & current_state,
		int center_x_min, int center_x_max, int center_y_min,
		int center_y_max, VideoCapture & cap, Mat & cdst, SOCKET & sclient) {
	if (current_state == vp_states::only_left_line)
	{
		DEBUG_COUT("too left");
		//verify_bias('l', cap, cdst, current_point, sclient);
		//make_a_turn('r', TURN_DEGREE, sclient, cap);
		/*don't do door detection */ 
		return false;
	}
	else if (current_state == vp_states::only_right_line)
	{
		DEBUG_COUT("too right");
		//verify_bias('r', cap, cdst, current_point, sclient);
		//make_a_turn('l', TURN_DEGREE, sclient, cap);
		/*don't do door detection */
		return false;
	}
	else if (current_state == vp_states::no_line)
	{
		//cout << "No line found" << endl;
		DEBUG_COUT("No line found");
		//verify_bias('n', cap, cdst, current_point, sclient);
		/*don't do door detection */
		return false;
	}
	else if (current_state == vp_states::got_vp)
	{
		///*adjust pwm */
		//if (current_point.x > center_x_max)
		//{
		//	DEBUG_COUT("Bias left");
		//	adjust_pwm('l', current_point.x, mark_point.x, sclient);
		//	send_forward_order(sclient);
		//}
		//else if (current_point.x < center_x_min)
		//{
		//	DEBUG_COUT("Bias right");
		//	adjust_pwm('r', current_point.x, mark_point.x, sclient);
		//	send_forward_order(sclient);
		//}
		//else
		//{
		//	DEBUG_COUT("Center");
		//	send_forward_order(sclient);
		//}
		/*continue to door detection*/
		DEBUG_COUT("got vp");
		return true;
	}
	else
	{
		throw std::invalid_argument("invalid line state");
	}
}
