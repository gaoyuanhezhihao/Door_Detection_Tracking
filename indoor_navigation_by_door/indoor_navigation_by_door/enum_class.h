#ifndef ENUM_CLASS_h
#define ENUM_CLASS_h
enum class vp_states {
	got_vp,
	only_left_line,
	only_right_line,
	no_line,
};

enum class door_states{
	Get_Door,
	No_Door,
	Found_a_door,
	Not_a_door
};
enum class system_mode
{
	detect_vp_dp,
	tracking_door,
	verifing_door,
};

#endif // ENUM_CLASS_h