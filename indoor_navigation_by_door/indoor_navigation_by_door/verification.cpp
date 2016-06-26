#include "verification.h"

bool verify_door(const Mat &img, char side_of_door, system_mode & current_mode,
		SOCKET & sclient) {
	char order;
	cout << "verify door>>>\n input any char to continue>>>" << endl;
	cin >> order;
	if (side_of_door == 'l') {
		make_a_turn('r', 90, sclient);
	}
	else if (side_of_door == 'r') {
		make_a_turn('l', 90, sclient);
	}
	else {
		cout << "BUG: side_of_door" << endl;
	}
	current_mode = system_mode::detect_vp_dp;
	return true;
}