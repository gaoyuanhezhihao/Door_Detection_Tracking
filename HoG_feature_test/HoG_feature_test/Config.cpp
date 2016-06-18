#include "Config.h"
#include <string>
#include "stdlib.h"
#include <iostream>
#include <fstream>
using namespace std;

int & operator << (int & dst, universal_type & src){
    dst = atoi(src.raw_data.c_str());
    return dst;
}

double & operator << (double & dst, universal_type & src) {
    dst = atof(src.raw_data.c_str());
    return dst;
}

bool read_config_file(char * file_name, map<string, universal_type> & config_map) {
    string line;
    string feature_name;
    universal_type feature_value;
    size_t split_id = 0;
    ifstream config_file;
    config_file.open(file_name);
    if(config_file.is_open()) {
        while(getline(config_file, line)) {
            split_id = line.find_first_of(':');
            if(string::npos == split_id) {
                break;
            }
            else {
                feature_name = line.substr(0, split_id);
                feature_value.set_value(line.substr(split_id + 1));
                config_map.insert(pair<string, universal_type> (feature_name, feature_value));
            }
        }
    }
	return true;
}

ostream & operator << (ostream & std_out, universal_type & src) {
    std_out << src.raw_data;
    return std_out;
}

string & operator <<(string & dst, universal_type & src) {
	dst = src.raw_data;
	return dst;
}

