#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include <map>
#include <string>
using namespace std;
class universal_type{
    private:
        string raw_data;
    public:
        universal_type(char * raw):raw_data(raw) {
            ;
        }
        universal_type(){
            ;
        }
        bool set_value(string  raw_string){
            raw_data = raw_string;
            return true;
        }

     friend int & operator << (int & dst, universal_type &);
     friend double & operator << (double & dst, universal_type & src);
     friend ostream & operator << (ostream & std_out, universal_type & src);
	 friend string & operator <<(string & dst, universal_type & src);
};

double & operator << (double & dst, universal_type & src);
int & operator << (int & dst, universal_type &src);
string & operator <<(string & dst, universal_type & src);
bool read_config_file(char * file_name, map<string, universal_type> & config_map);

ostream & operator << (ostream & std_out, universal_type & src);
#endif // CONFIG_H_INCLUDED
