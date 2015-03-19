#include "ros/ros.h"
#include <iostream>
#include <string>

using namespace std;

class NaoCommunication {

public:
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;

	ros::NodeHandle *n;

	void init(int argc, char **argv);
	void say(string str);
	void recognizeWord(string dictionary[], int size);
	void record(int time);

	template<typename T>
	inline vector<T> copyTable(T table[], int size);

};
