#include "ros/ros.h"
#include <iostream>
#include <string>

#include "rapp_robot_agent/Say.h"
#include "rapp_robot_agent/RecognizeWord.h"
#include "rapp_robot_agent/Record.h"

using namespace std;

class NaoCommunication {

public:
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;

	ros::NodeHandle *n;

	void init(int argc, char **argv);
	bool say(string str);
	string recognizeWord(string dictionary[], int size);
	string record(int time);

	template<typename T>
	inline vector<basic_string<char> > copyTable(T table[], int size);

};
