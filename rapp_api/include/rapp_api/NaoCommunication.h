#include "ros/ros.h"
#include <iostream>

#include "rapp_ros_naoqi_wrappings/Say.h"
#include "rapp_ros_naoqi_wrappings/RecognizeWord.h"
#include "rapp_ros_naoqi_wrappings/Record.h"

#include "NaoSendEmail.h"


using namespace std;

class NaoCommunication {

public:
	~NaoCommunication(){}
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;

	ros::NodeHandle *n;

	void init(int argc, char **argv);
	bool textToSpeech(string str);
	string wordSpotting(string dictionary[], int size);
	string captureAudio(int time);
	static void sendEmail(string login, string password, string sendTo);

	template<typename T>
	inline vector<basic_string<char> > copyTable(T table[], int size);
};
