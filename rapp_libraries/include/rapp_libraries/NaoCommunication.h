//#####################
// written by Jan Figat
//#####################

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <fstream>

#include <cstdio>
#include <ostream>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>

//#include "ros/ros.h"
#include <iostream>
#include <string>

#include "rapp_robot_agent/Say.h"
#include "rapp_robot_agent/RecognizeWord.h"
#include "rapp_robot_agent/Record.h"
#include "rapp_robot_agent/VoiceRecord.h"

using namespace std;

class NaoCommunication {

public:
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
	ros::ServiceClient client_textToSpeech;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;
	ros::ServiceClient client_voiceRecord;

	ros::NodeHandle *n;

	void init(int argc, char **argv);
	bool say(string str);
	void textToSpeech( std::string str, std::string language);// Method which calls ros service rapp_say. It causes nao robot says provided string message.
	string recognizeWord(string dictionary[], int size);
	string record(int time);
	void voiceRecord(string filename, int microphoneEnergy, int silenceTime, std::vector<std::string> &vectorFileName);// Function from Rapp API that calls voice record service from core agent on NAO robot. Robot records the sound. The recording stops when sound is not detected during the time equal to silenceTime [s]
	// vectorFileName -- vector with the recorded audio file names

	template<typename T>
	inline vector<basic_string<char> > copyTable(T table[], int size);

};
