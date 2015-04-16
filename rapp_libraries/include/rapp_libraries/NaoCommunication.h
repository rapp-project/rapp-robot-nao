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
#include "rapp_robot_agent/MicrophoneEnergy.h"



#define BUFSIZE 8192//48000  // Size of one element
#define BUFNUMBER 16 // Number of elements
#define AUDIO_BUFFER_FORMAT char


using namespace std;

class NaoCommunication {

public:
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
	ros::ServiceClient client_textToSpeech;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;
	ros::ServiceClient client_voiceRecord;
	ros::ServiceClient client_microphoneEnergy;

	ros::NodeHandle *n;

	//std::vector<AUDIO_BUFFER_FORMAT> audioBuffer;//[BUFSIZE];
	/*
	std::vector<std::string> audioBuffer;
	struct audio_buffer_struct
	{
		char ch[BUFSIZE];
	};
	*/

	//std::vector<audio_buffer_struct> audio_buffer_vector;
	std::vector<unsigned char> audio_buffer_vector;

	void init(int argc, char **argv);
	bool say(string str);
	void textToSpeech( std::string str, std::string language);// Method which calls ros service rapp_say. It causes nao robot says provided string message.
	string recognizeWord(string dictionary[], int size);
	string record(int time);
	int microphoneEnergy(std::string name);
	void voiceRecord(bool startRecording, std::vector<unsigned char> &audio_buffer_vector);//[BUFSIZE] );// Function from Rapp API that calls voice record service from core agent on NAO robot. Robot records the sound. The recording stops when sound is not detected during the time equal to silenceTime [s]
	
	template<typename T>
	inline vector<basic_string<char> > copyTable(T table[], int size);

};
