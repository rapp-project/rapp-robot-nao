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
#include "rapp_robot_agent/PlayAudio.h"
#include "rapp_robot_agent/RecognizeWord.h"
#include "rapp_robot_agent/Record.h"
#include "rapp_robot_agent/VoiceRecord.h"
#include "rapp_robot_agent/MicrophoneEnergy.h"
#include "rapp_robot_agent/RecordWithSoundDetection.h"



//#define BUFSIZE 8192//48000  // Size of one element
//#define BUFNUMBER 16 // Number of elements
#define AUDIO_BUFFER_FORMAT char


//using namespace std;

class NaoCommunication {

public:
	NaoCommunication (int argc, char **argv);

	ros::ServiceClient client_say;
	ros::ServiceClient client_textToSpeech;
	ros::ServiceClient client_playAudio;
 	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;
	ros::ServiceClient client_voiceRecord;
	ros::ServiceClient client_microphoneEnergy;
	ros::ServiceClient client_recordWithSoundDetection;
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
	//std::vector<unsigned char> audio_buffer_vector;
	std::vector< std::vector<unsigned char> > audio_buffer_vector;

	void init(int argc, char **argv);
	bool textToSpeech(std::string str);
	bool textToSpeech(std::string str, std::string language);// Method which calls ros service rapp_say. It causes nao robot says provided string message.
	bool playAudio(std::string file_path, double position, double volume, double balance, bool play_in_loop);//Method which calls ros service rapp_play_audio. It causes nao robot plays desired audio file. 
		/*
		std::string file_path - absolute path of the file
		double position - position in second where the playing should begin
		double volume - volume requested [0.0 - 1.0]
		double balance - Stereo panorama requested (-1.0 : left / 1.0 : right / 0.0 : center)
		bool play_in_loop - true -- play file in loop; false -- play once from given position
		*/
	std::string wordSpotting(std::string dictionary[], int size);
	std::string captureAudio(int time); // Recording stops after a specified time (time in [s]) // records in ogg format
	std::string captureAudio(std::string file_path, float waiting_time, int microphone_energy);// Recording stops after a specified time (waiting_time in [s]), if sound with a sufficient energy level (microphone_energy) was not detected during this time // microphone_energy should be > 1700 (noise); // records in ogg format
	int microphoneEnergy(std::string name);
	void voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> >  &audio_buffer_vector);// Function from Rapp API that calls voice record service from core agent on NAO robot. Robot records the sound. The recording stops when sound is not detected during the time equal to silenceTime [s]
	
	template<typename T>
	inline std::vector<std::basic_string<char> > copyTable(T table[], int size);

};
