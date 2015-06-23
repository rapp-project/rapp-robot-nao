#include "ros/ros.h"
#include <iostream>

#include "rapp_ros_naoqi_wrappings/Say.h"
#include "rapp_ros_naoqi_wrappings/PlayAudio.h"
#include "rapp_ros_naoqi_wrappings/RecognizeWord.h"
#include "rapp_ros_naoqi_wrappings/Record.h"
#include "rapp_ros_naoqi_wrappings/VoiceRecord.h"
#include "rapp_ros_naoqi_wrappings/MicrophoneEnergy.h"
#include "rapp_ros_naoqi_wrappings/RecordWithSoundDetection.h"

#include "NaoSendEmail.h"


//using namespace std;

class NaoCommunication {

public:
	~NaoCommunication(){}
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

	void init(int argc, char **argv);
	bool textToSpeech(std::string str);
	bool textToSpeech(std::string str, std::string language);// Method which calls ros service rapp_say. It causes nao robot says provided string message.
	bool playAudio(std::string file_path, double position, double volume, double balance, bool play_in_loop);//Method which calls ros service rapp_play_audio. It causes nao robot plays desired audio file. 	//file_path - Absolute path of the file; Volume requested [0.0 - 1.0]; balance -1.0 : left / 1.0 : right / 0.0 : center;
	std::string wordSpotting(std::string dictionary[], int size);
	std::string captureAudio(int time);
	std::string captureAudio(std::string file_path, float waiting_time, int microphone_energy);// Recording stops after a specified time (waiting_time in [s]), if sound with a sufficient energy level (microphone_energy) was not detected during this time // microphone_energy should be > 1700 (noise); // records in ogg format
	int microphoneEnergy(std::string name);
	void voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector); // Function from Rapp API that calls voice record service from core agent on NAO robot. Robot records the sound. The recording stops when sound is not detected during the time equal to silenceTime [s]
	static void sendEmail(std::string login, std::string password, std::string sendTo);

	template<typename T>
	T getParameter(std::string name)
	{
		T val;
		n->getParam(name, val);
		return val;
	}

	template<typename T>
	inline std::vector<std::basic_string<char> > copyTable(T table[], int size);
};
