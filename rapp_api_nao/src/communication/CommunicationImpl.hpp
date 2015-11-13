#include "ros/ros.h"
#include <iostream>

#include "rapp_ros_naoqi_wrappings/Say.h"
#include "rapp_ros_naoqi_wrappings/PlayAudio.h"
#include "rapp_ros_naoqi_wrappings/RecognizeWord.h"
#include "rapp_ros_naoqi_wrappings/Record.h"
#include "rapp_ros_naoqi_wrappings/VoiceRecord.h"
#include "rapp_ros_naoqi_wrappings/MicrophoneEnergy.h"
#include "rapp_ros_naoqi_wrappings/RecordWithSoundDetection.h"

#include <rapp/robot/communication/communication.hpp>

typedef rapp::robot::Communication::Language Language;

namespace rapp {
namespace robot {

class CommunicationImpl {

public:

	CommunicationImpl (int argc, char **argv);
	~CommunicationImpl();
	
	ros::ServiceClient client_say;
	ros::ServiceClient client_playAudio;
	ros::ServiceClient client_recognizeWord;
 	ros::ServiceClient client_record;
	ros::ServiceClient client_voiceRecord;
	ros::ServiceClient client_microphoneEnergy;
	ros::ServiceClient client_recordWithSoundDetection;

	ros::NodeHandle *n;
	
	//bool textToSpeech(const std::string & str);
	
	bool textToSpeech(const std::string & str, Language language);
	
	bool playAudio(std::string file_path, double position, double volume, double balance, bool play_in_loop);
	
	std::string wordSpotting(std::string dictionary[], int size);
	
	std::string captureAudio(int time);
	
	int microphoneEnergy(std::string name);
	
	std::string captureAudio(std::string file_path, float waiting_time, int microphone_energy);
	
	void voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector);
	
	
private:
	
	template<typename T>
	T getParameter(std::string name)
	{
		T val;
		n->getParam(name, val);
		return val;
	}

	template<typename T>
	inline std::vector<std::basic_string<char> > copyTable(T table[], int size) {
		std::vector<std::basic_string<char> > tmp;
		for(int i=0; i<size; i++)
		{
			tmp.push_back(table[i].c_str());
			std::cout<<tmp[i];
		}
		return tmp;
	}

};

} // namespace robot
} // namespace rapp
