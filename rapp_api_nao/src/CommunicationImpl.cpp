#include "CommunicationImpl.hpp"

namespace rapp {
namespace robot {

CommunicationImpl::CommunicationImpl(int argc,char **argv) {
	ros::init(argc, argv,"Communication_client");
	n = new ros::NodeHandle();
}

CommunicationImpl::~CommunicationImpl() {
}

bool CommunicationImpl::textToSpeech(const std::string & str, Language language) {
	client_say = n->serviceClient<rapp_ros_naoqi_wrappings::Say>("rapp_say");
	if (!client_say) {
		ROS_ERROR("Can't connect to service rapp_say!");
		return false;
	}
	rapp_ros_naoqi_wrappings::Say srv;
	bool successful = false;
	
	// slower and lower voice
	std::string sentence;
	sentence = "\\RSPD=" + std::string("80") + "\\ "; // speed
	sentence += "\\VCT="+ std::string("43") + "\\ ";  // pitch
	sentence += std::string(str);
	sentence += "\\RST\\ ";
	
	ROS_DEBUG("Final sentence: %s", sentence.c_str());

	srv.request.request = sentence;  
	switch(language) {
		case Language::ENGLISH: srv.request.language = "english"; break;
		case Language::GREEK: srv.request.language = "greek"; break;
	}
	

	if (client_say.call(srv)) {
		successful = true;
	} else {
		//Failed to call service
		ROS_ERROR("textToSpeech call failed!");
		successful = false;
	}
	
	return successful;
}

bool CommunicationImpl::playAudio(std::string file_path, double position, double volume, double balance, bool play_in_loop) {
	client_playAudio = n->serviceClient<rapp_ros_naoqi_wrappings::PlayAudio>("rapp_play_audio");
	rapp_ros_naoqi_wrappings::PlayAudio srv;
	bool successful = false;
	
	srv.request.file_path = file_path;
	srv.request.begin_position = position;
	srv.request.volume = volume;
	srv.request.balance_LR = balance;
	srv.request.play_in_loop = play_in_loop;

	if (client_playAudio.call(srv)) {
		if (srv.response.success == true)
			successful = true;
		else {
			ROS_ERROR("playAudio call returned false!");
			successful = false;
		}
	} else {
		//Failed to call service
		ROS_ERROR("playAudio call failed!");
		successful = false;
	}
	
	return successful;
}

} // namespace robot
} // namespace rapp
