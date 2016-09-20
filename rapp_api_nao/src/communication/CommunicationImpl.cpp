#include "CommunicationImpl.hpp"

namespace rapp {
namespace robot {

CommunicationImpl::CommunicationImpl(int argc,char **argv) {
	ros::init(argc, argv,"Communication_client");
	n = new ros::NodeHandle();
}

CommunicationImpl::~CommunicationImpl() {
}


/*bool CommunicationImpl::textToSpeech(const std::string & str){	

	client_say = n->serviceClient<rapp_ros_naoqi_wrappings::Say>("rapp_say");
	if (!client_say) {
		ROS_ERROR("Can't connect to service rapp_say!");
		return false;
	}
	rapp_ros_naoqi_wrappings::Say srv;
	bool successful = false;
	
	srv.request.request = str;
	srv.request.language="English";
	if (client_say.call(srv))
	{
		ROS_INFO("Nao said");
		successful = true;
		return successful;
	}
	else
	{
		ROS_ERROR("Failed to call service Say"); 
		successful = false;
	}
	return successful;
}*/
	
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
	//sentence = "\\RSPD=" + std::string("80") + "\\ "; // speed
	//sentence += "\\VCT="+ std::string("43") + "\\ ";  // pitch
	//sentence += std::string(str);
	//sentence += "\\RST\\ ";
	
	ROS_DEBUG("Final sentence: %s", sentence.c_str());

	srv.request.request = str;  
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

std::string CommunicationImpl::wordSpotting(const std::vector<std::string>& dictionary){	
	client_recognizeWord = n->serviceClient<rapp_ros_naoqi_wrappings::RecognizeWord>("rapp_get_recognized_word");
	rapp_ros_naoqi_wrappings::RecognizeWord srv;
	
	srv.request.wordsList = dictionary;//copyTable(dictionary,size);
	
	if (client_recognizeWord.call(srv))
	{
		//ROS_INFO("Nao recognized word");
		ROS_INFO_STREAM("Nao recognized word: "<< srv.response.recognizedWord);
		return srv.response.recognizedWord;
	}
	else
	{
		ROS_ERROR("Failed to call service RecognizeWord"); 
	}
	return "";
}

std::string CommunicationImpl::captureAudio(int time){
	client_say = n->serviceClient<rapp_ros_naoqi_wrappings::Record>("rapp_record");
	rapp_ros_naoqi_wrappings::Record srv;
	
	srv.request.recordingTime = time;
	
	if (client_say.call(srv))
	{
		//ROS_INFO("Nao recorded audio message");
		ROS_INFO_STREAM("Recorded audio message to the file: " << srv.response.recordedFileDest);
		return srv.response.recordedFileDest;
	}
	else
	{
		ROS_ERROR("Failed to call service Record"); 
	}
	return "";
}


int CommunicationImpl::microphoneEnergy(std::string name){
	client_microphoneEnergy = n->serviceClient<rapp_ros_naoqi_wrappings::MicrophoneEnergy>("rapp_get_microphone_energy");
	rapp_ros_naoqi_wrappings::MicrophoneEnergy srv;
	
	srv.request.microphone = name;
	int energy;
	
	if (client_microphoneEnergy.call(srv))
	{
		ROS_INFO_STREAM("Detected microphone energy: " << srv.response.energy);
		energy = srv.response.energy;
	}
	else
	{
		ROS_ERROR("Failed to call service MicrophoneEnergy"); 
		energy = 0;
	}
	return energy;
}


std::string CommunicationImpl::captureAudio (std::string file_path, float waiting_time/*in sec*/, int microphone_energy/*2700*/){
	client_recordWithSoundDetection = n->serviceClient<rapp_ros_naoqi_wrappings::RecordWithSoundDetection>("rapp_record_with_sound_detection");
	rapp_ros_naoqi_wrappings::RecordWithSoundDetection srv;
	
	srv.request.file_dest = file_path;
	srv.request.waiting_time = waiting_time;
	srv.request.microphone_energy = microphone_energy;
	
	if (client_recordWithSoundDetection.call(srv))
	{
		ROS_INFO("Nao recorded audio message");
		return srv.response.output_file_path;
	}
	else
	{
		ROS_ERROR("Failed to call service Record with sound detection"); 
	}
	return "";
}

	
void CommunicationImpl::voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector )
{
	client_voiceRecord = n->serviceClient<rapp_ros_naoqi_wrappings::VoiceRecord>("rapp_voice_record");
	rapp_ros_naoqi_wrappings::VoiceRecord srv;
	
	srv.request.startRecording = startRecording; //if true, the recording will start if false, the recording will be stoped
	
	if (client_voiceRecord.call(srv))
	{
		// for buffer usage
		ROS_INFO_STREAM("Recorded audio message to the buffer. Buffer size: " << srv.response.buffer_.size()); //should be 8192 (microphone buffer size)
		
		audio_buffer_vector.push_back( srv.response.buffer_ ); //adds buffer to the vector of vectors
		return;
	}
	else
	{
		ROS_ERROR("Failed to call service Voice Record"); 
	}
	return;
}

} // namespace robot
} // namespace rapp
