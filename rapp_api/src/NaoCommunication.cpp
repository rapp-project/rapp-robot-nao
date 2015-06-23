	#include <rapp_api/NaoCommunication.h>


	// Class that provides simple and core Rapp API functions utilized for communication with Nao robot and cloud services.
	NaoCommunication::NaoCommunication(int argc,char **argv){
		ros::init(argc, argv,"Communication_client");
		n = new ros::NodeHandle();

	}

	// Function from Rapp API that calls say service from core agent on Nao robot. Robot says a given sentence.
	bool NaoCommunication::textToSpeech(std::string str){	

		client_say = n->serviceClient<rapp_ros_naoqi_wrappings::Say>("rapp_say");
		rapp_ros_naoqi_wrappings::Say srv;
		srv.request.request = str;
		srv.request.language="English";
		if (client_say.call(srv))
		{
			ROS_INFO("Nao said");
			return true;
		}
		else
		{
			ROS_ERROR("Failed to call service Say"); 
		}
		return false;
	}

	//#############
	bool NaoCommunication::textToSpeech( std::string str, std::string language)
	{
		client_textToSpeech = n->serviceClient<rapp_ros_naoqi_wrappings::Say>("rapp_say");
		rapp_ros_naoqi_wrappings::Say srv;
		bool successful=false;
		//## slower and lower voice
		std::string sentence;
		sentence = "\\RSPD=" + std::string("80") + "\\ ";
		sentence += "\\VCT="+ std::string("43") + "\\ ";
		sentence += std::string(str);
		sentence += "\\RST\\ ";
		std::cout<<sentence<<std::endl;
	
		srv.request.request=sentence;//a message, that will be said
		srv.request.language=language;//language selection

		if (client_textToSpeech.call(srv))
		{
			std::cout<<"[Text to Speech] - received:\t"<< srv.response.response <<"\n"<< std::flush;
			successful = true;
			return successful;
		}
		else
		{
			//Failed to call service rapp_say
			std::cout<<"[Text to Speech] - Error calling service rapp_say\n";
			successful = false;
			return successful;
		}
		return successful;
	}

	//#############
	bool NaoCommunication::playAudio(std::string file_path, double begin_position, double volume, double balance, bool play_in_loop)
	{
		client_playAudio = n->serviceClient<rapp_ros_naoqi_wrappings::PlayAudio>("rapp_play_audio");
		rapp_ros_naoqi_wrappings::PlayAudio srv;
		bool successful = false;
		srv.request.file_path = file_path;
		srv.request.begin_position = begin_position;
		srv.request.volume = volume;
		srv.request.balance_LR = balance;
		srv.request.play_in_loop = play_in_loop;

		if (client_playAudio.call(srv))
		{
			if (srv.response.success == true)
				std::cout<<"[Play audio] - success\t"<<"\n"<< std::flush;
			successful = true;
			return successful;
		}
		else
		{
			//Failed to call service rapp_say
			std::cout<<"[Play audio] - Error calling service rapp_play_audio\n";
			successful = false;
			return successful;
		}
		return successful;
	}
	
	//#############
	// Function from Rapp API that calls word recognition service from core agent on NAO robot. Robot recognizes word.
	// dictionary - table of words to be recognized
	// size - size of dictionary
	std::string NaoCommunication::wordSpotting(std::string dictionary[], int size){	
		client_recognizeWord = n->serviceClient<rapp_ros_naoqi_wrappings::RecognizeWord>("rapp_get_recognized_word");
		rapp_ros_naoqi_wrappings::RecognizeWord srv;
		srv.request.wordsList = copyTable(dictionary,size);
		if (client_recognizeWord.call(srv))
		{
			ROS_INFO("Nao recognized word");
			std::cout<<"Recognized word:" << srv.response.recognizedWord<<std::endl;
			return srv.response.recognizedWord;
		}
		else
		{
			ROS_ERROR("Failed to call service RecognizeWord"); 
		}
		return "";
	}

	// Function from Rapp API that calls record service from core agent on NAO robot. Robot records the sound.
	// time - a given time period for recording the sound
	std::string NaoCommunication::captureAudio(int time){
		client_say = n->serviceClient<rapp_ros_naoqi_wrappings::Record>("rapp_record");
		rapp_ros_naoqi_wrappings::Record srv;
		srv.request.recordingTime = time;
		if (client_say.call(srv))
		{
			ROS_INFO("Nao recorded sound");
			std::cout<<"Recorded sound:" << srv.response.recordedFileDest<<std::endl;
			return srv.response.recordedFileDest;
		}
		else
		{
			ROS_ERROR("Failed to call service Record"); 
		}
		return "";
	}

	std::string NaoCommunication::captureAudio (std::string file_path, float waiting_time/*in sec*/, int microphone_energy/*2700*/){
		client_recordWithSoundDetection = n->serviceClient<rapp_ros_naoqi_wrappings::RecordWithSoundDetection>("rapp_record_with_sound_detection");
		rapp_ros_naoqi_wrappings::RecordWithSoundDetection srv;
		srv.request.file_dest = file_path;
		srv.request.waiting_time = waiting_time;
		srv.request.microphone_energy = microphone_energy;
		if (client_recordWithSoundDetection.call(srv))
		{
			ROS_INFO("Nao recorded audio message");
			//std::cout<<"File path to the recorded sound:" << srv.response.output_file_path <<std::endl;
			return srv.response.output_file_path;
		}
		else
		{
			ROS_ERROR("Failed to call service Record with sound detection"); 
		}
		return "";
	}

		
	//#############
	// Function from Rapp API that calls voice record service from core agent on NAO robot. Robot records the sound. The recording stops when sound is not detected during the time equal to silenceTime [s]
	void NaoCommunication::voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector )
	{
		client_voiceRecord = n->serviceClient<rapp_ros_naoqi_wrappings::VoiceRecord>("rapp_voice_record");
		rapp_ros_naoqi_wrappings::VoiceRecord srv;
		srv.request.startRecording = startRecording; //if true, the recording will start if false, the recording will be stoped
		
		if (client_voiceRecord.call(srv))
		{
			// for buffer usage
			ROS_INFO("Nao recorded sound to the buffer");
			std::cout<<"buffer size :"<<srv.response.buffer_.size()<<std::endl; //should be 8192 (microphone buffer size)
			fflush(stdout);
			
			//for (int i=0; i<srv.response.buffer_.size();i++)
			//	audio_buffer_vector.push_back(srv.response.buffer_[i]); // is this correctly working?
			audio_buffer_vector.push_back( srv.response.buffer_ ); //adds buffer to the vector of vectors
			return;
		}
		else
		{
			ROS_ERROR("Failed to call service Voice Record"); 
		}
		return;
	}

	int NaoCommunication::microphoneEnergy(std::string name){
		client_microphoneEnergy = n->serviceClient<rapp_ros_naoqi_wrappings::MicrophoneEnergy>("rapp_get_microphone_energy");
		rapp_ros_naoqi_wrappings::MicrophoneEnergy srv;
		srv.request.microphone = name;
		//srv.request.start = start_recording;
		int energy;
		if (client_microphoneEnergy.call(srv))
		{
			//ROS_INFO("Nao microphone energy - check");
			std::cout<<"Energy detected:" << srv.response.energy<<std::endl;
			energy = srv.response.energy;
		}
		else
		{
			ROS_ERROR("Failed to call service MicrophoneEnergy"); 
			energy = 0;
		}
		return energy;
	}


	// Function from RAPP API that calls send email service using VMIME library and SMTP protocol (smtp://smtp.gmail.com)
	void NaoCommunication::sendEmail(std::string login, std::string password, std::string sendTo)
	{
		SendEmailClass::sendMessage(login, password, sendTo);
		return;
	}


	// Method that copies table of given type to vector
	template<typename T>
	inline std::vector<std::basic_string<char> > NaoCommunication::copyTable(T table[], int size){
		std::vector<std::basic_string<char> > tmp;
		for(int i=0; i<size; i++)
		{
			tmp.push_back(table[i].c_str());
			std::cout<<tmp[i];
		}
		return tmp;
	}



