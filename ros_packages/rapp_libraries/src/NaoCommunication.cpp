	#include <rapp_libraries/NaoCommunication.h>


	// Class that provides simple and core Rapp API functions utilized for communication with Nao robot and cloud services.
	NaoCommunication::NaoCommunication(int argc,char **argv){
		ros::init(argc, argv,"Communication_client");
		n = new ros::NodeHandle();

	}

	// Function from Rapp API that calls say service from core agent on Nao robot. Robot says a given sentence.
	bool NaoCommunication::say(string str){	

		client_say = n->serviceClient<rapp_robot_agent::Say>("rapp_say");
		rapp_robot_agent::Say srv;
		srv.request.request = str;
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

	// Function from Rapp API that calls word recognition service from core agent on NAO robot. Robot recognizes word.
	// dictionary - table of words to be recognized
	// size - size of dictionary
	string NaoCommunication::recognizeWord(string dictionary[], int size){	
		client_recognizeWord = n->serviceClient<rapp_robot_agent::RecognizeWord>("rapp_get_recognized_word");
		rapp_robot_agent::RecognizeWord srv;
		srv.request.wordsList = copyTable(dictionary,size);
		if (client_recognizeWord.call(srv))
		{
			ROS_INFO("Nao recognized word");
			cout<<"Recognized word:" << srv.response.recognizedWord<<endl;
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
	string NaoCommunication::record(int time){
		client_say = n->serviceClient<rapp_robot_agent::Record>("rapp_record");
		rapp_robot_agent::Record srv;
		srv.request.recordingTime = time;
		if (client_say.call(srv))
		{
			ROS_INFO("Nao recorded sound");
			cout<<"Recorded sound:" << srv.response.recordedFileDest<<endl;
			return srv.response.recordedFileDest;
		}
		else
		{
			ROS_ERROR("Failed to call service Record"); 
		}
		return "";
	}


	// Function from RAPP API that calls send email service using VMIME library and SMTP protocol (smtp://smtp.gmail.com)
	void NaoCommunication::sendEmail(string login, string password, string sendTo)
	{
		SendEmailClass::sendMessage(login, password, sendTo);
		return;
	}


	// Method that copies table of given type to vector
	template<typename T>
	inline vector<basic_string<char> > NaoCommunication::copyTable(T table[], int size){
		vector<basic_string<char> > tmp;
		for(int i=0; i<size; i++)
		{
			tmp.push_back(table[i].c_str());
			cout<<tmp[i];
		}
		return tmp;
	}



