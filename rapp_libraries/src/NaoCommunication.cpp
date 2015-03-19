#include "NaoCommunication.h"
#include "ros/ros.h"
#include "rapp_robot_agent/Say.h"
#include "rapp_robot_agent/RecognizeWord.h"
#include "rapp_robot_agent/Record.h"

// Class that provides simple and core Rapp API functions utilized for communication with Nao robot and cloud services.
NaoCommunication::NaoCommunication(int argc,char **argv){
		ros::init(argc, argv,"Communication_client");
		n = new ros::NodeHandle();

		}

	// Function from Rapp API that calls say service from core agent on Nao robot. Robot says a given sentence.
	void NaoCommunication::say(string str){	

		client_say = n->serviceClient<rapp_robot_agent::Say>("rapp_say");
		rapp_robot_agent::Say srv;
		srv.request.request = str;
		if (client_say.call(srv))
		{
			ROS_INFO("Nao said");
		}
		else
		{
			ROS_ERROR("Failed to call service Say"); 
		}
	}

	// Function from Rapp API that calls word recognition service from core agent on NAO robot. Robot recognizes word.
	// dictionary - table of words to be recognized
	// size - size of dictionary
	void NaoCommunication::recognizeWord(string dictionary[], int size){	

		client_recognizeWord = n->serviceClient<rapp_robot_agent::RecognizeWord>("rapp_get_recognized_word");
		rapp_robot_agent::RecognizeWord srv;
		
		vector<string> tmp=copyTable(dictionary,size);

		srv.request.wordsList = tmp;
		if (client_recognizeWord.call(srv))
		{
			ROS_INFO("Nao recognized word");
			cout<<"Recognized word:" << srv.response.recognizedWord<<endl;
		}
		else
		{
			ROS_ERROR("Failed to call service RecognizeWord"); 
		}
	}

	// Function from Rapp API that calls record service from core agent on NAO robot. Robot records the sound.
	// time - a given time period for recording the sound
	void NaoCommunication::record(int time){
		client_say = n->serviceClient<rapp_robot_agent::Record>("rapp_record");
		rapp_robot_agent::Record srv;
		srv.request.recordingTime = time;
		if (client_say.call(srv))
		{
			ROS_INFO("Nao recorded sound");
			cout<<"Recorded sound:" << srv.response.recordedFileDest<<endl;
		}
		else
		{
			ROS_ERROR("Failed to call service Record"); 
		}
	}


	// Method that copies table of given type to vector
	template<typename T>
	inline vector<T> NaoCommunication::copyTable(T table[], int size){
		vector<T> tmp;
		for(int i=0; i<size; i++)
		{
			tmp.push_back(table[i]);			
		}
		return tmp;
	}




