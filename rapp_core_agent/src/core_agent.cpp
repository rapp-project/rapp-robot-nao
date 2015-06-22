
// core_agent.cpp
// An example of RAPP core agent functionality: downloading
// and running applications from the cloud (RAPP store)
//
// Marcin Szlenk <m.szlenk@elka.pw.edu.pl>
// Maksym Figat <maksym.figat44@gmail.com>
// Copyright 2014 RAPP

#include "ros/ros.h"
#include "ros/service.h"
#include <sys/types.h>
#include <unistd.h>

#include "std_msgs/String.h"
#include "rapp_ros_naoqi_wrappings/RecognizeWord.h"
#include "rapp_ros_naoqi_wrappings/Say.h"
#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParam.h"
#include "rapp_ros_naoqi_wrappings/GetTransform.h"

#include "sensor_msgs/Image.h"

//#include "rapp_core_agent/Status.h"
#include "rapp_core_agent/DynamicAgentStatus.h"

#include <stdio.h>
#include <ostream>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include <map>



// Name of the ROS topic for receiving requests.
#define REQUEST_TOPIC "/rapp_core_agent/store_interaction/request"
// Name of the ROS topic for publishing requests.
#define RESPONSE_TOPIC "/rapp_core_agent/store_interaction/response"
// Name of the ROS topic for receiving status of dynamic agent
#define RESPONSE_STATUS_TOPIC "dynamic_agent_status"
// Name of servive used to recognized word
#define RECOGNIZEDWORD "rapp_get_recognized_word"


class CoreAgent {

public:
	//Constructor of class CoreAgent
	CoreAgent();

	//Destructor of CoreAgent class
	~CoreAgent() {}

	// Ask for the name of an application to download and publish the
	// request to HOP, where:
	//
	// p - a pointer to the ROS publisher object; must be a non-NULL
	//     on the first call to the function and is ignored on the
	//     subsequent calls.
	void sendRequest();

	// Gets commands
	std::string GetCommand();


	// Runs the script which launches dynamic agent
	void runScript(std::string path);

	// A callback function. Executes each time a new response
	// message from HOP arrives.
	void responseReceived(const std_msgs::String& msg);

	// A callback function. Executes each time a dynamic agent status arrives
	bool dynamicAgentStatusReceived(rapp_core_agent::DynamicAgentStatus::Request &req, rapp_core_agent::DynamicAgentStatus::Response &res);

protected:
	ros::NodeHandle nh_;

	ros::ServiceClient client_;
	// The server service that receives as a request a status and process id of a dynamic agent. As a response it returns an information of a currently executing dynamic agent.
	ros::ServiceServer serverSetStatus_;

	ros::Subscriber subHopCommunication_;
	//ros::Subscriber subDynamicAgentStatus_;
	ros::Publisher pub_;
	// PID number of dynamic agent. This value is set at the state of activation of dynamic agent.
	// After task is finished or error occurs core agent destroys dynamic agent.
	int dynamicAgentPid_;

	// Dictionary:
	// - key is word to be recognized e.g hello
	// - value is a hz name for a corresponding key e.g helloword-1.0.0.hz
	std::map<std::string, std::string> applications_;

	// list of words for recognition system
	std::vector<std::string> words_;

};

//Constructor of class CoreAgent
CoreAgent::CoreAgent() {

	// Wait for service rapp_get_recognized_word
	ros::service::waitForService(RECOGNIZEDWORD);

	// Getting dictionary from rosparam server
	nh_.getParam("applications", applications_);
	for (std::map<std::string,std::string>::iterator it=applications_.begin(); it!=applications_.end(); ++it) {
		words_.push_back(it->first);
	}

	// Create a client for the rapp_get_recognizes_word serivce
	client_ = nh_.serviceClient<rapp_ros_naoqi_wrappings::RecognizeWord>(RECOGNIZEDWORD);

	// Create a client for the rapp_get_recognizes_word serivce
	serverSetStatus_ = nh_.advertiseService(RESPONSE_STATUS_TOPIC, &CoreAgent::dynamicAgentStatusReceived, this);

	// Create a subscriber object.
	subHopCommunication_ = nh_.subscribe(RESPONSE_TOPIC, 100, &CoreAgent::responseReceived, this);

	// Create a publisher object.
	pub_ = nh_.advertise<std_msgs::String>(REQUEST_TOPIC, 100);

	// Send the initial request.
	sendRequest();

}

// Ask for the name of an application to download and publish the
// request to HOP, where:
//
// p - a pointer to the ROS publisher object; must be a non-NULL
//     on the first call to the function and is ignored on the
//     subsequent calls.
void CoreAgent::sendRequest() {
	std::string name;
	bool recognized=false;

	// Enter the name of package using voice command
	do {
		name=GetCommand();
	} while(name=="Empty");

	if(name == "exit") {
		// Shut down the node.
		ros::shutdown();
	} else {
		std_msgs::String msg;

		for (std::map<std::string,std::string>::iterator it=applications_.begin(); it!=applications_.end(); ++it) {
			if(name==it->first) {
				recognized=true;
				name =it->second;
			}
		}

		if (!recognized) {
			std::cout<<"Word was not recognized"<<std::endl;
			ros::shutdown();
		}


		// Publishes a message with the application name. A dynamic agent task of a given name is going tobe downloaded from Rapp Store.

		msg.data = name;
		pub_.publish(msg);
		std::cout << "Downloading...\n";
	}
}

// Gets commands
std::string CoreAgent::GetCommand() {
	std::cout << "Get command\n";
	int i=0;
	int size=applications_.size();

	rapp_ros_naoqi_wrappings::RecognizeWord srv;
	srv.request.wordsList=words_;


	if (client_.call(srv)) {
		//ROS_INFO("Word recognized: %s", srv.response.recognizedWord);
		std::cout<<srv.response.recognizedWord<<std::endl;
		return srv.response.recognizedWord;
	} else {
		//ROS_ERROR("Failed to call service rapp_get_recognizes_word");
		std::cout<<"Error\n";
		return "Error";
	}
	return "Error";//srv.response.recognizedWord;
}

// Runs the script which launches dynamic agent
void CoreAgent::runScript(std::string path) {
	int counter = 0;
	std::string new_path="/home/nao/";
	new_path.append(path.substr(1));
	std::cout<<new_path;
	pid_t pid = fork();
	if (pid == 0) {
		// child process
		execl("/bin/bash", "bash", new_path.c_str(), NULL);
	} else if (pid > 0) {
		// parent process
	} else {
		// fork failed
		std::cout << "fork() failed!\n";
		return ;
	}
	return;
}


// A callback function. Executes each time a new response
// message from HOP arrives.
void CoreAgent::responseReceived(const std_msgs::String& msg) {
	// Response from HOP.
	std::string path = msg.data;

	if(path.empty()) {
		std::cout << "\e[1m\e[31m" // bold and red font
				  << "Failed"
				  << "\e[0m\n"; // normal print mode

		// Word spotting if package was not downloaded
		//sendRequest();
	} else {
		std::cout << "\e[1m\e[31m"
				  << "Installed in "
				  << path << "\e[0m\n";

		// Build the path to the 'run' script of the application.
		path.append("/run");

		std::cout << "Starting...\n";
		// Run the script.
		runScript(path);

	}
}

// A callback function. Executes each time a dynamic agent status arrives
bool CoreAgent::dynamicAgentStatusReceived(rapp_core_agent::DynamicAgentStatus::Request &req, rapp_core_agent::DynamicAgentStatus::Response &res ) {
	std::cout << "[Core agent] - Dynamic agent status received\n";
	std::cout << "[Core agent] - Status = "<<req.da_status<<"\n";
	dynamicAgentPid_ = req.pid;
	std::cout << "[Core agent] - Pid = "<<dynamicAgentPid_<<"\n";
	if(req.da_status == "Init") {
		// Setting status of core and dynamic agent's connection to Initialized
		res.ca_status="Initialized";
	} else if(req.da_status == "Finished") {
		// Kill process of dynamic agent
		// todo - is it necessary?
		// Ask for the next application's name.
		res.ca_status="Finished";
		sendRequest();
	} else if(req.da_status == "Working") {
		// Setting status of core and dynamic agent's connection to Working
		res.ca_status="Working";
	} else if(req.da_status == "Error") {
		// Handle error of dynamic agent. Kill the process dynamicAgentPid_
		res.ca_status="Error";
	} else {
		res.ca_status="Status was not recognized";
	}
	return true;
}

void sigint_signal (int param) {
	ros::shutdown();
}

int main(int argc, char **argv) {
	// SIGINT - signal handler
	void (*prev_handler)(int);
	prev_handler = signal (SIGINT, sigint_signal);

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "core_agent");
	ros::NodeHandle nh;

	CoreAgent coreAgent_;

	// Let ROS take over and execute callbacks.
	ros::spin();
	return 0;
}
