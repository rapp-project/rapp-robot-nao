
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

enum State {
	Init,
	Register,
	Listen,
	Interpret,
	Inform,
	Unregister,
	Finish,
	LoadDA,
	WaitForDAPackage,
	ActivateDA,
	DestroyDA,
	WaitForDACommand,
	ExecuteDACommand
};

public:
	//Constructor of class CoreAgent
	CoreAgent();

	//Destructor of CoreAgent class
	~CoreAgent() {}

	bool state_init();
	bool state_register();
	bool state_listen();
	bool state_interpret();
	bool state_inform();
	bool state_unregister();
	bool state_finish();
	bool state_load_dynamic();
	bool state_wait_for_dynamic();
	bool state_activate_dynamic();
	bool state_destroy_dynamic();
	bool state_wait_for_dynamic_command();
	bool state_execute_dynamic_command();

	bool run();

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

	State next_state;

	bool finished;
	std::string recognized_word;
	std::string app_name;
	std::string app_path;
	
	int package_wait;
};

//Constructor of class CoreAgent
CoreAgent::CoreAgent() {
	next_state = Init;
	finished = false;
}

bool CoreAgent::run() {	
	while ( (!finished) && (ros::ok()) ) {
		switch(next_state) {
		case Init:
			state_init();
			break;
		case Register:
			state_register();
			break;
		case Listen:
			state_listen();
			break;
		case Interpret:
			state_interpret();
			break;
		case Inform:
			state_inform();
			break;
		case Unregister:
			state_unregister();
			break;
		case Finish:
			state_finish();
			break;
		case LoadDA:
			state_load_dynamic();
			break;
		case WaitForDAPackage:
			state_wait_for_dynamic();
			break;
		case ActivateDA:
			state_activate_dynamic();
			break;
		case DestroyDA:
			state_destroy_dynamic();
			break;
		case WaitForDACommand:
			state_wait_for_dynamic_command();
			break;
		case ExecuteDACommand:
			state_execute_dynamic_command();
			break;
		}
		
		ros::spinOnce();
	}
}

bool CoreAgent::state_init() {
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

	//
	next_state = Register;

	return true;
}

bool CoreAgent::state_register() {
	next_state = Listen;
	return true;
}

bool CoreAgent::state_listen() {
	ROS_INFO("State::Listen");

	rapp_ros_naoqi_wrappings::RecognizeWord srv;
	srv.request.wordsList=words_;

	if (client_.call(srv)) {
		recognized_word = srv.response.recognizedWord;
		ROS_INFO("Word recognized: %s", recognized_word.c_str());
	} else {
		ROS_ERROR("Failed to call service rapp_get_recognizes_word");
		return false;
	}

	if (recognized_word != "Empty") {
		next_state = Interpret;
	} else {
		next_state = Listen;
	}

	return true;
}

bool CoreAgent::state_interpret() {
	if (recognized_word == "exit") {
		next_state = Unregister;
	} else {
		// check, whether application for given keyword exists
		// note: in general, NaoQI should not recognize words 
		if (applications_.count(recognized_word) < 1) {
			next_state = Inform;
			return true;
		}

		// get app name associated to detected word
		app_name = applications_[recognized_word];
		ROS_INFO("Requested application: %s", app_name.c_str());
		next_state = LoadDA;
	}
	return true;
}

bool CoreAgent::state_inform() {
	ROS_INFO("State::Inform");
	next_state = Listen;
	return true;
}

bool CoreAgent::state_unregister() {
	ROS_INFO("State::Unregister");
	next_state = Finish;
	return true;
}

bool CoreAgent::state_finish() {
	ROS_INFO("State::Finish");
	finished = true;
	ros::shutdown();
	return true;
}

bool CoreAgent::state_load_dynamic() {
	ROS_INFO("State::LoadDA");
	
	app_path = "X";
	package_wait = 0;
	std_msgs::String msg;
	msg.data = app_name;
	pub_.publish(msg);

	next_state = WaitForDAPackage;

	return true;
}

bool CoreAgent::state_wait_for_dynamic() {
	ROS_INFO("State::WaitForDAPackage");
	std::string path = app_path;
	if (path == "X") {
		// still waiting
		++package_wait;
		
		if (package_wait > 10) {
			ROS_WARN("Waiting for package - timed out.");
			next_state = Inform;
		} else {		
			ros::Duration(0.5).sleep();
			next_state = WaitForDAPackage;
		}
	} else if (path == "") {
		// download failed
		ROS_ERROR("Download failed");
		next_state = Inform;
	} else {
		// activate DA
		next_state = ActivateDA;
	}

	return true;
}

bool CoreAgent::state_activate_dynamic() {
	ROS_INFO("State::ActivateDA");
	
	// Build the path to the 'run' script of the application.
	std::string path = app_path;
	app_path.append("/run");

	ROS_INFO("Running app from: %s", app_path.c_str());
	//runScript(path);
	next_state = Inform;

	//next_state = WaitForDACommand;
	return true;
}

bool CoreAgent::state_destroy_dynamic() {
	ROS_INFO("State::DestroyDA");
	next_state = Listen;
	return true;
}

bool CoreAgent::state_wait_for_dynamic_command() {
	ROS_INFO("State::WaitForDACommand");
	next_state = DestroyDA;
	return true;
}

bool CoreAgent::state_execute_dynamic_command() {
	ROS_INFO("State::ExecuteDACommand");
	next_state = WaitForDACommand;
	return true;
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
	app_path = msg.data;
	ROS_INFO("Got sth! %s", app_path.c_str());
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
		//sendRequest();
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
/*	void (*prev_handler)(int);
	prev_handler = signal (SIGINT, sigint_signal);*/

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "core_agent");
	ros::NodeHandle nh;

	CoreAgent coreAgent_;
	coreAgent_.run();

	// Let ROS take over and execute callbacks.
	return 0;
}
