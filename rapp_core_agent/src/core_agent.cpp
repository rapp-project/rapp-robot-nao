
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
#include <sys/types.h>
#include <sys/wait.h>

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


#define SILENT



// Name of the ROS topic for receiving requests.
#define REQUEST_TOPIC "/rapp_core_agent/store_interaction/request"
// Name of the ROS topic for publishing requests.
#define RESPONSE_TOPIC "/rapp_core_agent/store_interaction/response"
// Name of the ROS topic for receiving status of dynamic agent
#define RESPONSE_STATUS_TOPIC "dynamic_agent_status"
// Name of servive used to recognized word
#define RECOGNIZEDWORD "rapp_get_recognized_word"

#define SERVICE_SAY "rapp_say"

class CoreAgent;

CoreAgent * core_agent_ptr = NULL;


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
	ExecuteDACommand,
	ActivateHOP,
	ActivateCPP,
	WaitForHOP,
	WaitForCPP,
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
	
	bool state_activate_hop();
	bool state_activate_cpp();
	
	bool state_wait_hop();
	bool state_wait_cpp();

	void finish_ros();

	bool run();

	// Runs the script which launches dynamic agent
	void runScript(std::string path);
	
	// Runs the binary and returns process pid
	pid_t runBinary(std::string path);

	// A callback function. Executes each time a new response
	// message from HOP arrives.
	void responseReceived(const std_msgs::String& msg);

	// A callback function. Executes each time a dynamic agent status arrives
	bool dynamicAgentStatusReceived(rapp_core_agent::DynamicAgentStatus::Request &req, rapp_core_agent::DynamicAgentStatus::Response &res);

	// pid of C++ RApp
	pid_t cpp_pid;
	std::string entry_point;

protected:
	ros::NodeHandle nh_;

	ros::ServiceClient client_recognize_;
	ros::ServiceClient client_say_;
	
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
	bool finish_requested;
	std::string recognized_word;
	std::string app_name;
	std::string app_path;
	
	int package_wait;
	
	std::string error_msg;
	
	bool da_finished;
};

//Constructor of class CoreAgent
CoreAgent::CoreAgent() {
	next_state = Init;
	finished = false;
	finish_requested = false;
}

bool CoreAgent::run() {	
	while ( (!finished) && (ros::ok()) ) {
		if (finish_requested) next_state = Unregister;
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
		case ActivateCPP:
			state_activate_cpp();
			break;
		case ActivateHOP:
			state_activate_hop();
			break;
		case WaitForCPP:
			state_wait_cpp();
			break;
		case WaitForHOP:
			state_wait_hop();
			break;
		}
		
		ros::spinOnce();
	}
}

bool CoreAgent::state_init() {
	// Wait for service rapp_get_recognized_word
	ros::service::waitForService(RECOGNIZEDWORD);

	// Getting dictionary from rosparam server
	ros::param::get("applications", applications_);
	for (std::map<std::string,std::string>::iterator it=applications_.begin(); it!=applications_.end(); ++it) {
		words_.push_back(it->first);
		std::cout << "Known keyword: " << it->first << std::endl;
	}

	// Create a client for the rapp_get_recognizes_word serivce
	client_recognize_ = nh_.serviceClient<rapp_ros_naoqi_wrappings::RecognizeWord>(RECOGNIZEDWORD);
	
	client_say_ = nh_.serviceClient<rapp_ros_naoqi_wrappings::Say>(SERVICE_SAY);

	// Create a client for the rapp_get_recognizes_word serivce
	serverSetStatus_ = nh_.advertiseService(RESPONSE_STATUS_TOPIC, &CoreAgent::dynamicAgentStatusReceived, this);

	// Create a subscriber object.
	subHopCommunication_ = nh_.subscribe(RESPONSE_TOPIC, 100, &CoreAgent::responseReceived, this);

	// Create a publisher object.
	pub_ = nh_.advertise<std_msgs::String>(REQUEST_TOPIC, 100);

	//
	next_state = Register;
	
	cpp_pid = -1;

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

	if (client_recognize_.call(srv)) {
		recognized_word = srv.response.recognizedWord;
		ROS_INFO("Word recognized: %s", recognized_word.c_str());
	} else {
		ROS_ERROR("Failed to call service rapp_get_recognizes_word");
		return false;
	}
	
	if (recognized_word != "Empty") {
		next_state = Interpret;
	}

	return true;
}

bool CoreAgent::state_interpret() {
	if (recognized_word == "exit") {
		next_state = Unregister;
	} else {
		// check, whether application for given keyword exists
		// note: in general, NaoQI should not recognize words 
		// that are not in provided dictionary
		if (applications_.count(recognized_word) < 1) {
			error_msg = "Sorry, I don't know command ";
			error_msg += recognized_word;
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
	
	rapp_ros_naoqi_wrappings::Say srv;
	bool successful=false;
	//## slower and lower voice
	std::string sentence;
	sentence = "\\RSPD=" + std::string("100") + "\\ ";
	sentence += "\\VCT="+ std::string("60") + "\\ ";
	sentence += error_msg;
	sentence += "\\RST\\ ";
	ROS_INFO("Saying sentence: %s", error_msg.c_str());

#ifndef SILENT
	srv.request.request = sentence;//a message, that will be said
	srv.request.language = "English";//language selection

	client_say_.call(srv);
#endif
	
	next_state = Listen;
	return true;
}

bool CoreAgent::state_unregister() {
	ROS_INFO("State::Unregister");
	
	rapp_ros_naoqi_wrappings::Say srv;
	bool successful = false;
	//## slower and lower voice
	std::string sentence;
	sentence = "\\RSPD=" + std::string("100") + "\\ ";
	sentence += "\\VCT="+ std::string("100") + "\\ ";
	sentence += "Bye!";
	sentence += "\\RST\\ ";

	srv.request.request = sentence;//a message, that will be said
	srv.request.language = "English";//language selection

	client_say_.call(srv);
	
	finish_requested = false;
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
			error_msg = "Sorry, I can't download this package.";
			next_state = Inform;
		} else {		
			ros::Duration(0.5).sleep();
			next_state = WaitForDAPackage;
		}
	} else if (path == "") {
		// download failed
		ROS_ERROR("Download failed");
		error_msg = "Sorry, package download failed.";
		next_state = Inform;
	} else {
		// activate DA
		next_state = ActivateCPP;
		da_finished = false;
	}

	return true;
}

bool CoreAgent::state_activate_dynamic() {
	ROS_INFO("State::ActivateDA");
	
	// Build the path to the 'run' script of the application.
	std::string path = app_path;
	path.append("/run");

	ROS_INFO("Running app from: %s", app_path.c_str());
	runScript(path);
	

	next_state = WaitForDACommand;

	return true;
}

bool CoreAgent::state_destroy_dynamic() {
	ROS_INFO("State::DestroyDA");
	next_state = Listen;
	return true;
}

bool CoreAgent::state_wait_for_dynamic_command() {
	ROS_INFO("State::WaitForDACommand");
	
	if (da_finished) {
		next_state = DestroyDA;
		da_finished = false;
	} else {
		next_state = WaitForDACommand;
		ros::Duration(1.0).sleep();
	}
	return true;
}

bool CoreAgent::state_execute_dynamic_command() {
	ROS_INFO("State::ExecuteDACommand");
	next_state = WaitForDACommand;
	return true;
}

bool CoreAgent::state_activate_cpp() {
	ROS_INFO("State::ActivateCPP");
	
	entry_point = "a";
	// Build the path to the entry point of the application.
	std::string path = app_path + "/../" + entry_point;

	ROS_INFO("Running app from: %s", path.c_str());
	cpp_pid = runBinary(path);
	ROS_INFO("App %d is running", cpp_pid);

	next_state = WaitForCPP;

	return true;
}

bool CoreAgent::state_activate_hop() {
	next_state = DestroyDA;
	/*std_msgs::String msg;
	msg.data = app_name;
	pub_.publish(msg);*/
}

bool CoreAgent::state_wait_cpp() {
	ROS_INFO("State::WaitForCPP %d", cpp_pid);
	
	int status;
	if (!waitpid(cpp_pid, &status, WNOHANG)) {
		ROS_INFO("Status: %d", status);
		sleep(1);
	} else {
		if (WIFEXITED(status)) ROS_INFO("RApp %d ended", cpp_pid);
		if (WCOREDUMP(status)) ROS_INFO("RApp %d segfaulted", cpp_pid);
		next_state = DestroyDA;
	}
}

bool CoreAgent::state_wait_hop() {
	next_state = DestroyDA;
}








// Runs the script which launches dynamic agent
void CoreAgent::runScript(std::string path) {
	pid_t pid = fork();
	if (pid == 0) {
		// child process
		execl("/bin/bash", "bash", path.c_str(), (char*)0);
	} else if (pid > 0) {
		// parent process
		ROS_INFO("Spawned process: %d", pid);
	} else {
		// fork failed
		ROS_INFO("fork() failed!");
		return ;
	}
	return;
}

// Runs the script which launches dynamic agent
pid_t CoreAgent::runBinary(std::string path) {
	pid_t pid = fork();
	if (pid == 0) {
		// child process
		execl("/home/nao/RAPPCache/a", "a", (char*)0);
		ROS_INFO("Something went wrong");
		_Exit(EXIT_FAILURE);
	} else if (pid > 0) {
		// parent process
		ROS_INFO("Spawned process: %d", pid);
	} else {
		// fork failed
		ROS_INFO("fork() failed!");
		return pid;
	}
	
	return pid;
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
	ROS_INFO("[Core agent] - Dynamic agent [%d] status received: %s", req.pid, req.da_status.c_str());
	if(req.da_status == "Init") {
		// Setting status of core and dynamic agent's connection to Initialized
		res.ca_status="Initialized";
	} else if(req.da_status == "Finished") {
		// Kill process of dynamic agent
		// todo - is it necessary?
		res.ca_status="Finished";
		da_finished = true;
	} else if(req.da_status == "Working") {
		// Setting status of core and dynamic agent's connection to Working
		res.ca_status="Working";
	} else if(req.da_status == "Error") {
		// Handle error of dynamic agent. Kill the process dynamicAgentPid_
		res.ca_status="Error";
	} else {
		res.ca_status="Working";
	}
	return true;
}

void CoreAgent::finish_ros() {
	finish_requested = true;
}



void sigint_signal (int param) {
	core_agent_ptr->finish_ros();
}

int main(int argc, char **argv) {
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "core_agent");
	ros::NodeHandle nh;
	
	// SIGINT - signal handler
	void (*prev_handler)(int);
	prev_handler = signal (SIGINT, sigint_signal);

	CoreAgent coreAgent_;
	core_agent_ptr = &coreAgent_;
	coreAgent_.run();
	
	return 0;
}
