
// core_agent.cpp
// An example of RAPP core agent functionality: downloading 
// and running applications from the cloud (RAPP store)
//
// Marcin Szlenk <m.szlenk@elka.pw.edu.pl>
// Maksym Figat <maksym.figat44@gmail.com>
// Copyright 2014 RAPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rapp_ros_naoqi_wrappings/RecognizeWord.h"
#include "rapp_ros_naoqi_wrappings/Say.h"

#include "rapp_core_agent/Status.h"

#include <stdio.h>
#include <ostream>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>

// Name of the ROS topic for receiving requests.
#define REQUEST_TOPIC "/rapp_core_agent/store_interaction/request"
// Name of the ROS topic for publishing requests.
#define RESPONSE_TOPIC "/rapp_core_agent/store_interaction/response"
// Name of the ROS topic for receiving status of dynamic agent
#define RESPONSE_STATUS_TOPIC "/rapp_core_agent/dynamic_agent/status"




class CoreAgent{
	
public:
		//Constructor of class CoreAgent
		CoreAgent();
		
		//Destructor of CoreAgent class
		~CoreAgent(){}
		
		// Ask for the name of an application to download and publish the
		// request to HOP, where:
		//
		// p - a pointer to the ROS publisher object; must be a non-NULL
		//     on the first call to the function and is ignored on the
		//     subsequent calls.
		void sendRequest(ros::Publisher *p );

		// Gets commands
		std::string GetCommand();
		
	
		// Runs the script which launches dynamic agent
		void runScript(std::string path);

		// A callback function. Executes each time a new response
		// message from HOP arrives.
		void responseReceived(const std_msgs::String& msg);
		
		// A callback function. Executes each time a dynamic agent status arrives
		void dynamicAgentStatusReceived(const rapp_core_agent::Status &msg);

protected:
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
		ros::ServiceClient serverSetStatus_;
		ros::Subscriber subHopCommunication_;
		ros::Subscriber subDynamicAgentStatus_;
		ros::Publisher pub_;
		// PID number of dynamic agent. This value is set at the state of activation of dynamic agent.
		// After task is finished or error occurs core agent destroys dynamic agent. 
		int dynamicAgentPid_;
};

	//Constructor of class CoreAgent
	CoreAgent::CoreAgent(){
		// Create a client for the rapp_get_recognizes_word serivce
		client_ = nh_.serviceClient<rapp_ros_naoqi_wrappings::RecognizeWord>("rapp_get_recognizes_word");
			
		// Create a subscriber object.
		subHopCommunication_ = nh_.subscribe(RESPONSE_TOPIC, 100, &CoreAgent::responseReceived, this);
		
		// Create a subscriber object to a topic RESPONSE_STATUS_TOPIC
		subDynamicAgentStatus_ = nh_.subscribe(RESPONSE_STATUS_TOPIC, 100, &CoreAgent::dynamicAgentStatusReceived, this);
			
		// Create a publisher object.
		pub_ = nh_.advertise<std_msgs::String>(REQUEST_TOPIC, 100);
			
		// Send the initial request.
		sendRequest(&pub_);
	}
	
	
		
	// Ask for the name of an application to download and publish the
	// request to HOP, where:
	//
	// p - a pointer to the ROS publisher object; must be a non-NULL
	//     on the first call to the function and is ignored on the
	//     subsequent calls.
	void CoreAgent::sendRequest(ros::Publisher *p = NULL)
	{
		static ros::Publisher& pub = *p;

		std::string name;
		// Entering name of package from code
		//std::cout << "Enter the name of a RAPP application (or \'q' to quit): ";
		//std::cin >> name;
			
		// Enter the name of package using voice command
		name=GetCommand();
		

		if(name == "q")
		{
			// Shut down the node.
			ros::shutdown();
		}
		else
		{
			std_msgs::String msg;
			if(name == "rapp")
				name.append("_email-1.0.0.hz");
			else if(name == "email")
				name= "rapp_email-1.0.0.hz";
			
			// Publish a message with the application name.
			
			msg.data = name;
			pub.publish(msg);
			std::cout << "Downloading...\n";
		}
	}


	
	const std::vector<std::basic_string <char> > getVector(std::string dictionary [], int size)
	{
		int i=0;
		std::vector<std::basic_string<char> > tab;
		for (i=0;i<size;i++){
			tab.push_back(dictionary[i].c_str());
			std::cout<<tab[i]<<std::endl;
		}
			
		return tab;
	}

    // Gets commands
    std::string CoreAgent::GetCommand()
	{
		std::cout << "Get command\n";
		std::string dictionary_ [3] ={"rapp", "email", "exit"};
		rapp_ros_naoqi_wrappings::RecognizeWord srv;
		
		srv.request.wordsList=getVector(dictionary_,3);
		
		
		if (client_.call(srv))
		{
			//ROS_INFO("Word recognized: %s", srv.response.recognizedWord);
			std::cout<<srv.response.recognizedWord;
			return srv.response.recognizedWord;
		}
		else
		{
			//ROS_ERROR("Failed to call service rapp_get_recognizes_word");
			std::cout<<"Error\n";
			return "Error";
		}
		return "Error";//srv.response.recognizedWord;
	}
	
	// Runs the script which launches dynamic agent
	void CoreAgent::runScript(std::string path)
	{
		int counter = 0;
		std::string new_path="/home/nao/";
		new_path.append(path.substr(1));
		std::cout<<new_path;
		pid_t pid = fork();
		if (pid == 0)
		{
			// child process
			execl("/bin/bash", "bash", new_path.c_str(), NULL);
		}
		else if (pid > 0)
		{
			// parent process
		}
		else
		{
			// fork failed
			std::cout << "fork() failed!\n";
			return ;
		}
		return;
	}


	// A callback function. Executes each time a new response
	// message from HOP arrives.
	void CoreAgent::responseReceived(const std_msgs::String& msg)
	{
		// Response from HOP.
		std::string path = msg.data;

		if(path.empty())
		{
			std::cout << "\e[1m\e[31m" // bold and red font
				<< "Failed"
				<< "\e[0m\n"; // normal print mode
		}
		else
		{
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
	void CoreAgent::dynamicAgentStatusReceived(const rapp_core_agent::Status &msg)
	{
		std::cout << "[Core agent] - Dynamic agent status received\n";
		std::cout << "[Core agent] - Status = "<<msg.status<<"\n";
		if(msg.status == "Init")
		{
			dynamicAgentPid_ = msg.pid;
		}
		else if(msg.status == "Finished")
		{
			// Kill process of dynamic agent
			// todo - is it necessary?
			// Ask for the next application's name.
			sendRequest();
		}
		else if(msg.status == "Working")
		{
			
		}
		else if(msg.status == "Error")
		{
			// Handle error of dynamic agent. Kill the process dynamicAgentPid_
			// todo
		}
		
		
	}



int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "core_agent");
    ros::NodeHandle nh;
    
    CoreAgent coreAgent_;
    
    // Let ROS take over and execute callbacks.
    ros::spin();
    return 0;
}