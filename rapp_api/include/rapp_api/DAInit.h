#include "ros/ros.h"
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

#include<rapp_core_agent/DynamicAgentStatus.h>


using namespace std;

// Class NaoDAStatus sends status of dynamic agent to core agent. Core agent sends back as a response a desired status of core agent.
class DAInit {

private: 
	// Core agent sends as a response a status of dynamic agent
	string ca_status;
public:
	~DAInit();
	DAInit (int argc, char **argv);

	ros::ServiceClient client_status;

	ros::NodeHandle *n;

	void init(int argc, char **argv);
	
	// Sends status of DA - dynamic agent to CA - core agent
	void sendDAStatus(string status);

	// Returns ca_status
	string getCAStatus();
};
