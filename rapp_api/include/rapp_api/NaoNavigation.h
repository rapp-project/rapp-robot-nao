#include "ros/ros.h"
#include <iostream>
class NaoNavigation {

public:
	NaoNavigation (int argc, char **argv);
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveHead;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_removeStiffness;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_takePredefinedPosture;
	ros::NodeHandle *n;

void init(int argc, char **argv);
	void moveVel(float x, float y, float theta);
	void moveHead(float yaw,float pitch);
	void moveStop();
	void moveJoint(std::vector<std::string> joint, std::vector<float> angle);
	void removeStiffness(std::string joint);
	void takePredefinedPosture(std::string pose);
};
