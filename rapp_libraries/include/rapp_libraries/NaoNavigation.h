#include "ros/ros.h"
class NaoNavigation {

public:
	NaoNavigation (int argc, char **argv);
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::NodeHandle *n;

void init(int argc, char **argv);
	void moveTo(float x, float y, float theta);
	void moveVel(float x, float y, float theta);
	void moveHead(float roll, float pitch, float yaw);


};
