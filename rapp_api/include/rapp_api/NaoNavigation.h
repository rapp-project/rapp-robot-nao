#include "ros/ros.h"
class NaoNavigation {

public:
	NaoNavigation (int argc, char **argv);

	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveHead;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_moveGetCollisionStatus;
	ros::ServiceClient client_updatePose;
	ros::ServiceClient client_getPose;
	ros::ServiceClient client_getPlan;
	ros::ServiceClient client_removeStiffness;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_takePredefinedPose;
	ros::NodeHandle *n;

void init(int argc, char **argv);
	void moveTo(float x, float y, float theta);
	void moveVel(float x, float y, float theta);
	void moveHead(float yaw,float pitch);
	void moveStop();
	void moveGetCollisionStatus();
	void updatePose();
	void getPose();
	void getPlan();
	void moveJoint(std::string joint);
	void removeStiffness(std::string joint);
	void takePredefinedPose(std::string pose);
};
