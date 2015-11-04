#include "ros/ros.h"
#include <iostream>
class NaoNavigation {

public:
	NaoNavigation (int argc, char **argv);
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveHead;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_removeStiffness;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_takePredefinedPosture;
	ros::NodeHandle *n;

void init(int argc, char **argv);
	bool moveTo(float x, float y, float theta);
	bool moveVel(float x, float y, float theta);
	//void moveHead(float yaw,float pitch);
	bool moveStop();
	bool moveJoint(std::vector<std::string> joint, std::vector<float> angle);
	//void removeStiffness(std::string joint);
	bool takePredefinedPosture(std::string posture);
	bool rest();
	bool moveAlongPath(rapp::objects::Path path);
	rapp::objects::Pose getRobotPosition();
	bool globalLocalization(rapp::objects::Pose pose);
	rapp::objects::Path PathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map);
	rapp::objects::Pose QRcodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap);

};
