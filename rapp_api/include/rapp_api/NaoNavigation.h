#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <opencv>
#include "../"
class NaoNavigation {

public:
	NaoNavigation (int argc, char **argv);
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_takePredefinedPosture;
	ros::ServiceClient client_rest;
	ros::ServiceClient client_moveAlongPath;
	ros::ServiceClient client_getRobotPose;
	ros::ServiceClient client_setGlobalPose;
	// ros::ServiceClient client_pathPlanner_2D;
	// ros::ServiceClient client_qrCodeLocalization;
	ros::NodeHandle *n;

void init(int argc, char **argv);
	bool moveTo(float x, float y, float theta);
	bool moveVel(float x, float y, float theta);
	bool moveStop();
	bool moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speeds);
	bool takePredefinedPosture(std::string posture);
	bool rest(std::string posture);
	bool moveAlongPath(rapp::objects::Path path);
	rapp::objects::PoseStamped getRobotPose();
	bool setGlobalPose(rapp::objects::Pose pose);
	// rapp::objects::Path pathPlanner_2D(std::string algorithm, rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map);
	// rapp::objects::Pose qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap);

};
