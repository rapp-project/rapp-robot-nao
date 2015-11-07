#include "ros/ros.h"
#include "rapp_ros_naoqi_wrappings/MoveTo.h"
#include "rapp_ros_naoqi_wrappings/MoveVel.h"
#include "rapp_ros_naoqi_wrappings/MoveHead.h"
#include "rapp_ros_naoqi_wrappings/MoveStop.h"
#include "rapp_ros_naoqi_wrappings/MoveGetCollisionStatus.h"
#include "rapp_ros_naoqi_wrappings/UpdatePose.h"
#include "rapp_ros_naoqi_wrappings/GetPose.h"
#include "rapp_ros_naoqi_wrappings/GetPlan.h"
#include "rapp_ros_naoqi_wrappings/MoveJoint.h"
#include "rapp_ros_naoqi_wrappings/RemoveStiffness.h"
#include "rapp_ros_naoqi_wrappings/TakePredefinedPosture.h"
#include "rapp_ros_naoqi_wrappings/LookAtPoint.h"
#include <rapp/robot/Navigation.hpp>
//#include <iostream.h>
namespace rapp {
namespace robot {

class NavigationImpl {

public:

	NavigationImpl (int argc, char **argv);
	~NavigationImpl();
	
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveHead;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_removeStiffness;
	ros::ServiceClient client_takePredefinedPosture;
	ros::ServiceClient client_visOdom;
	ros::ServiceClient client_lookAtPoint;
	ros::NodeHandle *n;
	
	bool moveTo(float x, float y, float theta);
	bool moveVel(float x, float y, float theta);
	// bool moveHead(float yaw,float pitch);
	bool moveStop();
	bool moveJoint(std::vector<std::string> joint, std::vector<float> angle);
	// bool removeStiffness(std::string joint);
	bool takePredefinedPosture(std::string posture);
	// bool visOdom();
	bool lookAtPoint(float x, float y, float z);
	bool rest(std::string posture);
	bool moveAlongPath(rapp::objects::Path path);
	rapp::objects::PoseStamped getRobotPose();
	bool setGlobalPose(rapp::objects::Pose pose);
	// rapp::objects::Path pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map);
 //    rapp::objects::Pose qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap);
	
};	
	
} // namespace robot
} // namespace rapp
