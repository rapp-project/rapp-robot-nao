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
	
	void Navigation::moveTo(float x, float y, float theta);
	void Navigation::moveVel(float x, float y, float theta);
	void Navigation::moveHead(float yaw,float pitch);
	void Navigation::moveStop();
	void Navigation::moveJoint(std::string joint, float angle);
	void Navigation::removeStiffness(std::string joint);
	void Navigation::takePredefinedPosture(std::string pose);
	void Navigation::visOdom();
	void Navigation::lookAtPoint(float x, float y, float z);
	
};	
	
} // namespace robot
} // namespace rapp
