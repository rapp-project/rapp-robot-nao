/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


// search vector std::find
#include <algorithm>
// inverse matrix etc.
#include <Eigen/Core>
// ros includes
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
// conversions TF to eigen and eigen to TF
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
//include rapp service headers
#include "rapp_ros_naoqi_wrappings/MoveTo.h"
#include "rapp_ros_naoqi_wrappings/MoveVel.h"
#include "rapp_ros_naoqi_wrappings/MoveStop.h"
#include "rapp_ros_naoqi_wrappings/UpdatePose.h"
#include "rapp_ros_naoqi_wrappings/GetRobotPose.h"
#include "rapp_ros_naoqi_wrappings/SetGlobalPose.h"
#include "rapp_ros_naoqi_wrappings/MoveJoint.h"
#include "rapp_ros_naoqi_wrappings/Rest.h"
#include "rapp_ros_naoqi_wrappings/MoveAlongPath.h"
#include "rapp_ros_naoqi_wrappings/TakePredefinedPosture.h"
#include "rapp_ros_naoqi_wrappings/LookAtPoint.h"
//include rapp-api objects
#include <rapp/objects/path/path.hpp>
#include <rapp/objects/pose/pose.hpp>
#include <rapp/objects/poseStamped/poseStamped.hpp>
 #include <rapp/objects/qrCodeMap/qrCodeMap.hpp>
 #include <rapp/objects/qrCodeMap/qrCodeMap.hpp>


namespace rapp {
namespace robot {

class NavigationImpl {

public:

	NavigationImpl (int argc, char ** argv);
	~NavigationImpl();
	
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_rest;
	ros::ServiceClient client_takePredefinedPosture;
	ros::ServiceClient client_lookAtPoint;
	ros::ServiceClient client_getRobotPose;
	ros::ServiceClient client_setGlobalPose;
	ros::ServiceClient client_moveAlongPath;


	ros::NodeHandle *n;
	
	bool moveTo(float x, float y, float theta);
	bool moveVel(float x, float y, float theta);
	// bool moveHead(float yaw,float pitch);
	bool moveStop();
	bool moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speed);
	// bool removeStiffness(std::string joint);
	bool takePredefinedPosture(std::string posture, float speed);
	// bool visOdom();
	bool lookAtPoint(float x, float y, float z);
	bool rest(std::string posture);
	bool moveAlongPath(rapp::object::Path path);
	rapp::object::PoseStamped getRobotPose();
	bool setGlobalPose(rapp::object::PoseStamped rapp_pose);
	// rapp::objects::Path pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map);
 //    rapp::objects::Pose qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap);
	
};	
} // namespace robot
} // namespace rapp