#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParam.h"
#include "rapp_ros_naoqi_wrappings/GetTransform.h"

#include <rapp/robot/Vision.hpp>

namespace rapp {
namespace robot {

class VisionImpl {

public:

	VisionImpl (int argc, char **argv);
	~VisionImpl();
	
	ros::ServiceClient client_captureImage;
	ros::ServiceClient client_setCameraParam;
	ros::ServiceClient client_getTransform;

	ros::NodeHandle *n;
	
	cv::Mat captureImage(std::string cameraId, int cameraResolution);
	
	bool setCameraParams(int cameraId, int cameraParameterId, int newValue );
	
	cv::Mat getTransform(std::string chainName, int space);
	
};	
	
} // namespace robot
} // namespace rapp
