#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParam.h"
#include "rapp_ros_naoqi_wrappings/GetTransform.h"

#include <rapp/robot/Vision.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <zbar.h> // for QRcode detection - dynamic agent

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


//######################################################################
//######################################################################

namespace rappPlatform {
namespace robot {

class VisionImpl {

public:

	VisionImpl (int argc, char **argv);
	~VisionImpl();
	
	ros::ServiceClient client_faceDetect;

	ros::NodeHandle *n;
	
	std::vector< std::vector <double> > faceDetect(cv::Mat image, std::string cameraId, int cameraResolution); // face detection
	

	struct QRcodeDetection
	{
		bool isQRcodeFound;
		int numberOfQRcodes;//number of detected QRcodes
		std::vector< cv::Mat > LandmarkInCameraCoordinate;//Transformation matrix from camera to Landmark
		std::vector< cv::Mat > LandmarkInRobotCoordinate;//Transformation matrix from camera to robot
		std::vector<std::string> QRmessage; //vector for messages from QRcodes

		void clear()
		{
			isQRcodeFound = false;
			numberOfQRcodes = 0;
			LandmarkInCameraCoordinate.clear();
			LandmarkInRobotCoordinate.clear();
			QRmessage.clear();
		}
	};

	double camera_top_matrix_3[3][3]; double camera_top_matrix_2[3][3]; double camera_top_matrix_1[3][3]; // camera intinsic matrix
	float landmarkTheoreticalSize; //# QRcode real size in meters

	struct QRcodeDetection qrCodeDetection(sensor_msgs::Image &frame_, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix); // For QRcode detection
};	
	
} // namespace robot
} // namespace rappPlatform
