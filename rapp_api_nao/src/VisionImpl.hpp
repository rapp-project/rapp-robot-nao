#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParam.h"
#include "rapp_ros_naoqi_wrappings/GetTransform.h"
#include "rapp_ros_naoqi_wrappings/FaceDetect.h"

#include <rapp/robot/Vision.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <zbar.h> // for QRcode detection - dynamic agent
//#include "../../../../../objects/QRCode3D/QRCode3D.hpp"

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
	
	rapp::object::picture captureImage(std::string cameraId, int cameraResolution); /*
	Input: 
		cameraId: camera identifier,
		cameraResolution: camera resolution.
	Output: image (e.g. stored RGB image)
	Description: This function captures an image frame from the robot’s camera. The resolution of the captured image is set to cameraResolution. The returned color image is a kBGRColorSpace. The frame rate of the camera is set to 15 fps.
	*/
	
	bool setCameraParams(int cameraId, int cameraParameterId, int newValue );/*
	Input: 
		cameraId: the camera  identifier,
		cameraParameterId: the parameter  identifier,
		newValue: the desired parameter value.
	Output: result of operation – “failed/succeeded” (of type Boolean).
	Description. It sets acquisition parameters of camera device (exposure time, camera resolution, color space etc.). This function is required by light checking behaviour. The most important parameters (available for cameras mounted on Nao robot) are presented the Annex (adapted from Aldebaran documentation for software version 2.1).
	*/
	
	rapp::object::Matrix2D getTransform(std::string chainName, int space);/*
	Input:
		chainName: Name of the item. Could be: any joint or chain or sensor.
		space: Task frame {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2} 
	Output: The matrix, which contains homogeneous transform relative to the space (frame). Axis definition: the x axis is positive toward the robot’s front, the y from right to left and the z is vertical.
	Description. This function computes the transformation matrix from one frame to another (e.g. from camera frame to robot frame).
	*/
	
};	
	
} // namespace robot
} // namespace rapp


//######################################################################
//######################################################################

/**/
namespace rappPlatform {
namespace robot {

class VisionDynImpl {

public:

	VisionDynImpl (int argc, char **argv);
	~VisionDynImpl();
	
	ros::ServiceClient client_faceDetect;
	ros::ServiceClient client_captureImage;

	ros::NodeHandle *n;

	double camera_top_matrix_3[3][3];
	double camera_top_matrix_2[3][3];
	double camera_top_matrix_1[3][3]; // camera intinsic matrix
	const float landmarkTheoreticalSize=0.16; //# QRcode real size in meters

	std::vector< std::vector <float> > faceDetect(rapp::object::picture image, std::string cameraId, int cameraResolution); // Description: Given an RGB image, camera identifier and camera resolution. It detects human faces in the image. Provides a detection of all visible faces. As the output, for each detected face, the position of the center of the face is given and the face size in relation to the image.
	
	//rapp::object::QRCode3D qrCodeDetection(rapp::object::picture image_, rapp::object::Matrix4x4 robotToCameraMatrix, float landmarkTheoreticalSize = 0.16f);
	rapp::object::QRCode3D qrCodeDetection(rapp::object::picture imgFrame, cv::Mat robotToCameraMatrix, float landmarkTheoreticalSize);
	
	
};	
	
} // namespace robot
} // namespace rappPlatform
//*/
