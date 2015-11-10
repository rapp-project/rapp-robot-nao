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
	
	cv::Mat captureImage(std::string cameraId, int cameraResolution); /*
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
	
	cv::Mat getTransform(std::string chainName, int space);/*
	Input:
		chainName: Name of the item. Could be: any joint or chain or sensor.
		space: Task frame {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2} 
	Output: The matrix, which contains homogeneous transform relative to the space (frame). Axis definition: the x axis is positive toward the robot’s front, the y from right to left and the z is vertical.
	Description. This function computes the transformation matrix from one frame to another (e.g. from camera frame to robot frame).
	*/
	
	
	ros::ServiceClient client_faceDetect;
	
	std::vector< std::vector <float> > faceDetect(cv::Mat &image, std::string cameraId, int cameraResolution); // Description: Given an RGB image, camera identifier and camera resolution. It detects human faces in the image. Provides a detection of all visible faces. As the output, for each detected face, the position of the center of the face is given and the face size in relation to the image.
	double camera_top_matrix_3[3][3]; double camera_top_matrix_2[3][3]; double camera_top_matrix_1[3][3]; // camera intinsic matrix
	float landmarkTheoreticalSize; //# QRcode real size in meters
	
	template<typename _Tp> static  std::vector<std::vector<_Tp> > toVec(const cv::Mat_<_Tp> matIn);
	template<typename _Tp> static  cv::Mat toMat(const std::vector<std::vector<_Tp> > vecIn);
	rapp::object::QRCode3D qrCodeDetection(cv::Mat &cv_frame, cv::Mat &robotToCameraMatrix);
	
};	
	
} // namespace robot
} // namespace rapp


//######################################################################
//######################################################################

/*
namespace rappPlatform {
namespace robot {

class VisionDynImpl {

public:

	VisionDynImpl (int argc, char **argv);
	~VisionDynImpl();
	
	ros::ServiceClient client_faceDetect;

	ros::NodeHandle *n;
	
	std::vector< std::vector <double> > faceDetect(cv::Mat &image, std::string cameraId, int cameraResolution); // Description: Given an RGB image, camera identifier and camera resolution. It detects human faces in the image. Provides a detection of all visible faces. As the output, for each detected face, the position of the center of the face is given and the face size in relation to the image.
	

	struct QRcodeDetection //structure for the qrCodeDetection
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
	
	//struct rapp::object::QRCode3D qrCodeDetection(cv::Mat &cv_frame, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix);
	
	Input: Image – the RGB image; libraryFun() – a pointer to external function (e.g. in the ZBar library),  robot to camera transposition matrix,
	Output: A structure, which contains: number of detected QR-codes, vector of QR-code messages - QR-code messages, vector of coordinates vector<pair<float, float> >, in camera coordinate system, vector of coordinates (vector<pair<float, float> >) in the robot coordinate system.
	Description: Given an RGB image, it detects QR-codes. The results are: the number of detected QR-codes, messages contained in the QR-codes, localization matrices in the camera coordinate system, localization matrices in the robot coordinate system. 
	
};	
	
} // namespace robot
} // namespace rappPlatform
*/
