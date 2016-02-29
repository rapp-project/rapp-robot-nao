#include "ros/ros.h"
#include <iostream>

#include "std_msgs/String.h"

#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParam.h"
#include "rapp_ros_naoqi_wrappings/SetCameraParams.h"
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
	ros::ServiceClient client_setCameraParams;
	ros::ServiceClient client_getTransform;
	ros::ServiceClient client_faceDetect;

	ros::NodeHandle *n;
	
	rapp::object::picture::Ptr captureImage(int camera_id, int camera_resolution, const std::string & encoding); /*
	Input: 
		cameraId: camera identifier,
		cameraResolution: camera resolution.
	Output: image (e.g. stored RGB image)
	Description: This function captures an image frame from the robot’s camera. The resolution of the captured image is set to cameraResolution. The returned color image is a kBGRColorSpace. The frame rate of the camera is set to 15 fps.
	*/
	
	bool setCameraParam(int camera_id, int camera_parameter_id, int new_value );/*
	Modifies camera internal parameter.
	Input: 
		cameraId: the camera  identifier,
		cameraParameterId: the parameter  identifier,
		newValue: the desired parameter value.
	Output: result of operation – “failed/succeeded” (of type Boolean).
	Description. It sets acquisition parameters of camera device (exposure time, camera resolution, color space etc.). This function is required by light checking behaviour. The most important parameters (available for cameras mounted on Nao robot) are presented the Annex (adapted from Aldebaran documentation for software version 2.1).
	
	cameraId: 0-top; 1-bottom;
	cameraParameterId - see Camera parameters [http://doc.aldebaran.com/2-1/family/robots/video_robot.html#cameraparameter-mt9m114]
	Parameter			Min	Value	Max Value	Default Value	Camera parameter ID name	cameraParameterId	Remarks
	Brightness				0			255			55			kCameraBrightnessID					0	Auto Exposition must be enabled
	Contrast				16			64			32			kCameraContrastID					1	The contrast value represents the gradient of the contrast adjustment curve, measured at the target brightness point (as controlled by Brightness parameter). The device supports a range of gradients from 0.5 to 2.0; Device represents this as a contrast range from 16 (0.5) to 64 (2.0).
	Saturation				0			255			128			kCameraSaturationID					2	 
	Hue						-180		180			0			kCameraHueID						3	Disabled
	Gain					32			255			32			kCameraGainID						6	Auto Exposition must be disabled
	Horizontal Flip			0			1			0			kCameraHFlipID						7	 
	Vertical Flip			0			1			0			kCameraVFlipID						8	 
	Auto Exposition			0			1			1			kCameraAutoExpositionID				11	 
	Auto White Balance		0			1			1			kCameraAutoWhiteBalanceID			12	 
	Camera Resolution		kQVGA		k4VGA		kQVGA		kCameraResolutionID					14	Not to be set manually
	Frames Per Second		1			30			5			kCameraFrameRateID					15	Not to be set manually
	Exposure (time[ms]=value/10)	1	2500(250ms)	NA			kCameraExposureID					17	Auto Exposition must be disabled otherwise display last value before auto exposure was activated. Once auto exposure disabled paremeter is set to the last value used by the camera internal algorithm
	Camera Select			0			1			0			kCameraSelectID						18	0: Top Camera; 1: Bottom Camera;
	Reset camera registers	0			1			0			kCameraSetDefaultParamsID			19	1: reset camera and reset parameter to 0
	Auto Exposure Algorithm	0			3			1			kCameraExposureAlgorithmID			22	0: Average scene Brightness; 1: Weighted average scene Brightness; 2: Adaptive weighted auto exposure for hightlights; 3: Adaptive weighted auto exposure for lowlights
	Sharpness				-1			7			0			kCameraSharpnessID					24	-1: disabled
	White Balance (Kelvin)	2700		6500		NA			kCameraWhiteBalanceID				33	Read only if Auto White Balance is enabled Read/Write if Auto White Balance is disabled
	Back light compensation	0			4			1			kCameraBacklightCompensationID		34
	
	*/
	
	std::map<int, bool> setCameraParams(int camera_id, std::vector<unsigned int> camera_parameter_ids, std::vector<unsigned int> new_values );
	
	//##################################################################
	
	//for Dynamic Agent usage
	double camera_top_matrix_3[3][3];
	double camera_top_matrix_2[3][3];
	double camera_top_matrix_1[3][3]; // camera intinsic matrix
	const float landmarkTheoreticalSize=0.16; //# QRcode real size in meters

	std::vector< std::vector <float> > faceDetect(rapp::object::picture image, int camera_id, int camera_resolution); // Description: Given an RGB image, camera identifier and camera resolution. It detects human faces in the image. Provides a detection of all visible faces. As the output, for each detected face, the position of the center of the face is given and the face size in relation to the image.
	
	rapp::object::QRCode3D qrCodeDetection(rapp::object::picture::Ptr imgFrame, std::vector<std::vector<float>> robotToCameraMatrix, double camera_matrix[][3], float landmarkTheoreticalSize = 0.16f);/*
	double camera_matrix[3][3] - camera intrinsic matrix
	*/
	
};	
	
} // namespace robot
} // namespace rapp
