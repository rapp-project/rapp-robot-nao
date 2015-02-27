#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rapp_robot_agent/GetImage.h"//change it for core
#include "rapp_robot_agent/GetTransform.h"//change it for core
#include "sensor_msgs/Image.h"

#include <stdio.h>
#include <fstream>

#include <cstdio>
#include <ostream>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>

#include <zbar.h> // for QRcode detection

using namespace std;
using namespace cv;
using namespace zbar;

class NaoVision {

public:
	NaoVision (int argc, char **argv);
	ros::ServiceClient client_captureImage;
	ros::ServiceClient client_getTransform;
	//ros::NodeHandle *n;
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	
	std::string NAO_IP;// = "nao.local";
	int NAO_PORT;// = 9559;

	//std::vector< std::vector<float> > cameraTopMatrix(3, vector<float>(3,0.0));
	//double camera_top_matrix[3][3] = {{182.0992346/0.16, 0., 658.7582}, {0., 185.0952141/0.16, 484.2186}, {0., 0., 1.}};
	cv::Mat cameraTopMatrix;// = cv::Mat(3, 3, CV_64F, camera_top_matrix);

	struct QRcodeDetection
   {
		bool isQRcodeFound;
		int numberOfQRcodes; //number of detected QRcodes
		std::vector< cv::Mat > LandmarkInCameraCoordinate;//Transformation matrix from camera to Landmark
		std::vector< cv::Mat > LandmarkInRobotCoordinate;//Transformation matrix from camera to robot
		//std::vector< std::vector< std::vector<float> > > LandmarkInCameraCoordinate(0/*empty*/,std::vector<std::vector<float>>(4,std::vector<float>(4,0.0)));//Transformation matrix from camera to Landmark
		//std::vector< std::vector< std::vector<float> > > LandmarkInRobotCoordinate(0/*empty*/,std::vector<std::vector<float>>(4,std::vector<float>(4,0.0)));//Transformation matrix from camera to robot
		std::vector<std::string> QRmessage; //vector for messages from QRcodes	
		//Matrix4x4Camera;
		//Matrix4x4Robot;
   };
	
	struct QRcodeHazardDetection
   {
		bool isHazardFound;
		std::vector<float> hazardPosition_x; // in robot coordinate system - x-axis is directed to the front
		std::vector<float> hazardPosition_y; // in robot coordinate system - y-axis is directed to the left
		std::vector<std::string> openedObject; //message from QRcode of an open object
   };
	
	void init(int argc, char **argv);
	sensor_msgs::Image captureImage(std::string cameraId);	 // For frame capture from selected camera

	cv::Mat getTransform(std::string chainName, int space); // For computing transforamtion matrix from one coordinate system to another

	struct QRcodeDetection qrCodeDetection(sensor_msgs::Image frame_, zbar::ImageScanner set_zbar, /*std::vector< std::vector<float> >*/ cv::Mat robotToCameraMatrix); // For QRcode detection


protected:
	// For QRcode detection:
	const double landmarkTheoreticalSize = 0.16; //# QRcode real size in meters
	const std::string currentQRcodeCamera = "CameraTop";
};
