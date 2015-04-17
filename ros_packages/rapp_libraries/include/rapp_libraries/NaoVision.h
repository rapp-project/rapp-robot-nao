//#####################
// written by Jan Figat
// just to get an image from NAO robot
//#####################

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rapp_robot_agent/GetImage.h"


#include <stdio.h>
#include <fstream>

/*#include <cstdio>
#include <ostream>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>*/

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class NaoVision {

public:
	NaoVision (int argc, char **argv);
	ros::ServiceClient client_captureImage;
	ros::ServiceClient client_getTransform;

	ros::NodeHandle *n_;

	//#################---new---#################
	cv_bridge::CvImagePtr cv_ptr_;
	bool is_image_arrived_;
	int iteration_flag_;
	//############################################3
	
	std::string NAO_IP;
	int NAO_PORT;

	void init(int argc, char **argv);
	sensor_msgs::Image captureImage(std::string cameraId);// For frame
    /*
    ##############################################################################
    PUBLISHING image to topic - add this function to the target library NaoVision
	##############################################################################
	*/
    void publishImage(cv_bridge::CvImagePtr image_ptr, std::string topic);
    cv_bridge::CvImagePtr getImageFromTopic(std::string topic);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};


