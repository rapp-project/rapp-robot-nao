#include "ros/ros.h"

#include "cloud_services/FindObjects.h"
#include "cloud_services/UploadImage.h"

#include <opencv2/opencv.hpp>

class CloudServices {

public:
	CloudServices (int argc, char **argv);
	~CloudServices();
	
	std::string uploadImage(cv::Mat img);
	
private:
	ros::ServiceClient client_FindObjects;
	ros::ServiceClient client_UploadImage;
	
	ros::NodeHandle * n_;
};
