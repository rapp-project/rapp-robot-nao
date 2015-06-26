#include "ros/ros.h"

#include "cloud_services/FindObjects.h"
#include "cloud_services/UploadImage.h"
#include "cloud_services/LightCheck.h"

#include <opencv2/opencv.hpp>

class CloudServices {

public:
	CloudServices (int argc, char **argv);
	~CloudServices();
	
	std::string uploadImage(cv::Mat img);
	int lightCheck(const std::string & fname);
	int findObjects(std::string fname, std::vector<std::string> names, std::vector<std::string> files, unsigned int limit, 
	                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores);
	
private:
	ros::ServiceClient client_FindObjects;
	ros::ServiceClient client_UploadImage;
	ros::ServiceClient client_LightCheck;
	
	ros::NodeHandle * n_;
};
