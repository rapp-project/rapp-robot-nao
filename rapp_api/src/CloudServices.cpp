#include "rapp_api/CloudServices.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

CloudServices::CloudServices(int argc,char **argv)
{
	n_ = new ros::NodeHandle();

}

CloudServices::~CloudServices() {
	delete n_;
}


std::string CloudServices::uploadImage(cv::Mat img) {
	if (!client_UploadImage) {
		ROS_DEBUG("Invalid service client, creating new one...");
		double secs = ros::Time::now().toSec();
		client_UploadImage = n_->serviceClient<cloud_services::UploadImage>("upload_image", true);
		double sec2 = ros::Time::now().toSec();
		ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
	} else {
		ROS_DEBUG("Service client valid.");
	}
	
	cv_bridge::CvImage out_msg;
	out_msg.header   = std_msgs::Header();
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
	out_msg.image    = img;
	
	cloud_services::UploadImage srv;
	out_msg.toImageMsg(srv.request.image);
	
	std::string path = "";
	if (client_UploadImage.call(srv)) {
		path = srv.response.path;
		ROS_INFO("[CloudServices] - Image uploaded to %s", path.c_str());
	} else {
		//Failed to call service rapp_get_image
		ROS_ERROR("[CloudServices] - Error calling service upload_image");
	}
	
	return path;
}

int CloudServices::lightCheck(const std::string & fname) {
	if (!client_LightCheck) {
		ROS_DEBUG("Invalid service client, creating new one...");
		double secs = ros::Time::now().toSec();
		client_LightCheck = n_->serviceClient<cloud_services::LightCheck>("light_check", true);
		double sec2 = ros::Time::now().toSec();
		ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
	} else {
		ROS_DEBUG("Service client valid.");
	}
	
	cloud_services::LightCheck srv;
	srv.request.fname = fname;
	
	int result = -1;
	if (client_LightCheck.call(srv)) {
		result = srv.response.result;
		ROS_INFO("[CloudServices] - Light level %d", result);
	} else {
		//Failed to call service rapp_get_image
		ROS_ERROR("[CloudServices] - Error calling service light_check");
	}
	
	return result;
}

