#include "VisionImpl.hpp"

namespace rapp {
namespace robot {

VisionImpl::VisionImpl(int argc,char **argv) {
	ros::init(argc, argv,"Vision_client");
	n = new ros::NodeHandle();
}

VisionImpl::~VisionImpl() {
}

cv::Mat VisionImpl::captureImage(std::string cameraId, int cameraResolution) {
	if (!client_captureImage) {
		ROS_DEBUG("Invalid service client, creating new one...");
		double secs = ros::Time::now().toSec();
		client_captureImage = n->serviceClient<rapp_ros_naoqi_wrappings::GetImage>("rapp_capture_image", true);
		double sec2 = ros::Time::now().toSec();
		ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
	} else {
		ROS_DEBUG("Service client valid.");
	}

	rapp_ros_naoqi_wrappings::GetImage srv;
	srv.request.request = cameraId;
	srv.request.resolution = cameraResolution;
	sensor_msgs::Image img;
			
	if (client_captureImage.call(srv)) {
		img = srv.response.frame;
		ROS_INFO("[Vision] - Image captured");
	} else {
		//Failed to call service rapp_get_image
		ROS_ERROR("[Vision] - Error calling service rapp_capture_image");
	}

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("[Vision] cv_bridge exception: %s", e.what());
		return cv::Mat();
	}

	return cv_ptr->image.clone();
}


bool VisionImpl::setCameraParams(int cameraId, int cameraParameterId, int newValue )
{
	client_setCameraParam = n->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParam>("rapp_set_camera_parameter");
	rapp_ros_naoqi_wrappings::SetCameraParam srv;
	srv.request.cameraId = cameraId;
	srv.request.cameraParameterId = cameraParameterId;
	srv.request.newValue = newValue;
	bool isSet=false;
			
	if (client_setCameraParam.call(srv))
	{
		isSet = srv.response.isSet;
		if (isSet == true)
			ROS_INFO("[Rapp Set Camera Parameter] - New parameter value was set");
		else
			ROS_INFO("[Rapp Set Camera Parameter] - New parameter value wasn't set");
	}
	else
	{
		//Failed to call service rapp_set_camera_parameter
		ROS_ERROR("[Rapp Set Camera Parameter] - Error calling service rapp_set_camera_parameter");
	}
	return isSet;
}


cv::Mat VisionImpl::getTransform(std::string chainName, int space)
{
	client_getTransform = n->serviceClient<rapp_ros_naoqi_wrappings::GetTransform>("rapp_get_transform");
	rapp_ros_naoqi_wrappings::GetTransform srv;
	srv.request.chainName = chainName;
	srv.request.space = space;
	cv::Mat transformMatrix = cv::Mat::zeros(4,4,cv::DataType<double>::type);
	
	if (client_getTransform.call(srv)) //
	{
		transformMatrix.at<double>(0,0) = srv.response.transformMatrix.r11[0]; transformMatrix.at<double>(0,1) = srv.response.transformMatrix.r12[0];
		transformMatrix.at<double>(0,2) = srv.response.transformMatrix.r13[0]; transformMatrix.at<double>(0,3) = srv.response.transformMatrix.r14[0];
		transformMatrix.at<double>(1,0) = srv.response.transformMatrix.r21[0]; transformMatrix.at<double>(1,1) = srv.response.transformMatrix.r22[0];
		transformMatrix.at<double>(1,2) = srv.response.transformMatrix.r23[0]; transformMatrix.at<double>(1,3) = srv.response.transformMatrix.r24[0];
		transformMatrix.at<double>(2,0) = srv.response.transformMatrix.r31[0]; transformMatrix.at<double>(2,1) = srv.response.transformMatrix.r32[0];
		transformMatrix.at<double>(2,2) = srv.response.transformMatrix.r33[0]; transformMatrix.at<double>(2,3) = srv.response.transformMatrix.r34[0];
		transformMatrix.at<double>(3,0) = srv.response.transformMatrix.r41[0]; transformMatrix.at<double>(3,1) = srv.response.transformMatrix.r42[0];
		transformMatrix.at<double>(3,2) = srv.response.transformMatrix.r43[0]; transformMatrix.at<double>(3,3) = srv.response.transformMatrix.r44[0];
		ROS_INFO("[Rapp get transform] - Transformation matrix computed");
	}
	else
	{
		//Failed to call service rapp_get_image
		ROS_ERROR("[Rapp get transform] - Error calling service rapp_get_transform");
	}
	return transformMatrix;
}



} // namespace robot
} // namespace rapp

