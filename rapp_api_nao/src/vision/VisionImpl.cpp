#include "VisionImpl.hpp"

namespace rapp {
namespace robot {

VisionImpl::VisionImpl(int argc,char **argv) {
	ros::init(argc, argv,"Vision_client");
	n = new ros::NodeHandle();
}

VisionImpl::~VisionImpl() {
}

rapp::object::picture::Ptr VisionImpl::captureImage(int camera_id, int camera_resolution, const std::string & encoding)
{
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
	srv.request.request = camera_id;
	srv.request.resolution = camera_resolution;
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
		return rapp::object::picture("");
	}
	
	int size = cv_ptr->image.total() * cv_ptr->image.elemSize();
	std::vector<rapp::types::byte> bytes(cv_ptr->image.data, cv_ptr->image.data+size * sizeof(rapp::types::byte));
	return std::make_shared<rapp::object::picture>(bytes);
}


bool VisionImpl::setCameraParam(int camera_id, int camera_parameter_id, int new_value)
{
	client_setCameraParam = n->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParam>("rapp_set_camera_parameter");
	rapp_ros_naoqi_wrappings::SetCameraParam srv;
	srv.request.cameraId = camera_id;
	srv.request.cameraParameterId = camera_parameter_id;
	srv.request.newValue = new_value;
	bool isSet = false;
			
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

std::map<int, bool> VisionImpl::setCameraParams(int camera_id, std::vector<int> camera_parameter_ids, std::vector<int> new_values)
{
	client_setCameraParams = n->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParams>("rapp_set_camera_parameters");
	rapp_ros_naoqi_wrappings::SetCameraParams srv;
	srv.request.cameraId = camera_id;
	srv.request.cameraParameterIds = camera_parameter_ids;
	srv.request.newValues = new_values;
	std::vector<uint8_t> isSetList;
	std::map<int, bool> result;
	
	if (camera_parameter_ids.size() != new_values.size()) {
		ROS_ERROR("[Rapp Set Camera Parameters] - different size of vectors"); //camera_parameter_ids.size()!=new_values.size() 
		return result;
	}
	
	if (client_setCameraParams.call(srv)) {
		isSetList = srv.response.isSetList;
		for (int i=0; i<camera_parameter_ids.size(); i++) {
			if (isSetList[i] == true) {
				result[camera_parameter_ids[i]] = true;
				ROS_INFO("[Rapp Set Camera Parameters] - New parameter value was set");
			} else {
				result[camera_parameter_ids[i]] = false;
				ROS_WARN("[Rapp Set Camera Parameters] - New parameter value wasn't set");
			}
		}
	} else {
		ROS_ERROR("[Rapp Set Camera Parameters] - Error calling service rapp_set_camera_parameters");
	}
	
	
		
		
	return result;
}

} // namespace robot
} // namespace rapp

