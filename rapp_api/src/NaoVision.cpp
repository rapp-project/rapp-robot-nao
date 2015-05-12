//#####################
// written by Jan Figat
//#####################

#include "rapp_api/NaoVision.h"

#define MAX_IMAGE_QUEUE_ITER 10
#define FREQUENCY_OF_PUBLICATION 10

//#########################################################################
	NaoVision::NaoVision(int argc,char **argv)
	{
		ros::init(argc, argv,"NaoVision_client");
		n_ = new ros::NodeHandle();
		
		NaoVision::NAO_IP = "nao.local";
		NaoVision::NAO_PORT = 9559;

		//############# --- new --- ###############
		is_image_arrived_ = false;
		iteration_flag_ = 0;
		//#########################################

	}

//#########################################################################
//#########################################################################
	// CAMERA
	sensor_msgs::Image NaoVision::captureImage(std::string cameraId, int cameraResolution)
	{
		client_captureImage = n_->serviceClient<rapp_ros_naoqi_wrappings::GetImage>("rapp_capture_image");
		rapp_ros_naoqi_wrappings::GetImage srv;
		srv.request.request = cameraId;
		srv.request.resolution = cameraResolution;
		sensor_msgs::Image img;
				
		if (client_captureImage.call(srv))
		{
			img = srv.response.frame;
			std::cout << "[Rapp QR code test] - Image captured\n";
		}
		else
		{
			//Failed to call service rapp_get_image
			std::cout << "[Rapp QR code test] - Error calling service rapp_get_image\n";
		}
		return img;
	}

	//#########################################################################
	bool NaoVision::setCameraParams(int cameraId, int cameraParameterId, int newValue )
	{
		client_setCameraParam = n_->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParam>("rapp_set_camera_parameter");
		rapp_ros_naoqi_wrappings::SetCameraParam srv;
		srv.request.cameraId = cameraId;
		srv.request.cameraParameterId = cameraParameterId;
		srv.request.newValue = newValue;
		bool isSet=false;
				
		if (client_setCameraParam.call(srv))
		{
			isSet = srv.response.isSet;
			if (isSet == true)
				std::cout << "[Rapp Set Camera Parameter] - New parameter value was set\n";
			else
				std::cout << "[Rapp Set Camera Parameter] - New parameter value wasn't set\n";
		}
		else
		{
			//Failed to call service rapp_set_camera_parameter
			std::cout<<"[Rapp Set Camera Parameter] - Error calling service rapp_set_camera_parameter\n";
		}
		return isSet;
	}
//#########################################################################
	cv::Mat NaoVision::getTransform(std::string chainName, int space)
	{
		client_getTransform = n_->serviceClient<rapp_ros_naoqi_wrappings::GetTransform>("rapp_get_transform");
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
			std::cout << "[Rapp QR code test] - Transformation matrix computed\n";
		}
		else
		{
			//Failed to call service rapp_get_image
			std::cout<<"[Rapp QR code test] - Error calling service rapp_get_transform\n";
		}
		return transformMatrix;
	}
//  ###################------New------####################################
	/*
		Publishing an image to a given topic
	*/
	void NaoVision::publishImage(cv_bridge::CvImagePtr image_ptr, std::string topic)
	{
		image_transport::ImageTransport it(*n_);
		image_transport::Publisher publish_image;

		publish_image = it.advertise(topic, 1);

		cv::Mat image = image_ptr->image;//= cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
		//waitKey(0);
		//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_mat).toImageMsg();
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

		ros::Rate loop_rate(FREQUENCY_OF_PUBLICATION);
		while (n_->ok() && iteration_flag_ < MAX_IMAGE_QUEUE_ITER) {
			publish_image.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			iteration_flag_++;
			
		}
		iteration_flag_=0;
		
	}

	/*
		Gets an image from a given topic - as a cv::Mat image
	*/
	cv_bridge::CvImagePtr NaoVision::getImageFromTopic(std::string topic)
	{
		image_transport::ImageTransport it(*n_);
		image_transport::Subscriber image_sub;
		
  		// Subscrive to input video feed and publish output video feed
    	image_sub = it.subscribe(topic, 1, &NaoVision::imageCb, this);

    	while(ros::ok() && !is_image_arrived_)
    	{
    		ros::spinOnce();
    	}
    	is_image_arrived_=false;
    	return cv_ptr_;
	}

	/*
		In case of subscribtion to a given topic
	*/
	void NaoVision::imageCb(const sensor_msgs::ImageConstPtr& msg)
  	{
		try
		{
			cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			is_image_arrived_=true;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}
