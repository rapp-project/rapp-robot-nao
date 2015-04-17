//#####################
// written by Jan Figat
//#####################

#include "rapp_libraries/NaoVision.h"

#define MAX_IMAGE_QUEUE_ITER 10
#define FREQUENCY_OF_PUBLICATION 10

//#########################################################################
	NaoVision::NaoVision(int argc,char **argv)
	{
		ros::init(argc, argv,"QRcodeDetection_client");
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
	sensor_msgs::Image NaoVision::captureImage(std::string cameraId)
	{
		client_captureImage = n_->serviceClient<rapp_robot_agent::GetImage>("rapp_capture_image");
		rapp_robot_agent::GetImage srv;
		srv.request.request = cameraId;
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
