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
		
		// Top camera intrinsic matrix -- from camera calibration //for 1280x960
		NaoVision::camera_top_matrix_3[0][0]=182.0992346/0.16; NaoVision::camera_top_matrix_3[0][2]=658.7582;
		NaoVision::camera_top_matrix_3[1][1]=185.0952141/0.16; NaoVision::camera_top_matrix_3[1][2]=484.2186;
		NaoVision::camera_top_matrix_3[2][2]=1.0;
		NaoVision::camera_top_matrix_3[0][1]=0.0; NaoVision::camera_top_matrix_3[1][0]=0.0; NaoVision::camera_top_matrix_3[2][0]=0.0; NaoVision::camera_top_matrix_3[2][1]=0.0;

		//for 640x480
		NaoVision::camera_top_matrix_2[0][0]=91.0496173/0.16; NaoVision::camera_top_matrix_2[0][2]=329.3791;
		NaoVision::camera_top_matrix_2[1][1]=92.5476071/0.16; NaoVision::camera_top_matrix_2[1][2]=242.1093;
		NaoVision::camera_top_matrix_2[2][2]=1.0;
		NaoVision::camera_top_matrix_2[0][1]=0.0; NaoVision::camera_top_matrix_2[1][0]=0.0; NaoVision::camera_top_matrix_2[2][0]=0.0; NaoVision::camera_top_matrix_2[2][1]=0.0;

		//for 320x240
		NaoVision::camera_top_matrix_1[0][0]=0.5*91.0496173/0.16; NaoVision::camera_top_matrix_1[0][2]=0.5*329.3791;
		NaoVision::camera_top_matrix_1[1][1]=0.5*92.5476071/0.16; NaoVision::camera_top_matrix_1[1][2]=0.5*242.1093;
		NaoVision::camera_top_matrix_1[2][2]=1.0;
		NaoVision::camera_top_matrix_1[0][1]=0.0; NaoVision::camera_top_matrix_1[1][0]=0.0; NaoVision::camera_top_matrix_1[2][0]=0.0; NaoVision::camera_top_matrix_1[2][1]=0.0;

		NaoVision::landmarkTheoreticalSize = 0.16f; //# QRcode real size in meters
		NaoVision::currentQRcodeCamera = "CameraTop";
		
		
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
	
	//#########################################################################
	//#########################################################################
	struct NaoVision::QRcodeDetection NaoVision::qrCodeDetection(sensor_msgs::Image &frame_, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix_)
	{	
		// initializing the structure QRcodeDetection -- set to default
		NaoVision::QRcodeDetection QRcodeDetectionStruct;
		QRcodeDetectionStruct.clear();
				
		cv::Mat cv_frame,frame_grayscale;
		boost::shared_ptr<void const> tracked_object;
		cv_frame = cv_bridge::toCvShare(frame_, tracked_object, frame_.encoding)->image; //conversion from sensor_msgs::Image to cv::Mat

		// Convert to grayscale
		cv::cvtColor(cv_frame, frame_grayscale, CV_BGR2GRAY);
		// Obtain image data
		int width = frame_grayscale.cols;
		int height = frame_grayscale.rows;
		uchar *raw = (uchar *)(frame_grayscale.data);

		// // ZBar
		set_zbar.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
		// Wrap image data
		zbar::Image image(width, height, "Y800", raw, width * height);
		// Scan the image for barcodes
		set_zbar.scan(image);

		// Extract results
		int counter = 0;

		for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
			std::vector<cv::Point3f> object_points;
			std::vector<cv::Point2f> pixel_coords;           
		
			pixel_coords.clear();
			pixel_coords.push_back(cv::Point2f (symbol->get_location_x(0),symbol->get_location_y(0)));
			pixel_coords.push_back(cv::Point2f (symbol->get_location_x(1),symbol->get_location_y(1)));
			pixel_coords.push_back(cv::Point2f (symbol->get_location_x(2),symbol->get_location_y(2)));
			pixel_coords.push_back(cv::Point2f (symbol->get_location_x(3),symbol->get_location_y(3)));
			
			cv::Mat rvec;//(3,1,cv::DataType<float>::type);
			cv::Mat tvec;//(3,1,cv::DataType<float>::type);
			cv::Mat rotationMatrix;//(3,3,cv::DataType<float>::type);
			cv::Mat cameraIntrinsicMatrix;
			cv::Mat distCoeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
			cv::Mat rotation_matrix, translation_matrix;
			cv::Mat landmarkToCameraTransform = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat cameraToLandmarkTransformMatrix = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat robotToLandmarkMatrix = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotx_minus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotz_minus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Mat_I = cv::Mat::zeros(4,4,cv::DataType<double>::type);

			//$$$$$$$$$$$$$$$$$$
			std::vector<double> m00,m01,m02,m10,m12,m20,m21,m22,euler1, euler2, euler3;
			const double PI = 3.14159265359f;
		
			// Model for SolvePnP // x-> y^
			object_points.clear();
						
			//z^y<-
			object_points.push_back(cv::Point3f (0,NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(cv::Point3f (0,NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(cv::Point3f (0,-NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(cv::Point3f (0,-NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2));
			
			// Camera Intrinsic Matrix -- from Camera calibration
			if (width==1280 && height==960)
				cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix_3);
			else if (width==640 && height==480)
				cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix_2);
			else if (width==320 && height==240)
				cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix_1);
			else
			{
				std::cout<<"Wrong camera intrinsic matrix, the position may be computed wrongly"<<std::endl;
				cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix_2);
			}
			

			cv::solvePnP(cv::Mat(object_points), cv::Mat(pixel_coords), cameraIntrinsicMatrix, distCoeffs, rvec, tvec, false);//, CV_ITERATIVE );
						
			cv::Rodrigues(rvec,rotationMatrix);
			
			
			// landmarkToCameraTransform computation
			for (int i=0;i<3;i++)
			{
				for (int j=0;j<3;j++)
					landmarkToCameraTransform.at<double>(i,j)=rotationMatrix.at<double>(i,j);
			}
			landmarkToCameraTransform.at<double>(0,3)=tvec.at<double>(0,0); //x->y^ : x
			landmarkToCameraTransform.at<double>(1,3)=tvec.at<double>(1,0); //x->y^ : y
			landmarkToCameraTransform.at<double>(2,3)=tvec.at<double>(2,0); //x->y^ : z
			landmarkToCameraTransform.at<double>(3,3)=1.0;


			Rotx_minus90.at<double>(0,0)=1.f;
			Rotx_minus90.at<double>(1,1)=0.f;Rotx_minus90.at<double>(1,2)=+1.f;
			Rotx_minus90.at<double>(2,1)=-1.f;Rotx_minus90.at<double>(2,2)=0.f;
			Rotx_minus90.at<double>(3,3)=1.f;

			Rotz_minus90.at<double>(0,0)=0.f;Rotz_minus90.at<double>(0,1)=+1.f;
			Rotz_minus90.at<double>(1,0)=-1.f;Rotz_minus90.at<double>(1,1)=0.f;
			Rotz_minus90.at<double>(2,2)=1.f;
			Rotz_minus90.at<double>(3,3)=1.f;

			
			//#####################			
			//## Transformation from the QR-code coordinate system to the camera coordinate system
			cameraToLandmarkTransformMatrix = Rotz_minus90*Rotx_minus90*landmarkToCameraTransform;
			//#####################

			//#####################						
			//## Transformation from the camera coordinate system to the robot coordinate system
			robotToLandmarkMatrix = robotToCameraMatrix_*cameraToLandmarkTransformMatrix;
			//#####################			


			QRcodeDetectionStruct.LandmarkInCameraCoordinate.push_back(cameraToLandmarkTransformMatrix);
			QRcodeDetectionStruct.LandmarkInRobotCoordinate.push_back(robotToLandmarkMatrix);
			
			counter++;
			QRcodeDetectionStruct.QRmessage.push_back(symbol->get_data());
			QRcodeDetectionStruct.isQRcodeFound = true;

		}
		QRcodeDetectionStruct.numberOfQRcodes = counter;

		return QRcodeDetectionStruct;
	}
//#########################################################################
//#########################################################################
	
	std::vector<double> NaoVision::compute_euler_x( std::vector<double> &m21, std::vector<double> &m22, std::vector<double> &euler_y)//( std::vector<double> &m12, std::vector<double> &m22) //For the computation of the 1-st euler angle
	{
		std::vector<double> euler1;
		euler1.clear();
		for (int i=0;i<m21.size();i++)
			if (cos(euler_y[i])!=0.f && i<euler_y.size())
				euler1.push_back( atan2(m21[i]/cos(euler_y[i]),m22[i]/cos(euler_y[i])) );
			
		return euler1;
	}
	std::vector<double> NaoVision::compute_euler_y(std::vector<double> &m20)//( std::vector<double> &m00, std::vector<double> &m01, std::vector<double> &m02) //For the computation of the 2-nd euler angle
	{
		std::vector<double> euler2;
		const double PI = 3.14159265359f;
		euler2.clear();
		for (int i=0;i<m20.size();i++)
			euler2.push_back( -asin(m20[i]) );//in radians PI+asin(m20[i]) v -asin(m20[i])
		return euler2;
	}
	std::vector<double> NaoVision::compute_euler_z(std::vector<double> &m10, std::vector<double> &m00, std::vector<double> &euler_y)//(std::vector<double> &m00, std::vector<double> &m01) //For the computation of the 3-rd euler angle
	{
		std::vector<double> euler3;
		euler3.clear();
		for (int i=0;i<m00.size();i++)
			if (cos(euler_y[i])!=0.f && i<euler_y.size())
			{
				euler3.push_back( atan2(m10[i]/cos(euler_y[i]), m00[i]/cos(euler_y[i])) );// in radians
				//euler3.push_back( atan2(m10[i], m00[i]) );// in radians
			}
		return euler3;
	}

//#########################################################################

	// Hazard -- open door detection	
	struct NaoVision::QRcodeHazardDetection NaoVision::openDoorDetection(std::vector< cv::Mat > &LandmarkInRobotCoordinate, std::vector<std::string> &QRmessage)
	{
		// initializing the structure QRcodeHazardDetection -- set to default
		NaoVision::QRcodeHazardDetection QRcodeHazardStruct;
		QRcodeHazardStruct.clear();
		
		std::vector<double> euler1;
		std::vector<double> euler2;
		std::vector<double> euler3;
		std::vector<double> m00, m01, m02, m10, m12, m20, m21, m22;

		const double PI = 3.14159265359f;

		double precision = 6*PI/180.f; //established precision of QRcodes detection -- in degrees
		std::vector<int> wall_number;

		//computation of the 3-rd euler angle
		euler3.clear();
		euler1.clear();euler2.clear();
		m00.clear(); m01.clear();m02.clear(); m10.clear(); m12.clear(); m20.clear(); m21.clear(); m22.clear();
		wall_number.clear();

		// creates vectors with selected one value, per detected QRcode, from vector of localization matrices
		for (int i=0; i<LandmarkInRobotCoordinate.size(); i++)
		{
			m00.push_back(LandmarkInRobotCoordinate[i].at<double>(0,0));
			m01.push_back(LandmarkInRobotCoordinate[i].at<double>(0,1));
			m02.push_back(LandmarkInRobotCoordinate[i].at<double>(0,2));
			m10.push_back(LandmarkInRobotCoordinate[i].at<double>(1,0));
			m12.push_back(LandmarkInRobotCoordinate[i].at<double>(1,2));
			m20.push_back(LandmarkInRobotCoordinate[i].at<double>(2,0));
			m21.push_back(LandmarkInRobotCoordinate[i].at<double>(2,1));
			m22.push_back(LandmarkInRobotCoordinate[i].at<double>(2,2));
		}
		
		euler2 = NaoVision::compute_euler_y(m20);
		euler3 = NaoVision::compute_euler_z(m10, m00, euler2);
		euler1 = NaoVision::compute_euler_x(m21, m22, euler2);


		for (int i=0; i<LandmarkInRobotCoordinate.size(); i++)
		{
			//std::cout<<"Euler 1:\t"<<euler1[i]*180/PI<<"\tEuler 2:\t"<<euler2[i]*180/PI<<"\tEuler 3:\t"<<euler3[i]*180/PI<<std::endl;

			if ( (QRmessage[i].find("Wall") != std::string::npos) || (QRmessage[i].find("Stable object") != std::string::npos) || (QRmessage[i].find("stable object") != std::string::npos) ) //checks if "Wall" or "Stable object" are a substring of QRmessage[i]
				wall_number.push_back(i);
		}
		for (int i=0; i<wall_number.size(); i++)
			for (int k=-2;k<=2;k++)
			{
				if ( (k!=0) && (wall_number[i]+k>=0) && (wall_number[i]+k<LandmarkInRobotCoordinate.size()) )
				{
					//comparing the angles (euler2)
					if ( ! ( ((euler3[wall_number[i]]+precision > euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision < euler3[wall_number[i]+k])) || ((euler3[wall_number[i]]+precision - PI/2> euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision - PI/2< euler3[wall_number[i]+k])) || ((euler3[wall_number[i]]+precision + PI/2> euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision + PI/2< euler3[wall_number[i]+k])) ) )
					{
						//detected an open door
						QRcodeHazardStruct.isHazardFound = true;
						QRcodeHazardStruct.hazardPosition_x.push_back( LandmarkInRobotCoordinate[wall_number[i]+k].at<double>(0,3) );
						QRcodeHazardStruct.hazardPosition_y.push_back( LandmarkInRobotCoordinate[wall_number[i]+k].at<double>(1,3) );
						QRcodeHazardStruct.openedObject.push_back( QRmessage[wall_number[i]+k] );
					}
				}
			}

		return QRcodeHazardStruct;
	}

//#########################################################################
