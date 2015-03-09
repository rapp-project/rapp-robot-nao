//#####################
// written by Jan Figat
//#####################

#include "NaoVision.h"

//#########################################################################
	NaoVision::NaoVision(int argc,char **argv)
	{
		ros::init(argc, argv,"QRcodeDetection_client");
		n = new ros::NodeHandle();
		
		NaoVision::NAO_IP = "nao.local";
		NaoVision::NAO_PORT = 9559;
		// Top camera intrinsic matrix -- from camera calibration
		NaoVision::camera_top_matrix[0][0]=182.0992346/0.16; NaoVision::camera_top_matrix[0][2]=658.7582;
		NaoVision::camera_top_matrix[1][1]=185.0952141/0.16; NaoVision::camera_top_matrix[1][2]=484.2186;
		NaoVision::camera_top_matrix[2][2]=1.0;
		NaoVision::camera_top_matrix[0][1]=0.0; NaoVision::camera_top_matrix[1][0]=0.0; NaoVision::camera_top_matrix[2][0]=0.0; NaoVision::camera_top_matrix[2][1]=0.0;

		//cout<<cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix)<<endl;
		
		NaoVision::landmarkTheoreticalSize = 0.16f; //# QRcode real size in meters
		//cout<<NaoVision::landmarkTheoreticalSize<<endl;
		NaoVision::currentQRcodeCamera = "CameraTop";

	}

//#########################################################################
//#########################################################################

	struct NaoVision::QRcodeDetection NaoVision::qrCodeDetection(sensor_msgs::Image &frame_, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix_)
	{	
		//cout<<cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix)<<endl;

		// initializing the structure QRcodeDetection -- set to default
		NaoVision::QRcodeDetection QRcodeDetectionStruct;
		QRcodeDetectionStruct.clear();
		//QRcodeDetectionStruct.isQRcodeFound = false;
		//QRcodeDetectionStruct.numberOfQRcodes = 0;
		//QRcodeDetectionStruct.LandmarkInCameraCoordinate.clear();
		//QRcodeDetectionStruct.LandmarkInRobotCoordinate.clear();
		//QRcodeDetectionStruct.QRmessage.clear();
		
		cv::Mat cv_frame,frame_grayscale;
		boost::shared_ptr<void const> tracked_object;
		cv_frame = cv_bridge::toCvShare(frame_, tracked_object, frame_.encoding)->image; //conversion from sensor_msgs::Image to cv::Mat

		// Convert to grayscale
		cvtColor(cv_frame, frame_grayscale, CV_BGR2GRAY);
		// Obtain image data
		int width = frame_grayscale.cols;
		int height = frame_grayscale.rows;
		uchar *raw = (uchar *)(frame_grayscale.data);

		// // ZBar
		set_zbar.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
		// Wrap image data
      Image image(width, height, "Y800", raw, width * height);
      // Scan the image for barcodes
	   set_zbar.scan(image);

		// Extract results
      int counter = 0;


      for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
			//vector<vector<Point3f> > object_points;
			//vector<vector<Point2f> > image_points;
			//vector<Point3f> world_coords;
			std::vector<cv::Point3f> object_points;
			std::vector<cv::Point2f> pixel_coords;           
		
			pixel_coords.clear();
			pixel_coords.push_back(Point2f (symbol->get_location_x(0),symbol->get_location_y(0)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(1),symbol->get_location_y(1)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(2),symbol->get_location_y(2)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(3),symbol->get_location_y(3)));
			//cout<<pixel_coords<<endl;

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
			cv::Mat Rotx_plus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotz_plus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Roty_plus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Roty_minus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotz_180 = cv::Mat::zeros(4,4,cv::DataType<double>::type);

			//$$$$$$$$$$$$$$$$$$
			vector<double> m00,m01,m02,m12,m22,euler1, euler2, euler3;
			const double PI = 3.14159265359f;
		
			// Model for SolvePnP // x-> y^
			object_points.clear();
			/*object_points.push_back(Point3f (-NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (-NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2,0.));
			*/
			//z^y->
			object_points.push_back(Point3f (0.,-NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(Point3f (0.,-NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(Point3f (0.,NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2));
			object_points.push_back(Point3f (0.,NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2));
						

			// Camera Intrinsic Matrix -- from Camera calibration
			cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix);

			cv::solvePnP(cv::Mat(object_points), cv::Mat(pixel_coords), cameraIntrinsicMatrix, distCoeffs, rvec, tvec, false);//, CV_ITERATIVE );
						
			Rodrigues(rvec,rotationMatrix);
			
			//cout<<cameraIntrinsicMatrix<<endl<<endl;
			//cout<<tvec<<endl;
			//cout<<rotationMatrix<<endl;
			
			// landmarkToCameraTransform computation
			for (int i=0;i<3;i++)
			{
				for (int j=0;j<3;j++)
					landmarkToCameraTransform.at<double>(i,j)=rotationMatrix.at<double>(i,j);
			}
			landmarkToCameraTransform.at<double>(0,3)=tvec.at<double>(2,0); //y->z^ : x
			landmarkToCameraTransform.at<double>(1,3)=tvec.at<double>(0,0); //y->z^ : y
			landmarkToCameraTransform.at<double>(2,3)=tvec.at<double>(1,0); //y->z^ : z
			landmarkToCameraTransform.at<double>(3,3)=1.0;

									
			Rotz_180.at<double>(0,0)=-1.f; Rotz_180.at<double>(0,1)=0.f;
			Rotz_180.at<double>(1,0)=0.f;	Rotz_180.at<double>(1,1)=-1.f;
			Rotz_180.at<double>(2,2)=1.f;
			Rotz_180.at<double>(3,3)=1.f;
			
			//## Transformation from the QR-code coordinate system to the camera coordinate system
			cameraToLandmarkTransformMatrix = Rotz_180*landmarkToCameraTransform;
			//cameraToLandmarkTransformMatrix.at<double>(0,3)*=-1.; //instead of translation about vector
			cameraToLandmarkTransformMatrix.at<double>(0,3)=tvec.at<double>(2,0); //translation about x axis from QRcode coordinate system to Camera coordinate system
			cameraToLandmarkTransformMatrix.at<double>(1,3)=-tvec.at<double>(0,0); //translation about y axis from QRcode coordinate system to Camera coordinate system
			cameraToLandmarkTransformMatrix.at<double>(2,3)=-tvec.at<double>(1,0); //translation about z axis from QRcode coordinate system to Camera coordinate system

						
			//## Transformation from the camera coordinate system to the robot coordinate system
			robotToLandmarkMatrix = robotToCameraMatrix_*cameraToLandmarkTransformMatrix;
			

			
			// ####################			for tests
			//&&&&&&&&&&&&&&&&&&&&&
			std::cout<<"landmarkToCameraTransform"<<landmarkToCameraTransform<<std::endl;
			m00.clear();m01.clear();m02.clear();m12.clear();m22.clear();euler1.clear();euler2.clear();euler3.clear();
			m00.push_back(landmarkToCameraTransform.at<double>(0,0)); m01.push_back(landmarkToCameraTransform.at<double>(0,1));
			m02.push_back(landmarkToCameraTransform.at<double>(0,2)); m12.push_back(landmarkToCameraTransform.at<double>(1,2));
			m22.push_back(landmarkToCameraTransform.at<double>(2,2));
			euler3 = NaoVision::compute_euler_z(m00, m01);
			euler1 = NaoVision::compute_euler_x(m12, m22);
			euler2 = NaoVision::compute_euler_y(m00, m01, m02);
			std::cout<<"landmarkToCameraTransform >> Euler 1:\t"<<euler1[0]*180.f/PI<<"\tEuler 2:\t"<<euler2[0]*180.f/PI<<"\tEuler 3:\t"<<euler3[0]*180.f/PI<<std::endl;
			//&&&&&&&&&&&&&&&&&&&&&
			std::cout<<"cameraToLandmarkTransformMatrix"<<cameraToLandmarkTransformMatrix<<std::endl;
			m00.clear();m01.clear();m02.clear();m12.clear();m22.clear();euler1.clear();euler2.clear();euler3.clear();
			m00.push_back(cameraToLandmarkTransformMatrix.at<double>(0,0)); m01.push_back(cameraToLandmarkTransformMatrix.at<double>(0,1));
			m02.push_back(cameraToLandmarkTransformMatrix.at<double>(0,2)); m12.push_back(cameraToLandmarkTransformMatrix.at<double>(1,2));
			m22.push_back(cameraToLandmarkTransformMatrix.at<double>(2,2));
			euler3 = NaoVision::compute_euler_z(m00, m01);
			euler1 = NaoVision::compute_euler_x(m12, m22);
			euler2 = NaoVision::compute_euler_y(m00, m01, m02);
			std::cout<<"cameraToLandmarkTransformMatrix >> Euler 1:\t"<<euler1[0]*180.f/PI<<"\tEuler 2:\t"<<euler2[0]*180.f/PI<<"\tEuler 3:\t"<<euler3[0]*180.f/PI<<std::endl;
			//&&&&&&&&&&&&&&&&&&&&&
			std::cout<<"robotToCameraMatrix \n\t"<<robotToCameraMatrix_<<std::endl;
			m00.clear();m01.clear();m02.clear();m12.clear();m22.clear();euler1.clear();euler2.clear();euler3.clear();
			m00.push_back(robotToCameraMatrix_.at<double>(0,0));	m01.push_back(robotToCameraMatrix_.at<double>(0,1));
			m02.push_back(robotToCameraMatrix_.at<double>(0,2));	m12.push_back(robotToCameraMatrix_.at<double>(1,2));
			m22.push_back(robotToCameraMatrix_.at<double>(2,2));
			euler3 = NaoVision::compute_euler_z(m00, m01);
			euler1 = NaoVision::compute_euler_x(m12, m22);
			euler2 = NaoVision::compute_euler_y(m00, m01, m02);
			std::cout<<"robotToCameraMatrix >> Euler 1:\t"<<euler1[0]*180.f/PI<<"\tEuler 2:\t"<<euler2[0]*180.f/PI<<"\tEuler 3:\t"<<euler3[0]*180.f/PI<<std::endl;
			//&&&&&&&&&&&&&&&&&&&&&
			std::cout<<"robotToLandmarkMatrix \n\t"<<robotToLandmarkMatrix<<std::endl;
			m00.clear();m01.clear();m02.clear();m12.clear();m22.clear();euler1.clear();euler2.clear();euler3.clear();
			m00.push_back(robotToLandmarkMatrix.at<double>(0,0));	m01.push_back(robotToLandmarkMatrix.at<double>(0,1));
			m02.push_back(robotToLandmarkMatrix.at<double>(0,2));	m12.push_back(robotToLandmarkMatrix.at<double>(1,2));
			m22.push_back(robotToLandmarkMatrix.at<double>(2,2));
			euler3 = NaoVision::compute_euler_z(m00, m01);
			euler1 = NaoVision::compute_euler_x(m12, m22);
			euler2 = NaoVision::compute_euler_y(m00, m01, m02);
			std::cout<<"robotToLandmarkMatrix >> Euler 1:\t"<<euler1[0]*180.f/PI<<"\tEuler 2:\t"<<euler2[0]*180.f/PI<<"\tEuler 3:\t"<<euler3[0]*180.f/PI<<std::endl;
			//&&&&&&&&&&&&&&&&&&&&&
			// ####################
			//*/

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
	
	std::vector<double> NaoVision::compute_euler_x( std::vector<double> &m12, std::vector<double> &m22) //For the computation of the 1-st euler angle
	{
		std::vector<double> euler1;
		euler1.clear();
		for (int i=0;i<m12.size();i++)
			euler1.push_back( atan2(m12[i], m22[i]) );// in radians
		return euler1;
	}
	std::vector<double> NaoVision::compute_euler_y( std::vector<double> &m00, std::vector<double> &m01, std::vector<double> &m02) //For the computation of the 2-nd euler angle
	{
		std::vector<double> euler2;
		euler2.clear();
		for (int i=0;i<m00.size();i++)
			euler2.push_back( atan2(-m02[i], sqrt(pow(m00[i],2) + pow(m01[i],2))) );// in radians
		return euler2;
	}
	std::vector<double> NaoVision::compute_euler_z( std::vector<double> &m00, std::vector<double> &m01) //For the computation of the 3-rd euler angle
	{
		std::vector<double> euler3;
		euler3.clear();
		for (int i=0;i<m00.size();i++)
			euler3.push_back( atan2(m01[i], m00[i]) );// in radians
		return euler3;
	}

//#########################################################################

	// Hazard -- open door detection	
	struct NaoVision::QRcodeHazardDetection NaoVision::openDoorDetection(std::vector< cv::Mat > &LandmarkInRobotCoordinate, std::vector<std::string> &QRmessage)
	{
		// initializing the structure QRcodeHazardDetection -- set to default
		NaoVision::QRcodeHazardDetection QRcodeHazardStruct;
		QRcodeHazardStruct.clear();
		//QRcodeHazardStruct.isHazardFound = false;
		//QRcodeHazardStruct.hazardPosition_x.clear();
		//QRcodeHazardStruct.hazardPosition_y.clear();
		//QRcodeHazardStruct.openedObject.clear();
		
		//vector<double> euler1;
		//vector<double> euler2;
		vector<double> euler3;
		vector<double> m00, m01;//, m02, m12, m22;

		const double PI = 3.14159265359f;

		double precision = 4*PI/180.f; //established precision of QRcodes detection -- in degrees
		vector<int> wall_number;

		//computation of the 3-rd euler angle
		euler3.clear();
		//euler1.clear();euler2.clear();
		m00.clear(); m01.clear();
		wall_number.clear();

		// creates vectors with selected one value, per detected QRcode, from vector of localization matrices
		for (int i=0; i<LandmarkInRobotCoordinate.size(); i++)
		{
			m00.push_back(LandmarkInRobotCoordinate[i].at<double>(0,0));
			m01.push_back(LandmarkInRobotCoordinate[i].at<double>(0,1));
			//m02.push_back(LandmarkInRobotCoordinate[i].at<double>(0,2));
			//m12.push_back(LandmarkInRobotCoordinate[i].at<double>(1,2));
			//m22.push_back(LandmarkInRobotCoordinate[i].at<double>(2,2));
		}
		euler3 = NaoVision::compute_euler_z(m00, m01);
		//euler1 = NaoVision::compute_euler_x(m12, m22);
		//euler2 = NaoVision::compute_euler_y(m00, m01, m02);

		for (int i=0; i<LandmarkInRobotCoordinate.size(); i++)
		{
			//std::cout<<"Euler 1:\t"<<euler1[i]*180/PI<<"\tEuler 2:\t"<<euler2[i]*180/PI<<"\tEuler 3:\t"<<euler3[i]*180/PI<<std::endl;

			if ( (QRmessage[i].find("Wall") != std::string::npos) || (QRmessage[i].find("Stable object") != std::string::npos) ) //checks if "Wall" or "Stable object" are a substring of QRmessage[i]
				wall_number.push_back(i);
		}
		for (int i=0; i<wall_number.size(); i++)
			for (int k=-2;k<=2;k++)
			{
				if ( (k!=0) && (wall_number[i]+k>=0) && (wall_number[i]+k<LandmarkInRobotCoordinate.size()) )
				{
					//comparing the angles (euler2)
					if ( ! ( ((euler3[wall_number[i]]+precision > euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision < euler3[wall_number[i]+k])) || ((euler3[wall_number[i]]+precision - PI> euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision - PI< euler3[wall_number[i]+k])) || ((euler3[wall_number[i]]+precision + PI> euler3[wall_number[i]+k]) && (euler3[wall_number[i]]-precision + PI< euler3[wall_number[i]+k])) ) )
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
//#########################################################################
	// CAMERA
	sensor_msgs::Image NaoVision::captureImage(std::string cameraId)
	{
		client_captureImage = n->serviceClient<rapp_robot_agent::GetImage>("rapp_capture_image");
		rapp_robot_agent::GetImage srv;
		srv.request.request = cameraId;
		sensor_msgs::Image img;
				
		if (client_captureImage.call(srv))//NaoVision::
		{
			img = srv.response.frame;
			std::cout << "[Rapp QR code test] - Image captured\n";
		}
		else
		{
			//Failed to call service rapp_get_image
			std::cout<<"[Rapp QR code test] - Error calling service rapp_get_image\n";
		}
		return img;
	}

//#########################################################################
	cv::Mat NaoVision::getTransform(std::string chainName, int space)
	{
		//ros::NodeHandle nh;
		//ros::ServiceClient client_getTransform_;
		//NaoVision::client_getTransform = NaoVision::nh_.serviceClient<rapp_robot_agent::GetTransform>("rapp_get_transform");
		//client_getTransform_ = nh.serviceClient<rapp_robot_agent::GetTransform>("rapp_get_transform");
		client_getTransform = n->serviceClient<rapp_robot_agent::GetTransform>("rapp_get_transform");
		rapp_robot_agent::GetTransform srv;
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
	
////
/*
main(int argc, char **argv)
{
	//NaoVision Nao(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;

	NaoVision::QRcodeDetection QRcodeDetectionStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;
	
	frame = NaoVision::captureImage("top - adaptive auto exposure 1");
	robotToCameraMatrix = NaoVision::getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system
	QRcodeDetectionStruct = NaoVision::qrCodeDetection(frame, scanner, robotToCameraMatrix);
	for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		std::cout<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;


    return 0;
}
*/
