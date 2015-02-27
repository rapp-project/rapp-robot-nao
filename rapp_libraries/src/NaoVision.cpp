#include "NaoVision.h"
#include "ros/ros.h"
//#include "rapp_robot_agent/DetectQRcode.h"

using namespace std;
using namespace cv;
using namespace zbar;

	NaoVision::NaoVision(int argc,char **argv)
	{
		//ros::init(argc, argv,"QRcodeDetection_client");
		//n = new ros::NodeHandle();
		
		NaoVision::NAO_IP = "nao.local";
		NaoVision::NAO_PORT = 9559;
		double camera_top_matrix[3][3] = {{182.0992346/0.16, 0., 658.7582}, {0., 185.0952141/0.16, 484.2186}, {0., 0., 1.}};
		NaoVision::cameraTopMatrix = cv::Mat(3, 3, CV_64F, camera_top_matrix);
	}

	

	//////////////////////
		// Create a zbar reader
    	//ImageScanner scanner;
    
    	// Configure the reader
    	//scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

	struct NaoVision::QRcodeDetection NaoVision::qrCodeDetection(sensor_msgs::Image frame_, zbar::ImageScanner set_zbar, /*vector< vector<float> >*/ cv::Mat robotToCameraMatrix_)
	{	
		// initializing the structure QRcodeDetection -- set to default
		NaoVision::QRcodeDetection QRcodeDetectionStruct;
		QRcodeDetectionStruct.isQRcodeFound = false;
		QRcodeDetectionStruct.numberOfQRcodes = 0;
		QRcodeDetectionStruct.LandmarkInCameraCoordinate.clear();
		QRcodeDetectionStruct.LandmarkInRobotCoordinate.clear();
		QRcodeDetectionStruct.QRmessage.clear();
		
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
			vector<Point3f> object_points;
			vector<Point2f> pixel_coords;           

			pixel_coords.push_back(Point2f (symbol->get_location_x(0),symbol->get_location_y(0)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(1),symbol->get_location_y(1)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(2),symbol->get_location_y(2)));
			pixel_coords.push_back(Point2f (symbol->get_location_x(3),symbol->get_location_y(3)));

			//cameraMatrix = initCameraMatrix2D(object_points, image_points, frame.size());

			vector<cv::Mat> rvec;
			vector<cv::Mat> tvec;
			//cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);
			cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
			cv::Mat rotation_matrix, translation_matrix;
			cv::Mat landmarkToCameraTransform = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat cameraToLandmarkTransformMatrix = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat robotToLandmarkMatrix = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotx_minus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat Rotz_plus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

			double w= NaoVision::landmarkTheoreticalSize;
			// Model for SolvePnP // x-> y^
			object_points.push_back(Point3f (-w/2,w/2,0.0));
			object_points.push_back(Point3f (-w/2,-w/2,0.0));
			object_points.push_back(Point3f (w/2,-w/2,0.0));
			object_points.push_back(Point3f (w/2,w/2,0.0));
				
			cv::solvePnP(object_points, pixel_coords, NaoVision::cameraTopMatrix, distCoeffs, rvec, tvec, false, ITERATIVE );
			//calibrateCamera(object_points, image_points, frame.size(), cameraMatrix, distCoeffs, rvec, tvec, CV_CALIB_USE_INTRINSIC_GUESS);

			Rodrigues(rvec[0],rotationMatrix);
			// landmarkToCameraTransform computation
			for (int i=0;i<3;i++)
			{
				for (int j=0;j<3;j++)
					landmarkToCameraTransform.at<double>(i,j)=rotationMatrix.at<double>(i,j);
				landmarkToCameraTransform.at<double>(i,3)=tvec[i].at<double>(0);
			}
			landmarkToCameraTransform.at<double>(3,3)=1.0;

			Rotx_minus90.at<double>(0,0)=1.0;
			Rotx_minus90.at<double>(1,1)=cos(-CV_PI/2); Rotx_minus90.at<double>(1,2)=-sin(-CV_PI/2);
			Rotx_minus90.at<double>(2,1)=sin(-CV_PI/2); Rotx_minus90.at<double>(2,2)=cos(-CV_PI/2);
			Rotx_minus90.at<double>(3,3)=1.0;

			Rotz_plus90.at<double>(0,0)=cos(CV_PI/2); Rotz_plus90.at<double>(0,1)=-sin(CV_PI/2);
			Rotz_plus90.at<double>(0,0)=sin(CV_PI/2); Rotz_plus90.at<double>(0,1)=cos(CV_PI/2);
			Rotz_plus90.at<double>(2,2)=1.0;
			Rotz_plus90.at<double>(3,3)=1.0;

			//## Transformation from the QR-code coordinate system to the camera coordinate system
			cameraToLandmarkTransformMatrix = Rotx_minus90*landmarkToCameraTransform;
			cameraToLandmarkTransformMatrix = Rotz_plus90*cameraToLandmarkTransformMatrix;
			cameraToLandmarkTransformMatrix.at<double>(0,3)*=-1.;	cameraToLandmarkTransformMatrix.at<double>(2,3)*=-1.;

			//## Transformation from the camera coordinate system to the robot coordinate system
			robotToLandmarkMatrix = cameraToLandmarkTransformMatrix*robotToCameraMatrix_;

			QRcodeDetectionStruct.LandmarkInCameraCoordinate.push_back(cameraToLandmarkTransformMatrix);
			QRcodeDetectionStruct.LandmarkInRobotCoordinate.push_back(robotToLandmarkMatrix);
			counter++;
			QRcodeDetectionStruct.QRmessage.push_back(symbol->get_data());
			QRcodeDetectionStruct.isQRcodeFound = true;

		}
		QRcodeDetectionStruct.numberOfQRcodes = counter;

		return QRcodeDetectionStruct;
	}
	

	// CAMERA
	sensor_msgs::Image NaoVision::captureImage(std::string cameraId)
	{
		NaoVision::client_captureImage = NaoVision::nh_.serviceClient<rapp_robot_agent::GetImage>("rapp_get_image");		
		rapp_robot_agent::GetImage srv;
		srv.request.request = cameraId;
		sensor_msgs::Image img;
				
		if (NaoVision::client_captureImage.call(srv))
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

	/*std::vector< std::vector<float> >*/
	cv::Mat getTransform(std::string chainName, int space)
	{
		ros::NodeHandle nh;
		ros::ServiceClient client_getTransform_;
		//NaoVision::client_getTransform = NaoVision::nh_.serviceClient<rapp_robot_agent::GetTransform>("rapp_get_transform");
		client_getTransform_ = nh.serviceClient<rapp_robot_agent::GetTransform>("rapp_get_transform");
		rapp_robot_agent::GetTransform srv;
		srv.request.chainName = chainName;
		srv.request.space = space;
		cv::Mat transformMatrix = cv::Mat::zeros(4,4,CV_64F);
		
		if (client_getTransform_.call(srv)) //
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
