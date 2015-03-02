#include "NaoVision.h"
#include "ros/ros.h"
//#include "rapp_robot_agent/DetectQRcode.h"

#include <cv_bridge/cv_bridge.h>
//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <zbar.h> // for QRcode detection

using namespace std;
using namespace cv;
using namespace zbar;

//const double PI = 3.14159265359;

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
		cout<<NaoVision::landmarkTheoreticalSize<<endl;
		NaoVision::currentQRcodeCamera = "CameraTop";

	}

	

	//////////////////////
		// Create a zbar reader
    	//ImageScanner scanner;
    
    	// Configure the reader
    	//scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

	struct NaoVision::QRcodeDetection NaoVision::qrCodeDetection(sensor_msgs::Image &frame_, zbar::ImageScanner &set_zbar, /*vector< vector<float> >*/ cv::Mat robotToCameraMatrix_)
	{	
		//cout<<cv::Mat(3, 3, cv::DataType<double>::type, NaoVision::camera_top_matrix)<<endl;

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
			cv::Mat Rotz_plus90 = cv::Mat::zeros(4,4,cv::DataType<double>::type);
			
			//float w= NaoVision::landmarkTheoreticalSize;
			// Model for SolvePnP // x-> y^
			object_points.clear();
			object_points.push_back(Point3f (-NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (-NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (NaoVision::landmarkTheoreticalSize/2,-NaoVision::landmarkTheoreticalSize/2,0.));
			object_points.push_back(Point3f (NaoVision::landmarkTheoreticalSize/2,NaoVision::landmarkTheoreticalSize/2,0.));
						
			//object_points.push_back(Point3f (-w/2,-w/2,0));
			//object_points.push_back(Point3f (-w/2,w/2,0));
			//object_points.push_back(Point3f (w/2,w/2,0));
			//object_points.push_back(Point3f (w/2,-w/2,0));
			
			

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
				landmarkToCameraTransform.at<double>(i,3)=tvec.at<double>(i,0);
			}
			landmarkToCameraTransform.at<double>(3,3)=1.0;
			
			Rotx_minus90.at<double>(0,0)=1.0;
			//Rotx_minus90.at<float>(1,1)=cos(-CV_PI/2); Rotx_minus90.at<float>(1,2)=-sin(-CV_PI/2);
			//Rotx_minus90.at<float>(2,1)=sin(-CV_PI/2); Rotx_minus90.at<float>(2,2)=cos(-CV_PI/2);
			Rotx_minus90.at<double>(1,1)=0.; 	Rotx_minus90.at<double>(1,2)=1.;
			Rotx_minus90.at<double>(2,1)=-1.; 	Rotx_minus90.at<double>(2,2)=0.;
			Rotx_minus90.at<double>(3,3)=1.0;

			//Rotz_plus90.at<float>(0,0)=cos(CV_PI/2); Rotz_plus90.at<float>(0,1)=-sin(CV_PI/2);
			//Rotz_plus90.at<float>(1,0)=sin(CV_PI/2); Rotz_plus90.at<float>(1,1)=cos(CV_PI/2);
			Rotz_plus90.at<double>(0,0)=0.; 	Rotz_plus90.at<double>(0,1)=-1.;
			Rotz_plus90.at<double>(1,0)=1.;	Rotz_plus90.at<double>(1,1)=0.;
			Rotz_plus90.at<double>(2,2)=1.0;
			Rotz_plus90.at<double>(3,3)=1.0;
			
			//## Transformation from the QR-code coordinate system to the camera coordinate system
			cameraToLandmarkTransformMatrix = Rotx_minus90*landmarkToCameraTransform;
			cameraToLandmarkTransformMatrix = Rotz_plus90*cameraToLandmarkTransformMatrix;
			cameraToLandmarkTransformMatrix.at<double>(0,3)*=-1.;	cameraToLandmarkTransformMatrix.at<double>(2,3)*=-1.;

			//## Transformation from the camera coordinate system to the robot coordinate system
			robotToLandmarkMatrix = cameraToLandmarkTransformMatrix*robotToCameraMatrix_;

			//cout<<landmarkToCameraTransform<<endl;
			//cout<<Rotx_minus90<<endl;
			//cout<<Rotz_plus90<<endl;
			//cout<<cameraToLandmarkTransformMatrix<<endl;	
			//cout<<robotToCameraMatrix_<<endl;		
			//cout<<robotToLandmarkMatrix<<endl;



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

	/*std::vector< std::vector<float> >*/
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
