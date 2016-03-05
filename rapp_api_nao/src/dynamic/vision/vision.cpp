#include <rapp/dynamic/vision/vision.hpp>

namespace rapp {
namespace dynamic {
  	vision::vision(int argc, char ** argv){
  	}
  	vision::~vision(){
  	}

	rapp::object::QRCode3D vision::qrCodeDetection(rapp::object::picture::Ptr imgFrame, std::vector<std::vector<float>> robotToCameraMatrix, double camera_matrix[][3], float landmarkTheoreticalSize)
	{
		zbar::ImageScanner set_zbar;
	
		// initializing the structure QRcodeDetectionStruct -- set to default
		rapp::object::QRCode3D QRcodeDetectionStruct;
		if (sizeof((*imgFrame))<10) return QRcodeDetectionStruct;
		QRcodeDetectionStruct.clear();
	
		////cv::Mat from the std::vector<byte>
		cv::Mat cv_mat((*imgFrame).bytearray(),true);
		////decoding the image
		//cv::Mat cv_frame(cv::imdecode(cv_mat,1)); //put 0 if you want greyscale
		cv::Mat frame_grayscale(cv::imdecode(cv_mat,1));//decoding the image to the gray scale
		//frame_grayscale = bytesToMat(image_,width,height);
		
		//boost::shared_ptr<void const> tracked_object;
		//cv_frame = cv_bridge::toCvShare(frame_, tracked_object, frame_.encoding)->image; //conversion from sensor_msgs::Image to cv::Mat
	
		// Convert to grayscale
		//cv::cvtColor(cv_frame, frame_grayscale, CV_BGR2GRAY);
		
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
		
		// Camera Intrinsic Matrix -- from Camera calibration
		/*
	//	double camera_matrix[3][3];
		if (width == 1280 && height == 960){
			for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			camera_matrix[i][j] = camera_top_matrix_3[i][j];
			//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_3);
		}else if (width == 640 && height == 480){
			for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			camera_matrix[i][j] = camera_top_matrix_2[i][j];
			//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_2);
		}else if (width == 320 && height == 240){
			for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			camera_matrix[i][j] = camera_top_matrix_1[i][j];
			//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_1);
		}else
		{
			std::cout << "Wrong camera intrinsic matrix, the position may be computed wrongly" << std::endl;
			for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			camera_matrix[i][j] = camera_top_matrix_2[i][j];
			//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, rapp::robot::VisionImpl::camera_top_matrix_2);
		}
		*/

		for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
	
			std::vector<cv::Point3f> object_points;
			std::vector<cv::Point2f> pixel_coords;
	
			pixel_coords.clear();
			pixel_coords.push_back(cv::Point2f(symbol->get_location_x(0), symbol->get_location_y(0)));
			pixel_coords.push_back(cv::Point2f(symbol->get_location_x(1), symbol->get_location_y(1)));
			pixel_coords.push_back(cv::Point2f(symbol->get_location_x(2), symbol->get_location_y(2)));
			pixel_coords.push_back(cv::Point2f(symbol->get_location_x(3), symbol->get_location_y(3)));

			std::vector<std::vector<double>> landmarkInCameraCoordinate;
			std::vector<std::vector<double>> landmarkInRobotCoordinate;
			std::vector<double> row_matrix;

			cv::Mat rvec;//(3,1,cv::DataType<float>::type);
			cv::Mat tvec;//(3,1,cv::DataType<float>::type);
			cv::Mat rotationMatrix;//(3,3,cv::DataType<float>::type);
			cv::Mat cameraIntrinsicMatrix;
			cv::Mat distCoeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
			cv::Mat rotation_matrix, translation_matrix;
			cv::Mat landmarkToCameraTransform = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat cameraToLandmarkTransformMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat robotToLandmarkMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat Rotx_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat Rotz_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat Mat_I = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
			cv::Mat robotToCameraMat = cv::Mat(4, 4, cv::DataType<float>::type);//cv::CV_32FC1, robotToCameraMatrix);//initialization from the given data
			//cv::Mat robotToCameraMat = cv::Mat::zeros(4, 4, cv::DataType<float>::type);
			//robotToCameraMat = robotToCameraMatrix;
			try{
				for(unsigned int i=0;i<4;i++)
				for(unsigned int j=0;j<4;j++)
				robotToCameraMat.at<float>(i,j)=robotToCameraMatrix[i][j]; //copying the given data from robotToCameraMatrix to robotToCameraMat
				
			}catch(const std::runtime_error& re)
			{
				// speciffic handling for runtime_error
				std::cerr << "Runtime error: " << re.what() << std::endl;
			}catch(const std::exception& ex)
			{
				// speciffic handling for all exceptions extending std::exception, except
				// std::runtime_error which is handled explicitly
				std::cerr << "Error occurred: " << ex.what() << std::endl;
			}catch(...){
				std::cerr << "Unknown failure occured. Possible memory corruption" << std::endl;
				return QRcodeDetectionStruct;
			}
	
			//initialization from the given data
			//for(int i=0;i<4;i++)
			//for(int j=0;j<4;j++)
			//robotToCameraMat.at<float>(i,j)=robotToCameraMatrix[i][j];
			
			//$$$$$$$$$$$$$$$$$$
			std::vector<double> m00, m01, m02, m10, m12, m20, m21, m22, euler1, euler2, euler3;
			//const double PI = 3.14159265359f;
	
			// Model for SolvePnP // x-> y^
			object_points.clear();
	
			//z^y<-
			object_points.push_back(cv::Point3f(0, landmarkTheoreticalSize / 2, landmarkTheoreticalSize / 2));
			object_points.push_back(cv::Point3f(0, landmarkTheoreticalSize / 2, -landmarkTheoreticalSize / 2));
			object_points.push_back(cv::Point3f(0, -landmarkTheoreticalSize / 2, -landmarkTheoreticalSize / 2));
			object_points.push_back(cv::Point3f(0, -landmarkTheoreticalSize / 2, landmarkTheoreticalSize / 2));
	
			// Camera Intrinsic Matrix -- from Camera calibration
			cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_matrix);
	
			cv::solvePnP(cv::Mat(object_points), cv::Mat(pixel_coords), cameraIntrinsicMatrix, distCoeffs, rvec, tvec, false);//, CV_ITERATIVE );
	
			cv::Rodrigues(rvec, rotationMatrix);


			// landmarkToCameraTransform computation
			for (int i = 0; i<3; i++)
			{
				for (int j = 0; j<3; j++)
					landmarkToCameraTransform.at<double>(i, j) = rotationMatrix.at<double>(i, j);
			}
			landmarkToCameraTransform.at<double>(0, 3) = tvec.at<double>(0, 0); //x->y^ : x
			landmarkToCameraTransform.at<double>(1, 3) = tvec.at<double>(1, 0); //x->y^ : y
			landmarkToCameraTransform.at<double>(2, 3) = tvec.at<double>(2, 0); //x->y^ : z
			landmarkToCameraTransform.at<double>(3, 3) = 1.0;


			Rotx_minus90.at<double>(0, 0) = 1.f;
			Rotx_minus90.at<double>(1, 1) = 0.f; Rotx_minus90.at<double>(1, 2) = +1.f;
			Rotx_minus90.at<double>(2, 1) = -1.f; Rotx_minus90.at<double>(2, 2) = 0.f;
			Rotx_minus90.at<double>(3, 3) = 1.f;

			Rotz_minus90.at<double>(0, 0) = 0.f; Rotz_minus90.at<double>(0, 1) = +1.f;
			Rotz_minus90.at<double>(1, 0) = -1.f; Rotz_minus90.at<double>(1, 1) = 0.f;
			Rotz_minus90.at<double>(2, 2) = 1.f;
			Rotz_minus90.at<double>(3, 3) = 1.f;


			//#####################			
			//## Transformation from the QR-code coordinate system to the camera coordinate system
			cameraToLandmarkTransformMatrix = Rotz_minus90*Rotx_minus90*landmarkToCameraTransform;
			//#####################

			//#####################						
			//## Transformation from the camera coordinate system to the robot coordinate system
			robotToLandmarkMatrix = robotToCameraMat*cameraToLandmarkTransformMatrix;
			//#####################			
		
			//cv::Mat -> vector<vector<double>>
			landmarkInCameraCoordinate.clear();
			landmarkInCameraCoordinate.resize(cameraToLandmarkTransformMatrix.rows);
			for(int i=0;i<cameraToLandmarkTransformMatrix.rows;i++){
				landmarkInCameraCoordinate[i].resize(cameraToLandmarkTransformMatrix.cols);
				for(int j=0;j<cameraToLandmarkTransformMatrix.cols;j++){
					landmarkInCameraCoordinate[i][j]=cameraToLandmarkTransformMatrix.at<double>(i,j);
				}
			}
			landmarkInRobotCoordinate.clear();
			landmarkInRobotCoordinate.resize(robotToLandmarkMatrix.rows);
			for(int i=0;i<robotToLandmarkMatrix.rows;i++){
				landmarkInRobotCoordinate[i].resize(robotToLandmarkMatrix.cols);
				for(int j=0;j<robotToLandmarkMatrix.cols;j++){
					landmarkInRobotCoordinate[i][j]=robotToLandmarkMatrix.at<double>(i,j);
				}
			}
		
			QRcodeDetectionStruct.LandmarkInCameraCoordinate.push_back(landmarkInCameraCoordinate);
			QRcodeDetectionStruct.LandmarkInRobotCoordinate.push_back(landmarkInRobotCoordinate);

			counter++;
			QRcodeDetectionStruct.QRmessage.push_back(symbol->get_data());
			QRcodeDetectionStruct.isQRcodeFound = true;

		}
		QRcodeDetectionStruct.numberOfQRcodes = counter;

		return QRcodeDetectionStruct;
	}

} // namespace rapp
} // namespace dynamic
