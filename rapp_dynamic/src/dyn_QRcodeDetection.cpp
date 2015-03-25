//#####################
// written by Jan Figat
//#####################

#include "NaoVision.h"

main(int argc, char **argv)
{
	//NaoVision NaoVision_(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;
	std::vector<double> R00,R10,R20,R21,R22; //vectors containing the values of the corresponding cells of the matrix
	std::vector<double> euler_x, euler_y, euler_z; //vectors cantaining the values of euler angles;

	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;
	
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 1"/*Auto Exposure Algorithm*/, 3/*camera resolution*/);//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3;; 3-4VGA,2-VGA,1-QVGA (camera resolution)
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
	if (frame.height == 0 || frame.width == 0) //frame is empty
		std::cout<<"Camera frame is empty"<<std::endl;
	else
	{
		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix);

		// for displaying the rotation angles of QR-codes in robot coordinate system
		R00.clear();R10.clear();R20.clear();R21.clear();R22.clear();
		euler_x.clear();euler_y.clear();euler_z.clear();
		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		{
			R00.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[i].at<double>(0,0));
			R10.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[i].at<double>(1,0)); 
			R20.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[i].at<double>(2,0)); 
			R21.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[i].at<double>(2,1)); 
			R22.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[i].at<double>(2,2)); 
		}
		// computation of euler angles
		euler_y = NAO_Vision.compute_euler_y(R20);
		euler_z = NAO_Vision.compute_euler_z(R10,R00,euler_y);
		euler_x = NAO_Vision.compute_euler_x(R21,R22,euler_y);

		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		{
			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;
			std::cout<<"rotation of QR-code around OZ axis in robot frame: "<<euler_z[i]*180.f/M_PI<<" in degrees"<<std::endl;

		}
	}

   return 0;
}
