#include "NaoVision.h"
//#include <zbar.h> // for QRcode detection

main(int argc, char **argv)
{
	//NaoVision NaoVision_(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;

	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;
	
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 1");
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system
	if (frame.height == 0 || frame.width == 0) //frame is empty
		std::cout<<"Camera frame is empty"<<std::endl;
	else
	{
		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix);
		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		{
			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;
		}
	}

   return 0;
}
