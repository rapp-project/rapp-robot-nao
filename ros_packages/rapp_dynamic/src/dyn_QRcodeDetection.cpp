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

	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;
	
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 1");//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
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
