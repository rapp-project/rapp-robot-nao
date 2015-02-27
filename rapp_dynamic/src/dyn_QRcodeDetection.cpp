#include "NaoVision.h"

main(int argc, char **argv)
{
	NaoVision NaoVision_(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;

	NaoVision_.QRcodeDetection QRcodeDetectionStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;
	
	frame = NaoVision_.captureImage("top - adaptive auto exposure 1");
	robotToCameraMatrix = NaoVision_.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system
	QRcodeDetectionStruct = NaoVision_.qrCodeDetection(frame, scanner, robotToCameraMatrix);
	for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		std::cout<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;

    return 0;
}
